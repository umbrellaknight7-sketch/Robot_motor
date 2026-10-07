#include "erob_driver/engine.hpp"
#include "erob_driver/config.hpp"
#include "erob_driver/pdo.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace erob {
Engine::Engine(BusConfig c, Transport &t) : config_(std::move(c)), transport_(t) {
  const auto n = config_.drives.size();
  state_.joints.resize(n);
  outputs_.resize(n);
  hold_tau_.resize(n);
  inputs_.resize(n);
  desired_.resize(n);
  commands_.resize(n);
  stationary_since_.assign(n, 0);
  switching_since_.assign(n, 0);
  stop_q_.resize(n);
  stop_v_.resize(n);
  hold_claimed_.resize(n);
}

void Engine::configure() {
  validate_config(config_);
  devices_ = transport_.discover(config_);
  mapping_ = match_devices(config_, devices_);
  transport_.configure(config_, mapping_);
  for (size_t i = 0; i < desired_.size(); ++i) {
    desired_[i].mode = config_.drives[i].initial_mode;
    outputs_[i] = encode(desired_[i]);
  }
}

bool Engine::activate(Nanoseconds now) {
  if (state_.health.phase != Phase::Inactive || !state_.health.feedback_valid)
    return false;
  validate_activation(config_);
  for (const auto &s : state_.joints)
    if (s.error || !s.valid || (s.status & 8))
      return false;

  // Seed activation from feedback before following the drive enable sequence.
  state_.health.phase = Phase::Enabling;
  state_.health.hold_reason = Fault::None;
  state_.health.resume_ready = false;
  healthy_since_ = 0;
  transition_start_ = now;
  enabled_since_ = 0;
  have_command_ = command_watchdog_armed_ = false;
  disabled_confirmed_ = false;
  for (size_t i = 0; i < commands_.size(); ++i) {
    commands_[i] = {config_.drives[i].initial_mode, state_.joints[i].position, 0, 0, false};
    desired_[i].position = decode(inputs_[i]).position;
    desired_[i].velocity = 0;
    desired_[i].torque = 0;
    desired_[i].velocity_offset = desired_[i].torque_offset = 0;
    desired_[i].mode = config_.drives[i].initial_mode;
  }
  return true;
}

void Engine::deactivate(Nanoseconds now) {
  if (state_.health.phase == Phase::Inactive || state_.health.phase == Phase::Fault ||
      state_.health.phase == Phase::Disabling)
    return;
  state_.health.phase = Phase::Disabling;
  state_.health.resume_ready = false;
  transition_start_ = now;
  have_command_ = command_watchdog_armed_ = false;
  disabled_confirmed_ = false;
  for (auto &c : commands_)
    c.claimed = false;
}

bool Engine::reset_fault() {
  if (state_.health.phase != Phase::Fault || !disabled_confirmed_ || !state_.health.feedback_valid)
    return false;
  for (const auto &s : state_.joints)
    if (s.error || (s.status & 8))
      return false;
  state_.health.phase = Phase::Inactive;
  state_.health.fault = Fault::None;
  state_.health.fault_motor = -1;
  have_command_ = false;
  return true;
}

void Engine::trip(Fault fault, int motor, Nanoseconds now) noexcept {
  if (state_.health.phase == Phase::Fault)
    return;
  state_.health.fault = fault;
  state_.health.fault_motor = motor;
  state_.health.phase = Phase::Fault;
  state_.health.resume_ready = false;
  fault_since_ = now;
  have_command_ = command_watchdog_armed_ = false;
  disabled_confirmed_ = false;
  for (auto &c : commands_)
    c.claimed = false;
  std::fill(switching_since_.begin(), switching_since_.end(), 0);
}

void Engine::reject(RejectReason reason, int joint, uint64_t count) noexcept {
  state_.health.rejected_commands += count;
  state_.health.rejection = reason;
  state_.health.rejected_joint = joint;
}

// Recoverable CSP overruns enter hold first and hard-fault at three times the threshold.
void Engine::monitor_deadline(Nanoseconds now, Nanoseconds scheduled) noexcept {
  if (now - scheduled > config_.lateness_ns) {
    ++state_.health.missed_deadlines;
    ++late_cycles_;
    healthy_since_ = 0;
    const bool recoverable = config_.csp_hold_enabled &&
                             (state_.health.phase == Phase::Active || holding(state_.health.phase));
    if (late_cycles_ >= uint64_t(config_.missed_deadlines) * (recoverable ? 3 : 1))
      trip(Fault::DeadlineMissed, -1, now);
    else if (recoverable && late_cycles_ >= config_.missed_deadlines)
      deadline_pending_ = true;
  } else
    late_cycles_ = 0;
  state_.health.late_cycles = late_cycles_;
}

// Latch the hold generation once; feedback must confirm the stopping trajectory.
void Engine::begin_hold(Fault reason, Nanoseconds now) noexcept {
  auto &h = state_.health;
  if (holding(h.phase))
    return;
  if (!config_.csp_hold_enabled || h.phase != Phase::Active) {
    trip(reason, -1, now);
    return;
  }
  for (size_t i = 0; i < commands_.size(); ++i) {
    hold_claimed_[i] = commands_[i].claimed;
    hold_tau_[i] = joint_effort(desired_[i].torque_offset, config_.drives[i]);
    state_.joints[i].hold_start = state_.joints[i].position;
    state_.joints[i].hold_target = joint_position(desired_[i].position, config_.drives[i]);
  }
  offset_stop_start_ = now;
  h.hold_reason = reason;
  ++h.hold_generation;
  h.resume_ready = h.claims_released = false;
  healthy_since_ = settled_since_ = 0;
  h.phase = Phase::HoldUnconfirmed;
  if (h.feedback_valid)
    plan_stop(now);
}

// Use the longest per-joint acceleration/jerk stopping time for the whole group.
void Engine::plan_stop(Nanoseconds now) noexcept {
  auto &h = state_.health;
  double duration = 0;
  for (size_t i = 0; i < desired_.size(); ++i) {
    auto &s = state_.joints[i];
    const auto &d = config_.drives[i];
    if (!s.valid || !enabled(s.status) || s.error || s.mode != Mode::CSP ||
        !std::isfinite(s.position) || !std::isfinite(s.velocity) ||
        std::abs(s.velocity) > (d.csp_feedforward ? std::min(d.max_velocity, d.max_velocity_offset)
                                                  : d.max_velocity) ||
        std::abs(s.position - joint_position(desired_[i].position, d)) > d.hold_tracking_error) {
      trip(Fault::HoldFailed, int(i), now);
      return;
    }
    stop_q_[i] = s.position;
    stop_v_[i] = std::abs(s.velocity) <= d.standstill_velocity ? 0 : s.velocity;
    duration = std::max({duration, 1.5 * std::abs(stop_v_[i]) / d.hold_acceleration,
                         std::sqrt(6 * std::abs(stop_v_[i]) / d.hold_jerk)});
  }
  // Round duration upward to whole cycles, preserving the acceleration/jerk bounds.
  const double cycles = std::ceil(duration * 1e9 / config_.cycle_ns);
  if (!std::isfinite(cycles) || cycles * config_.cycle_ns > double(INT64_MAX / 4)) {
    trip(Fault::HoldFailed, -1, now);
    return;
  }
  stop_duration_ = Nanoseconds(cycles) * config_.cycle_ns;
  duration = double(stop_duration_) / 1e9;
  for (size_t i = 0; i < desired_.size(); ++i) {
    const auto &d = config_.drives[i];
    auto &s = state_.joints[i];
    const double end = stop_q_[i] + stop_v_[i] * duration / 2;
    Command c{Mode::CSP, end, 0, 0, true};
    Command start{Mode::CSP, stop_q_[i], 0, 0, true};
    if (!command_valid(c, d) || !command_valid(start, d)) {
      trip(Fault::HoldFailed, int(i), now);
      return;
    }
    s.hold_start = stop_q_[i];
    s.hold_target = end;
  }
  stop_start_ = now;
  settling_deadline_ = now + stop_duration_ + config_.transition_timeout_ns;
  settled_since_ = 0;
  h.phase = Phase::Decelerating;
}

void Engine::hold_tick(Nanoseconds now) noexcept {
  auto &h = state_.health;
  h.resume_ready = false;
  // Withdraw feedforward even when feedback cannot confirm a stop. Position
  // targets remain under the existing hold/fault state machine.
  for (size_t i = 0; i < desired_.size(); ++i) {
    const auto &d = config_.drives[i];
    if (!d.csp_feedforward)
      continue;
    const double u = std::clamp((now - offset_stop_start_) / (d.offset_ramp_seconds * 1e9), 0., 1.);
    const double blend = 1 - (10 * u * u * u - 15 * u * u * u * u + 6 * u * u * u * u * u);
    desired_[i].velocity_offset = 0;
    desired_[i].torque_offset =
        int16_t(std::llround(d.direction * hold_tau_[i] * blend / d.nm_per_raw));
  }
  if (!h.feedback_valid) {
    h.phase = Phase::HoldUnconfirmed;
    for (size_t i = 0; i < desired_.size(); ++i)
      state_.joints[i].hold_target = joint_position(desired_[i].position, config_.drives[i]);
    settled_since_ = healthy_since_ = 0;
    return; // Freeze position; torque withdrawal continues independently.
  }
  if (h.phase == Phase::HoldUnconfirmed)
    plan_stop(now);
  if (!holding(h.phase))
    return;
  bool settled = true;
  const double duration = double(stop_duration_) / 1e9;
  const double u =
      stop_duration_ ? std::clamp(double(now - stop_start_) / stop_duration_, 0., 1.) : 1.;
  for (size_t i = 0; i < desired_.size(); ++i) {
    auto &out = desired_[i];
    const auto &s = state_.joints[i];
    const auto &d = config_.drives[i];
    if (std::abs(s.position - joint_position(out.position, d)) > d.hold_tracking_error) {
      trip(Fault::HoldFailed, int(i), now);
      return;
    }
    out.control = 15;
    out.mode = Mode::CSP;
    out.velocity = out.torque = 0;
    if (h.phase == Phase::Decelerating) {
      const double q = stop_q_[i] + stop_v_[i] * duration * (u - u * u * u + .5 * u * u * u * u);
      const auto torque_offset = out.torque_offset;
      apply_command({Mode::CSP, q, 0, 0, true}, d, out);
      out.torque_offset = torque_offset;
      if (d.csp_feedforward)
        out.velocity_offset =
            int32_t(std::llround(d.direction * stop_v_[i] * (1 - 3 * u * u + 2 * u * u * u) *
                                 d.velocity_counts_per_rad_s));
    }
    settled = settled && out.torque_offset == 0 && std::abs(s.velocity) <= d.standstill_velocity &&
              std::abs(s.position - s.hold_target) <= d.hold_tracking_error;
  }
  if (u >= 1 && settled) {
    if (!settled_since_)
      settled_since_ = now;
    if (now - settled_since_ >= config_.standstill_ns)
      h.phase = Phase::Holding;
  } else {
    settled_since_ = 0;
    if (h.phase == Phase::Holding) {
      // A small disturbance after a long hold gets a new settling interval;
      // the fixed target and original stopping profile are not changed.
      h.phase = Phase::Decelerating;
      settling_deadline_ = now + config_.transition_timeout_ns;
    }
  }
  if (h.phase != Phase::Holding && now > settling_deadline_) {
    trip(Fault::HoldFailed, -1, now);
    return;
  }
  h.resume_ready = ready_to_resume(now);
}

// Require released/reacquired claims, settled feedback and a fresh measured-position command.
bool Engine::ready_to_resume(Nanoseconds now) const noexcept {
  const auto &h = state_.health;
  if (h.phase != Phase::Holding || !h.feedback_valid || !h.claims_released ||
      now - h.feedback_stamp_ns >= config_.feedback_timeout_ns || !healthy_since_ ||
      now - healthy_since_ < config_.standstill_ns || !settled_since_ ||
      now - settled_since_ < config_.standstill_ns || !have_command_ ||
      now - last_command_ >= config_.command_timeout_ns)
    return false;
  for (size_t i = 0; i < commands_.size(); ++i) {
    const auto &c = commands_[i];
    if (hold_claimed_[i] && !c.claimed)
      return false;
    if (c.claimed &&
        (c.velocity_offset != 0 || c.torque_offset != 0 || c.mode != Mode::CSP ||
         std::abs(c.position - state_.joints[i].position) > config_.drives[i].hold_tracking_error))
      return false;
  }
  return true;
}

// Only an explicit request for the current hold generation can resume control.
bool Engine::resume(uint64_t generation, Nanoseconds now) noexcept {
  if (generation != state_.health.hold_generation || !ready_to_resume(now))
    return false;
  state_.health.phase = Phase::Active;
  state_.health.resume_ready = false;
  state_.health.hold_reason = Fault::None;
  last_command_ = resume_after_ = now;
  state_.health.last_command_ns = now;
  have_command_ = false; // Discard the last held frame; only post-resume submissions may move.
  // Preserve claims for the command watchdog, but discard all held motion values.
  return true;
}

void Engine::tick(Nanoseconds now, const CommandFrame *frame) noexcept {
  auto &h = state_.health;
  h.stamp_ns = now;
  ++h.cycle;
  const auto ex = transport_.exchange(outputs_, inputs_);
  h.wkc = ex.wkc;
  h.expected_wkc = ex.expected_wkc;
  h.feedback_valid = ex.valid && ex.wkc == ex.expected_wkc && ex.expected_wkc > 0;
  // A repeated DC timestamp means the bus returned stale process data, even
  // if its WKC is otherwise plausible. Unchanged positions alone are valid.
  if (h.feedback_valid && last_dc_time_ && ex.dc_time <= last_dc_time_)
    h.feedback_valid = false;
  if (h.feedback_valid)
    last_dc_time_ = ex.dc_time;

  if (h.feedback_valid && ex.dc_time) {
    auto delta = ex.dc_time % config_.cycle_ns;
    if (delta > config_.cycle_ns / 2)
      delta -= config_.cycle_ns;
    h.dc_offset_ns =
        std::clamp<Nanoseconds>(-delta / 100, -config_.cycle_ns / 10, config_.cycle_ns / 10);
  }

  if (!h.feedback_valid) {
    ++bad_cycles_;
    for (auto &s : state_.joints)
      s.valid = false;
    healthy_since_ = 0;
    const bool recoverable =
        config_.csp_hold_enabled && (h.phase == Phase::Active || holding(h.phase));
    const auto age = h.feedback_stamp_ns ? now - h.feedback_stamp_ns : 0;
    if (bad_cycles_ >= uint64_t(config_.bad_cycles) * (recoverable ? 3 : 1) ||
        (h.feedback_stamp_ns && age >= config_.feedback_timeout_ns * (recoverable ? 3 : 1)))
      trip(Fault::FeedbackLost, ex.fault_motor, now);
    else if (recoverable &&
             (bad_cycles_ >= config_.bad_cycles || age >= config_.feedback_timeout_ns))
      begin_hold(Fault::FeedbackLost, now);
  } else {
    bad_cycles_ = 0;
    h.feedback_stamp_ns = now;
    for (size_t i = 0; i < inputs_.size(); ++i) {
      const auto p = decode(inputs_[i]);
      auto &s = state_.joints[i];
      const auto &d = config_.drives[i];
      s.position = joint_position(p.position, d);
      s.velocity = joint_velocity(p.velocity, d);
      s.raw_torque = p.torque;
      s.effort_valid = d.effort_verified;
      s.effort =
          d.effort_verified ? joint_effort(p.torque, d) : std::numeric_limits<double>::quiet_NaN();
      s.status = p.status;
      s.error = p.error;
      s.mode = p.mode;
      s.valid = true;
      if (std::abs(s.velocity) <= d.standstill_velocity) {
        if (!stationary_since_[i])
          stationary_since_[i] = now;
      } else
        stationary_since_[i] = 0;
      if ((h.phase == Phase::Active || h.phase == Phase::Enabling || holding(h.phase)) &&
          ((p.status & 8) || p.error))
        trip(Fault::DriveFault, int(i), now);
      if ((h.phase == Phase::Active || holding(h.phase)) && !enabled(p.status))
        trip(Fault::DriveFault, int(i), now);
      if (config_.csp_hold_enabled && (h.phase == Phase::Active || holding(h.phase)) &&
          p.mode != Mode::CSP)
        trip(Fault::ModeTimeout, int(i), now);
    }
  }
  h.bad_cycles = bad_cycles_;
  if (h.feedback_valid && late_cycles_ == 0) {
    if (!healthy_since_)
      healthy_since_ = now;
  } else
    healthy_since_ = 0;
  if (deadline_pending_) {
    if (h.phase == Phase::Active)
      begin_hold(Fault::DeadlineMissed, now);
    deadline_pending_ = false;
  }

  // Reject invalid frames without replacing the accepted command or extending its watchdog.
  if (frame && (h.phase == Phase::Active || holding(h.phase))) {
    RejectReason why = RejectReason::None;
    int joint = -1;
    if (frame->joints.size() != commands_.size())
      why = RejectReason::Shape;
    else if (frame->generation <= min_generation_)
      why = RejectReason::Order;
    else if (frame->stamp_ns < active_since_ || frame->stamp_ns <= resume_after_ ||
             frame->stamp_ns > now || now - frame->stamp_ns >= config_.command_timeout_ns)
      why = RejectReason::Stale;
    else
      for (size_t i = 0; i < commands_.size(); ++i) {
        if (!command_valid(frame->joints[i], config_.drives[i])) {
          why = RejectReason::Value;
          joint = int(i);
          break;
        }
        if (config_.csp_hold_enabled && frame->joints[i].mode != Mode::CSP) {
          why = RejectReason::Mode;
          joint = int(i);
          break;
        }
      }
    if (why != RejectReason::None)
      reject(why, joint);
    else {
      commands_ = frame->joints;
      min_generation_ = frame->generation;
      last_command_ = frame->stamp_ns;
      h.last_command_ns = last_command_;
      have_command_ = true;
      command_watchdog_armed_ =
          std::any_of(commands_.begin(), commands_.end(), [](const auto &c) { return c.claimed; });
      if (holding(h.phase) &&
          std::none_of(commands_.begin(), commands_.end(), [](const auto &c) { return c.claimed; }))
        h.claims_released = true;
    }
  }

  if (h.phase == Phase::Active && command_watchdog_armed_ &&
      now - last_command_ >= config_.command_timeout_ns)
    begin_hold(Fault::CommandExpired, now);

  // Enter active control after all drives report enabled in the requested mode for the delay.
  if (h.phase == Phase::Enabling) {
    bool all = true;
    for (size_t i = 0; i < desired_.size(); ++i) {
      auto &out = desired_[i];
      const auto &s = state_.joints[i];
      out.control = enable_word(s.status);
      out.velocity = 0;
      out.torque = 0;
      out.velocity_offset = out.torque_offset = 0;
      out.position = decode(inputs_[i]).position;
      all = all && s.valid && enabled(s.status) && s.mode == out.mode;
    }
    if (all) {
      if (!enabled_since_)
        enabled_since_ = now;
      if (now - enabled_since_ >= config_.enable_delay_ns) {
        h.phase = Phase::Active;
        active_since_ = now;
        have_command_ = false;
      }
    } else
      enabled_since_ = 0;
    if (now - transition_start_ > config_.transition_timeout_ns)
      trip(Fault::TransitionTimeout, -1, now);
  }

  if (h.phase == Phase::Active && h.feedback_valid) {
    for (size_t i = 0; i < desired_.size(); ++i) {
      auto &out = desired_[i];
      auto &s = state_.joints[i];
      const auto &c = commands_[i];
      out.control = 15;
      out.velocity = 0;
      out.torque = 0;
      out.velocity_offset = out.torque_offset = 0;
      const auto requested = c.claimed ? c.mode : out.mode;
      s.requested_mode = requested;
      if (requested != out.mode) {
        if (!stationary_since_[i] || now - stationary_since_[i] < config_.standstill_ns) {
          trip(Fault::MovingModeSwitch, int(i), now);
          break;
        }
        out.mode = requested;
        out.position = decode(inputs_[i]).position;
        switching_since_[i] = now;
      }
      if (s.mode != out.mode) {
        if (!switching_since_[i])
          switching_since_[i] = now;
        s.switching = true;
        out.position = decode(inputs_[i]).position;
        if (now - switching_since_[i] >= config_.mode_timeout_ns) {
          trip(Fault::ModeTimeout, int(i), now);
          break;
        }
      } else if (switching_since_[i]) {
        // Acknowledge mode first. The new target is applied on the following
        // valid cycle.
        switching_since_[i] = 0;
        s.switching = false;
        out.position = decode(inputs_[i]).position;
      } else if (c.claimed && have_command_)
        apply_command(c, config_.drives[i], out);
    }
  }
  if (holding(h.phase))
    hold_tick(now);
  if (h.phase == Phase::Inactive) {
    for (size_t i = 0; i < desired_.size(); ++i) {
      auto &out = desired_[i];
      out.control = 0;
      out.velocity = 0;
      out.torque = 0;
      out.velocity_offset = out.torque_offset = 0;
      if (h.feedback_valid)
        out.position = decode(inputs_[i]).position;
    }
  }

  // Send quick-stop first, then disable after the configured interval.
  if (h.phase == Phase::Fault || h.phase == Phase::Disabling) {
    const auto start = h.phase == Phase::Fault ? fault_since_ : transition_start_;
    bool disabled = h.feedback_valid;
    for (size_t i = 0; i < desired_.size(); ++i) {
      auto &out = desired_[i];
      out.control = now - start >= config_.disable_delay_ns ? 0 : 2;
      out.velocity = 0;
      out.torque = 0;
      out.velocity_offset = out.torque_offset = 0;
      if (h.feedback_valid)
        out.position = decode(inputs_[i]).position;
      state_.joints[i].switching = false;
      disabled = disabled && ((state_.joints[i].status & 0x6f) == 0x40 ||
                              (state_.joints[i].status & 0x6f) == 8);
    }
    disabled_confirmed_ = disabled && now - start >= config_.disable_delay_ns;
    if (h.phase == Phase::Disabling && disabled_confirmed_)
      h.phase = Phase::Inactive;
    else if (h.phase == Phase::Disabling && now - start > config_.transition_timeout_ns)
      trip(Fault::TransitionTimeout, -1, now);
  }
  for (size_t i = 0; i < outputs_.size(); ++i)
    outputs_[i] = encode(desired_[i]);
  for (size_t i = 0; i < desired_.size(); ++i) {
    state_.joints[i].commanded_velocity_offset =
        joint_velocity(desired_[i].velocity_offset, config_.drives[i]);
    state_.joints[i].commanded_torque_offset =
        joint_effort(desired_[i].torque_offset, config_.drives[i]);
  }
}
} // namespace erob
