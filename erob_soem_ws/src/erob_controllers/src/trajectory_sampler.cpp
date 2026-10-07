#include "erob_controllers/trajectory_sampler.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace erob_controllers {
namespace {
double blend(double u) {
  return u * u * u * (10 + u * (-15 + 6 * u));
}
}

const char *phase_name(Phase p) {
  switch (p) {
  case Phase::Inactive:
    return "inactive";
  case Phase::Holding:
    return "holding";
  case Phase::Moving:
    return "moving";
  case Phase::Stopping:
    return "stopping";
  case Phase::Fault:
    return "fault";
  }
  return "fault";
}

void TrajectorySampler::configure(size_t size, const std::array<Limits, capacity> &limits) {
  if (!size || size >= capacity)
    throw std::invalid_argument("Invalid joint count");
  size_ = size;
  limits_ = limits;
}

void TrajectorySampler::activate(const Values &q, const Values &v, double now) {
  state_ = Snapshot{};
  state_.phase = Phase::Holding;
  state_.position = state_.actual = q;
  state_.velocity = v;
  state_.stamp = now;
}

// The quintic peak velocity, acceleration and jerk must all fit the joint limits.
bool TrajectorySampler::feasible(const Request &r, const Values &q,
                                 const Values &v) const noexcept {
  if (!std::isfinite(r.duration) || r.duration < 0.001 || r.duration > 3600)
    return false;
  for (size_t i = 0; i < size_; ++i) {
    const auto &l = limits_[i];
    const double distance = std::abs(r.position[i] - q[i]);
    if (!std::isfinite(q[i]) || !std::isfinite(v[i]) || !std::isfinite(r.position[i]) ||
        !std::isfinite(r.torque[i]) || std::abs(v[i]) > l.standstill || q[i] < l.lower ||
        q[i] > l.upper || r.position[i] < l.lower || r.position[i] > l.upper ||
        std::abs(r.torque[i]) > l.torque || 1.875 * distance / r.duration > l.velocity ||
        5.773502691896258 * distance / (r.duration * r.duration) > l.acceleration ||
        60 * distance / (r.duration * r.duration * r.duration) > l.jerk)
      return false;
  }
  return true;
}

// Choose a shared stopping time, then check each joint's resulting endpoint.
void TrajectorySampler::stop(double now, const Values &q, const Values &v,
                             Outcome result) noexcept {
  began_ = now;
  duration_ = 0;
  settled_since_ = -1;
  start_ = q;
  start_velocity_ = v;
  start_torque_ = state_.torque;
  stopping_outcome_ = result;
  state_.outcome = Outcome::None;
  state_.phase = Phase::Stopping;
  for (size_t i = 0; i < size_; ++i) {
    const auto &l = limits_[i];
    if (!std::isfinite(q[i]) || !std::isfinite(v[i]) || std::abs(v[i]) > l.velocity) {
      state_.phase = Phase::Fault;
      break;
    }
    if (std::abs(v[i]) <= l.standstill)
      start_velocity_[i] = 0;
    duration_ = std::max({duration_, l.ramp, 1.5 * std::abs(start_velocity_[i]) / l.acceleration,
                          std::sqrt(6 * std::abs(start_velocity_[i]) / l.jerk)});
  }
  for (size_t i = 0; i < size_; ++i) {
    const double end = start_[i] + start_velocity_[i] * duration_ / 2;
    if (!std::isfinite(end) || end < limits_[i].lower || end > limits_[i].upper)
      state_.phase = Phase::Fault;
  }
  if (state_.phase == Phase::Fault) {
    state_.torque.fill(0);
    state_.velocity_offset.fill(0);
    state_.outcome = Outcome::Aborted;
  }
}

void TrajectorySampler::tick(double now, const Values &q, const Values &v, const Request *r,
                             bool heartbeat, double tolerance, double timeout) noexcept {
  state_.stamp = now;
  state_.actual = q;
  state_.velocity = v;
  if (state_.phase == Phase::Fault)
    return;

  // Consume each request once; older generations cannot replace the current motion.
  if (r && r->sequence > state_.generation) {
    state_.generation = r->sequence;
    if (r->stop) {
      if (r->cancel_goal)
        state_.goal = r->cancel_goal;
      if (state_.phase != Phase::Stopping)
        stop(now, q, v, Outcome::Canceled);
    } else {
      state_.goal = r->sequence;
      state_.outcome = Outcome::None;
      if (state_.phase != Phase::Holding || !heartbeat || !feasible(*r, q, v)) {
        stop(now, q, v, Outcome::Aborted);
      } else {
        start_ = q;
        target_ = r->position;
        start_torque_ = state_.torque;
        end_torque_ = r->torque;
        began_ = now;
        duration_ = r->duration;
        settled_since_ = -1;
        state_.phase = Phase::Moving;
      }
    }
  }

  // Retained torque still needs a heartbeat after the position goal has finished.
  if (!heartbeat &&
      (state_.phase == Phase::Moving || (state_.phase == Phase::Holding && !state_.neutral)))
    stop(now, q, v, Outcome::Aborted);
  if (state_.phase == Phase::Moving || state_.phase == Phase::Holding) {
    for (size_t i = 0; i < size_; ++i) {
      if (!std::isfinite(q[i]) || !std::isfinite(v[i]) ||
          std::abs(q[i] - state_.position[i]) > limits_[i].tracking) {
        stop(now, q, v, Outcome::Aborted);
        break;
      }
    }
  }

  if (state_.phase == Phase::Moving || state_.phase == Phase::Stopping) {
    const bool stopping = state_.phase == Phase::Stopping;
    const double u = std::clamp((now - began_) / duration_, 0., 1.);
    state_.progress = u;
    bool settled = true;
    for (size_t i = 0; i < size_; ++i) {
      if (stopping) {
        state_.position[i] =
            start_[i] + start_velocity_[i] * duration_ * (u - u * u * u + 0.5 * u * u * u * u);
        state_.velocity_offset[i] = start_velocity_[i] * (1 - 3 * u * u + 2 * u * u * u);
        state_.torque[i] = start_torque_[i] * (1 - blend(u));
      } else {
        const double delta = target_[i] - start_[i];
        state_.position[i] = start_[i] + delta * blend(u);
        state_.velocity_offset[i] = delta * 30 * u * u * (1 - u) * (1 - u) / duration_;
        state_.torque[i] = start_torque_[i] + (end_torque_[i] - start_torque_[i]) * blend(u);
      }
      settled = settled && std::abs(q[i] - state_.position[i]) <= tolerance &&
                std::abs(v[i]) <= limits_[i].standstill;
    }

    // Elapsed trajectory time alone is insufficient; measured feedback must settle.
    if (u >= 1 && settled) {
      if (settled_since_ < 0)
        settled_since_ = now;
      if (now - settled_since_ >= 0.1) {
        state_.phase = Phase::Holding;
        state_.outcome = stopping ? stopping_outcome_ : Outcome::Succeeded;
      }
    } else
      settled_since_ = -1;
    if (now - began_ > duration_ + timeout) {
      if (!stopping)
        stop(now, q, v, Outcome::Aborted);
      else {
        state_.phase = Phase::Fault;
        state_.outcome = Outcome::Aborted;
        state_.torque.fill(0);
        state_.velocity_offset.fill(0);
      }
    }
  }

  // Neutral means both feedforward channels are effectively zero.
  state_.neutral = true;
  for (size_t i = 0; i < size_; ++i)
    state_.neutral = state_.neutral && std::abs(state_.torque[i]) < 1e-9 &&
                     std::abs(state_.velocity_offset[i]) < 1e-9;
}
} // namespace erob_controllers
