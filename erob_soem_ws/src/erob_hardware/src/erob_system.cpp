#include "erob_hardware/erob_system.hpp"

#include <algorithm>
#include <cmath>
#include <erob_driver/config.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <set>
#include <sstream>

namespace erob_hardware {
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace {
const auto logger = rclcpp::get_logger("erob_hardware");
} // namespace

ErobSystem::~ErobSystem() {
  stop();
}

CallbackReturn ErobSystem::on_init(const hardware_interface::HardwareInfo &info) {
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;
  try {
    config_ = erob::load_config(info.hardware_parameters.at("config_file"));
    auto backend = info.hardware_parameters.find("backend");
    if (backend != info.hardware_parameters.end())
      config_.backend = backend->second;
    mock_fault_injection_ = info.hardware_parameters.count("enable_mock_fault_injection") &&
                            info.hardware_parameters.at("enable_mock_fault_injection") == "true";
    if (mock_fault_injection_ && config_.backend != "mock")
      throw std::runtime_error("Fault injection is available only with backend=mock");
    config_.csp_hold_enabled = true;
    erob::validate_config(config_);
    for (const auto &d : config_.drives)
      if (d.initial_mode != erob::Mode::CSP)
        throw std::runtime_error("ErobSystem supports CSP position commands only");

    // URDF interfaces and order must match the motor profile before allocating state.
    if (info.joints.size() != config_.drives.size())
      throw std::runtime_error("URDF/YAML joint count mismatch");
    for (size_t i = 0; i < info.joints.size(); ++i) {
      const auto &joint = info.joints[i];
      const auto &d = config_.drives[i];
      if (joint.name != d.joint)
        throw std::runtime_error("URDF joint order/name differs from configured logical mapping");
      std::set<std::string> expected{"position"};
      if (config_.csp_feedforward) {
        expected.insert("velocity_offset");
        expected.insert("torque_offset");
      }
      std::set<std::string> actual;
      for (const auto &x : joint.command_interfaces)
        actual.insert(x.name);
      if (actual != expected || actual.size() != joint.command_interfaces.size())
        throw std::runtime_error("Unexpected command interfaces for " + d.joint);
      expected = {"position", "velocity"};
      if (d.effort_verified)
        expected.insert("effort");
      actual.clear();
      for (const auto &x : joint.state_interfaces)
        actual.insert(x.name);
      if (actual != expected || actual.size() != joint.state_interfaces.size())
        throw std::runtime_error("Unexpected state interfaces for " + d.joint);
    }

    const auto n = config_.drives.size();
    position_.assign(n, 0);
    velocity_.assign(n, 0);
    effort_.assign(n, 0);
    cmd_position_.assign(n, 0);
    cmd_velocity_.assign(n, 0);
    cmd_effort_.assign(n, 0);
    cmd_velocity_offset_.assign(n, 0);
    cmd_torque_offset_.assign(n, 0);
    commands_.resize(n);
    claimed_.assign(n, 0);
    pending_ = claimed_;
    snapshot_.joints.resize(n);
    switch_snapshot_.joints.resize(n);
    switch_state_ = std::make_unique<erob::Latest<erob::Snapshot>>(snapshot_);
    diagnostics_state_ = std::make_unique<erob::Latest<erob::Snapshot>>(snapshot_);
    return CallbackReturn::SUCCESS;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger, "Initialization failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
}

std::vector<hardware_interface::StateInterface> ErobSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> v;
  for (size_t i = 0; i < config_.drives.size(); ++i) {
    const auto &d = config_.drives[i];
    v.emplace_back(d.joint, "position", &position_[i]);
    v.emplace_back(d.joint, "velocity", &velocity_[i]);
    if (d.effort_verified)
      v.emplace_back(d.joint, "effort", &effort_[i]);
  }
  return v;
}

std::vector<hardware_interface::CommandInterface> ErobSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> v;
  for (size_t i = 0; i < config_.drives.size(); ++i) {
    const auto &d = config_.drives[i];
    v.emplace_back(d.joint, "position", &cmd_position_[i]);
    if (config_.csp_feedforward) {
      v.emplace_back(d.joint, "velocity_offset", &cmd_velocity_offset_[i]);
      v.emplace_back(d.joint, "torque_offset", &cmd_torque_offset_[i]);
    }
  }
  return v;
}

CallbackReturn ErobSystem::on_configure(const rclcpp_lifecycle::State &) {
  try {
    driver_ = std::make_unique<erob::Driver>(config_);
    driver_->configure();
    driver_->read(snapshot_);
    switch_state_->publish(snapshot_);
    diagnostics_node_ = std::make_shared<rclcpp::Node>(info_.name + "_diagnostics");
    diagnostics_pub_ = diagnostics_node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 10);
    resume_service_ = diagnostics_node_->create_service<std_srvs::srv::Trigger>(
        "/" + info_.name + "/resume",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          response->success =
              diagnostics_running_.load() && driver_ && driver_->resume(service_generation_);
          response->message =
              response->success
                  ? "Resumed; submit a new trajectory"
                  : "Hold not ready, generation changed, or worker unavailable; hold retained";
        });
    if (mock_fault_injection_) {
      test_expire_service_ = diagnostics_node_->create_service<std_srvs::srv::Trigger>(
          "/" + info_.name + "/test_expire_commands",
          [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            pause_commands_until_ = erob::monotonic_now() + config_.command_timeout_ns * 2;
            response->success = true;
            response->message = "Mock command handoff suppressed for two timeout intervals";
          });
    }
    diagnostics_running_ = true;
    diagnostics_thread_ = std::thread(&ErobSystem::diagnostics, this);
    return CallbackReturn::SUCCESS;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger, "Configure failed: %s", e.what());
    stop();
    return CallbackReturn::ERROR;
  }
}

CallbackReturn ErobSystem::on_activate(const rclcpp_lifecycle::State &) {
  try {
    if (!driver_ || !driver_->activate())
      return CallbackReturn::ERROR;
    driver_->read(snapshot_);
    switch_state_->publish(snapshot_);
    for (size_t i = 0; i < commands_.size(); ++i) {
      position_[i] = snapshot_.joints[i].position;
      cmd_position_[i] = position_[i];
      cmd_velocity_[i] = cmd_effort_[i] = 0;
      cmd_velocity_offset_[i] = cmd_torque_offset_[i] = 0;
      claimed_[i] = 0;
    }
    active_ = true;
    return CallbackReturn::SUCCESS;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger, "Activation failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
}

CallbackReturn ErobSystem::on_deactivate(const rclcpp_lifecycle::State &) {
  active_ = false;
  std::fill(claimed_.begin(), claimed_.end(), 0);
  return !driver_ || driver_->deactivate() ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

void ErobSystem::stop() noexcept {
  active_ = false;
  diagnostics_running_ = false;
  // Publish Stop before joining the service thread so a concurrent resume
  // cannot delay or supersede shutdown. Keep the driver alive until it joins.
  if (driver_)
    driver_->shutdown();
  if (diagnostics_thread_.joinable())
    diagnostics_thread_.join();
  test_expire_service_.reset();
  resume_service_.reset();
  diagnostics_pub_.reset();
  diagnostics_node_.reset();
  driver_.reset();
}

CallbackReturn ErobSystem::on_cleanup(const rclcpp_lifecycle::State &) {
  stop();
  return CallbackReturn::SUCCESS;
}

CallbackReturn ErobSystem::on_shutdown(const rclcpp_lifecycle::State &) {
  stop();
  return CallbackReturn::SUCCESS;
}

CallbackReturn ErobSystem::on_error(const rclcpp_lifecycle::State &) {
  stop();
  return CallbackReturn::SUCCESS;
}

// Read a coherent worker snapshot; ROS read/write callbacks never perform bus I/O.
return_type ErobSystem::read(const rclcpp::Time &, const rclcpp::Duration &) {
  if (!driver_)
    return return_type::OK;
  driver_->read(snapshot_);
  for (size_t i = 0; i < commands_.size(); ++i) {
    const auto &s = snapshot_.joints[i];
    position_[i] = s.position;
    velocity_[i] = s.velocity;
    effort_[i] = s.effort;
  }
  diagnostics_state_->publish(snapshot_);
  switch_state_->publish(snapshot_);
  if (snapshot_.health.phase == erob::Phase::Fault) {
    RCLCPP_ERROR(logger, "Motor group stopped: %s (motor index %d)",
                 erob::fault_text(snapshot_.health.fault), snapshot_.health.fault_motor);
    return return_type::ERROR;
  }
  return return_type::OK;
}

return_type ErobSystem::write(const rclcpp::Time &, const rclcpp::Duration &) {
  if (!driver_ || !active_)
    return return_type::OK;
  if (erob::monotonic_now() < pause_commands_until_.load())
    return return_type::OK;
  for (size_t i = 0; i < commands_.size(); ++i) {
    auto &c = commands_[i];
    c.claimed = claimed_[i] != 0;
    c.mode = erob::Mode::CSP;
    c.position = cmd_position_[i];
    c.velocity = cmd_velocity_[i];
    c.effort = cmd_effort_[i];
    c.velocity_offset = c.claimed ? cmd_velocity_offset_[i] : 0;
    c.torque_offset = c.claimed ? cmd_torque_offset_[i] : 0;
  }

  // Rejected values retain the previous valid command; only an unavailable driver is an error.
  if (driver_->submit(commands_).code == erob::SubmitResult::Unavailable) {
    RCLCPP_ERROR(logger, "Driver unavailable for command handoff");
    return return_type::ERROR;
  }
  return return_type::OK;
}

// Validate a proposed set of claims before committing it in perform_command_mode_switch.
return_type ErobSystem::prepare_command_mode_switch(const std::vector<std::string> &start,
                                                    const std::vector<std::string> &stop_keys) {
  switch_state_->read(switch_snapshot_);
  pending_ = claimed_;
  prepared_ = false;
  auto apply = [&](const std::vector<std::string> &keys, bool starting) {
    std::set<std::string> seen;
    for (const auto &key : keys) {
      if (!seen.insert(key).second)
        return false;
      auto slash = key.rfind('/');
      if (slash == std::string::npos)
        continue;
      const auto name = key.substr(0, slash);
      const auto interface = key.substr(slash + 1);
      auto it = std::find_if(config_.drives.begin(), config_.drives.end(),
                             [&](const auto &d) { return d.joint == name; });
      if (it == config_.drives.end())
        continue;
      const int bit =
          interface == "position"
              ? 1
              : (config_.csp_feedforward && interface == "velocity_offset"
                     ? 2
                     : (config_.csp_feedforward && interface == "torque_offset" ? 4 : 0));
      if (!bit)
        return false;
      const size_t i = size_t(it - config_.drives.begin());
      if (starting) {
        if (pending_[i] & bit)
          return false;
        pending_[i] |= bit;
      } else {
        if (!(pending_[i] & bit))
          return false;
        pending_[i] &= ~bit;
      }
    }
    return true;
  };
  if (!apply(stop_keys, false) || !apply(start, true))
    return return_type::ERROR;
  for (auto mask : pending_)
    if (mask != 0 && mask != (config_.csp_feedforward ? 7 : 1))
      return return_type::ERROR;
  prepared_ = true;
  return return_type::OK;
}

return_type ErobSystem::perform_command_mode_switch(const std::vector<std::string> &,
                                                    const std::vector<std::string> &) {
  if (!prepared_)
    return return_type::ERROR;
  for (size_t i = 0; i < claimed_.size(); ++i)
    if (pending_[i] != claimed_[i]) {
      cmd_position_[i] = position_[i];
      cmd_velocity_[i] = cmd_effort_[i] = 0;
      cmd_velocity_offset_[i] = cmd_torque_offset_[i] = 0;
    }
  claimed_ = pending_;
  prepared_ = false;
  return return_type::OK;
}

void ErobSystem::diagnostics() {
  erob::Snapshot s;
  s.joints.resize(config_.drives.size());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(diagnostics_node_);
  while (diagnostics_running_) {
    diagnostics_state_->read(s);
    service_generation_ = s.health.hold_generation;
    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = diagnostics_node_->now();
    for (size_t i = 0; i < s.joints.size(); ++i) {
      diagnostic_msgs::msg::DiagnosticStatus d;
      d.name = info_.name + "/" + config_.drives[i].joint;
      d.hardware_id = config_.drives[i].branch + ":" + std::to_string(config_.drives[i].logical_id);
      const auto &j = s.joints[i];
      d.level =
          s.health.fault != erob::Fault::None
              ? 2
              : (!j.valid || erob::holding(s.health.phase) || s.health.rejected_commands ? 1 : 0);
      d.message = s.health.fault != erob::Fault::None
                      ? erob::fault_text(s.health.fault)
                      : (erob::holding(s.health.phase)
                             ? erob::phase_text(s.health.phase)
                             : (s.health.rejected_commands ? "command rejected (see counters)"
                                                           : erob::phase_text(s.health.phase)));
      auto add = [&](const char *name, auto value) {
        diagnostic_msgs::msg::KeyValue x;
        x.key = name;
        std::ostringstream text;
        text << value;
        x.value = text.str();
        d.values.push_back(x);
      };
      add("branch", config_.drives[i].branch);
      add("branch_position", config_.drives[i].branch_position);
      add("logical_id", config_.drives[i].logical_id);
      add("status_word", j.status);
      add("error_code", j.error);
      add("mode", int(j.mode));
      add("requested_mode", int(j.requested_mode));
      add("commanded_velocity_offset", j.commanded_velocity_offset);
      add("commanded_torque_offset", j.commanded_torque_offset);
      add("switching", j.switching);
      add("wkc", s.health.wkc);
      add("expected_wkc", s.health.expected_wkc);
      add("feedback_valid", s.health.feedback_valid);
      add("phase", int(s.health.phase));
      add("phase_name", erob::phase_text(s.health.phase));
      add("hold_reason", erob::fault_text(s.health.hold_reason));
      add("hold_generation", s.health.hold_generation);
      add("hold_start", j.hold_start);
      add("hold_target", j.hold_target);
      add("resume_ready", s.health.resume_ready);
      add("claims_released", s.health.claims_released);
      add("bad_cycles", s.health.bad_cycles);
      add("late_cycles", s.health.late_cycles);
      add("rejected_commands", s.health.rejected_commands);
      add("rejection_reason", erob::reject_text(s.health.rejection));
      add("rejected_joint_index", s.health.rejected_joint);
      add("feedback_age_ms", double(erob::monotonic_now() - s.health.feedback_stamp_ns) / 1e6);
      add("missed_deadlines", s.health.missed_deadlines);
      add("fault_motor_index", s.health.fault_motor);
      if (s.health.fault_motor >= 0 && size_t(s.health.fault_motor) < config_.drives.size()) {
        const auto &culprit = config_.drives[size_t(s.health.fault_motor)];
        add("fault_joint", culprit.joint);
        add("fault_branch", culprit.branch);
      } else if (s.health.fault != erob::Fault::None) {
        add("fault_joint", "unknown (bus/group fault)");
        add("fault_branch", "unknown (bus/group fault)");
      }
      message.status.push_back(d);
    }
    try {
      if (rclcpp::ok())
        diagnostics_pub_->publish(message);
      executor.spin_some();
    } catch (const std::exception &) {
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}
} // namespace erob_hardware
PLUGINLIB_EXPORT_CLASS(erob_hardware::ErobSystem, hardware_interface::SystemInterface)
