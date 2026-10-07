#include "erob_controllers/csp_trajectory_controller.hpp"

#include <erob_driver/config.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <set>

namespace erob_controllers {
namespace {
double steady() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}

using Callback = controller_interface::CallbackReturn;

Callback CspTrajectoryController::on_init() {
  auto_declare<std::string>("config_file", "");
  auto_declare<double>("heartbeat_timeout", 0.5);
  auto_declare<double>("goal_tolerance", 0.01);
  auto_declare<double>("settle_timeout", 3.0);
  // The recovery helper checks these invariants for both supported controllers.
  auto_declare<bool>("open_loop_control", false);
  auto_declare<bool>("set_last_command_interface_value_as_state_on_activation", false);
  return Callback::SUCCESS;
}

Callback CspTrajectoryController::on_configure(const rclcpp_lifecycle::State &) {
  const std::lock_guard<std::recursive_mutex> guard(callbacks_);
  try {
    const auto node = get_node();
    const auto config = erob::load_config(node->get_parameter("config_file").as_string());
    if (!config.csp_feedforward || config.drives.empty() || config.drives.size() >= capacity)
      throw std::runtime_error("Controller requires a verified-size CSP feedforward profile");
    if (node->get_parameter("open_loop_control").as_bool() ||
        node->get_parameter("set_last_command_interface_value_as_state_on_activation").as_bool())
      throw std::runtime_error("Measured-state activation is required");
    heartbeat_timeout_ = node->get_parameter("heartbeat_timeout").as_double();
    tolerance_ = node->get_parameter("goal_tolerance").as_double();
    settle_timeout_ = node->get_parameter("settle_timeout").as_double();
    if (!std::isfinite(heartbeat_timeout_) || heartbeat_timeout_ <= 0 ||
        !std::isfinite(tolerance_) || tolerance_ <= 0 || !std::isfinite(settle_timeout_) ||
        settle_timeout_ <= 0)
      throw std::runtime_error("Invalid controller timing/tolerance");

    // Match command slots and motion limits to the configured logical joint order.
    names_.clear();
    count_ = config.drives.size();
    for (size_t i = 0; i < count_; ++i) {
      const auto &d = config.drives[i];
      names_.push_back(d.joint);
      if (d.hold_acceleration <= 0 || d.hold_jerk <= 0 || d.hold_tracking_error <= 0)
        throw std::runtime_error("Positive acceleration/jerk/tracking limits required");
      limits_[i] = {d.min_position,
                    d.max_position,
                    std::min(d.max_velocity, d.max_velocity_offset),
                    d.hold_acceleration,
                    d.hold_jerk,
                    d.max_torque_offset,
                    d.offset_ramp_seconds,
                    d.standstill_velocity,
                    d.hold_tracking_error};
    }
    sampler_.configure(count_, limits_);
    validator_.configure(count_, limits_);

    server_ = rclcpp_action::create_server<Action>(
        node, "~/follow_joint_trajectory",
        [this](const rclcpp_action::GoalUUID &id, std::shared_ptr<const Action::Goal> g) {
          return goal(id, g);
        },
        [this](std::shared_ptr<Goal> g) {
          const std::lock_guard<std::recursive_mutex> guard(callbacks_);
          if (g != goal_)
            return rclcpp_action::CancelResponse::REJECT;
          request_stop();
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](std::shared_ptr<Goal> g) { accepted(g); });
    heartbeat_sub_ = node->create_subscription<std_msgs::msg::Empty>(
        "~/heartbeat", 1,
        [this](std_msgs::msg::Empty::ConstSharedPtr) { heartbeat_.store(steady()); });
    stop_service_ = node->create_service<std_srvs::srv::Trigger>(
        "~/stop", [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          response->success = active_;
          response->message = active_ ? std::to_string(request_stop()) : "Controller inactive";
        });
    state_pub_ = rclcpp::create_publisher<control_msgs::msg::JointTrajectoryControllerState>(
        node, "~/controller_state", 10);
    status_pub_ =
        rclcpp::create_publisher<diagnostic_msgs::msg::DiagnosticArray>(node, "~/status", 10);
    timer_ = node->create_wall_timer(std::chrono::milliseconds(20), [this] { publish(); });
    return Callback::SUCCESS;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", e.what());
    return Callback::ERROR;
  }
}

controller_interface::InterfaceConfiguration
CspTrajectoryController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration result;
  result.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &name : names_)
    for (const auto &field : {"position", "velocity_offset", "torque_offset"})
      result.names.push_back(name + "/" + field);
  return result;
}

controller_interface::InterfaceConfiguration
CspTrajectoryController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration result;
  result.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &name : names_)
    for (const auto &field : {"position", "velocity"})
      result.names.push_back(name + "/" + field);
  return result;
}

Callback CspTrajectoryController::on_activate(const rclcpp_lifecycle::State &) {
  const std::lock_guard<std::recursive_mutex> guard(callbacks_);
  auto locate = [](const auto &interfaces, const std::string &name) {
    for (size_t i = 0; i < interfaces.size(); ++i)
      if (interfaces[i].get_name() == name)
        return i;
    throw std::runtime_error("Missing interface: " + name);
  };
  try {
    Values q{}, v{};
    for (size_t i = 0; i < count_; ++i) {
      q_cmd_[i] = locate(command_interfaces_, names_[i] + "/position");
      v_cmd_[i] = locate(command_interfaces_, names_[i] + "/velocity_offset");
      t_cmd_[i] = locate(command_interfaces_, names_[i] + "/torque_offset");
      q_state_[i] = locate(state_interfaces_, names_[i] + "/position");
      v_state_[i] = locate(state_interfaces_, names_[i] + "/velocity");
      q[i] = state_interfaces_[q_state_[i]].get_value();
      v[i] = state_interfaces_[v_state_[i]].get_value();
      if (!std::isfinite(q[i]) || !std::isfinite(v[i]))
        throw std::runtime_error("Invalid activation state");
      command_interfaces_[q_cmd_[i]].set_value(q[i]);
      command_interfaces_[v_cmd_[i]].set_value(0);
      command_interfaces_[t_cmd_[i]].set_value(0);
    }

    // Seed the sampler from measured feedback and discard requests from the old lifecycle.
    sampler_.activate(q, v, steady());
    cached_ = sampler_.state();
    Request discarded;
    requests_.read(discarded); // Lifecycle excludes update().
    pending_ = false;
    goal_.reset();
    stopping_sequence_ = 0;
    heartbeat_ = 0;
    active_ = true;
    return Callback::SUCCESS;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s", e.what());
    return Callback::ERROR;
  }
}

Callback CspTrajectoryController::on_deactivate(const rclcpp_lifecycle::State &) {
  const std::lock_guard<std::recursive_mutex> guard(callbacks_);
  active_ = false;
  for (size_t i = 0; i < count_; ++i) {
    command_interfaces_[v_cmd_[i]].set_value(0);
    command_interfaces_[t_cmd_[i]].set_value(0);
  }
  if (goal_) {
    auto r = std::make_shared<Action::Result>();
    r->error_code = Action::Result::INVALID_GOAL;
    r->error_string = "Controller deactivated";
    goal_->abort(r);
    goal_.reset();
  }
  pending_ = false;
  cached_.phase = Phase::Inactive;
  return Callback::SUCCESS;
}

rclcpp_action::GoalResponse CspTrajectoryController::goal(const rclcpp_action::GoalUUID &,
                                                          std::shared_ptr<const Action::Goal> g) {
  const std::lock_guard<std::recursive_mutex> guard(callbacks_);
  const auto reject = [this](const char *why) {
    RCLCPP_WARN(get_node()->get_logger(), "Goal rejected: %s", why);
    return rclcpp_action::GoalResponse::REJECT;
  };
  if (!active_ || pending_ || goal_ || cached_.phase != Phase::Holding ||
      steady() - cached_.stamp > 0.5 || steady() - heartbeat_ > heartbeat_timeout_)
    return reject("not ready or busy");

  // This controller accepts one immediate, full-joint, rest-to-rest endpoint.
  const auto &t = g->trajectory;
  if (t.joint_names != names_ || t.points.size() != 1 || t.header.stamp.sec ||
      t.header.stamp.nanosec)
    return reject("requires one immediate, ordered, full-joint endpoint");
  if (!g->path_tolerance.empty() || !g->goal_tolerance.empty() || g->goal_time_tolerance.sec ||
      g->goal_time_tolerance.nanosec)
    return reject("use configured tolerances");
  if (!g->multi_dof_trajectory.points.empty() || !g->multi_dof_trajectory.joint_names.empty())
    return reject("multi-DOF trajectories unsupported");
  const auto &p = t.points[0];
  if (p.time_from_start.nanosec >= 1000000000u)
    return reject("invalid duration encoding");
  if (p.positions.size() != count_ || (!p.effort.empty() && p.effort.size() != count_))
    return reject("wrong vector size");
  for (const auto *values : {&p.velocities, &p.accelerations}) {
    if (!values->empty() && values->size() != count_)
      return reject("wrong derivative size");
    for (double x : *values)
      if (!std::isfinite(x) || x != 0)
        return reject("requires rest-to-rest endpoint");
  }
  pending_request_ = Request{};
  pending_request_.duration = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9;
  for (size_t i = 0; i < count_; ++i) {
    pending_request_.position[i] = p.positions[i];
    pending_request_.torque[i] = p.effort.empty() ? 0 : p.effort[i];
  }
  if (!validator_.feasible(pending_request_, cached_.actual, cached_.velocity))
    return reject("limits or infeasible duration");
  pending_ = true;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void CspTrajectoryController::accepted(std::shared_ptr<Goal> g) {
  const std::lock_guard<std::recursive_mutex> guard(callbacks_);
  pending_ = false;
  if (!active_) {
    auto result = std::make_shared<Action::Result>();
    result->error_code = Action::Result::INVALID_GOAL;
    result->error_string = "Controller deactivated before dispatch";
    g->abort(result);
    return;
  }
  goal_ = g;
  stopping_sequence_ = 0;
  goal_sequence_ = ++sequence_;
  pending_request_.sequence = goal_sequence_;
  requests_.publish(pending_request_);
}

// A new sequence distinguishes this stop from an earlier goal completion.
uint64_t CspTrajectoryController::request_stop() {
  const std::lock_guard<std::recursive_mutex> guard(callbacks_);
  Request r;
  r.stop = true;
  r.sequence = ++sequence_;
  r.cancel_goal = goal_ ? goal_sequence_ : 0;
  stopping_sequence_ = r.sequence;
  requests_.publish(r);
  return r.sequence;
}

// The update loop exchanges preallocated requests and snapshots with ROS callbacks.
controller_interface::return_type CspTrajectoryController::update(const rclcpp::Time &,
                                                                  const rclcpp::Duration &) {
  if (!active_)
    return controller_interface::return_type::OK;
  Values q{}, v{};
  for (size_t i = 0; i < count_; ++i) {
    q[i] = state_interfaces_[q_state_[i]].get_value();
    v[i] = state_interfaces_[v_state_[i]].get_value();
  }

  Request request;
  const bool changed = requests_.read(request);
  const double now = steady();
  sampler_.tick(now, q, v, changed ? &request : nullptr,
                now - heartbeat_.load() <= heartbeat_timeout_, tolerance_, settle_timeout_);

  const auto &s = sampler_.state();
  for (size_t i = 0; i < count_; ++i) {
    command_interfaces_[q_cmd_[i]].set_value(s.position[i]);
    command_interfaces_[v_cmd_[i]].set_value(s.velocity_offset[i]);
    command_interfaces_[t_cmd_[i]].set_value(s.torque[i]);
  }
  snapshots_.publish(s);
  // Keep publishing fault status; no new goal is accepted until lifecycle recovery.
  return controller_interface::return_type::OK;
}

void CspTrajectoryController::publish() {
  const std::lock_guard<std::recursive_mutex> guard(callbacks_);
  if (active_)
    snapshots_.read(cached_);
  control_msgs::msg::JointTrajectoryControllerState msg;
  msg.header.stamp = get_node()->now();
  msg.joint_names = names_;
  msg.actual.positions.assign(cached_.actual.begin(), cached_.actual.begin() + count_);
  msg.actual.velocities.assign(cached_.velocity.begin(), cached_.velocity.begin() + count_);
  const double elapsed = cached_.progress * pending_request_.duration;
  msg.desired.time_from_start.sec = static_cast<int32_t>(elapsed);
  msg.desired.time_from_start.nanosec =
      static_cast<uint32_t>((elapsed - msg.desired.time_from_start.sec) * 1e9);
  msg.desired.positions.assign(cached_.position.begin(), cached_.position.begin() + count_);
  msg.desired.velocities.assign(cached_.velocity_offset.begin(),
                                cached_.velocity_offset.begin() + count_);
  msg.desired.effort.assign(cached_.torque.begin(), cached_.torque.begin() + count_);
  state_pub_->publish(msg);

  diagnostic_msgs::msg::DiagnosticArray status;
  status.header = msg.header;
  diagnostic_msgs::msg::DiagnosticStatus d;
  d.name = get_node()->get_name();
  d.message = active_ ? phase_name(cached_.phase) : "inactive";
  d.level = cached_.phase == Phase::Fault ? 2 : 0;
  auto add = [&](const std::string &k, const std::string &v) {
    diagnostic_msgs::msg::KeyValue x;
    x.key = k;
    x.value = v;
    d.values.push_back(x);
  };
  add("generation", std::to_string(cached_.generation));
  add("neutral", cached_.neutral ? "true" : "false");
  add("heartbeat_valid", steady() - heartbeat_ <= heartbeat_timeout_ ? "true" : "false");
  add("state_age", std::to_string(steady() - cached_.stamp));
  status.status.push_back(d);
  status_pub_->publish(status);

  // Match the goal and stop generation; faults may finish without a stop acknowledgement.
  if (goal_ && cached_.goal == goal_sequence_) {
    auto feedback = std::make_shared<Action::Feedback>();
    feedback->header = msg.header;
    feedback->joint_names = names_;
    feedback->actual = msg.actual;
    feedback->desired = msg.desired;
    goal_->publish_feedback(feedback);
    if (cached_.outcome != Outcome::None &&
        (!stopping_sequence_ || cached_.generation >= stopping_sequence_ ||
         cached_.phase == Phase::Fault)) {
      auto result = std::make_shared<Action::Result>();
      if (cached_.outcome == Outcome::Succeeded) {
        result->error_code = 0;
        goal_->succeed(result);
      } else if (goal_->is_canceling() && cached_.phase == Phase::Holding) {
        result->error_code = 0;
        result->error_string = "Stopped and offsets neutral";
        goal_->canceled(result);
      } else {
        result->error_code = Action::Result::PATH_TOLERANCE_VIOLATED;
        result->error_string = "Stopped, watchdog expired, or controller fault";
        goal_->abort(result);
      }
      goal_.reset();
    }
  }
}
} // namespace erob_controllers
PLUGINLIB_EXPORT_CLASS(erob_controllers::CspTrajectoryController,
                       controller_interface::ControllerInterface)
