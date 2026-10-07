#pragma once

#include "erob_controllers/trajectory_sampler.hpp"

#include <controller_interface/controller_interface.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <erob_driver/handoff.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <mutex>

namespace erob_controllers {
class CspTrajectoryController final : public controller_interface::ControllerInterface {
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  using Action = control_msgs::action::FollowJointTrajectory;
  using Goal = rclcpp_action::ServerGoalHandle<Action>;
  rclcpp_action::GoalResponse goal(const rclcpp_action::GoalUUID &,
                                   std::shared_ptr<const Action::Goal>);
  void accepted(std::shared_ptr<Goal>);
  uint64_t request_stop();
  void publish();
  // Lifecycle and executor callbacks may overlap. The cyclic update never locks.
  std::recursive_mutex callbacks_;
  std::vector<std::string> names_;
  size_t count_ = 0;
  std::array<Limits, capacity> limits_{};
  std::array<size_t, capacity> q_cmd_{}, v_cmd_{}, t_cmd_{}, q_state_{}, v_state_{};
  TrajectorySampler sampler_, validator_;
  erob::Latest<Request> requests_{Request{}};
  erob::Latest<Snapshot> snapshots_{Snapshot{}};
  Request pending_request_{};
  Snapshot cached_{};
  uint64_t sequence_ = 0, goal_sequence_ = 0, stopping_sequence_ = 0;
  bool pending_ = false;
  std::atomic<bool> active_{false};
  std::atomic<double> heartbeat_{0};
  double heartbeat_timeout_ = 0.5, tolerance_ = 0.01, settle_timeout_ = 3;
  std::shared_ptr<Goal> goal_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Publisher<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr state_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace erob_controllers
