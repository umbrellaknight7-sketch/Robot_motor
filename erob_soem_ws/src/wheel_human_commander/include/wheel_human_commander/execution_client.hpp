#pragma once

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <functional>

namespace wheel_human_commander {
class ExecutionClient {
public:
  using Action = control_msgs::action::FollowJointTrajectory;
  using Handle = rclcpp_action::ClientGoalHandle<Action>;
  ExecutionClient(rclcpp::Node *, const std::string &);
  bool ready() const;

  bool pending() const {
    return pending_;
  }

  void execute(const trajectory_msgs::msg::JointTrajectory &,
               std::function<void(const Handle::WrappedResult &)>, std::function<void(bool)>,
               std::function<void(const Action::Feedback &)>);
  void stop(std::function<void(bool, uint64_t)>);

private:
  rclcpp_action::Client<Action>::SharedPtr action_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_;
  Handle::SharedPtr handle_;
  bool pending_ = false;
  std::function<void(bool, uint64_t)> deferred_stop_;
};
}
