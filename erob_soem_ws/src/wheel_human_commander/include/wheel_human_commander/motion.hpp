#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <string>
#include <vector>

namespace wheel_human_commander {
enum class Code {
  Success = 0,
  NotImplemented = 1,
  InvalidGoal = 2,
  NotOwner = 3,
  Busy = 4,
  NotReady = 5,
  ExecutionFailed = 6,
  Canceled = 7
};

struct MotionResult {
  Code code = Code::NotImplemented;
  std::string message;
  std::vector<double> values;

  explicit operator bool() const {
    return code == Code::Success;
  }
};

MotionResult gravity_compensation(const std::vector<double> &);
MotionResult generate_advanced_trajectory(const std::vector<double> &, const std::vector<double> &);
trajectory_msgs::msg::JointTrajectory discrete_endpoint(const std::vector<std::string> &,
                                                        const std::vector<double> &,
                                                        const std::vector<double> &, double);
}
