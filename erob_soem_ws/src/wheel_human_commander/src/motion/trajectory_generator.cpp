#include "wheel_human_commander/motion.hpp"

#include <cmath>

namespace wheel_human_commander {
MotionResult generate_advanced_trajectory(const std::vector<double> &,
                                          const std::vector<double> &) {
  // TODO: collision-aware planning and automatic time parameterization.
  return {Code::NotImplemented, "Advanced trajectory generation is not implemented", {}};
}

// The controller generates the curve from one endpoint with zero terminal derivatives.
trajectory_msgs::msg::JointTrajectory discrete_endpoint(const std::vector<std::string> &names,
                                                        const std::vector<double> &q,
                                                        const std::vector<double> &torque,
                                                        double duration) {
  trajectory_msgs::msg::JointTrajectory t;
  t.joint_names = names;
  trajectory_msgs::msg::JointTrajectoryPoint p;
  p.positions = q;
  p.effort = torque;
  p.velocities.assign(q.size(), 0);
  p.accelerations.assign(q.size(), 0);
  const int64_t ns = std::llround(duration * 1e9);
  p.time_from_start.sec = int32_t(ns / 1000000000);
  p.time_from_start.nanosec = uint32_t(ns % 1000000000);
  t.points.push_back(p);
  return t;
}
}
