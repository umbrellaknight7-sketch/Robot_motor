#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <array>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace wheel_human_commander {

enum class KinematicsCode {
  Success = 0,
  InvalidInput = 1,
  NoConvergence = 2,
  Timeout = 3,
  InternalError = 4
};
enum class IkMode { PositionOnly = 0, FullPose = 1 };

struct KinematicsGroup {
  std::string name;
  std::vector<std::string> joints;
  std::string tip;
};

struct IkOptions {
  int max_iterations = 200;
  double timeout_seconds = 0.1;
  double max_joint_step = 0.1;
  double position_tolerance = 0.0001;
  double orientation_tolerance = 0.001;
  double orientation_scale = 0.1;
  double damping = 0.001;
};

struct KinematicsResult {
  KinematicsCode code = KinematicsCode::InvalidInput;
  std::string message;
  geometry_msgs::msg::PoseStamped pose;

  explicit operator bool() const {
    return code == KinematicsCode::Success;
  }
};

struct IkResult : KinematicsResult {
  sensor_msgs::msg::JointState solution;
  double position_error = std::numeric_limits<double>::quiet_NaN();
  double orientation_error = std::numeric_limits<double>::quiet_NaN();
  int iterations = 0;
};

struct JacobianResult : KinematicsResult {
  std::vector<std::string> joint_names;
  // Linear XYZ then angular XYZ, expressed in base_link at the requested origin.
  std::array<std::vector<double>, 6> rows;
};

// One immutable model and mutable calculation workspace. Calls are serialized by
// the owner; use separate instances for concurrent workers. No ROS I/O or motors.
class KinematicsModel {
public:
  KinematicsModel(const std::string &urdf, const std::vector<KinematicsGroup> &groups,
                  const IkOptions &options = IkOptions{});
  ~KinematicsModel();
  KinematicsModel(const KinematicsModel &) = delete;
  KinematicsModel &operator=(const KinematicsModel &) = delete;
  const std::vector<std::string> &joint_names() const;
  const std::string &fingerprint() const;
  static std::string pinocchio_version();
  KinematicsResult forward(const sensor_msgs::msg::JointState &, const std::string &frame);
  JacobianResult jacobian(const sensor_msgs::msg::JointState &, const std::string &frame);
  IkResult inverse(const sensor_msgs::msg::JointState &seed, const std::string &group,
                   const geometry_msgs::msg::PoseStamped &target, IkMode mode);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
