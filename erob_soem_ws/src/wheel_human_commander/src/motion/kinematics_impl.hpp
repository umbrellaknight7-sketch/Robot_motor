#pragma once
// Pinocchio forward declarations must precede other Boost-dependent headers.
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include "wheel_human_commander/kinematics.hpp"

#include <map>
#include <set>

namespace wheel_human_commander {
struct KinematicsModel::Impl {
  struct Group {
    std::vector<int> q_indices, v_indices;
    pinocchio::FrameIndex tip;
  };

  pinocchio::Model model;
  std::unique_ptr<pinocchio::Data> data;
  IkOptions options;
  std::vector<std::string> names;
  std::map<std::string, int> q_indices, v_indices;
  std::map<std::string, pinocchio::FrameIndex> frames;
  std::map<std::string, Group> groups;
  std::string fingerprint;
  Impl(const std::string &, const std::vector<KinematicsGroup> &, const IkOptions &);
  Eigen::VectorXd configuration(const sensor_msgs::msg::JointState &) const;
  pinocchio::FrameIndex frame(const std::string &) const;
  geometry_msgs::msg::PoseStamped pose(pinocchio::FrameIndex) const;
  void evaluate(const Eigen::VectorXd &);
};
}
