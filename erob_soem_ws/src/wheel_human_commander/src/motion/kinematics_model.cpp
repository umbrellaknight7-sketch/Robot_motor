#include "kinematics_impl.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/config.hpp>
#include <urdf_parser/urdf_parser.h>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace wheel_human_commander {
KinematicsModel::Impl::Impl(const std::string &xml, const std::vector<KinematicsGroup> &configs,
                            const IkOptions &settings)
    : options(settings) {
  if (options.max_iterations <= 0)
    throw std::invalid_argument("max_iterations must be positive");
  for (double value : {options.timeout_seconds, options.max_joint_step, options.position_tolerance,
                       options.orientation_tolerance, options.orientation_scale, options.damping})
    if (!std::isfinite(value) || value <= 0)
      throw std::invalid_argument("IK settings must be finite and positive");
  auto tree = urdf::parseURDF(xml);
  if (!tree || !tree->getRoot() || tree->getRoot()->name != "base_link")
    throw std::invalid_argument("Expected a valid URDF rooted at base_link");
  for (const auto &item : tree->joints_) {
    const auto &j = *item.second;
    const auto &origin = j.parent_to_joint_origin_transform;
    for (double value : {origin.position.x, origin.position.y, origin.position.z, origin.rotation.x,
                         origin.rotation.y, origin.rotation.z, origin.rotation.w})
      if (!std::isfinite(value))
        throw std::invalid_argument("Non-finite joint origin: " + j.name);
    if (j.type == urdf::Joint::FIXED)
      continue;
    if ((j.type != urdf::Joint::REVOLUTE && j.type != urdf::Joint::PRISMATIC) || !j.limits ||
        j.mimic)
      throw std::invalid_argument(
          "Only independent bounded revolute/prismatic joints are supported: " + j.name);
    if (!std::isfinite(j.limits->lower) || !std::isfinite(j.limits->upper) ||
        j.limits->lower >= j.limits->upper)
      throw std::invalid_argument("Invalid position limits: " + j.name);
    double norm = j.axis.x * j.axis.x + j.axis.y * j.axis.y + j.axis.z * j.axis.z;
    if (!std::isfinite(norm) || std::abs(norm - 1) > 1e-6)
      throw std::invalid_argument("Joint axis must be a finite unit vector: " + j.name);
  }
  pinocchio::urdf::buildModelFromXML(xml, model);

  // Build name-to-configuration and name-to-velocity maps; their indices need not match.
  for (pinocchio::JointIndex i = 1; i < model.joints.size(); ++i) {
    const auto &j = model.joints[i];
    if (j.nq() != 1 || j.nv() != 1)
      throw std::invalid_argument("Expected scalar joint: " + model.names[i]);
    names.push_back(model.names[i]);
    q_indices.emplace(model.names[i], j.idx_q());
    v_indices.emplace(model.names[i], j.idx_v());
  }
  for (const auto &link : tree->links_) {
    auto id = model.getFrameId(link.first, pinocchio::BODY);
    if (id >= model.frames.size())
      throw std::invalid_argument("Missing link frame: " + link.first);
    frames.emplace(link.first, id);
  }
  if (configs.empty())
    throw std::invalid_argument("At least one IK group is required");

  // Each planning group may move only joints on its tip's ancestor chain.
  for (const auto &c : configs) {
    if (c.name.empty() || c.joints.empty() || groups.count(c.name))
      throw std::invalid_argument("Empty or duplicate IK group");
    Group group;
    group.tip = frame(c.tip);
    std::set<pinocchio::JointIndex> ancestors;
    for (auto id = model.frames[group.tip].parentJoint; id != 0; id = model.parents[id])
      ancestors.insert(id);
    std::set<std::string> unique;
    for (const auto &name : c.joints) {
      if (!q_indices.count(name) || !unique.insert(name).second ||
          !ancestors.count(model.getJointId(name)))
        throw std::invalid_argument("IK group joint is duplicate, unknown or outside TCP chain: " +
                                    name);
      group.q_indices.push_back(q_indices.at(name));
      group.v_indices.push_back(v_indices.at(name));
    }
    groups.emplace(c.name, std::move(group));
  }
  // Stable model fingerprint, not an authentication/security hash.
  uint64_t hash = 14695981039346656037ull;
  for (unsigned char byte : xml) {
    hash ^= byte;
    hash *= 1099511628211ull;
  }
  std::ostringstream out;
  out << "fnv1a64:" << std::hex << std::setfill('0') << std::setw(16) << hash;
  fingerprint = out.str();
  data = std::make_unique<pinocchio::Data>(model);
}

// Queries provide every movable joint explicitly, including joints outside the solved group.
Eigen::VectorXd
KinematicsModel::Impl::configuration(const sensor_msgs::msg::JointState &state) const {
  if (state.name.size() != names.size() || state.position.size() != names.size())
    throw std::invalid_argument("Supply every movable joint exactly once with a position");
  Eigen::VectorXd q(model.nq);
  std::set<std::string> seen;
  for (size_t i = 0; i < state.name.size(); ++i) {
    auto it = q_indices.find(state.name[i]);
    if (it == q_indices.end() || !seen.insert(state.name[i]).second)
      throw std::invalid_argument("Unknown or duplicate joint: " + state.name[i]);
    double value = state.position[i];
    int index = it->second;
    if (!std::isfinite(value) || value < model.lowerPositionLimit[index] ||
        value > model.upperPositionLimit[index])
      throw std::invalid_argument("Non-finite or out-of-limit position: " + state.name[i]);
    q[index] = value;
  }
  return q;
}

pinocchio::FrameIndex KinematicsModel::Impl::frame(const std::string &name) const {
  auto it = frames.find(name);
  if (it == frames.end())
    throw std::invalid_argument("Unknown URDF link frame: " + name);
  return it->second;
}

geometry_msgs::msg::PoseStamped KinematicsModel::Impl::pose(pinocchio::FrameIndex id) const {
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = "base_link";
  const auto &transform = data->oMf[id];
  const auto &p = transform.translation();
  result.pose.position.x = p.x();
  result.pose.position.y = p.y();
  result.pose.position.z = p.z();
  Eigen::Quaterniond q(transform.rotation());
  q.normalize();
  result.pose.orientation.x = q.x();
  result.pose.orientation.y = q.y();
  result.pose.orientation.z = q.z();
  result.pose.orientation.w = q.w();
  return result;
}

KinematicsModel::KinematicsModel(const std::string &xml, const std::vector<KinematicsGroup> &groups,
                                 const IkOptions &options)
    : impl_(std::make_unique<Impl>(xml, groups, options)) {}

KinematicsModel::~KinematicsModel() = default;

const std::vector<std::string> &KinematicsModel::joint_names() const {
  return impl_->names;
}

const std::string &KinematicsModel::fingerprint() const {
  return impl_->fingerprint;
}

std::string KinematicsModel::pinocchio_version() {
  return PINOCCHIO_VERSION;
}
}
