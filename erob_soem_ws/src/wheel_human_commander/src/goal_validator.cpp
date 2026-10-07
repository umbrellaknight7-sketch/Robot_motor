#include "wheel_human_commander/goal_validator.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace wheel_human_commander {
ValidatedGoal validate_goal(const wheel_human_interfaces::action::MoveJoints::Goal &g,
                            const erob::BusConfig &config, const RobotStateCache &state) {
  ValidatedGoal r;
  if (g.gravity_compensation) {
    r.code = Code::NotImplemented;
    r.message = "Automatic gravity compensation is not implemented";
    return r;
  }
  r.duration = g.duration.sec + g.duration.nanosec * 1e-9;
  if (!std::isfinite(r.duration) || r.duration < .001 || r.duration > 3600 ||
      g.duration.nanosec >= 1000000000) {
    r.message = "Duration must be in [0.001,3600] seconds";
    return r;
  }
  if (g.joint_names.empty() || g.joint_names.size() != g.positions.size() ||
      g.torque_joint_names.size() != g.torque_offsets.size()) {
    r.message = "Invalid vector sizes";
    return r;
  }
  if (state.positions().size() != config.drives.size() ||
      state.velocities().size() != config.drives.size()) {
    r.message = "Incomplete state";
    return r;
  }

  // Start from measured positions so omitted body joints hold their measured positions.
  std::map<std::string, size_t> indices;
  for (size_t i = 0; i < config.drives.size(); ++i)
    indices[config.drives[i].joint] = i;
  std::set<std::string> requested, torque_names;
  r.position = state.positions();
  r.torque.assign(config.drives.size(), 0);
  for (size_t j = 0; j < g.joint_names.size(); ++j) {
    const auto &name = g.joint_names[j];
    if (!indices.count(name) || !requested.insert(name).second || !std::isfinite(g.positions[j])) {
      r.message = "Unknown, duplicate, or nonfinite joint target";
      return r;
    }
    r.position[indices[name]] = g.positions[j];
  }

  // Torque offsets may address only joints explicitly targeted by this goal.
  for (size_t j = 0; j < g.torque_joint_names.size(); ++j) {
    const auto &name = g.torque_joint_names[j];
    if (!requested.count(name) || !torque_names.insert(name).second ||
        !std::isfinite(g.torque_offsets[j])) {
      r.message = "Torque names must be a unique subset of target joints";
      return r;
    }
    r.torque[indices[name]] = g.torque_offsets[j];
  }

  // Peak derivatives of the rest-to-rest quintic bound the requested duration.
  for (size_t i = 0; i < config.drives.size(); ++i) {
    const auto &d = config.drives[i];
    double delta = std::abs(r.position[i] - state.positions()[i]);
    if (std::abs(state.velocities()[i]) > d.standstill_velocity || r.position[i] < d.min_position ||
        r.position[i] > d.max_position || state.positions()[i] < d.min_position ||
        state.positions()[i] > d.max_position || std::abs(r.torque[i]) > d.max_torque_offset ||
        1.875 * delta / r.duration > std::min(d.max_velocity, d.max_velocity_offset) ||
        5.773502691896258 * delta / (r.duration * r.duration) > d.hold_acceleration ||
        60 * delta / (r.duration * r.duration * r.duration) > d.hold_jerk) {
      r.message = "Joint limits, moving start, or infeasible duration: " + d.joint;
      return r;
    }
  }
  r.code = Code::Success;
  r.message = "Valid";
  return r;
}
}
