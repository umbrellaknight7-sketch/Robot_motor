#pragma once

#include "wheel_human_commander/motion.hpp"
#include "wheel_human_commander/robot_state_cache.hpp"

#include <erob_driver/types.hpp>
#include <wheel_human_interfaces/action/move_joints.hpp>

namespace wheel_human_commander {
struct ValidatedGoal {
  Code code = Code::InvalidGoal;
  std::string message;
  std::vector<double> position, torque;
  double duration = 0;

  explicit operator bool() const {
    return code == Code::Success;
  }
};

ValidatedGoal validate_goal(const wheel_human_interfaces::action::MoveJoints::Goal &,
                            const erob::BusConfig &, const RobotStateCache &);
}
