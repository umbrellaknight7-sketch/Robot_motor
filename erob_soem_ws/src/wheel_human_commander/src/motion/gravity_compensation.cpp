#include "wheel_human_commander/motion.hpp"

namespace wheel_human_commander {
MotionResult gravity_compensation(const std::vector<double> &) {
  // TODO: Gravity compensation requires calibrated dynamics and torque conversion.
  return {Code::NotImplemented,
          "Gravity compensation requires calibrated dynamics and torque conversion",
          {}};
}
}
