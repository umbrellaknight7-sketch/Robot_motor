#pragma once

#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <vector>
#include <algorithm>

namespace wheel_human_commander {
class RobotStateCache {
public:
  explicit RobotStateCache(std::vector<std::string> names)
      : names_(std::move(names)), q_(names_.size()), v_(names_.size()), received_(names_.size()),
        valid_(names_.size(), false) {}

  void invalidate() {
    std::fill(valid_.begin(), valid_.end(), false);
  }

  void invalidate(const std::vector<std::string> &);
  void update(const sensor_msgs::msg::JointState &, double received);
  bool fresh(double now, double maximum_age) const;
  bool fresh(const std::string &, double now, double maximum_age) const;

  const std::vector<double> &positions() const {
    return q_;
  }

  const std::vector<double> &velocities() const {
    return v_;
  }

private:
  std::vector<std::string> names_;
  std::vector<double> q_, v_, received_;
  std::vector<bool> valid_;
};
}
