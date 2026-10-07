#include "wheel_human_commander/robot_state_cache.hpp"

#include <cmath>
#include <map>

namespace wheel_human_commander {
void RobotStateCache::invalidate(const std::vector<std::string> &names) {
  for (size_t i = 0; i < names_.size(); ++i)
    if (std::find(names.begin(), names.end(), names_[i]) != names.end())
      valid_[i] = false;
}

bool RobotStateCache::fresh(const std::string &name, double now, double age) const {
  auto it = std::find(names_.begin(), names_.end(), name);
  if (it == names_.end())
    return false;
  auto i = std::distance(names_.begin(), it);
  return valid_[i] && now >= received_[i] && now - received_[i] <= age;
}

bool RobotStateCache::fresh(double now, double age) const {
  for (const auto &name : names_) {
    if (!fresh(name, now, age))
      return false;
  }
  return true;
}

void RobotStateCache::update(const sensor_msgs::msg::JointState &m, double now) {
  if (m.name.size() != m.position.size() || m.name.size() != m.velocity.size()) {
    invalidate(m.name);
    return;
  }
  std::map<std::string, size_t> index;
  for (size_t i = 0; i < m.name.size(); ++i)
    if (!index.emplace(m.name[i], i).second) {
      invalidate(m.name);
      return;
    }

  // Partial updates refresh only their named joints; other timestamps keep aging.
  for (size_t i = 0; i < names_.size(); ++i) {
    auto it = index.find(names_[i]);
    if (it == index.end())
      continue;
    double q = m.position[it->second], v = m.velocity[it->second];
    valid_[i] = std::isfinite(q) && std::isfinite(v);
    if (valid_[i]) {
      q_[i] = q;
      v_[i] = v;
      received_[i] = now;
    }
  }
}
}
