#pragma once

#include <string>

namespace wheel_human_commander {
// Lease times use monotonic seconds; an expired token cannot renew itself.
class OwnershipManager {
public:
  explicit OwnershipManager(double lease_seconds = 2);
  std::string acquire(const std::string &source, double now);
  bool valid(const std::string &token, double now) const;
  bool renew(const std::string &token, double now);
  bool expired(double now) const;
  void revoke();
  double remaining(double now) const;

  const std::string &owner() const {
    return owner_;
  }

private:
  std::string owner_, token_;
  double lease_, deadline_ = 0;
};
}
