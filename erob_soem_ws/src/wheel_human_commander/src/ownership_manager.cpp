#include "wheel_human_commander/ownership_manager.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <random>
#include <sstream>
#include <stdexcept>

namespace wheel_human_commander {
OwnershipManager::OwnershipManager(double seconds) : lease_(seconds) {
  if (!std::isfinite(seconds) || seconds < 0.2 || seconds > 60)
    throw std::invalid_argument("lease_seconds must be in [0.2,60]");
}

std::string OwnershipManager::acquire(const std::string &source, double now) {
  if (source.empty() || source.size() > 128 || !owner_.empty())
    return {};
  std::random_device random;
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (int i = 0; i < 4; ++i)
    out << std::setw(8) << random();
  token_ = out.str();
  owner_ = source;
  deadline_ = now + lease_;
  return token_;
}

bool OwnershipManager::valid(const std::string &token, double now) const {
  return !owner_.empty() && !token.empty() && token == token_ && now < deadline_;
}

bool OwnershipManager::renew(const std::string &token, double now) {
  if (!valid(token, now))
    return false;
  deadline_ = now + lease_;
  return true;
}

bool OwnershipManager::expired(double now) const {
  return !owner_.empty() && now >= deadline_;
}

void OwnershipManager::revoke() {
  owner_.clear();
  token_.clear();
  deadline_ = 0;
}

double OwnershipManager::remaining(double now) const {
  return owner_.empty() ? 0 : std::max(0., deadline_ - now);
}
}
