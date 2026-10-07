#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace erob_controllers {
constexpr size_t capacity = 200;
using Values = std::array<double, capacity>;
enum class Phase { Inactive, Holding, Moving, Stopping, Fault };
enum class Outcome { None, Succeeded, Canceled, Aborted };

struct Limits {
  double lower = 0, upper = 0, velocity = 0, acceleration = 0, jerk = 0;
  double torque = 0, ramp = 0.2, standstill = 0.01, tracking = 0.05;
};

// Sequence identifies a request; cancel_goal identifies the action being stopped.
struct Request {
  bool stop = false;
  uint64_t sequence = 0, cancel_goal = 0;
  double duration = 0;
  Values position{}, torque{};
};

// Published state keeps the request generation separate from the active goal.
struct Snapshot {
  Phase phase = Phase::Inactive;
  Outcome outcome = Outcome::None;
  uint64_t generation = 0, goal = 0;
  double stamp = 0, progress = 0;
  bool neutral = true;
  Values actual{}, velocity{}, position{}, velocity_offset{}, torque{};
};

class TrajectorySampler {
public:
  void configure(size_t size, const std::array<Limits, capacity> &limits);
  void activate(const Values &q, const Values &v, double now);
  bool feasible(const Request &, const Values &q, const Values &v) const noexcept;
  void tick(double now, const Values &q, const Values &v, const Request *request, bool heartbeat,
            double goal_tolerance, double settle_timeout) noexcept;

  const Snapshot &state() const {
    return state_;
  }

private:
  void stop(double now, const Values &q, const Values &v, Outcome outcome) noexcept;
  size_t size_ = 0;
  std::array<Limits, capacity> limits_{};
  Snapshot state_{};
  Values start_{}, target_{}, start_velocity_{}, start_torque_{}, end_torque_{};
  double began_ = 0, duration_ = 0, settled_since_ = -1;
  Outcome stopping_outcome_ = Outcome::Canceled;
};

const char *phase_name(Phase);
} // namespace erob_controllers
