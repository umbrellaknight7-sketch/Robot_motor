#pragma once

#include "erob_driver/transport.hpp"

namespace erob {
class Engine {
public:
  Engine(BusConfig, Transport &);
  void configure();
  bool activate(Nanoseconds now);
  void deactivate(Nanoseconds now);
  bool reset_fault(); // Inactive only after drive disable; never resets encoder
                      // or writes flash.
  void trip(Fault, int motor, Nanoseconds now) noexcept;
  void tick(Nanoseconds now, const CommandFrame *command = nullptr) noexcept;
  bool resume(uint64_t generation, Nanoseconds now) noexcept;
  void reject(RejectReason, int joint = -1, uint64_t count = 1) noexcept;
  void monitor_deadline(Nanoseconds now, Nanoseconds scheduled) noexcept;

  const Snapshot &state() const noexcept {
    return state_;
  }

  const std::vector<Device> &devices() const {
    return devices_;
  }

  const std::vector<uint16_t> &mapping() const {
    return mapping_;
  }

private:
  void begin_hold(Fault reason, Nanoseconds now) noexcept;
  void plan_stop(Nanoseconds now) noexcept;
  void hold_tick(Nanoseconds now) noexcept;
  bool ready_to_resume(Nanoseconds now) const noexcept;
  BusConfig config_;
  Transport &transport_;
  Snapshot state_;
  std::vector<Device> devices_;
  std::vector<uint16_t> mapping_;
  std::vector<RxBytes> outputs_;
  std::vector<TxBytes> inputs_;
  std::vector<RxPdo> desired_;
  std::vector<Command> commands_;
  std::vector<Nanoseconds> stationary_since_, switching_since_;
  Nanoseconds transition_start_ = 0, enabled_since_ = 0, last_command_ = 0, fault_since_ = 0,
              active_since_ = 0;
  std::vector<double> stop_q_, stop_v_, hold_tau_;
  Nanoseconds offset_stop_start_ = 0;
  std::vector<bool> hold_claimed_;
  Nanoseconds stop_start_ = 0, stop_duration_ = 0, settled_since_ = 0, healthy_since_ = 0,
              resume_after_ = 0, settling_deadline_ = 0;
  bool deadline_pending_ = false, command_watchdog_armed_ = false;
  Nanoseconds last_dc_time_ = 0;
  unsigned late_cycles_ = 0;
  unsigned bad_cycles_ = 0;
  bool have_command_ = false, disabled_confirmed_ = false;
  uint64_t min_generation_ = 0;
};
} // namespace erob
