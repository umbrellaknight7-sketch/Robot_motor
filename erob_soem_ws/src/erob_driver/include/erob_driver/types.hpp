#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace erob {
using Nanoseconds = int64_t;

constexpr Nanoseconds ms(int64_t n) {
  return n * 1000000;
}
enum class Mode : int8_t { CSP = 8, CSV = 9, CST = 10 };

inline bool valid_mode(Mode m) {
  return m == Mode::CSP || m == Mode::CSV || m == Mode::CST;
}
enum class Phase {
  Inactive,
  Enabling,
  Active,
  Disabling,
  Fault,
  Decelerating,
  Holding,
  HoldUnconfirmed
};

inline bool holding(Phase p) {
  return p == Phase::Decelerating || p == Phase::Holding || p == Phase::HoldUnconfirmed;
}

const char *phase_text(Phase) noexcept;
enum class RejectReason { None, Shape, Value, Stale, Order, Mode };
const char *reject_text(RejectReason) noexcept;

struct SubmitResult {
  enum Code { Accepted, Rejected, Unavailable } code;

  explicit operator bool() const noexcept {
    return code == Accepted;
  }
};
enum class Fault {
  None,
  InvalidCommand,
  CommandExpired,
  FeedbackLost,
  DriveFault,
  ModeTimeout,
  MovingModeSwitch,
  TransitionTimeout,
  QueueOverflow,
  DeadlineMissed,
  TransportError,
  HoldFailed
};
const char *fault_text(Fault fault) noexcept;

struct Identity {
  uint32_t vendor = 0, product = 0, revision = 0, serial = 0;
};

struct Device {
  uint16_t index = 0, address = 0;
  Identity identity;
  std::vector<uint8_t> path;
  std::string name, firmware;
  uint32_t supported_modes = 0;
  bool drive = false, has_dc = false;
};

struct DriveConfig {
  unsigned logical_id = 0, branch_position = 0;
  std::string joint, branch;
  std::vector<uint8_t> path;
  Identity identity;
  bool require_serial = false, verified = false, effort_verified = false;
  double position_counts_per_rad = 0, velocity_counts_per_rad_s = 0;
  double zero_counts = 0, direction = 1, nm_per_raw = 0;
  double min_position = 0, max_position = 0, max_velocity = 0, max_effort = 0;
  double standstill_velocity = 0.01;
  double hold_acceleration = 0, hold_jerk = 0, hold_tracking_error = 0;
  bool hold_verified = false;
  bool csp_feedforward = false, feedforward_verified = false;
  double max_velocity_offset = 0, max_torque_offset = 0;
  double offset_ramp_seconds = 0;
  uint16_t max_torque_raw = 0;
  uint32_t quick_stop_deceleration = 0;
  Mode initial_mode = Mode::CSP;
};

struct BusConfig {
  std::string backend = "mock", interface, hub_model, esi_reference;
  bool topology_verified = false, stop_verified = false, watchdog_verified = false;
  bool padding_verified = false, require_realtime = false;
  bool csp_hold_enabled = false; // Selected by the CSP-only ROS plugin.
  bool csp_feedforward = false;  // Selects the extended 20-byte RxPDO layout.
  unsigned expected_motors = 15;
  Nanoseconds cycle_ns = ms(1), command_timeout_ns = ms(20), feedback_timeout_ns = ms(5);
  Nanoseconds enable_delay_ns = ms(500), disable_delay_ns = ms(301);
  Nanoseconds transition_timeout_ns = ms(10000), mode_timeout_ns = ms(1000);
  Nanoseconds standstill_ns = ms(100), lateness_ns = ms(2);
  unsigned bad_cycles = 5, missed_deadlines = 5;
  int cpu = -1, priority = 60;
  std::vector<DriveConfig> drives;
};

// Joint-space values use radians, rad/s and Nm; conversion to drive units is separate.
struct Command {
  Mode mode = Mode::CSP;
  double position = 0, velocity = 0, effort = 0;
  bool claimed = false;
  double velocity_offset = 0, torque_offset = 0;
};

// Raw torque remains available even when calibrated effort is not verified.
struct JointState {
  double position = 0, velocity = 0, effort = 0;
  int16_t raw_torque = 0;
  uint16_t status = 0, error = 0;
  Mode mode = Mode::CSP, requested_mode = Mode::CSP;
  bool valid = false, effort_valid = false, switching = false;
  double hold_start = 0, hold_target = 0;
  double commanded_velocity_offset = 0, commanded_torque_offset = 0;
};

struct Health {
  Phase phase = Phase::Inactive;
  Fault fault = Fault::None;
  int fault_motor = -1, wkc = 0, expected_wkc = 0;
  Nanoseconds stamp_ns = 0, feedback_stamp_ns = 0, dc_offset_ns = 0;
  uint64_t cycle = 0, missed_deadlines = 0;
  bool feedback_valid = false;
  Fault hold_reason = Fault::None;
  uint64_t hold_generation = 0, rejected_commands = 0;
  RejectReason rejection = RejectReason::None;
  int rejected_joint = -1;
  unsigned bad_cycles = 0, late_cycles = 0;
  bool resume_ready = false, claims_released = false;
  Nanoseconds last_command_ns = 0;
};

struct Snapshot {
  std::vector<JointState> joints;
  Health health;
};

struct CommandFrame {
  std::vector<Command> joints;
  Nanoseconds stamp_ns = 0;
  uint64_t generation = 0;
};

// Storage capacity; legacy wire transfers remain exactly 14 bytes.
using RxBytes = std::array<uint8_t, 20>;

inline size_t rx_wire_size(bool feedforward) {
  return feedforward ? 20 : 14;
}

using TxBytes = std::array<uint8_t, 16>;

struct RxPdo {
  uint16_t control = 0;
  int32_t position = 0, velocity = 0;
  int16_t torque = 0;
  Mode mode = Mode::CSP;
  int32_t velocity_offset = 0;
  int16_t torque_offset = 0;
};

struct TxPdo {
  uint16_t status = 0, error = 0;
  int32_t position = 0, velocity = 0;
  int16_t torque = 0;
  Mode mode = Mode::CSP;
};
} // namespace erob
