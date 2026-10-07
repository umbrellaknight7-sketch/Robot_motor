#include "erob_driver/config.hpp"
#include "erob_driver/pdo.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace erob {
namespace {
template <class T> T value(const YAML::Node &n, const char *key, T fallback) {
  return n[key] ? n[key].as<T>() : fallback;
}

uint32_t number(const YAML::Node &n) {
  if (!n)
    return 0;
  const auto s = n.as<std::string>();
  size_t used = 0;
  auto v = std::stoull(s, &used, 0);
  if (used != s.size() || v > UINT32_MAX)
    throw std::runtime_error("Invalid unsigned integer: " + s);
  return uint32_t(v);
}

void require(bool v, const std::string &text) {
  if (!v)
    throw std::runtime_error(text);
}
} // namespace

std::string path_string(const std::vector<uint8_t> &path) {
  std::ostringstream s;
  for (auto p : path) {
    if (s.tellp() > 0)
      s << '/';
    s << unsigned(p);
  }
  return s.str();
}

BusConfig load_config(const std::string &file) {
  auto root = YAML::LoadFile(file);
  auto b = root["bus"];
  auto p = root["profile"];
  BusConfig c;
  c.backend = value<std::string>(b, "backend", "mock");
  c.csp_feedforward = value<bool>(b, "csp_feedforward", false);
  c.interface = value<std::string>(b, "interface", "");
  c.expected_motors = value<unsigned>(b, "expected_motors", 15);
  c.cycle_ns = value<int64_t>(b, "cycle_us", 1000) * 1000;
  c.command_timeout_ns = ms(value<int64_t>(b, "command_timeout_ms", 20));
  c.feedback_timeout_ns = ms(value<int64_t>(b, "feedback_timeout_ms", 5));
  c.bad_cycles = value<unsigned>(b, "bad_cycles", 5);
  c.missed_deadlines = value<unsigned>(b, "missed_deadlines", 5);
  c.lateness_ns = value<int64_t>(b, "lateness_us", 2000) * 1000;
  c.cpu = value<int>(b, "cpu", -1);
  c.priority = value<int>(b, "priority", 60);
  c.require_realtime = value<bool>(b, "require_realtime", false);
  c.hub_model = value<std::string>(b, "hub_model", "");
  c.esi_reference = value<std::string>(b, "esi_reference", "");
  c.topology_verified = value<bool>(b, "topology_verified", false);
  c.stop_verified = value<bool>(b, "stop_verified", false);
  c.watchdog_verified = value<bool>(b, "watchdog_verified", false);
  c.padding_verified = value<bool>(b, "padding_verified", false);
  c.enable_delay_ns = ms(value<int64_t>(b, "enable_delay_ms", 500));
  c.disable_delay_ns = ms(value<int64_t>(b, "disable_delay_ms", 301));
  c.transition_timeout_ns = ms(value<int64_t>(b, "transition_timeout_ms", 10000));
  c.mode_timeout_ns = ms(value<int64_t>(b, "mode_timeout_ms", 1000));
  c.standstill_ns = ms(value<int64_t>(b, "standstill_ms", 100));

  // Per-motor settings override the shared profile defaults.
  for (const auto &n : root["motors"]) {
    DriveConfig d;
    d.logical_id = number(n["logical_id"]);
    d.joint = n["joint"].as<std::string>();
    d.branch = n["branch"].as<std::string>();
    d.branch_position = n["branch_position"].as<unsigned>();
    for (const auto &port : n["topology_path"]) {
      auto k = port.as<unsigned>();
      require(k <= 3, "Port must be 0..3");
      d.path.push_back(uint8_t(k));
    }
    auto id = n["identity"];
    d.identity = {number(id["vendor"]), number(id["product"]), number(id["revision"]),
                  number(id["serial"])};
    d.require_serial = value<bool>(id, "require_serial", false);
    auto get = [&](const char *key, double fallback) {
      return value<double>(n, key, value<double>(p, key, fallback));
    };
    d.verified = value<bool>(n, "verified", value<bool>(p, "verified", false));
    d.effort_verified = value<bool>(n, "effort_verified", value<bool>(p, "effort_verified", false));
    d.position_counts_per_rad = get("position_counts_per_rad", 0);
    d.velocity_counts_per_rad_s = get("velocity_counts_per_rad_s", 0);
    d.zero_counts = get("zero_counts", 0);
    d.direction = get("direction", 1);
    d.nm_per_raw = get("nm_per_raw", 0);
    d.min_position = get("min_position", 0);
    d.max_position = get("max_position", 0);
    d.max_velocity = get("max_velocity", 0);
    d.max_effort = get("max_effort", 0);
    d.standstill_velocity = get("standstill_velocity", 0.01);
    d.hold_acceleration = get("hold_acceleration", 0);
    d.hold_jerk = get("hold_jerk", 0);
    d.hold_tracking_error = get("hold_tracking_error", 0);
    d.hold_verified = value<bool>(n, "hold_verified", value<bool>(p, "hold_verified", false));
    d.csp_feedforward = c.csp_feedforward;
    d.feedforward_verified =
        value<bool>(n, "feedforward_verified", value<bool>(p, "feedforward_verified", false));
    d.max_velocity_offset = get("max_velocity_offset", 0);
    d.max_torque_offset = get("max_torque_offset", 0);
    d.offset_ramp_seconds = get("offset_ramp_seconds", 0);
    d.max_torque_raw =
        value<uint16_t>(n, "max_torque_raw", value<uint16_t>(p, "max_torque_raw", 0));
    d.quick_stop_deceleration = value<uint32_t>(n, "quick_stop_deceleration",
                                                value<uint32_t>(p, "quick_stop_deceleration", 0));
    d.initial_mode = Mode(value<int>(n, "initial_mode", 8));
    c.drives.push_back(d);
  }
  validate_config(c);
  return c;
}

void validate_config(const BusConfig &c) {
  require(c.backend == "mock" || c.backend == "soem", "backend must be mock or soem");
  require(c.expected_motors > 0 && c.drives.size() == c.expected_motors,
          "Configured motor count mismatch");
  require(c.drives.size() < 200, "Too many motors");
  require(c.cycle_ns >= 500000 && c.cycle_ns % 500000 == 0 && c.cycle_ns <= ms(10),
          "Cycle must be a 500 us multiple in [0.5,10] ms");
  require(c.command_timeout_ns > c.cycle_ns && c.feedback_timeout_ns >= c.cycle_ns &&
              c.bad_cycles > 0 && c.missed_deadlines > 0,
          "Invalid watchdog parameters");
  require(c.enable_delay_ns >= ms(500) && c.disable_delay_ns > ms(300) &&
              c.transition_timeout_ns > c.enable_delay_ns + c.disable_delay_ns &&
              c.mode_timeout_ns > 0 && c.standstill_ns > 0 && c.lateness_ns > 0,
          "Invalid lifecycle timing");
  require(c.cpu >= -1 && c.cpu < 1024 && c.priority >= 1 && c.priority <= 99,
          "Invalid realtime settings");
  if (c.backend == "soem")
    require(!c.interface.empty() && c.interface.size() < 16 &&
                c.interface.find_first_not_of(
                    "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ012345"
                    "6789_.:-") == std::string::npos,
            "Invalid network interface");

  // Logical names, physical paths and branch slots must each be unambiguous.
  std::set<std::string> names, paths, slots;
  std::set<unsigned> ids;
  for (const auto &d : c.drives) {
    require(!d.joint.empty() && names.insert(d.joint).second && d.logical_id > 0 &&
                ids.insert(d.logical_id).second,
            "Duplicate/invalid joint or logical ID");
    require(!d.branch.empty() && d.branch_position > 0 &&
                slots.insert(d.branch + ":" + std::to_string(d.branch_position)).second,
            "Duplicate/invalid branch position");
    require(!d.path.empty() && paths.insert(path_string(d.path)).second,
            "Duplicate/missing topology path");
    require(d.identity.vendor && d.identity.product && d.identity.revision,
            "Expected device identity is required");
    require(!d.require_serial || d.identity.serial, "Required serial cannot be zero");
    for (double x : {d.position_counts_per_rad, d.velocity_counts_per_rad_s, d.zero_counts,
                     d.direction, d.nm_per_raw, d.min_position, d.max_position, d.max_velocity,
                     d.max_effort, d.standstill_velocity})
      require(std::isfinite(x), "Non-finite profile value");
    require(d.position_counts_per_rad > 0 && d.velocity_counts_per_rad_s > 0 &&
                std::abs(d.direction) == 1 && d.min_position < d.max_position &&
                d.max_velocity > 0 && d.standstill_velocity > 0,
            "Invalid conversion/limits");
    require(d.max_torque_raw > 0 && d.max_torque_raw <= 32767 && d.quick_stop_deceleration > 0,
            "Drive stop/torque limits are required");
    require(!d.effort_verified || (d.nm_per_raw > 0 && d.max_effort > 0 &&
                                   d.max_effort / d.nm_per_raw <= d.max_torque_raw),
            "Invalid effort calibration");
    require(valid_mode(d.initial_mode), "Invalid initial mode");
    require(d.csp_feedforward == c.csp_feedforward, "Inconsistent feedforward layout");
    if (c.csp_feedforward) {
      require(d.effort_verified && d.nm_per_raw > 0 && std::isfinite(d.max_velocity_offset) &&
                  d.max_velocity_offset > 0 && d.max_velocity_offset <= d.max_velocity &&
                  fits32(d.max_velocity_offset * d.velocity_counts_per_rad_s) &&
                  std::isfinite(d.max_torque_offset) && d.max_torque_offset > 0 &&
                  d.max_torque_offset <= d.max_effort &&
                  d.max_torque_offset / d.nm_per_raw <= d.max_torque_raw &&
                  std::isfinite(d.offset_ramp_seconds) && d.offset_ramp_seconds >= 0.001 &&
                  d.offset_ramp_seconds <= 10,
              "Invalid CSP feedforward conversion/limits");
    }
    require(d.initial_mode != Mode::CST || d.effort_verified,
            "CST requires verified effort calibration");
  }
}

void validate_activation(const BusConfig &c) {
  if (c.csp_hold_enabled) {
    for (const auto &d : c.drives) {
      require(d.initial_mode == Mode::CSP, "CSP hold requires CSP startup mode");
      require(std::isfinite(d.hold_acceleration) && d.hold_acceleration > 0 &&
                  std::isfinite(d.hold_jerk) && d.hold_jerk > 0 &&
                  std::isfinite(d.hold_tracking_error) && d.hold_tracking_error > 0,
              "CSP hold requires positive finite acceleration, jerk and tracking limits");
      require(c.backend == "mock" || d.hold_verified, "Unverified hold profile: " + d.joint);
    }
  }
  if (c.backend == "mock")
    return;
  require(c.topology_verified && c.stop_verified && c.watchdog_verified && c.padding_verified &&
              !c.hub_model.empty() && !c.esi_reference.empty(),
          "Real activation requires verified hub/ESI, topology, padding, stop "
          "and drive watchdog profile");
  for (const auto &d : c.drives)
    require(d.verified, "Unverified hardware profile: " + d.joint);
  if (c.csp_feedforward)
    for (const auto &d : c.drives)
      require(d.feedforward_verified, "Unverified CSP offsets/mapping/stop behavior: " + d.joint);
}

std::vector<uint16_t> match_devices(const BusConfig &c, const std::vector<Device> &devices) {
  std::vector<uint16_t> out;
  std::set<uint16_t> matched, indexes;
  size_t drives = 0;
  for (const auto &dev : devices) {
    require(dev.index > 0 && indexes.insert(dev.index).second, "Duplicate discovered slave index");
    if (dev.drive)
      ++drives;
  }
  require(drives == c.expected_motors,
          "Discovered motor count mismatch (infrastructure is counted separately)");
  for (const auto &d : c.drives) {
    const Device *found = nullptr;
    for (const auto &dev : devices)
      if (dev.drive && dev.path == d.path) {
        require(!found, "Ambiguous topology path for " + d.joint);
        found = &dev;
      }
    require(found, "Missing/misplaced motor: " + d.joint);
    const auto &id = found->identity;
    require(id.vendor == d.identity.vendor && id.product == d.identity.product &&
                id.revision == d.identity.revision &&
                (!d.require_serial || id.serial == d.identity.serial),
            "Identity mismatch: " + d.joint);
    require(found->has_dc, "Drive lacks DC support: " + d.joint);
    require((found->supported_modes & 0x380) == 0x380,
            "Drive does not advertise CSP/CSV/CST: " + d.joint);
    require(matched.insert(found->index).second, "Duplicate motor mapping");
    out.push_back(found->index);
  }
  return out;
}

const char *fault_text(Fault f) noexcept {
  switch (f) {
  case Fault::None:
    return "none";
  case Fault::InvalidCommand:
    return "invalid command";
  case Fault::CommandExpired:
    return "command handoff expired";
  case Fault::FeedbackLost:
    return "feedback/WKC lost";
  case Fault::DriveFault:
    return "drive fault or unexpected drive state";
  case Fault::ModeTimeout:
    return "mode acknowledgment timeout";
  case Fault::MovingModeSwitch:
    return "mode switch while moving";
  case Fault::TransitionTimeout:
    return "lifecycle transition timeout";
  case Fault::QueueOverflow:
    return "command queue overflow";
  case Fault::DeadlineMissed:
    return "cycle deadline missed";
  case Fault::TransportError:
    return "transport failure";
  case Fault::HoldFailed:
    return "CSP hold path, tracking or settling failure";
  }
  return "unknown";
}
} // namespace erob
