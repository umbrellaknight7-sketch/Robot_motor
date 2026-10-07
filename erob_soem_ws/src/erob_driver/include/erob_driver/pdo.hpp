#pragma once

#include "erob_driver/types.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace erob {
inline void put16(uint8_t *p, uint16_t v) {
  p[0] = v & 255;
  p[1] = (v >> 8) & 255;
}

inline void put32(uint8_t *p, uint32_t v) {
  for (int i = 0; i < 4; ++i)
    p[i] = (v >> (8 * i)) & 255;
}

inline uint16_t get16(const uint8_t *p) {
  return uint16_t(p[0]) | (uint16_t(p[1]) << 8);
}

inline uint32_t get32(const uint8_t *p) {
  uint32_t v = 0;
  for (int i = 0; i < 4; ++i)
    v |= uint32_t(p[i]) << (8 * i);
  return v;
}

inline int16_t signed16(const uint8_t *p) {
  auto v = get16(p);
  int16_t s;
  std::memcpy(&s, &v, 2);
  return s;
}

inline int32_t signed32(const uint8_t *p) {
  auto v = get32(p);
  int32_t s;
  std::memcpy(&s, &v, 4);
  return s;
}

// Pack explicit little-endian wire offsets; do not depend on C++ struct padding.
inline RxBytes encode(const RxPdo &v) {
  RxBytes b{};
  put16(b.data(), v.control);
  put32(b.data() + 2, uint32_t(v.position));
  put32(b.data() + 6, uint32_t(v.velocity));
  put16(b.data() + 10, uint16_t(v.torque));
  b[12] = uint8_t(v.mode);
  put32(b.data() + 14, uint32_t(v.velocity_offset));
  put16(b.data() + 18, uint16_t(v.torque_offset));
  return b;
}

inline TxBytes encode(const TxPdo &v) {
  TxBytes b{};
  put16(b.data(), v.status);
  put16(b.data() + 2, v.error);
  put32(b.data() + 4, uint32_t(v.position));
  put32(b.data() + 8, uint32_t(v.velocity));
  put16(b.data() + 12, uint16_t(v.torque));
  b[14] = uint8_t(v.mode);
  return b;
}

inline RxPdo decode(const RxBytes &b) {
  return {get16(b.data()), signed32(b.data() + 2),  signed32(b.data() + 6), signed16(b.data() + 10),
          Mode(b[12]),     signed32(b.data() + 14), signed16(b.data() + 18)};
}

inline TxPdo decode(const TxBytes &b) {
  return {get16(b.data()),        get16(b.data() + 2),     signed32(b.data() + 4),
          signed32(b.data() + 8), signed16(b.data() + 12), Mode(b[14])};
}

inline bool enabled(uint16_t s) {
  return (s & 0x006f) == 0x0027;
}

// Decode the CiA 402 state bits before selecting the next enable controlword.
inline uint16_t enable_word(uint16_t s) {
  switch (s & 0x006f) {
  case 0x0040:
    return 6;
  case 0x0021:
    return 7;
  case 0x0023:
  case 0x0027:
    return 15;
  default:
    return 0;
  }
}

inline bool fits32(double x) {
  return std::isfinite(x) && x >= std::numeric_limits<int32_t>::min() &&
         x <= std::numeric_limits<int32_t>::max();
}

// Apply the calibrated zero and direction at the raw-counts/SI boundary.
inline double joint_position(int32_t raw, const DriveConfig &d) {
  return d.direction * (double(raw) - d.zero_counts) / d.position_counts_per_rad;
}

inline double joint_velocity(int32_t raw, const DriveConfig &d) {
  return d.direction * double(raw) / d.velocity_counts_per_rad_s;
}

inline double joint_effort(int16_t raw, const DriveConfig &d) {
  return d.direction * double(raw) * d.nm_per_raw;
}

inline bool command_valid(const Command &c, const DriveConfig &d) {
  if (!valid_mode(c.mode) || !std::isfinite(c.position) || !std::isfinite(c.velocity) ||
      !std::isfinite(c.effort) || !std::isfinite(c.velocity_offset) ||
      !std::isfinite(c.torque_offset))
    return false;
  const bool has_offset = c.velocity_offset != 0 || c.torque_offset != 0;
  if (has_offset && (!c.claimed || c.mode != Mode::CSP || !d.csp_feedforward))
    return false;
  if (d.csp_feedforward) {
    const double raw = d.nm_per_raw > 0 ? c.torque_offset / d.nm_per_raw : 0;
    if (!d.effort_verified || d.nm_per_raw <= 0 ||
        std::abs(c.velocity_offset) > d.max_velocity_offset ||
        !fits32(d.direction * c.velocity_offset * d.velocity_counts_per_rad_s) ||
        std::abs(c.torque_offset) > d.max_torque_offset || !std::isfinite(raw) ||
        std::abs(raw) > d.max_torque_raw || std::abs(raw) > 32767)
      return false;
  }
  if (!c.claimed)
    return true;
  if (c.mode == Mode::CSP)
    return c.position >= d.min_position && c.position <= d.max_position &&
           fits32(d.zero_counts + d.direction * c.position * d.position_counts_per_rad);
  if (c.mode == Mode::CSV)
    return std::abs(c.velocity) <= d.max_velocity &&
           fits32(d.direction * c.velocity * d.velocity_counts_per_rad_s);
  const double raw = d.nm_per_raw > 0 ? d.direction * c.effort / d.nm_per_raw
                                      : std::numeric_limits<double>::infinity();
  return d.effort_verified && std::abs(c.effort) <= d.max_effort && std::isfinite(raw) &&
         raw >= -32768 && raw <= 32767 && std::abs(raw) <= d.max_torque_raw;
}

inline void apply_command(const Command &c, const DriveConfig &d, RxPdo &out) {
  out.velocity_offset = 0;
  out.torque_offset = 0;
  if (c.mode == Mode::CSP && d.csp_feedforward) {
    out.velocity_offset =
        int32_t(std::llround(d.direction * c.velocity_offset * d.velocity_counts_per_rad_s));
    out.torque_offset = int16_t(std::llround(d.direction * c.torque_offset / d.nm_per_raw));
  }
  if (c.mode == Mode::CSP)
    out.position =
        int32_t(std::llround(d.zero_counts + d.direction * c.position * d.position_counts_per_rad));
  if (c.mode == Mode::CSV)
    out.velocity = int32_t(std::llround(d.direction * c.velocity * d.velocity_counts_per_rad_s));
  if (c.mode == Mode::CST)
    out.torque = int16_t(std::llround(d.direction * c.effort / d.nm_per_raw));
}

constexpr std::array<uint32_t, 6> rx_mapping{
    {0x60400010, 0x607a0020, 0x60ff0020, 0x60710010, 0x60600008, 0x00000008}};
constexpr std::array<uint32_t, 8> rx_feedforward_mapping{{0x60400010, 0x607a0020, 0x60ff0020,
                                                          0x60710010, 0x60600008, 0x00000008,
                                                          0x60b10020, 0x60b20010}};
constexpr std::array<uint32_t, 7> tx_mapping{
    {0x60410010, 0x603f0010, 0x60640020, 0x606c0020, 0x60770010, 0x60610008, 0x00000008}};
} // namespace erob
