#pragma once

#include "erob_driver/pdo.hpp"

#include <algorithm>
#include <stdexcept>

namespace erob {
// Configuration-only protocol shared by real SDO transport and the fake
// dictionary. The callbacks are never used in the cyclic path.
template <class Read, class Write>
void configure_drive_objects(const BusConfig &c, const std::vector<uint16_t> &map, Read read,
                             Write write) {
  auto set = [&](uint16_t slave, uint16_t index, uint8_t sub, uint8_t size, uint32_t value) {
    write(slave, index, sub, size, value);
    if (read(slave, index, sub, size) != value)
      throw std::runtime_error("SDO configuration readback mismatch");
  };
  auto map_entries = [&](uint16_t slave, uint16_t index, const auto &entries) {
    set(slave, index, 0, 1, 0);
    for (size_t j = 0; j < entries.size(); ++j)
      set(slave, index, uint8_t(j + 1), 4, entries[j]);
    set(slave, index, 0, 1, uint32_t(entries.size()));
  };
  for (size_t k = 0; k < map.size(); ++k) {
    const auto i = map[k];
    const auto &d = c.drives[k];
    set(i, 0x1c12, 0, 1, 0);
    set(i, 0x1c13, 0, 1, 0);
    if (c.csp_feedforward)
      map_entries(i, 0x1600, rx_feedforward_mapping);
    else
      map_entries(i, 0x1600, rx_mapping);
    map_entries(i, 0x1a00, tx_mapping);
    set(i, 0x1c12, 1, 2, 0x1600);
    set(i, 0x1c13, 1, 2, 0x1a00);
    set(i, 0x1c12, 0, 1, 1);
    set(i, 0x1c13, 0, 1, 1);
    const double p1 = d.zero_counts + d.direction * d.min_position * d.position_counts_per_rad;
    const double p2 = d.zero_counts + d.direction * d.max_position * d.position_counts_per_rad;
    if (!fits32(p1) || !fits32(p2) || !fits32(d.max_velocity * d.velocity_counts_per_rad_s))
      throw std::runtime_error("Drive limits cannot be represented");
    set(i, 0x607d, 1, 4, uint32_t(int32_t(std::llround(std::min(p1, p2)))));
    set(i, 0x607d, 2, 4, uint32_t(int32_t(std::llround(std::max(p1, p2)))));
    set(i, 0x6080, 0, 4, uint32_t(std::llround(d.max_velocity * d.velocity_counts_per_rad_s)));
    set(i, 0x6072, 0, 2, d.max_torque_raw);
    set(i, 0x605a, 0, 2, 2);
    set(i, 0x6085, 0, 4, d.quick_stop_deceleration);
    set(i, 0x1c32, 1, 2, 2);
    set(i, 0x1c33, 1, 2, 2);
    auto minimum = read(i, 0x1c32, 5, 4);
    if (minimum && c.cycle_ns < minimum)
      throw std::runtime_error("Requested cycle below drive minimum");
  }
}

inline void verify_process_image(size_t outputs, size_t inputs, bool feedforward = false) {
  if (outputs != rx_wire_size(feedforward) || inputs != TxBytes{}.size())
    throw std::runtime_error("PDO process-image size mismatch");
}
} // namespace erob
