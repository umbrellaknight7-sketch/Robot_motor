#include "erob_driver/pdo.hpp"
#include "erob_driver/protocol.hpp"
#include "erob_driver/transport.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace erob {
std::vector<Device> MockTransport::discover(const BusConfig &c) {
  if (devices.empty()) {
    devices.push_back({1, 0x1000, {1, 1, 1, 0}, {}, "Synthetic junction", "mock", 0, false, true});
    for (size_t i = 0; i < c.drives.size(); ++i) {
      const auto &d = c.drives[i];
      devices.push_back({uint16_t(i + 2), uint16_t(0x1001 + i), d.identity, d.path, "Mock eRob",
                         "mock-v1", 0x380, true, true});
    }
  }
  return devices;
}

void MockTransport::configure(const BusConfig &c, const std::vector<uint16_t> &map) {

  configure_drive_objects(
      c, map,
      [&](auto slave, auto index, auto sub, auto size) {
        return read_object(slave, index, sub, size);
      },
      [&](uint16_t slave, uint16_t index, uint8_t sub, uint8_t size, uint32_t value) {
        if (refuse_mapping && index == 0x1600)
          throw std::runtime_error("Mock SDO abort 0x06040041: object cannot be PDO mapped");
        const uint64_t key = (uint64_t(slave) << 32) | (uint64_t(index) << 8) | sub;
        dictionary_[key] = {size, value};
      });
  for (auto slave : map) {
    auto bits = [&](uint16_t index) {
      uint32_t total = 0;
      const auto count = read_object(slave, index, 0, 1);
      for (uint8_t sub = 1; sub <= count; ++sub)
        total += read_object(slave, index, sub, 4) & 255;
      return (total + 7) / 8;
    };
    verify_process_image(bits(0x1600) + (wrong_size ? 1 : 0), bits(0x1a00), c.csp_feedforward);
  }
  cycle_ = c.cycle_ns;
  values.resize(map.size());
  last_outputs.resize(map.size());
  pending_cycles_.assign(map.size(), 0);
  pending_modes_.assign(map.size(), Mode::CSP);
  gains_.resize(map.size());
  for (size_t i = 0; i < map.size(); ++i) {
    values[i] = {0x40, 0, int32_t(c.drives[i].zero_counts), 0, 0, Mode::CSP};
  }
}

Exchange MockTransport::exchange(const std::vector<RxBytes> &tx,
                                 std::vector<TxBytes> &rx) noexcept {
  if (!freeze_feedback)
    now_ += cycle_;
  const int expected = int(tx.size()) * 3;
  for (size_t i = 0; i < tx.size(); ++i)
    last_outputs[i] = decode(tx[i]);
  if (bad_wkc_cycles > 0 || lost_motor >= 0) {
    if (bad_wkc_cycles > 0)
      --bad_wkc_cycles;
    return {expected - 1, expected, now_, false, lost_motor};
  }
  for (size_t i = 0; i < tx.size(); ++i) {
    const auto c = decode(tx[i]);
    last_outputs[i] = c;
    auto &v = values[i];
    if (v.error)
      v.status = 8;
    else if (!freeze_enable) {
      if (c.control == 0)
        v.status = 0x40;
      else if (c.control == 6)
        v.status = 0x21;
      else if (c.control == 7)
        v.status = 0x23;
      else if (c.control == 15 && ((v.status & 0x6f) == 0x23 || enabled(v.status)))
        v.status = 0x27;
      else if (c.control == 2) {
        v.status = 0x07;
        v.velocity = 0;
        v.torque = 0;
      }
    }
    if (c.mode != pending_modes_[i]) {
      pending_modes_[i] = c.mode;
      pending_cycles_[i] = mode_delay_cycles;
    }
    if (pending_cycles_[i] > 0)
      --pending_cycles_[i];
    else if (!refuse_mode)
      v.mode = c.mode;
    if (enabled(v.status) && !freeze_motion) {
      if (v.mode == Mode::CSP) {
        v.velocity = int32_t((double(c.position) - v.position) * 1e9 / double(cycle_));
        v.position = c.position;
        v.torque = 0;
      }
      if (v.mode == Mode::CSV) {
        v.velocity = c.velocity;
        v.position += int32_t(std::llround(double(c.velocity) * double(cycle_) / 1e9));
        v.torque = 0;
      }
      if (v.mode == Mode::CST) {
        v.torque = c.torque;
        v.velocity = 0;
      }
    } else if (!enabled(v.status)) {
      v.velocity = 0;
      v.torque = 0;
    }
    rx[i] = encode(v);
  }
  return {expected, expected, now_, true, -1};
}

uint32_t MockTransport::read_object(uint16_t slave, uint16_t index, uint8_t sub, uint8_t size) {
  if (size != 1 && size != 2 && size != 4)
    throw std::runtime_error("Object width must be 1, 2 or 4");
  auto it = std::find_if(devices.begin(), devices.end(),
                         [slave](const auto &d) { return d.index == slave; });
  if (it == devices.end())
    throw std::runtime_error("Unknown slave");
  const uint64_t key = (uint64_t(slave) << 32) | (uint64_t(index) << 8) | sub;
  auto object = dictionary_.find(key);
  if (object != dictionary_.end()) {
    if (object->second.first != size)
      throw std::runtime_error("SDO width mismatch");
    return object->second.second + (corrupt_readback ? 1 : 0);
  }
  if (index == 0x1c32 && sub == 5)
    return 500000;
  if (index == 0x1018) {
    const auto &d = it->identity;
    switch (sub) {
    case 1:
      return d.vendor;
    case 2:
      return d.product;
    case 3:
      return d.revision;
    case 4:
      return d.serial;
    }
  }
  if (index == 0x6502)
    return it->supported_modes;
  if (index == 0x6075)
    return 1000;
  if (index == 0x6076)
    return 1000;
  if (index == 0x603f)
    return 0;
  if (index == 0x6041)
    return 0x40;
  if (index == 0x6061)
    return 8;
  if (index == 0x1600)
    return sub == 0 ? rx_mapping.size() : rx_mapping.at(sub - 1);
  if (index == 0x1a00)
    return sub == 0 ? tx_mapping.size() : tx_mapping.at(sub - 1);
  if (index == 0x2381 || index == 0x2382) {
    auto i = size_t(std::distance(devices.begin(), it));
    if (gains_.size() <= i)
      gains_.resize(i + 1);
    if (index == 0x2381 && (sub == 1 || sub == 2))
      return gains_[i][sub - 1];
    if (index == 0x2382 && (sub == 1 || sub == 3))
      return gains_[i][sub == 1 ? 2 : 3];
  }
  throw std::runtime_error("SDO abort 0x06020000: object does not exist in mock dictionary");
}

void MockTransport::write_gain(uint16_t slave, uint16_t index, uint8_t sub, uint16_t value) {
  auto it = std::find_if(devices.begin(), devices.end(),
                         [slave](const auto &d) { return d.index == slave; });
  if (it == devices.end() || !it->drive)
    throw std::runtime_error("Unknown drive");
  int slot = -1;
  if (index == 0x2381 && (sub == 1 || sub == 2))
    slot = sub - 1;
  if (index == 0x2382 && (sub == 1 || sub == 3))
    slot = sub == 1 ? 2 : 3;
  if (slot < 0)
    throw std::runtime_error("Only velocity PI, position P, and velocity "
                             "feedforward gains are allowlisted");
  auto i = size_t(std::distance(devices.begin(), it));
  if (gains_.size() <= i)
    gains_.resize(i + 1);
  gains_[i][slot] = value;
}

std::unique_ptr<Transport> make_transport(const BusConfig &c) {
  if (c.backend == "mock")
    return std::make_unique<MockTransport>();
  if (c.backend == "soem")
    return make_soem_transport();
  throw std::runtime_error("Unknown backend");
}
} // namespace erob
