#pragma once

#include "erob_driver/types.hpp"

#include <map>
#include <memory>

namespace erob {
struct Exchange {
  int wkc = 0, expected_wkc = 0;
  Nanoseconds dc_time = 0;
  bool valid = false;
  int fault_motor = -1;
};

class Transport {
public:
  virtual ~Transport() = default;
  virtual std::vector<Device> discover(const BusConfig &) = 0;
  virtual void configure(const BusConfig &, const std::vector<uint16_t> &) = 0;
  virtual Exchange exchange(const std::vector<RxBytes> &, std::vector<TxBytes> &) noexcept = 0;
  virtual uint32_t read_object(uint16_t slave, uint16_t index, uint8_t sub, uint8_t size) = 0;
  virtual void write_gain(uint16_t slave, uint16_t index, uint8_t sub, uint16_t value) = 0;
  virtual void close() noexcept = 0;
};

std::unique_ptr<Transport> make_transport(const BusConfig &);
std::unique_ptr<Transport> make_soem_transport();

// Deterministic fake bus below the production PDO/state-machine implementation.
class MockTransport final : public Transport {
public:
  std::vector<Device> devices;
  std::vector<TxPdo> values;
  std::vector<RxPdo> last_outputs;
  int bad_wkc_cycles = 0, lost_motor = -1, mode_delay_cycles = 0;
  bool freeze_feedback = false, freeze_motion = false;
  bool corrupt_readback = false, refuse_mapping = false, wrong_size = false, freeze_enable = false,
       refuse_mode = false;
  std::vector<Device> discover(const BusConfig &) override;
  void configure(const BusConfig &, const std::vector<uint16_t> &) override;
  Exchange exchange(const std::vector<RxBytes> &, std::vector<TxBytes> &) noexcept override;
  uint32_t read_object(uint16_t, uint16_t, uint8_t, uint8_t) override;
  void write_gain(uint16_t, uint16_t, uint8_t, uint16_t) override;

  void close() noexcept override {}

private:
  std::map<uint64_t, std::pair<uint8_t, uint32_t>> dictionary_;
  Nanoseconds now_ = 0, cycle_ = ms(1);
  std::vector<int> pending_cycles_;
  std::vector<Mode> pending_modes_;
  std::vector<std::array<uint16_t, 4>> gains_;
};
} // namespace erob
