#include "erob_driver/config.hpp"
#include "erob_driver/pdo.hpp"
#include "erob_driver/protocol.hpp"
#include "erob_driver/topology.hpp"
#include "erob_driver/transport.hpp"
extern "C" {
#include <ethercat.h>
}
#include <algorithm>
#include <array>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sstream>
#include <stdexcept>
#include <sys/file.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

namespace erob {
namespace {
// Own the SOEM context and its backing storage for the lifetime of this transport.
class SoemTransport final : public Transport {
  ecx_portt port_{};
  std::array<ec_slavet, EC_MAXSLAVE> slaves_{};
  int count_ = 0;
  std::array<ec_groupt, EC_MAXGROUP> groups_{};
  std::array<uint8, EC_MAXEEPBUF> esibuf_{};
  std::array<uint32, EC_MAXEEPBITMAP> esimap_{};
  ec_eringt errors_{};
  ec_idxstackT stack_{};
  size_t rx_size_ = 14;
  boolean error_ = FALSE;
  int64 dc_ = 0;
  std::array<ec_SMcommtypet, EC_MAX_MAPT> smcomm_{};
  std::array<ec_PDOassignt, EC_MAX_MAPT> assigns_{};
  std::array<ec_PDOdesct, EC_MAX_MAPT> descs_{};
  ec_eepromSMt sm_{};
  ec_eepromFMMUt fmmu_{};
  ecx_contextt ctx_{};
  std::vector<uint8_t> io_ = std::vector<uint8_t>(1024 * 1024);
  std::vector<uint16_t> map_;
  int lock_ = -1, expected_ = 0;
  bool open_ = false, configured_ = false;
  int receive_timeout_ = 500;

  [[noreturn]] void fail(const std::string &text) {
    std::string message = text;
    while (ecx_iserror(&ctx_)) {
      const char *e = ecx_elist2string(&ctx_);
      if (e)
        message += "; " + std::string(e);
    }
    throw std::runtime_error(message);
  }

  void write(uint16_t slave, uint16_t index, uint8_t sub, uint8_t size, uint32_t value) {
    uint8_t bytes[4]{};
    put32(bytes, value);
    if (ecx_SDOwrite(&ctx_, slave, index, sub, FALSE, size, bytes, EC_TIMEOUTRXM) <= 0)
      fail("SDO write failed at " + address(slave, index, sub));
  }

  void set_verify(uint16_t slave, uint16_t index, uint8_t sub, uint8_t size, uint32_t value) {
    write(slave, index, sub, size, value);
    if (read_object(slave, index, sub, size) != value)
      fail("SDO readback mismatch at " + address(slave, index, sub));
  }

  static std::string address(uint16_t slave, uint16_t index, uint8_t sub) {
    std::ostringstream s;
    s << "slave " << slave << " object 0x" << std::hex << index << ":" << unsigned(sub);
    return s.str();
  }

public:
  SoemTransport() {
    ctx_.port = &port_;
    ctx_.slavelist = slaves_.data();
    ctx_.slavecount = &count_;
    ctx_.maxslave = EC_MAXSLAVE;
    ctx_.grouplist = groups_.data();
    ctx_.maxgroup = EC_MAXGROUP;
    ctx_.esibuf = esibuf_.data();
    ctx_.esimap = esimap_.data();
    ctx_.elist = &errors_;
    ctx_.idxstack = &stack_;
    ctx_.ecaterror = &error_;
    ctx_.DCtime = &dc_;
    ctx_.SMcommtype = smcomm_.data();
    ctx_.PDOassign = assigns_.data();
    ctx_.PDOdesc = descs_.data();
    ctx_.eepSM = &sm_;
    ctx_.eepFMMU = &fmmu_;
    ctx_.manualstatechange = 1;
  }

  ~SoemTransport() override {
    close();
  }

  std::vector<Device> discover(const BusConfig &c) override {
    if (open_)
      fail("Bus already open");
    validate_config(c);
    const std::string path = "/tmp/erob-ethercat-" + c.interface + ".lock";
    lock_ = ::open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0666);
    if (lock_ < 0 || flock(lock_, LOCK_EX | LOCK_NB) != 0)
      fail("EtherCAT interface is locked or lock unavailable: " + c.interface);
    // The lock file is never unlinked: deleting it would permit concurrent
    // inode locks.
    if (ecx_init(&ctx_, c.interface.c_str()) <= 0)
      fail("Cannot open EtherCAT interface: " + c.interface);
    open_ = true;
    if (ecx_config_init(&ctx_, FALSE) <= 0)
      fail("No EtherCAT slaves discovered");
    slaves_[0].state = EC_STATE_PRE_OP;
    ecx_writestate(&ctx_, 0);
    if (ecx_statecheck(&ctx_, 0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 4) != EC_STATE_PRE_OP)
      fail("Cannot enter PRE_OP for mailbox discovery");
    // Establish DC entry-port metadata without enabling SYNC0 or any drive.
    ecx_configdc(&ctx_);
    std::vector<PortNode> ports(size_t(count_) + 1);
    for (int i = 1; i <= count_; ++i)
      ports[i] = {slaves_[i].parent, slaves_[i].activeports, slaves_[i].entryport};

    // Physical port paths identify motors independently of discovery order.
    const auto paths = physical_paths(ports);
    std::vector<Device> result;
    for (int i = 1; i <= count_; ++i) {
      Device d;
      d.index = i;
      d.address = slaves_[i].configadr;
      d.identity = {slaves_[i].eep_man, slaves_[i].eep_id, slaves_[i].eep_rev, 0};
      d.name = slaves_[i].name;
      d.has_dc = slaves_[i].hasdc;
      d.path = paths[i];
      d.drive = (d.identity.vendor == 0x5a65726f);
      if (d.drive) {
        d.identity = {read_object(i, 0x1018, 1, 4), read_object(i, 0x1018, 2, 4),
                      read_object(i, 0x1018, 3, 4), read_object(i, 0x1018, 4, 4)};
        d.supported_modes = read_object(i, 0x6502, 0, 4);
        std::array<char, 128> fw{};
        int size = 127;
        if (ecx_SDOread(&ctx_, i, 0x100a, 0, FALSE, &size, fw.data(), EC_TIMEOUTRXM) > 0 &&
            size >= 0 && size <= 127)
          d.firmware.assign(fw.data(), size);
        else
          fail("Cannot read drive firmware 0x100A");
      }
      result.push_back(d);
    }
    return result;
  }

  void configure(const BusConfig &c, const std::vector<uint16_t> &map) override {
    if (!open_)
      fail("Bus is not open");
    // Software may inspect an unknown hub, but must not guess a real PDO/stop
    // profile.
    validate_activation(c);
    map_ = map;
    rx_size_ = rx_wire_size(c.csp_feedforward);
    receive_timeout_ = int(std::min<Nanoseconds>(EC_TIMEOUTRET, c.cycle_ns / 2000));
    configure_drive_objects(
        c, map,
        [&](auto slave, auto index, auto sub, auto size) {
          return read_object(slave, index, sub, size);
        },
        [&](auto slave, auto index, auto sub, auto size, auto value) {
          write(slave, index, sub, size, value);
        });

    // Bound the process image before asking SOEM to map it into the allocated buffer.
    size_t predicted = 0;
    for (int i = 1; i <= count_; ++i)
      for (int sm = 0; sm < EC_MAXSM; ++sm)
        predicted += etohs(slaves_[i].SM[sm].SMlength);
    if (predicted > io_.size())
      fail("Process image exceeds allocated capacity");
    const int bytes = ecx_config_map_group(&ctx_, io_.data(), 0);
    if (bytes <= 0 || size_t(bytes) > io_.size())
      fail("Invalid process image size");
    ecx_configdc(&ctx_);
    for (auto i : map_) {
      verify_process_image(slaves_[i].Obytes, slaves_[i].Ibytes, c.csp_feedforward);
      if (!slaves_[i].outputs || !slaves_[i].inputs)
        fail("Missing process image pointers");
      ecx_dcsync0(&ctx_, i, TRUE, uint32_t(c.cycle_ns), 0);
      const auto period = read_object(i, 0x1c32, 2, 4);
      if (period != uint32_t(c.cycle_ns))
        fail("SYNC0 cycle readback mismatch");
    }
    slaves_[0].state = EC_STATE_SAFE_OP;
    ecx_writestate(&ctx_, 0);
    if (ecx_statecheck(&ctx_, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4) != EC_STATE_SAFE_OP)
      fail("Cannot enter SAFE_OP");
    expected_ = groups_[0].outputsWKC * 2 + groups_[0].inputsWKC;
    for (auto i : map_) {
      auto b = encode(RxPdo{});
      std::memcpy(slaves_[i].outputs, b.data(), rx_size_);
    }
    // Establish cyclic DC exchange with disabled drives before requesting OP.
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    for (int cycle = 0; cycle < 3000; ++cycle) {
      ts.tv_nsec += c.cycle_ns;
      while (ts.tv_nsec >= 1000000000) {
        ++ts.tv_sec;
        ts.tv_nsec -= 1000000000;
      }
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
      ecx_send_processdata(&ctx_);
      ecx_receive_processdata(&ctx_, receive_timeout_);
    }
    slaves_[0].state = EC_STATE_OPERATIONAL;
    ecx_writestate(&ctx_, 0);
    bool op = false;
    for (int retry = 0; retry < 1000 && !op; ++retry) {
      ecx_send_processdata(&ctx_);
      ecx_receive_processdata(&ctx_, receive_timeout_);
      if (retry % 20 == 0) {
        ecx_readstate(&ctx_);
        op = true;
        for (int i = 1; i <= count_; ++i)
          op = op && slaves_[i].state == EC_STATE_OPERATIONAL;
      }
      timespec delay{0, long(c.cycle_ns)};
      nanosleep(&delay, nullptr);
    }
    if (!op)
      fail("Cannot enter OP with disabled drives");
    configured_ = true;
  }

  // Cyclic traffic copies only the configured logical drives into their PDO slots.
  Exchange exchange(const std::vector<RxBytes> &tx, std::vector<TxBytes> &rx) noexcept override {
    if (!configured_ || tx.size() != map_.size() || rx.size() != map_.size())
      return {};
    for (size_t k = 0; k < map_.size(); ++k)
      std::memcpy(slaves_[map_[k]].outputs, tx[k].data(), rx_size_);
    ecx_send_processdata(&ctx_);
    int wkc = ecx_receive_processdata(&ctx_, receive_timeout_);
    if (wkc == expected_)
      for (size_t k = 0; k < map_.size(); ++k)
        std::memcpy(rx[k].data(), slaves_[map_[k]].inputs, rx[k].size());
    return {wkc, expected_, dc_, wkc == expected_, -1};
  }

  uint32_t read_object(uint16_t slave, uint16_t index, uint8_t sub, uint8_t size) override {
    if (!open_ || slave < 1 || slave > count_ || (size != 1 && size != 2 && size != 4))
      fail("Invalid SDO read request");
    std::array<uint8_t, 4> data{};
    int actual = size;
    if (ecx_SDOread(&ctx_, slave, index, sub, FALSE, &actual, data.data(), EC_TIMEOUTRXM) <= 0 ||
        actual != size)
      fail("SDO read/width failed at " + address(slave, index, sub));
    return get32(data.data());
  }

  void write_gain(uint16_t slave, uint16_t index, uint8_t sub, uint16_t value) override {
    if (configured_)
      fail("Tuning is only permitted in a separate offline session");
    if (!((index == 0x2381 && (sub == 1 || sub == 2)) ||
          (index == 0x2382 && (sub == 1 || sub == 3))))
      fail("Gain is not allowlisted");
    if (read_object(slave, 0x6041, 0, 2) & 4)
      fail("Cannot tune an enabled/quick-stopping drive");
    const bool gated = !(index == 0x2382 && sub == 3);
    uint32_t gate = 0;
    if (gated) {
      gate = read_object(slave, 0x2383, 0, 4);
      set_verify(slave, 0x2383, 0, 4, 1);
    }
    try {
      set_verify(slave, index, sub, 2, value);
    } catch (...) {
      if (gated) {
        try {
          set_verify(slave, 0x2383, 0, 4, gate);
        } catch (...) {
        }
      }
      throw;
    }
    if (gated)
      set_verify(slave, 0x2383, 0, 4, gate);
  }

  void close() noexcept override {
    if (open_) {
      slaves_[0].state = EC_STATE_INIT;
      ecx_writestate(&ctx_, 0);
      ecx_close(&ctx_);
      open_ = false;
      configured_ = false;
    }
    if (lock_ >= 0) {
      flock(lock_, LOCK_UN);
      ::close(lock_);
      lock_ = -1;
    }
  }
};
} // namespace

std::unique_ptr<Transport> make_soem_transport() {
  return std::make_unique<SoemTransport>();
}
} // namespace erob
