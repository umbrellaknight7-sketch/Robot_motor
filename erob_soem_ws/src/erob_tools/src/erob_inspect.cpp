#include <algorithm>
#include <erob_driver/config.hpp>
#include <erob_driver/transport.hpp>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>

uint32_t parse(const char *text, uint32_t max) {
  size_t end = 0;
  auto n = std::stoull(text, &end, 0);
  if (end != std::string(text).size() || n > max)
    throw std::runtime_error("Invalid numeric argument");
  return uint32_t(n);
}

int main(int argc, char **argv) {
  if (argc < 3) {
    std::cerr << "Usage: erob_inspect CONFIG discover|inspect|read|gain [slave "
                 "index subindex width_or_value]\n"
              << "read: width is 1, 2 or 4 bytes; output is raw unsigned hex.\n"
              << "gain: slave index subindex raw_uint16_value. Disabled drives "
                 "only.\n";
    return 1;
  }
  try {
    auto c = erob::load_config(argv[1]);
    auto transport = erob::make_transport(c);
    auto devices = transport->discover(c);
    std::string action = argv[2];
    if (action == "discover" || action == "inspect") {
      std::cout << "slave address path name vendor product revision serial "
                   "firmware\n";
      for (const auto &d : devices) {
        std::cout << std::dec << d.index << " 0x" << std::hex << d.address << " "
                  << erob::path_string(d.path) << " " << d.name << " 0x" << d.identity.vendor
                  << " 0x" << d.identity.product << " 0x" << d.identity.revision << " 0x"
                  << d.identity.serial << " " << d.firmware << "\n";
        if (action == "inspect" && d.drive) {
          for (auto object : {0x6502, 0x6075, 0x6076, 0x603f, 0x6041, 0x6061}) {
            uint8_t size = object == 0x6061 ? 1 : ((object == 0x603f || object == 0x6041) ? 2 : 4);
            std::cout << "  0x" << std::hex << object << " = 0x"
                      << transport->read_object(d.index, object, 0, size) << "\n";
          }
          for (auto object : {0x1600, 0x1a00}) {
            auto n = transport->read_object(d.index, object, 0, 1);
            std::cout << "  PDO 0x" << std::hex << object << ":";
            for (uint32_t sub = 1; sub <= n; ++sub)
              std::cout << " 0x" << transport->read_object(d.index, object, sub, 4);
            std::cout << "\n";
          }
        }
      }
      // Inspection remains useful before a valid profile exists.
      try {
        auto map = erob::match_devices(c, devices);
        for (size_t i = 0; i < map.size(); ++i) {
          const auto &d = c.drives[i];
          std::cout << d.joint << " logical=0x" << std::hex << d.logical_id
                    << " branch=" << d.branch << ":" << std::dec << d.branch_position
                    << " slave=" << map[i] << "\n";
        }
      } catch (const std::exception &e) {
        std::cerr << "Profile match unavailable: " << e.what() << "\n";
        return 2;
      }
    } else if (action == "read" || action == "gain") {
      if (argc != 7)
        throw std::runtime_error("Expected: slave index subindex width_or_value");
      auto slave = parse(argv[3], 65535), index = parse(argv[4], 65535), sub = parse(argv[5], 255);
      if (action == "gain") {
        // Require the supplied physical mapping/identity to match before any
        // tuning write.
        auto map = erob::match_devices(c, devices);
        if (std::find(map.begin(), map.end(), slave) == map.end())
          throw std::runtime_error("Slave is not a configured motor");
        transport->write_gain(slave, index, sub, parse(argv[6], 65535));
        std::cout << "Readback: " << transport->read_object(slave, index, sub, 2) << "\n";
      } else
        std::cout << "0x" << std::hex
                  << transport->read_object(slave, index, sub, parse(argv[6], 4)) << "\n";
    } else
      throw std::runtime_error("Unknown action");
    transport->close();
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << "\n";
    return 1;
  }
}
