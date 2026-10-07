#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace erob {
struct PortNode {
  uint16_t parent;
  uint8_t active_ports, entry_port;
};

// Nodes use EtherCAT discovery indexes; element zero represents the master.
// Walk immediate physical parents, including non-DC infrastructure. SOEM's
// parentport is a DC-parent timing field and cannot describe this physical
// tree.
inline std::vector<std::vector<uint8_t>> physical_paths(const std::vector<PortNode> &nodes) {
  std::vector<std::vector<uint8_t>> paths(nodes.size());
  std::vector<uint8_t> consumed(nodes.size(), 0);
  constexpr std::array<uint8_t, 4> traversal{0, 3, 1, 2};
  for (size_t i = 1; i < nodes.size(); ++i) {
    const auto &node = nodes[i];
    if (node.parent >= i || node.entry_port > 3 || !(node.active_ports & (1u << node.entry_port)))
      throw std::runtime_error("Invalid or unknown physical topology entry port");
    consumed[i] = uint8_t(1u << node.entry_port);
    if (!node.parent) {
      if (i != 1)
        throw std::runtime_error("Multiple master roots in physical topology");
      continue;
    }
    const auto &parent = nodes[node.parent];
    auto entry =
        std::find(traversal.begin(), traversal.end(), parent.entry_port) - traversal.begin();
    bool found = false;
    for (size_t offset = 1; offset < 4; ++offset) {
      uint8_t port = traversal[(entry + offset) % 4];
      uint8_t bit = uint8_t(1u << port);
      if ((parent.active_ports & bit) && !(consumed[node.parent] & bit)) {
        consumed[node.parent] |= bit;
        paths[i] = paths[node.parent];
        paths[i].push_back(port);
        found = true;
        break;
      }
    }
    if (!found)
      throw std::runtime_error("Physical parent has no available port");
  }
  return paths;
}
} // namespace erob
