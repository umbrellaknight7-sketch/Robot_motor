#pragma once

#include "erob_driver/types.hpp"

namespace erob {
BusConfig load_config(const std::string &filename);
void validate_config(const BusConfig &config);
void validate_activation(const BusConfig &config);
std::vector<uint16_t> match_devices(const BusConfig &, const std::vector<Device> &);
std::string path_string(const std::vector<uint8_t> &);
} // namespace erob
