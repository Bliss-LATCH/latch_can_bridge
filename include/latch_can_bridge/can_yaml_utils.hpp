#pragma once

#include <string>
#include <unordered_map>
#include <vector>

struct IdMapping {
  std::string msg_name;
  int can_id;
  std::string topic;
};

struct Device {
  std::string name;
  std::vector<IdMapping> id_mappings;
};

// get device static can_id map
std::unordered_map<std::string, IdMapping> getDeviceIDMap(
    const std::string& device_name);