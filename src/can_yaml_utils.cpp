#include "latch_can_bridge/can_yaml_utils.hpp"

#include <stdexcept>
#include <string>
#include <unordered_map>

#include "yaml-cpp/yaml.h"

std::unordered_map<std::string, IdMapping> getDeviceIDMap(
    const std::string& device_name) {
  static const auto device_map = []() {
    std::unordered_map<std::string, std::unordered_map<std::string, IdMapping>>
        map;

    try {
      YAML::Node config = YAML::LoadFile("config/can_mappings.yaml");

      if (config["can_bridge"] && config["can_bridge"]["devices"]) {
        for (const auto& device : config["can_bridge"]["devices"]) {
          std::string dev_name = device["name"].as<std::string>();
          std::unordered_map<std::string, IdMapping> msg_map;

          if (device["id_mappings"]) {
            for (const auto& mapping : device["id_mappings"]) {
              std::string msg_name = mapping["msg_name"].as<std::string>();

              IdMapping id_map;
              id_map.msg_name =
                  msg_name;  // Storing the name inside the struct too
              id_map.can_id = mapping["can_id"].as<uint32_t>();
              id_map.topic = mapping["topic"].as<std::string>();

              // Keying the map by msg_name so you can index it later
              msg_map[msg_name] = id_map;
            }
          }
          map[dev_name] = msg_map;
        }
      }
    } catch (const std::exception& e) {
      throw std::runtime_error("Failed to load CAN mappings: " +
                               std::string(e.what()));
    }

    return map;
  }();

  auto it = device_map.find(device_name);
  if (it != device_map.end()) {
    return it->second;
  }

  throw std::runtime_error("Device not found: " + device_name);
}