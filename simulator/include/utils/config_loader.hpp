#pragma once

#include <nlohmann/json.hpp>
#include <string>

namespace utils {

class ConfigLoader {
public:
    // Load unified config.jsonc from directory
    static bool load(const std::string& config_dir);

    // Get full configuration
    static const nlohmann::json& get_config();

    // Get simulation-related config (everything except network)
    static nlohmann::json get_sim_config();

    // Get network configuration section
    static nlohmann::json get_network_config();

    // Get specific sensor address
    static std::string get_sensor_address(const std::string& sensor_name);

    // Get specific actuator address
    static std::string get_actuator_address(const std::string& actuator_name);

    // Get service address
    static std::string get_service_address(const std::string& service_name);

    // Check if loaded
    static bool is_loaded() { return loaded_; }

private:
    static nlohmann::json config_;
    static bool loaded_;
};

} // namespace utils
