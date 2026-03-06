#include "utils/config_loader.hpp"
#include "utils/json_utils.hpp"
#include <iostream>
#include <filesystem>

namespace utils {

nlohmann::json ConfigLoader::config_;
bool ConfigLoader::loaded_ = false;

bool ConfigLoader::load(const std::string& config_dir) {
    try {
        std::filesystem::path base_path(config_dir);
        auto config_path = base_path / "config.jsonc";
        config_ = JsonUtils::load_from_file(config_path.string());
        std::cout << "Loaded configuration from " << config_path << std::endl;

        loaded_ = true;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "Failed to load configuration: " << e.what() << std::endl;
        loaded_ = false;
        return false;
    }
}

const nlohmann::json& ConfigLoader::get_config() {
    return config_;
}

nlohmann::json ConfigLoader::get_sim_config() {
    // Return the full config (callers extract what they need)
    return config_;
}

nlohmann::json ConfigLoader::get_network_config() {
    if (config_.contains("network")) {
        return config_["network"];
    }
    return nlohmann::json();
}

std::string ConfigLoader::get_sensor_address(const std::string& sensor_name) {
    if (config_.contains("network") &&
        config_["network"].contains("sensors") &&
        config_["network"]["sensors"].contains(sensor_name)) {
        return config_["network"]["sensors"][sensor_name].get<std::string>();
    }
    return "";
}

std::string ConfigLoader::get_actuator_address(const std::string& actuator_name) {
    if (config_.contains("network") &&
        config_["network"].contains("actuators") &&
        config_["network"]["actuators"].contains(actuator_name)) {
        return config_["network"]["actuators"][actuator_name].get<std::string>();
    }
    return "";
}

std::string ConfigLoader::get_service_address(const std::string& service_name) {
    if (config_.contains("network") &&
        config_["network"].contains("services") &&
        config_["network"]["services"].contains(service_name)) {
        return config_["network"]["services"][service_name].get<std::string>();
    }
    return "";
}

} // namespace utils
