#include "utils/json_utils.hpp"
#include <fstream>
#include <iostream>
#include <sstream>

namespace utils {

nlohmann::json JsonUtils::load_from_file(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file: " + filepath);
    }

    nlohmann::json json;
    try {
        std::ostringstream ss;
        ss << file.rdbuf();
        json = nlohmann::json::parse(ss.str(), nullptr, true, true);
    } catch (const nlohmann::json::exception& e) {
        throw std::runtime_error("Failed to parse JSON from " + filepath + ": " + e.what());
    }

    return json;
}

bool JsonUtils::save_to_file(const std::string& filepath, const nlohmann::json& json) {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Cannot open file for writing: " << filepath << std::endl;
        return false;
    }

    try {
        file << json.dump(4);  // Pretty print with 4 spaces
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to write JSON: " << e.what() << std::endl;
        return false;
    }
}

nlohmann::json JsonUtils::get_nested(const nlohmann::json& json,
                                      const std::vector<std::string>& path,
                                      const nlohmann::json& default_value) {
    nlohmann::json current = json;
    for (const auto& key : path) {
        if (current.contains(key)) {
            current = current[key];
        } else {
            return default_value;
        }
    }
    return current;
}

} // namespace utils
