#pragma once

#include <nlohmann/json.hpp>
#include <string>

namespace utils {

class JsonUtils {
public:
    // Load JSON from file
    static nlohmann::json load_from_file(const std::string& filepath);

    // Save JSON to file
    static bool save_to_file(const std::string& filepath, const nlohmann::json& json);

    // Get value with default
    template<typename T>
    static T get_or_default(const nlohmann::json& json, const std::string& key, const T& default_value) {
        if (json.contains(key)) {
            return json[key].get<T>();
        }
        return default_value;
    }

    // Get nested value with default
    static nlohmann::json get_nested(const nlohmann::json& json,
                                     const std::vector<std::string>& path,
                                     const nlohmann::json& default_value = nlohmann::json());
};

} // namespace utils
