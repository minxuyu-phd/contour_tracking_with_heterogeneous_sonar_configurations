#pragma once

#include <nlohmann/json.hpp>
#include <vector>
#include <cstdint>

namespace nng_comm {

class Message {
public:
    // JSON to byte stream serialization
    static std::vector<uint8_t> serialize(const nlohmann::json& json);

    // Byte stream to JSON deserialization
    static nlohmann::json deserialize(const uint8_t* data, size_t size);

    // Add timestamp to message
    static void add_timestamp(nlohmann::json& msg);

    // Get current timestamp in seconds
    static double get_timestamp();
};

} // namespace nng_comm
