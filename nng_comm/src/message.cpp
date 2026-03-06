#include "nng_comm/message.hpp"
#include <nng/nng.h>
#include <chrono>
#include <stdexcept>

namespace nng_comm {

// Static initializer to ensure nng_init is called once
static class NNGInitializer {
public:
    NNGInitializer() {
        nng_init(NULL);
    }
    ~NNGInitializer() {
        nng_fini();
    }
} nng_initializer;

std::vector<uint8_t> Message::serialize(const nlohmann::json& json) {
    std::string json_str = json.dump();
    return std::vector<uint8_t>(json_str.begin(), json_str.end());
}

nlohmann::json Message::deserialize(const uint8_t* data, size_t size) {
    try {
        std::string json_str(reinterpret_cast<const char*>(data), size);
        return nlohmann::json::parse(json_str);
    } catch (const nlohmann::json::exception& e) {
        throw std::runtime_error("Failed to deserialize JSON: " + std::string(e.what()));
    }
}

void Message::add_timestamp(nlohmann::json& msg) {
    msg["timestamp"] = get_timestamp();
}

double Message::get_timestamp() {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double>(duration).count();
}

} // namespace nng_comm
