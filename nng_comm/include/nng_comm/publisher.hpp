#pragma once

#include <nng/nng.h>
#include <nlohmann/json.hpp>
#include <string>

namespace nng_comm {

class Publisher {
public:
    Publisher();
    ~Publisher();

    // Initialize publisher, bind to specified address
    bool init(const std::string& address);

    // Publish JSON message
    bool publish(const nlohmann::json& msg);

    // Close publisher
    void close();

    // Check if initialized
    bool is_initialized() const { return initialized_; }

private:
    nng_socket socket_;
    std::string address_;
    bool initialized_;
    bool error_logged_ = false;  // Avoid spamming "not initialized" errors
};

} // namespace nng_comm
