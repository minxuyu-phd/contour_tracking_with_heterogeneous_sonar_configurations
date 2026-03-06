#pragma once

#include <nng/nng.h>
#include <nlohmann/json.hpp>
#include <string>

namespace nng_comm {

class Requester {
public:
    Requester();
    ~Requester();

    // Initialize requester, connect to server address
    bool init(const std::string& server_address);

    // Send request and wait for reply
    bool request(const nlohmann::json& req, nlohmann::json& rep, int timeout_ms = 5000);

    // Close requester
    void close();

    // Check if initialized
    bool is_initialized() const { return initialized_; }

private:
    nng_socket socket_;
    std::string address_;
    bool initialized_;
};

} // namespace nng_comm
