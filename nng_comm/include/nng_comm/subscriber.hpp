#pragma once

#include <nng/nng.h>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>
#include <functional>
#include <atomic>

namespace nng_comm {

class Subscriber {
public:
    Subscriber();
    ~Subscriber();

    // Initialize subscriber, connect to publisher address
    bool init(const std::string& address);

    // Receive JSON message (blocking or non-blocking)
    bool receive(nlohmann::json& msg, int timeout_ms = -1);

    // Set callback function
    void set_callback(std::function<void(const nlohmann::json&)> callback);

    // Start async receive thread
    void start_async();

    // Stop async receive thread
    void stop_async();

    // Close subscriber
    void close();

    // Check if initialized
    bool is_initialized() const { return initialized_; }

private:
    nng_socket socket_;
    std::string address_;
    bool initialized_;

    std::thread recv_thread_;
    std::atomic<bool> running_;
    std::function<void(const nlohmann::json&)> callback_;

    void receive_loop();
};

} // namespace nng_comm
