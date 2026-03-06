#pragma once

#include <nng/nng.h>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>
#include <functional>
#include <atomic>

namespace nng_comm {

class Replier {
public:
    Replier();
    ~Replier();

    // Initialize replier, bind to address
    bool init(const std::string& bind_address);

    // Set request handler callback
    void set_handler(std::function<nlohmann::json(const nlohmann::json&)> handler);

    // Start reply service thread
    void start();

    // Stop reply service thread
    void stop();

    // Close replier
    void close();

    // Check if initialized
    bool is_initialized() const { return initialized_; }

private:
    nng_socket socket_;
    std::string address_;
    bool initialized_;

    std::thread service_thread_;
    std::atomic<bool> running_;
    std::function<nlohmann::json(const nlohmann::json&)> handler_;

    void service_loop();
};

} // namespace nng_comm
