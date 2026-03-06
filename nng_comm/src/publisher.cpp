#include "nng_comm/publisher.hpp"
#include "nng_comm/message.hpp"
#include <iostream>

namespace nng_comm {

Publisher::Publisher() : initialized_(false) {
    socket_.id = 0;
}

Publisher::~Publisher() {
    close();
}

bool Publisher::init(const std::string& address) {
    int rv;

    // Create publisher socket
    if ((rv = nng_pub0_open(&socket_)) != 0) {
        std::cerr << "nng_pub0_open: " << nng_strerror(static_cast<nng_err>(rv)) << std::endl;
        return false;
    }

    // Bind to address
    if ((rv = nng_listen(socket_, address.c_str(), nullptr, 0)) != 0) {
        std::cerr << "nng_listen: " << nng_strerror(static_cast<nng_err>(rv)) << std::endl;
        nng_socket_close(socket_);
        return false;
    }

    address_ = address;
    initialized_ = true;
    return true;
}

bool Publisher::publish(const nlohmann::json& msg) {
    if (!initialized_) {
        // Only log once per publisher instance to avoid spamming
        if (!error_logged_) {
            std::cerr << "Publisher not initialized for " << address_ << std::endl;
            error_logged_ = true;
        }
        return false;
    }

    // Serialize JSON to bytes
    std::vector<uint8_t> data = Message::serialize(msg);

    // Send message
    int rv = nng_send(socket_, data.data(), data.size(), 0);
    if (rv != 0) {
        std::cerr << "nng_send: " << nng_strerror(static_cast<nng_err>(rv)) << std::endl;
        return false;
    }

    return true;
}

void Publisher::close() {
    if (initialized_) {
        nng_socket_close(socket_);
        initialized_ = false;
    }
}

} // namespace nng_comm
