#include "nng_comm/subscriber.hpp"
#include "nng_comm/message.hpp"
#include <iostream>

namespace nng_comm {

Subscriber::Subscriber() : initialized_(false), running_(false) {
    socket_.id = 0;
}

Subscriber::~Subscriber() {
    stop_async();
    close();
}

bool Subscriber::init(const std::string& address) {
    int rv;

    // Create subscriber socket
    if ((rv = nng_sub0_open(&socket_)) != 0) {
        std::cerr << "nng_sub0_open: " << nng_strerror(static_cast<nng_err>(rv)) << std::endl;
        return false;
    }

    // Subscribe to all topics (empty string)
    if ((rv = nng_sub0_socket_subscribe(socket_, "", 0)) != 0) {
        std::cerr << "nng_sub0_socket_subscribe: " << nng_strerror(static_cast<nng_err>(rv)) << std::endl;
        nng_socket_close(socket_);
        return false;
    }

    // Dial to publisher with NNG_FLAG_NONBLOCK for auto-reconnect
    // This allows the subscriber to be created before the publisher is up
    if ((rv = nng_dial(socket_, address.c_str(), nullptr, NNG_FLAG_NONBLOCK)) != 0) {
        std::cerr << "nng_dial: " << nng_strerror(static_cast<nng_err>(rv)) << std::endl;
        nng_socket_close(socket_);
        return false;
    }

    address_ = address;
    initialized_ = true;
    return true;
}

bool Subscriber::receive(nlohmann::json& msg, int timeout_ms) {
    if (!initialized_) {
        std::cerr << "Subscriber not initialized" << std::endl;
        return false;
    }

    // Set receive timeout
    if (timeout_ms >= 0) {
        nng_duration timeout = timeout_ms;
        nng_socket_set_ms(socket_, NNG_OPT_RECVTIMEO, timeout);
    }

    // Receive message using message API
    nng_msg* msgp = nullptr;
    int rv = nng_recvmsg(socket_, &msgp, 0);

    if (rv != 0) {
        if (rv != NNG_ETIMEDOUT) {
            std::cerr << "nng_recvmsg: " << nng_strerror(static_cast<nng_err>(rv)) << std::endl;
        }
        return false;
    }

    // Deserialize
    try {
        void* buf = nng_msg_body(msgp);
        size_t size = nng_msg_len(msgp);
        msg = Message::deserialize(static_cast<uint8_t*>(buf), size);
        nng_msg_free(msgp);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Deserialize error: " << e.what() << std::endl;
        nng_msg_free(msgp);
        return false;
    }
}

void Subscriber::set_callback(std::function<void(const nlohmann::json&)> callback) {
    callback_ = callback;
}

void Subscriber::start_async() {
    if (!initialized_ || !callback_) {
        std::cerr << "Cannot start async: not initialized or no callback set" << std::endl;
        return;
    }

    running_ = true;
    recv_thread_ = std::thread(&Subscriber::receive_loop, this);
}

void Subscriber::stop_async() {
    if (running_) {
        running_ = false;
        if (recv_thread_.joinable()) {
            recv_thread_.join();
        }
    }
}

void Subscriber::receive_loop() {
    while (running_) {
        nlohmann::json msg;
        if (receive(msg, 100)) {  // 100ms timeout
            if (callback_) {
                callback_(msg);
            }
        }
    }
}

void Subscriber::close() {
    if (initialized_) {
        nng_socket_close(socket_);
        initialized_ = false;
    }
}

} // namespace nng_comm
