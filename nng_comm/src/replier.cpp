#include "nng_comm/replier.hpp"
#include "nng_comm/message.hpp"
#include <iostream>

namespace nng_comm {

Replier::Replier() : initialized_(false), running_(false) {
    socket_.id = 0;
}

Replier::~Replier() {
    stop();
    close();
}

bool Replier::init(const std::string& bind_address) {
    int rv;

    // Create replier socket
    if ((rv = nng_rep0_open(&socket_)) != 0) {
        std::cerr << "nng_rep0_open: " << nng_strerror(static_cast<nng_err>(rv)) << std::endl;
        return false;
    }

    // Bind to address
    if ((rv = nng_listen(socket_, bind_address.c_str(), nullptr, 0)) != 0) {
        std::cerr << "nng_listen: " << nng_strerror(static_cast<nng_err>(rv)) << std::endl;
        nng_socket_close(socket_);
        return false;
    }

    address_ = bind_address;
    initialized_ = true;
    return true;
}

void Replier::set_handler(std::function<nlohmann::json(const nlohmann::json&)> handler) {
    handler_ = handler;
}

void Replier::start() {
    if (!initialized_ || !handler_) {
        std::cerr << "Cannot start: not initialized or no handler set" << std::endl;
        return;
    }

    running_ = true;
    service_thread_ = std::thread(&Replier::service_loop, this);
}

void Replier::stop() {
    if (running_) {
        running_ = false;
        if (service_thread_.joinable()) {
            service_thread_.join();
        }
    }
}

void Replier::service_loop() {
    while (running_) {
        // Receive request using message API
        nng_msg* msgp = nullptr;

        // Set receive timeout to allow checking running_ flag
        nng_socket_set_ms(socket_, NNG_OPT_RECVTIMEO, 100);

        int rv = nng_recvmsg(socket_, &msgp, 0);
        if (rv != 0) {
            if (rv != NNG_ETIMEDOUT) {
                std::cerr << "nng_recvmsg: " << nng_strerror(static_cast<nng_err>(rv)) << std::endl;
            }
            continue;
        }

        // Deserialize request
        nlohmann::json req;
        try {
            void* buf = nng_msg_body(msgp);
            size_t size = nng_msg_len(msgp);
            req = Message::deserialize(static_cast<uint8_t*>(buf), size);
            nng_msg_free(msgp);
        } catch (const std::exception& e) {
            std::cerr << "Deserialize error: " << e.what() << std::endl;
            nng_msg_free(msgp);
            continue;
        }

        // Process request with handler
        nlohmann::json rep;
        if (handler_) {
            rep = handler_(req);
        } else {
            rep = {{"status", "error"}, {"message", "no handler"}};
        }

        // Serialize reply
        std::vector<uint8_t> rep_data = Message::serialize(rep);

        // Send reply
        rv = nng_send(socket_, rep_data.data(), rep_data.size(), 0);
        if (rv != 0) {
            std::cerr << "nng_send: " << nng_strerror(static_cast<nng_err>(rv)) << std::endl;
        }
    }
}

void Replier::close() {
    if (initialized_) {
        nng_socket_close(socket_);
        initialized_ = false;
    }
}

} // namespace nng_comm
