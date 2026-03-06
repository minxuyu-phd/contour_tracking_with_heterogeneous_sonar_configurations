#include "nng_comm/requester.hpp"
#include "nng_comm/message.hpp"
#include <iostream>

namespace nng_comm {

Requester::Requester() : initialized_(false) {
    socket_.id = 0;
}

Requester::~Requester() {
    close();
}

bool Requester::init(const std::string& server_address) {
    int rv;

    // Create requester socket
    if ((rv = nng_req0_open(&socket_)) != 0) {
        std::cerr << "nng_req0_open: " << nng_strerror(static_cast<nng_err>(rv)) << std::endl;
        return false;
    }

    // Dial to server with NNG_FLAG_NONBLOCK for auto-reconnect
    // This allows the requester to be created before the server is up
    if ((rv = nng_dial(socket_, server_address.c_str(), nullptr, NNG_FLAG_NONBLOCK)) != 0) {
        std::cerr << "nng_dial: " << nng_strerror(static_cast<nng_err>(rv)) << std::endl;
        nng_socket_close(socket_);
        return false;
    }

    address_ = server_address;
    initialized_ = true;
    return true;
}

bool Requester::request(const nlohmann::json& req, nlohmann::json& rep, int timeout_ms) {
    if (!initialized_) {
        std::cerr << "Requester not initialized" << std::endl;
        return false;
    }

    // Serialize request
    std::vector<uint8_t> req_data = Message::serialize(req);

    // Set send/receive timeout
    nng_duration timeout = timeout_ms;
    nng_socket_set_ms(socket_, NNG_OPT_SENDTIMEO, timeout);
    nng_socket_set_ms(socket_, NNG_OPT_RECVTIMEO, timeout);

    // Send request
    int rv = nng_send(socket_, req_data.data(), req_data.size(), 0);
    if (rv != 0) {
        std::cerr << "nng_send: " << nng_strerror(static_cast<nng_err>(rv)) << std::endl;
        return false;
    }

    // Receive reply using message API
    nng_msg* msgp = nullptr;
    rv = nng_recvmsg(socket_, &msgp, 0);
    if (rv != 0) {
        std::cerr << "nng_recvmsg: " << nng_strerror(static_cast<nng_err>(rv)) << std::endl;
        return false;
    }

    // Deserialize reply
    try {
        void* buf = nng_msg_body(msgp);
        size_t size = nng_msg_len(msgp);
        rep = Message::deserialize(static_cast<uint8_t*>(buf), size);
        nng_msg_free(msgp);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Deserialize error: " << e.what() << std::endl;
        nng_msg_free(msgp);
        return false;
    }
}

void Requester::close() {
    if (initialized_) {
        nng_socket_close(socket_);
        initialized_ = false;
    }
}

} // namespace nng_comm
