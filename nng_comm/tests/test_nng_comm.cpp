#include "nng_comm/publisher.hpp"
#include "nng_comm/subscriber.hpp"
#include "nng_comm/requester.hpp"
#include "nng_comm/replier.hpp"
#include "nng_comm/message.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <cassert>

using namespace nng_comm;

void test_pubsub() {
    std::cout << "\n=== Testing Pub/Sub ===" << std::endl;

    Publisher pub;
    assert(pub.init("tcp://127.0.0.1:15560"));
    std::cout << "Publisher initialized" << std::endl;

    // Allow time for socket to bind
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    Subscriber sub;
    assert(sub.init("tcp://127.0.0.1:15560"));
    std::cout << "Subscriber initialized" << std::endl;

    // Allow time for connection
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Publish message
    nlohmann::json test_msg = {
        {"type", "test"},
        {"value", 42},
        {"text", "Hello NNG"}
    };
    Message::add_timestamp(test_msg);

    assert(pub.publish(test_msg));
    std::cout << "Published: " << test_msg.dump() << std::endl;

    // Receive message
    nlohmann::json received_msg;
    assert(sub.receive(received_msg, 1000));
    std::cout << "Received: " << received_msg.dump() << std::endl;

    // Verify content
    assert(received_msg["type"] == "test");
    assert(received_msg["value"] == 42);
    assert(received_msg["text"] == "Hello NNG");

    std::cout << "Pub/Sub test PASSED" << std::endl;
}

void test_reqrep() {
    std::cout << "\n=== Testing Req/Rep ===" << std::endl;

    // Start replier
    Replier rep;
    assert(rep.init("tcp://127.0.0.1:15561"));

    rep.set_handler([](const nlohmann::json& req) -> nlohmann::json {
        std::cout << "Replier received: " << req.dump() << std::endl;

        if (req["command"] == "add") {
            int result = req["a"].get<int>() + req["b"].get<int>();
            return {{"status", "ok"}, {"result", result}};
        }

        return {{"status", "error"}, {"message", "unknown command"}};
    });

    rep.start();
    std::cout << "Replier started" << std::endl;

    // Allow time for server to start
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Create requester
    Requester req;
    assert(req.init("tcp://127.0.0.1:15561"));
    std::cout << "Requester initialized" << std::endl;

    // Send request
    nlohmann::json request = {
        {"command", "add"},
        {"a", 10},
        {"b", 32}
    };

    nlohmann::json reply;
    assert(req.request(request, reply, 5000));
    std::cout << "Reply: " << reply.dump() << std::endl;

    // Verify reply
    assert(reply["status"] == "ok");
    assert(reply["result"] == 42);

    rep.stop();
    std::cout << "Req/Rep test PASSED" << std::endl;
}

void test_async_subscriber() {
    std::cout << "\n=== Testing Async Subscriber ===" << std::endl;

    Publisher pub;
    assert(pub.init("tcp://127.0.0.1:15562"));

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    Subscriber sub;
    assert(sub.init("tcp://127.0.0.1:15562"));

    int message_count = 0;
    sub.set_callback([&message_count](const nlohmann::json& msg) {
        std::cout << "Async received: " << msg.dump() << std::endl;
        message_count++;
    });

    sub.start_async();
    std::cout << "Async subscriber started" << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Publish multiple messages
    for (int i = 0; i < 5; i++) {
        nlohmann::json msg = {{"id", i}, {"data", "message_" + std::to_string(i)}};
        pub.publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Wait for messages to be processed
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    sub.stop_async();

    assert(message_count == 5);
    std::cout << "Async Subscriber test PASSED" << std::endl;
}

void test_pubsub_subscriber_first() {
    std::cout << "\n=== Testing Pub/Sub (Subscriber First) ===" << std::endl;

    // Create subscriber FIRST — dials with auto-reconnect
    Subscriber sub;
    assert(sub.init("tcp://127.0.0.1:15563"));
    std::cout << "Subscriber initialized (before publisher)" << std::endl;

    // Wait a bit, then create publisher
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    Publisher pub;
    assert(pub.init("tcp://127.0.0.1:15563"));
    std::cout << "Publisher initialized (after subscriber)" << std::endl;

    // Allow time for reconnection
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // Publish messages in a loop — early messages may be lost during
    // subscription propagation, so keep publishing until one is received
    nlohmann::json test_msg = {
        {"type", "sub_first_test"},
        {"value", 99}
    };

    nlohmann::json received_msg;
    bool received = false;
    for (int attempt = 0; attempt < 20 && !received; attempt++) {
        pub.publish(test_msg);
        received = sub.receive(received_msg, 200);
    }

    assert(received);
    std::cout << "Received: " << received_msg.dump() << std::endl;

    // Verify content
    assert(received_msg["type"] == "sub_first_test");
    assert(received_msg["value"] == 99);

    std::cout << "Pub/Sub (Subscriber First) test PASSED" << std::endl;
}

void test_reqrep_client_first() {
    std::cout << "\n=== Testing Req/Rep (Client First) ===" << std::endl;

    // Create requester FIRST — dials with auto-reconnect
    Requester req;
    assert(req.init("tcp://127.0.0.1:15564"));
    std::cout << "Requester initialized (before server)" << std::endl;

    // Wait a bit, then create replier
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    Replier rep;
    assert(rep.init("tcp://127.0.0.1:15564"));

    rep.set_handler([](const nlohmann::json& request) -> nlohmann::json {
        std::cout << "Replier received: " << request.dump() << std::endl;
        if (request["command"] == "multiply") {
            int result = request["a"].get<int>() * request["b"].get<int>();
            return {{"status", "ok"}, {"result", result}};
        }
        return {{"status", "error"}, {"message", "unknown command"}};
    });

    rep.start();
    std::cout << "Replier started (after requester)" << std::endl;

    // Allow time for reconnection
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // Send request
    nlohmann::json request = {
        {"command", "multiply"},
        {"a", 6},
        {"b", 7}
    };

    nlohmann::json reply;
    assert(req.request(request, reply, 5000));
    std::cout << "Reply: " << reply.dump() << std::endl;

    // Verify reply
    assert(reply["status"] == "ok");
    assert(reply["result"] == 42);

    rep.stop();
    std::cout << "Req/Rep (Client First) test PASSED" << std::endl;
}

int main() {
    std::cout << "=== NNG Communication Tests ===" << std::endl;

    try {
        test_pubsub();
        test_reqrep();
        test_async_subscriber();
        test_pubsub_subscriber_first();
        test_reqrep_client_first();

        std::cout << "\n=== All tests PASSED ===" << std::endl;
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
}
