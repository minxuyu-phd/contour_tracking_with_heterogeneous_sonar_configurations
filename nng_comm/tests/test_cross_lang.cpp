#include "nng_comm/nng_comm.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <cassert>
#include <cstring>

using namespace nng_comm;

// C++ 发布，等待对端订阅后发送消息
void run_publisher(const std::string& address) {
    Publisher pub;
    assert(pub.init(address));
    std::cout << "[C++ Publisher] listening on " << address << std::endl;

    // 等待订阅端连接
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    for (int i = 0; i < 20; i++) {
        nlohmann::json msg = {
            {"source", "cpp"},
            {"type", "cross_lang_test"},
            {"seq", i},
            {"value", 42}
        };
        Message::add_timestamp(msg);
        pub.publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 等待对端处理完毕
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    pub.close();
    std::cout << "[C++ Publisher] done" << std::endl;
}

// C++ 订阅，接收消息并验证
void run_subscriber(const std::string& address) {
    Subscriber sub;
    assert(sub.init(address));
    std::cout << "[C++ Subscriber] dialing " << address << std::endl;

    nlohmann::json msg;
    bool received = false;
    for (int attempt = 0; attempt < 30 && !received; attempt++) {
        received = sub.receive(msg, 500);
    }

    assert(received);
    assert(msg["source"] == "python");
    assert(msg["type"] == "cross_lang_test");
    std::cout << "[C++ Subscriber] received: " << msg.dump() << std::endl;

    sub.close();
    std::cout << "[C++ Subscriber] PASSED" << std::endl;
}

// C++ 提供服务（Replier）
void run_replier(const std::string& address) {
    Replier rep;
    assert(rep.init(address));

    rep.set_handler([](const nlohmann::json& req) -> nlohmann::json {
        std::cout << "[C++ Replier] received: " << req.dump() << std::endl;
        if (req["command"] == "add") {
            int result = req["a"].get<int>() + req["b"].get<int>();
            return {{"status", "ok"}, {"result", result}, {"server", "cpp"}};
        }
        return {{"status", "error"}, {"message", "unknown command"}};
    });

    rep.start();
    std::cout << "[C++ Replier] listening on " << address << std::endl;

    // 运行一段时间等待请求
    std::this_thread::sleep_for(std::chrono::seconds(5));

    rep.stop();
    rep.close();
    std::cout << "[C++ Replier] done" << std::endl;
}

// C++ 请求服务（Requester）
void run_requester(const std::string& address) {
    // 等待服务端启动
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    Requester req;
    assert(req.init(address));
    std::cout << "[C++ Requester] dialing " << address << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    nlohmann::json request = {
        {"command", "add"},
        {"a", 100},
        {"b", 200}
    };

    nlohmann::json reply;
    assert(req.request(request, reply, 5000));
    std::cout << "[C++ Requester] reply: " << reply.dump() << std::endl;

    assert(reply["status"] == "ok");
    assert(reply["result"] == 300);
    assert(reply["server"] == "python");

    req.close();
    std::cout << "[C++ Requester] PASSED" << std::endl;
}

void print_usage(const char* prog) {
    std::cerr << "Usage: " << prog << " <mode> <address>" << std::endl;
    std::cerr << "  mode: pub | sub | rep | req" << std::endl;
    std::cerr << "  address: e.g. tcp://127.0.0.1:15580" << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc != 3) {
        print_usage(argv[0]);
        return 1;
    }

    std::string mode = argv[1];
    std::string address = argv[2];

    try {
        if (mode == "pub") {
            run_publisher(address);
        } else if (mode == "sub") {
            run_subscriber(address);
        } else if (mode == "rep") {
            run_replier(address);
        } else if (mode == "req") {
            run_requester(address);
        } else {
            print_usage(argv[0]);
            return 1;
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "FAILED: " << e.what() << std::endl;
        return 1;
    }
}
