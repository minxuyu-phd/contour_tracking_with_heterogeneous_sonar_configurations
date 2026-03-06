# nng_comm — 基于 NNG 的 JSON 通信模块

## 概述

`nng_comm` 是对 [nng](https://github.com/nanomsg/nng) 库的封装，提供两种通信模式：

- **Pub/Sub**（发布/订阅）— 一对多消息广播
- **Req/Rep**（请求/应答）— 一对一 RPC 调用

同时支持 **C++** 和 **Python (pynng)** 两套 API，所有消息统一使用 **JSON** 序列化。

## 目录结构

```
nng_comm/
├── CMakeLists.txt
├── include/nng_comm/          # C++ 头文件
│   ├── nng_comm.hpp           # 统一入口 (include 此文件即可)
│   ├── message.hpp
│   ├── publisher.hpp
│   ├── subscriber.hpp
│   ├── requester.hpp
│   └── replier.hpp
├── src/                       # C++ 实现
│   ├── message.cpp
│   ├── publisher.cpp
│   ├── subscriber.cpp
│   ├── requester.cpp
│   └── replier.cpp
├── scripts/                   # Python 实现
│   ├── __init__.py
│   ├── message.py
│   ├── publisher.py
│   ├── subscriber.py
│   ├── requester.py
│   └── replier.py
└── tests/
    ├── test_nng_comm.cpp      # C++ 测试
    ├── test_nng_comm.py       # Python 测试
    ├── test_cross_lang.cpp    # 跨语言测试（C++ 端）
    ├── test_cross_lang.py     # 跨语言测试（Python 端）
    └── test_cross_lang.sh     # 跨语言测试运行脚本
```

## 依赖安装

### 1. nng C 库（v2.0.0-alpha.7）

```bash
sudo apt-get update && sudo apt-get install -y cmake build-essential

git clone --branch v2.0.0-alpha.7 https://github.com/nanomsg/nng.git /tmp/nng
cd /tmp/nng
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_SHARED_LIBS=ON ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

### 2. nlohmann/json（v3.11.3）

```bash
git clone --branch v3.11.3 https://github.com/nlohmann/json.git /tmp/nlohmann_json
cd /tmp/nlohmann_json
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DJSON_BuildTests=OFF ..
make -j$(nproc)
sudo make install
```

### 3. pynng（Python，0.9.0）

> 安装前请确认已激活正确的 conda / venv 环境。

```bash
pip install pynng==0.9.0
```

## C++ 编译

```bash
cd nng_comm
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

编译产物：

- `libnng_comm.so` — 共享库
- `test_nng_comm` — 单语言测试
- `test_cross_lang` — 跨语言测试（C++ 端）

运行测试：

```bash
./test_nng_comm
```

## API 参考

### 通信模式概览

| 模式 | 角色 | 行为 | 地址 |
|------|------|------|------|
| Pub/Sub | Publisher | `listen(address)` | `tcp://IP:PORT` |
| Pub/Sub | Subscriber | `dial(address)` | 同上 |
| Req/Rep | Replier（服务端） | `listen(address)` | `tcp://IP:PORT` |
| Req/Rep | Requester（客户端） | `dial(address)` | 同上 |

> Publisher 和 Replier 负责 `listen`；Subscriber 和 Requester 负责 `dial`。
> 启动顺序不限——NNG 内置自动重连机制。

---

### C++ API

头文件入口：`#include "nng_comm/nng_comm.hpp"`

命名空间：`nng_comm`

#### Message

```cpp
class Message {
public:
    static std::vector<uint8_t> serialize(const nlohmann::json& json);
    static nlohmann::json deserialize(const uint8_t* data, size_t size);
    static void add_timestamp(nlohmann::json& msg);
    static double get_timestamp();
};
```

#### Publisher

```cpp
class Publisher {
public:
    bool init(const std::string& address);
    bool publish(const nlohmann::json& msg);
    void close();
    bool is_initialized() const;
};
```

#### Subscriber

```cpp
class Subscriber {
public:
    bool init(const std::string& address);
    bool receive(nlohmann::json& msg, int timeout_ms = -1);
    void set_callback(std::function<void(const nlohmann::json&)> callback);
    void start_async();
    void stop_async();
    void close();
    bool is_initialized() const;
};
```

#### Requester

```cpp
class Requester {
public:
    bool init(const std::string& server_address);
    bool request(const nlohmann::json& req, nlohmann::json& rep, int timeout_ms = 5000);
    void close();
    bool is_initialized() const;
};
```

#### Replier

```cpp
class Replier {
public:
    bool init(const std::string& bind_address);
    void set_handler(std::function<nlohmann::json(const nlohmann::json&)> handler);
    void start();
    void stop();
    void close();
    bool is_initialized() const;
};
```

#### C++ 用法示例

**Pub/Sub：**

```cpp
#include "nng_comm/nng_comm.hpp"
using namespace nng_comm;

// 发布端
Publisher pub;
pub.init("tcp://127.0.0.1:15560");

nlohmann::json msg = {{"type", "test"}, {"value", 42}};
Message::add_timestamp(msg);
pub.publish(msg);

// 订阅端
Subscriber sub;
sub.init("tcp://127.0.0.1:15560");

nlohmann::json received;
sub.receive(received, 1000);  // 超时 1000ms
```

**Pub/Sub（异步回调）：**

```cpp
Subscriber sub;
sub.init("tcp://127.0.0.1:15562");

sub.set_callback([](const nlohmann::json& msg) {
    std::cout << "Async received: " << msg.dump() << std::endl;
});
sub.start_async();

// ... 处理其他逻辑 ...

sub.stop_async();
```

**Req/Rep：**

```cpp
// 服务端
Replier rep;
rep.init("tcp://127.0.0.1:15561");
rep.set_handler([](const nlohmann::json& req) -> nlohmann::json {
    if (req["command"] == "add") {
        int result = req["a"].get<int>() + req["b"].get<int>();
        return {{"status", "ok"}, {"result", result}};
    }
    return {{"status", "error"}, {"message", "unknown command"}};
});
rep.start();

// 客户端
Requester req;
req.init("tcp://127.0.0.1:15561");

nlohmann::json request = {{"command", "add"}, {"a", 10}, {"b", 32}};
nlohmann::json reply;
req.request(request, reply, 5000);  // 超时 5000ms
// reply: {"result": 42, "status": "ok"}
```

---

### Python API

导入方式：`from scripts import Publisher, Subscriber, Requester, Replier, Message`

所有类均支持 **context manager**（`with` 语句）。

#### Message

```python
class Message:
    @staticmethod
    def serialize(data: dict) -> bytes
    @staticmethod
    def deserialize(data: bytes) -> dict
    @staticmethod
    def add_timestamp(msg: dict) -> None
    @staticmethod
    def get_timestamp() -> float
```

#### Publisher

```python
class Publisher:
    def init(self, address: str) -> bool
    def publish(self, msg: dict) -> bool
    def close(self)
    def is_initialized(self) -> bool
```

#### Subscriber

```python
class Subscriber:
    def init(self, address: str) -> bool
    def receive(self, timeout_ms: int = -1) -> Optional[dict]
    def set_callback(self, callback: Callable[[dict], None])
    def start_async(self)
    def stop_async(self)
    def close(self)
    def is_initialized(self) -> bool
```

#### Requester

```python
class Requester:
    def init(self, server_address: str) -> bool
    def request(self, req: dict, timeout_ms: int = 5000) -> Optional[dict]
    def close(self)
    def is_initialized(self) -> bool
```

#### Replier

```python
class Replier:
    def init(self, bind_address: str) -> bool
    def set_handler(self, handler: Callable[[dict], dict])
    def start(self)
    def stop(self)
    def close(self)
    def is_initialized(self) -> bool
```

#### Python 用法示例

**Pub/Sub：**

```python
from scripts import Publisher, Subscriber, Message
import time

pub = Publisher()
pub.init("tcp://127.0.0.1:15570")

sub = Subscriber()
sub.init("tcp://127.0.0.1:15570")
time.sleep(0.1)

msg = {"topic": "test", "value": 42}
Message.add_timestamp(msg)
pub.publish(msg)

received = sub.receive(timeout_ms=2000)
# received: {"topic": "test", "value": 42, "timestamp": ...}

pub.close()
sub.close()
```

**Pub/Sub（异步回调 + context manager）：**

```python
with Publisher() as pub, Subscriber() as sub:
    pub.init("tcp://127.0.0.1:15572")
    sub.init("tcp://127.0.0.1:15572")

    sub.set_callback(lambda msg: print(f"Received: {msg}"))
    sub.start_async()

    time.sleep(0.1)
    pub.publish({"seq": 0})

    time.sleep(0.5)
    sub.stop_async()
```

**Req/Rep：**

```python
from scripts import Requester, Replier

def handler(req):
    return {"result": req.get("x", 0) + req.get("y", 0)}

rep = Replier()
rep.init("tcp://127.0.0.1:15571")
rep.set_handler(handler)
rep.start()

req = Requester()
req.init("tcp://127.0.0.1:15571")
time.sleep(0.1)

reply = req.request({"x": 3, "y": 4}, timeout_ms=5000)
# reply: {"result": 7}

req.close()
rep.close()
```

## 测试

### C++ 测试

```bash
cd nng_comm/build
./test_nng_comm
```

使用端口：`15560`–`15564`

### Python 测试

```bash
cd nng_comm
python tests/test_nng_comm.py
```

使用端口：`15570`–`15574`

> C++ 和 Python 测试使用不同端口段，可同时运行互不冲突。

### 跨语言测试

验证 C++ 和 Python 之间的互操作性，共 4 个测试案例：

| 测试 | 服务端（listen） | 客户端（dial） | 端口 |
|------|------------------|----------------|------|
| 1 | C++ Publisher | Python Subscriber | 15580 |
| 2 | Python Publisher | C++ Subscriber | 15581 |
| 3 | C++ Replier | Python Requester | 15582 |
| 4 | Python Replier | C++ Requester | 15583 |

一键运行全部跨语言测试：

```bash
cd nng_comm
bash tests/test_cross_lang.sh
```

也可以手动分别运行两端进程（在两个终端中）：

```bash
# 示例：测试 1 — C++ 发布, Python 订阅
# 终端 1
./build/test_cross_lang pub tcp://127.0.0.1:15580

# 终端 2
python tests/test_cross_lang.py sub tcp://127.0.0.1:15580
```

支持的模式参数：`pub` | `sub` | `rep` | `req`

### 多机测试

`test_cross_host.py` 用于两台主机之间的网络联调。

| 模式 | 角色 | 启动命令（示例） |
|------|------|-----------------|
| `pub` | 发布端（listen） | `python tests/test_cross_host.py --mode pub --address <本机IP>` |
| `sub` | 订阅端（dial） | `python tests/test_cross_host.py --mode sub --address <pub端IP>` |
| `rep` | 服务端（listen） | `python tests/test_cross_host.py --mode rep --address <本机IP>` |
| `req` | 客户端（dial）  | `python tests/test_cross_host.py --mode req --address <rep端IP>` |

默认端口 `15590`，可通过 `--port` 指定：

```bash
# 机器 A（发布端）
python tests/test_cross_host.py --mode pub --address 192.168.1.100

# 机器 B（订阅端）
python tests/test_cross_host.py --mode sub --address 192.168.1.100
```

```bash
# 机器 A（服务端）
python tests/test_cross_host.py --mode rep --address 192.168.1.100

# 机器 B（客户端）
python tests/test_cross_host.py --mode req --address 192.168.1.100
```

两端均运行至 `Ctrl+C` 停止。
