#!/bin/bash
#
# 跨语言测试：C++ <-> Python
# 使用端口 15580-15583
#
# 用法：在 nng_comm/ 目录下执行
#   bash tests/test_cross_lang.sh
#
# 前置条件：
#   - 已编译 build/test_cross_lang
#   - 已安装 pynng

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CPP_BIN="$PROJECT_DIR/build/test_cross_lang"
PY_SCRIPT="$SCRIPT_DIR/test_cross_lang.py"

if [ ! -f "$CPP_BIN" ]; then
    echo "ERROR: $CPP_BIN not found. Please build first:"
    echo "  cd nng_comm/build && cmake .. && make -j\$(nproc)"
    exit 1
fi

PASSED=0
FAILED=0

run_test() {
    local test_name="$1"
    local cmd1="$2"
    local cmd2="$3"

    echo ""
    echo "=== $test_name ==="

    # 启动第一个进程（listen 端）
    eval "$cmd1" &
    local pid1=$!

    # 启动第二个进程（dial 端）
    eval "$cmd2" &
    local pid2=$!

    # 等待两个进程结束
    local fail=0
    wait $pid1 || fail=1
    wait $pid2 || fail=1

    if [ $fail -eq 0 ]; then
        echo "=== $test_name PASSED ==="
        PASSED=$((PASSED + 1))
    else
        echo "=== $test_name FAILED ==="
        FAILED=$((FAILED + 1))
    fi
}

# 测试 1: C++ 发布, Python 订阅
run_test "Test 1: C++ Pub -> Python Sub" \
    "$CPP_BIN pub tcp://127.0.0.1:15580" \
    "python3 $PY_SCRIPT sub tcp://127.0.0.1:15580"

# 测试 2: Python 发布, C++ 订阅
run_test "Test 2: Python Pub -> C++ Sub" \
    "python3 $PY_SCRIPT pub tcp://127.0.0.1:15581" \
    "$CPP_BIN sub tcp://127.0.0.1:15581"

# 测试 3: C++ 提供服务, Python 请求
run_test "Test 3: C++ Rep <- Python Req" \
    "$CPP_BIN rep tcp://127.0.0.1:15582" \
    "python3 $PY_SCRIPT req tcp://127.0.0.1:15582"

# 测试 4: Python 提供服务, C++ 请求
run_test "Test 4: Python Rep <- C++ Req" \
    "python3 $PY_SCRIPT rep tcp://127.0.0.1:15583" \
    "$CPP_BIN req tcp://127.0.0.1:15583"

echo ""
echo "==============================="
echo "Results: $PASSED passed, $FAILED failed out of 4"
echo "==============================="

[ $FAILED -eq 0 ] || exit 1
