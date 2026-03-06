#!/usr/bin/env python3
"""键盘遥控 SAM-AUV

使用状态机控制 AUV：
- W/↑: 前进（后退中按下则先停止）
- S/↓: 后退（前进中按下则先停止）
- A/←: 左转（右转中按下则先停止）
- D/→: 右转（左转中按下则先停止）
- N: 停止所有状态
- Q/Esc: 退出程序

前进/后退 和 左转/右转 互不干扰，可以同时激活。
反向操作需要先经过停止状态，避免突然反向。
"""

import sys
import os
import json
import time
import curses
import math

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_SCRIPT_DIR, '..', '..', 'nng_comm'))
from scripts import Publisher, Message


def load_jsonc(path: str) -> dict:
    """加载 JSONC 文件（支持 // 注释，正确跳过字符串内的 //）"""
    with open(path, 'r', encoding='utf-8') as f:
        content = f.read()
    result, i, in_string = [], 0, False
    while i < len(content):
        c = content[i]
        if c == '\\' and in_string:
            result.append(c); result.append(content[i + 1]); i += 2; continue
        if c == '"':
            in_string = not in_string
        if not in_string and c == '/' and i + 1 < len(content) and content[i + 1] == '/':
            while i < len(content) and content[i] != '\n':
                i += 1
            continue
        result.append(c); i += 1
    return json.loads(''.join(result))


# 状态定义
class ThrustState:
    STOP = 0
    FORWARD = 1
    BACKWARD = 2


class TurnState:
    STOP = 0
    LEFT = 1
    RIGHT = 2


def send_thruster_cmd(pub: Publisher, rpm: float):
    """发送推进器指令"""
    pub.publish({"timestamp": time.time(), "rpm": float(rpm)})


def send_angle_cmd(pub: Publisher, horizontal: float, vertical: float = 0.0):
    """发送舵角指令"""
    pub.publish({
        "timestamp": time.time(),
        "horizontal_radians": float(horizontal),
        "vertical_radians": float(vertical)
    })


def safe_addstr(stdscr, y, x, text):
    """安全地添加字符串，忽略终端边界错误"""
    try:
        height, width = stdscr.getmaxyx()
        if y < height and x < width:
            # 截断超出宽度的文本
            max_len = width - x - 1
            if max_len > 0:
                stdscr.addstr(y, x, text[:max_len])
    except curses.error:
        pass


def main(stdscr):
    # 读取配置
    config = load_jsonc(os.path.join(_SCRIPT_DIR, '..', 'config', 'test_teleop.jsonc'))
    max_rpm       = config['teleop']['max_rpm']
    max_angle_rad = config['teleop']['max_angle_rad']
    max_angle_deg = max_angle_rad * 180 / math.pi
    loop_interval = 1.0 / config['teleop']['loop_hz']
    thruster1_addr = config['network']['thruster1']
    thruster2_addr = config['network']['thruster2']
    angle_addr     = config['network']['thrust_angle']

    # 设置 curses
    curses.curs_set(0)  # 隐藏光标
    stdscr.nodelay(True)  # 非阻塞输入
    stdscr.timeout(0)  # 不等待，立即返回

    # 检查终端大小
    height, width = stdscr.getmaxyx()
    if height < 17 or width < 55:
        stdscr.addstr(0, 0, f"终端太小! 需要至少 17x55, 当前 {height}x{width}")
        stdscr.refresh()
        stdscr.nodelay(False)
        stdscr.getch()
        return

    # 显示说明
    safe_addstr(stdscr, 0, 0, "=" * 50)
    safe_addstr(stdscr, 1, 0, "SAM-AUV 键盘遥控 (状态机模式)")
    safe_addstr(stdscr, 2, 0, "=" * 50)
    safe_addstr(stdscr, 3, 0, "操作说明:")
    safe_addstr(stdscr, 4, 0, "  W/上 : 前进    S/下 : 后退")
    safe_addstr(stdscr, 5, 0, "  A/左 : 左转    D/右 : 右转")
    safe_addstr(stdscr, 6, 0, "  N   : 停止    Q/Esc : 退出")
    safe_addstr(stdscr, 7, 0, "=" * 50)
    safe_addstr(stdscr, 8, 0, f"RPM: {max_rpm}, 舵角: {max_angle_deg:.1f} deg")
    safe_addstr(stdscr, 9, 0, "=" * 50)

    # 创建 Publisher 套接字（listen 模式，与仿真器 C++ Subscriber.dial 配对）
    pub1      = Publisher()
    pub2      = Publisher()
    pub_angle = Publisher()

    try:
        if not pub1.init(thruster1_addr):
            raise RuntimeError(f"pub1 init failed: {thruster1_addr}")
        if not pub2.init(thruster2_addr):
            raise RuntimeError(f"pub2 init failed: {thruster2_addr}")
        if not pub_angle.init(angle_addr):
            raise RuntimeError(f"pub_angle init failed: {angle_addr}")
        safe_addstr(stdscr, 11, 0, f"监听: {thruster1_addr}, {thruster2_addr}")
        safe_addstr(stdscr, 12, 0, f"      {angle_addr}")
    except Exception as e:
        safe_addstr(stdscr, 11, 0, f"初始化失败: {e}")
        stdscr.refresh()
        time.sleep(2)
        pub1.close(); pub2.close(); pub_angle.close()
        return

    time.sleep(0.1)  # 等待连接建立

    # 状态机
    thrust_state = ThrustState.STOP
    turn_state = TurnState.STOP

    running = True

    while running:
        loop_start = time.time()

        # 读取所有待处理的按键
        while True:
            try:
                key = stdscr.getch()
            except:
                key = -1

            if key == -1 or key == curses.ERR:
                break

            # 处理按键 - 状态切换（反向操作先经过停止状态）
            if key == curses.KEY_UP or key == ord('w') or key == ord('W'):
                if thrust_state == ThrustState.BACKWARD:
                    thrust_state = ThrustState.STOP
                else:
                    thrust_state = ThrustState.FORWARD
            elif key == curses.KEY_DOWN or key == ord('s') or key == ord('S'):
                if thrust_state == ThrustState.FORWARD:
                    thrust_state = ThrustState.STOP
                else:
                    thrust_state = ThrustState.BACKWARD
            elif key == curses.KEY_LEFT or key == ord('a') or key == ord('A'):
                if turn_state == TurnState.RIGHT:
                    turn_state = TurnState.STOP
                else:
                    turn_state = TurnState.LEFT
            elif key == curses.KEY_RIGHT or key == ord('d') or key == ord('D'):
                if turn_state == TurnState.LEFT:
                    turn_state = TurnState.STOP
                else:
                    turn_state = TurnState.RIGHT
            elif key == ord('n') or key == ord('N'):
                thrust_state = ThrustState.STOP
                turn_state = TurnState.STOP
            elif key == ord('q') or key == ord('Q') or key == 27:  # Q or ESC
                running = False
                break

        if not running:
            break

        # 根据状态计算控制量
        if thrust_state == ThrustState.FORWARD:
            rpm = max_rpm
        elif thrust_state == ThrustState.BACKWARD:
            rpm = -max_rpm
        else:
            rpm = 0.0

        if turn_state == TurnState.LEFT:
            horizontal = max_angle_rad
        elif turn_state == TurnState.RIGHT:
            horizontal = -max_angle_rad
        else:
            horizontal = 0.0

        # 发送指令
        send_thruster_cmd(pub1, rpm)
        send_thruster_cmd(pub2, rpm)
        send_angle_cmd(pub_angle, horizontal)

        # 显示状态
        thrust_str = {ThrustState.STOP: "停止", ThrustState.FORWARD: "前进", ThrustState.BACKWARD: "后退"}
        turn_str = {TurnState.STOP: "直行", TurnState.LEFT: "左转", TurnState.RIGHT: "右转"}

        safe_addstr(stdscr, 14, 0, f"推进状态: {thrust_str[thrust_state]:6}  转向状态: {turn_str[turn_state]:6}")
        safe_addstr(stdscr, 15, 0, f"RPM: {rpm:8.1f}  舵角: {horizontal:8.4f} rad")
        stdscr.refresh()

        # 控制循环频率
        elapsed = time.time() - loop_start
        if elapsed < loop_interval:
            time.sleep(loop_interval - elapsed)

    # 发送归零指令
    for _ in range(5):
        send_thruster_cmd(pub1, 0)
        send_thruster_cmd(pub2, 0)
        send_angle_cmd(pub_angle, 0)
        time.sleep(0.05)

    pub1.close()
    pub2.close()
    pub_angle.close()


if __name__ == "__main__":
    print("启动遥控...")
    curses.wrapper(main)
    print("已退出")
