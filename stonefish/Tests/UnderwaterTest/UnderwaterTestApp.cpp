/*    
    This file is a part of Stonefish.

    Stonefish is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Stonefish is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

//
//  UnderwaterTestApp.cpp
//  Stonefish
//
//  Created by Patryk Cieslak on 03/03/2014.
//  Copyright (c) 2014-2025 Patryk Cieslak. All rights reserved.
//

#include "UnderwaterTestApp.h"

#include <actuators/Servo.h>
#include <actuators/Thruster.h>
#include <actuators/VariableBuoyancy.h>
#include <core/Robot.h>
#include <sensors/scalar/Accelerometer.h>
#include <sensors/scalar/IMU.h>
#include <sensors/scalar/DVL.h>
#include <sensors/vision/FLS.h>
#include <sensors/vision/SSS.h>
#include <graphics/IMGUI.h>
#include <comms/USBL.h>
#include <core/Console.h>

UnderwaterTestApp::UnderwaterTestApp(std::string dataDirPath, sf::RenderSettings s, sf::HelperSettings h, UnderwaterTestManager* sim)
    : GraphicalSimulationApp("UnderwaterTest", dataDirPath, s, h, sim), surge_(0), yaw_(0)
{
}

// ============================================================================
// DoHUD() - 用户界面绘制回调函数
// ============================================================================
//
// 【触发时机】
//   DoHUD() 在每一帧渲染循环中被调用，具体调用链如下：
//   1. 主循环 Run() 调用 LoopInternal()
//   2. LoopInternal() 调用 RenderLoop()
//   3. RenderLoop() 中，当 displayHUD=true 且 displayConsole=false 时：
//      gui->Begin();
//      DoHUD();      <-- 在此处调用
//      gui->End();
//   4. 调用频率：与渲染帧率同步（通常60Hz左右）
//
// 【主要用途】
//   1. 绘制自定义GUI元素（滑块、按钮、标签、面板等）
//   2. 实时显示传感器数据和机器人状态
//   3. 提供用户交互控件来控制执行器（如推进器、伺服电机）
//   4. 调试信息可视化
//
// 【继承机制】
//   - 基类 GraphicalSimulationApp::DoHUD() 绘制默认调试面板：
//     * DEBUG面板：物理显示、坐标系、传感器、执行器、关节、碰撞等开关
//     * SUN POSITION面板：太阳方位角和仰角调节
//     * OCEAN面板：水体类型、悬浮颗粒开关
//     * VIEW面板：视角跟踪目标选择、曝光补偿
//     * SELECTION INFO面板：选中实体的详细信息
//     * 底部状态栏：绘制时间、CPU使用率、仿真时间
//   - 子类重写时应先调用基类方法，然后添加自定义控件
//
// 【GUI系统说明】
//   - 使用 IMGUI 即时模式GUI库（sf::IMGUI，非imgui.h库）
//   - 控件ID由 sf::Uid 结构体标识：owner(所有者)+item(项目编号)
//   - 常用控件：DoSlider(), DoCheckBox(), DoButton(), DoLabel(), DoPanel()
//   - 坐标系：左上角为原点，X向右，Y向下，单位为像素
//
// 【注意事项】
//   - 此函数在渲染线程中执行，避免执行耗时操作
//   - 按 [H] 键可切换HUD显示/隐藏
//   - 按 [C] 键切换到控制台模式时，DoHUD() 不会被调用
// ============================================================================
void UnderwaterTestApp::DoHUD()
{
    // 首先调用基类的DoHUD()，绘制默认的调试面板和状态栏
    // 基类会绘制：DEBUG、SUN POSITION、OCEAN、VIEW等面板
    GraphicalSimulationApp::DoHUD();

    // ========== 获取推进器执行器指针 ==========
    // 根据是否使用XML场景解析(PARSED_SCENARIO)，执行器名称前缀不同：
    // - XML解析模式：名称带机器人前缀 "GIRONA500/ThrusterSurgePort"
    // - 代码创建模式：名称不带前缀 "ThrusterSurgePort"
#ifdef PARSED_SCENARIO
    sf::Thruster* th1 = dynamic_cast<sf::Thruster*>(getSimulationManager()->getRobot("GIRONA500")->getActuator("GIRONA500/ThrusterSurgePort"));
    sf::Thruster* th2 = dynamic_cast<sf::Thruster*>(getSimulationManager()->getRobot("GIRONA500")->getActuator("GIRONA500/ThrusterSurgeStarboard"));
#else
    // 获取左舷(Port)和右舷(Starboard)的Surge方向推进器
    sf::Thruster* th1 = dynamic_cast<sf::Thruster*>(getSimulationManager()->getRobot("GIRONA500")->getActuator("ThrusterSurgePort"));
    sf::Thruster* th2 = dynamic_cast<sf::Thruster*>(getSimulationManager()->getRobot("GIRONA500")->getActuator("ThrusterSurgeStarboard"));
#endif

    // 只有在成功获取两个推进器后才绘制控制滑块
    if (th1 && th2)
    {
        // ========== GUI控件ID设置 ==========
        // sf::Uid 用于唯一标识GUI控件，防止状态混淆
        // - owner: 控件组所有者ID（这里使用10，与基类的0-4区分）
        // - item: 同一owner下的控件序号
        sf::Uid id;
        id.owner = 10;  // 自定义控件组ID，避免与基类冲突

        // ========== Surge(前进)控制滑块 ==========
        // DoSlider(id, x, y, width, min, max, currentValue, label)
        // - 位置: (180, 10)，宽度250像素
        // - 范围: [-1, 1]，-1=全速后退，0=停止，1=全速前进
        // - 返回值: 用户调整后的当前值
        id.item = 0;
        surge_ = getGUI()->DoSlider(id, 180.f, 10.f, 250.f, sf::Scalar(-1), sf::Scalar(1), surge_, "Surge");

        // ========== Yaw(偏航)控制滑块 ==========
        // - 位置: (180, 65)，在Surge滑块下方
        // - 范围: [-1, 1]，-1=左转，0=直行，1=右转
        id.item = 1;
        yaw_ = getGUI()->DoSlider(id, 180.f, 65.f, 250.f, sf::Scalar(-1), sf::Scalar(1), yaw_, "Yaw");

        // ========== 差速控制算法 ==========
        // 使用差速驱动原理控制AUV的前进和转向：
        // - 两个推进器位于AUV左右两侧，推力方向相同（都向后推）
        // - 前进：两个推进器同向同速 → th1 = th2 = -surge
        // - 转向：两个推进器差速 → 增加一侧推力，减少另一侧推力
        //
        // 控制公式：
        //   th1(左舷) = -surge - yaw  (前进分量 + 偏航分量)
        //   th2(右舷) = -surge + yaw  (前进分量 - 偏航分量)
        //
        // 示例：
        //   surge=1, yaw=0 → th1=-1, th2=-1 → 两侧同速，直线前进
        //   surge=0, yaw=1 → th1=-1, th2=1  → 左侧后推，右侧前推，原地右转
        //   surge=1, yaw=0.5 → th1=-1.5, th2=-0.5 → 前进同时右转
        //
        // 注意：setSetpoint()的参数是推进器转速设定值（归一化），
        // 负值表示反转（产生向后的推力使AUV前进）
        th1->setSetpoint(-surge_ - yaw_);
        th2->setSetpoint(-surge_ + yaw_);
    }
}
