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
//  UnderwaterTestManager.cpp
//  Stonefish
//
//  Created by Patryk Cieslak on 04/03/2014.
//  Copyright(c) 2014-2025 Patryk Cieslak. All rights reserved.
//

#include "UnderwaterTestManager.h"

#include "UnderwaterTestApp.h"
#include <core/FeatherstoneRobot.h>
#include <entities/statics/Plane.h>
#include <entities/statics/Obstacle.h>
#include <entities/solids/Polyhedron.h>
#include <entities/solids/Box.h>
#include <entities/solids/Sphere.h>
#include <entities/solids/Torus.h>
#include <entities/solids/Cylinder.h>
#include <entities/solids/Compound.h>
#include <entities/solids/Wing.h>
#include <graphics/OpenGLPointLight.h>
#include <graphics/OpenGLSpotLight.h>
#include <graphics/OpenGLTrackball.h>
#include <utils/SystemUtil.hpp>
#include <entities/statics/Obstacle.h>
#include <entities/statics/Terrain.h>
#include <actuators/Thruster.h>
#include <actuators/Servo.h>
#include <actuators/VariableBuoyancy.h>
#include <sensors/scalar/Pressure.h>
#include <sensors/scalar/Odometry.h>
#include <sensors/scalar/DVL.h>
#include <sensors/scalar/Compass.h>
#include <sensors/scalar/IMU.h>
#include <sensors/scalar/GPS.h>
#include <sensors/Contact.h>
#include <sensors/vision/ColorCamera.h>
#include <sensors/vision/DepthCamera.h>
#include <sensors/vision/Multibeam2.h>
#include <sensors/vision/FLS.h>
#include <sensors/vision/SSS.h>
#include <sensors/vision/MSIS.h>
#include <comms/AcousticModem.h>
#include <sensors/Sample.h>
#include <actuators/Light.h>
#include <sensors/scalar/RotaryEncoder.h>
#include <sensors/scalar/Accelerometer.h>
#include <entities/FeatherstoneEntity.h>
#include <entities/forcefields/Trigger.h>
#include <entities/forcefields/Pipe.h>
#include <entities/forcefields/Jet.h>
#include <entities/forcefields/Uniform.h>
#include <entities/AnimatedEntity.h>
#include <sensors/scalar/Profiler.h>
#include <sensors/scalar/Multibeam.h>
#include <sensors/scalar/EchoSounder.h>
#include <utils/UnitSystem.h>
#include <core/ScenarioParser.h>
#include <core/NED.h>
#include <iomanip>

UnderwaterTestManager::UnderwaterTestManager(sf::Scalar stepsPerSecond)
: SimulationManager(stepsPerSecond, sf::Solver::SI, sf::CollisionFilter::EXCLUSIVE)
{
}

void UnderwaterTestManager::BuildScenario()
{
#ifdef PARSED_SCENARIO
    sf::ScenarioParser parser(this);
    bool success = parser.Parse(sf::GetDataPath() + "underwater_test.scn");
    parser.SaveLog("underwater_test.log");
    if(!success)
        cCritical("Scenario parser: Parsing failed!");
#else
    // ============================================================================
    // 第一部分：材料定义 (MATERIALS)
    // ============================================================================
    // 材料定义了物体的物理属性，主要包括：
    // - 密度(density): 影响质量计算和浮力
    // - 恢复系数(restitution): 碰撞时的弹性，0=完全非弹性，1=完全弹性
    //
    // CreateMaterial(名称, 密度, 恢复系数)
    // sf::UnitSystem::Density(CGS, MKS, value) 将CGS单位(g/cm³)转换为MKS单位(kg/m³)
    // ============================================================================

    // Dummy材料：低密度(0.9g/cm³)，用于不重要的内部零件，比水轻会上浮
    CreateMaterial("Dummy", sf::UnitSystem::Density(sf::CGS, sf::MKS, 0.9), 0.3);
    // Fiberglass玻璃纤维：中等密度(1.5g/cm³)，用于AUV外壳，比水重会下沉
    CreateMaterial("Fiberglass", sf::UnitSystem::Density(sf::CGS, sf::MKS, 1.5), 0.9);
    // Rock岩石：高密度(3.0g/cm³)，用于海床地形
    CreateMaterial("Rock", sf::UnitSystem::Density(sf::CGS, sf::MKS, 3.0), 0.6);
    // Metal金属：高密度(7.8g/cm³ 钢铁)，高恢复系数(0.95)用于高声纳反射的障碍物
    CreateMaterial("Metal", sf::UnitSystem::Density(sf::CGS, sf::MKS, 7.8), 0.95);

    // ============================================================================
    // 材料交互定义：定义不同材料之间碰撞时的摩擦系数
    // SetMaterialsInteraction(材料1, 材料2, 静摩擦系数, 动摩擦系数)
    // - 静摩擦系数：物体从静止开始运动所需的摩擦力
    // - 动摩擦系数：物体运动中的摩擦力（通常小于静摩擦）
    // ============================================================================
    SetMaterialsInteraction("Dummy", "Dummy", 0.5, 0.2);
    SetMaterialsInteraction("Fiberglass", "Fiberglass", 0.5, 0.2);
    SetMaterialsInteraction("Rock", "Rock", 0.9, 0.7);      // 岩石间摩擦较大
    SetMaterialsInteraction("Metal", "Metal", 0.4, 0.3);    // 金属间摩擦较小
    SetMaterialsInteraction("Fiberglass", "Dummy", 0.5, 0.2);
    SetMaterialsInteraction("Rock", "Dummy", 0.6, 0.4);
    SetMaterialsInteraction("Rock", "Fiberglass", 0.6, 0.4);
    SetMaterialsInteraction("Metal", "Dummy", 0.5, 0.3);
    SetMaterialsInteraction("Metal", "Rock", 0.5, 0.4);
    SetMaterialsInteraction("Metal", "Fiberglass", 0.5, 0.3);

    // ============================================================================
    // 第二部分：外观定义 (LOOKS)
    // ============================================================================
    // 外观定义了物体的渲染属性，用于视觉显示（不影响物理仿真）
    // CreateLook(名称, 颜色, 粗糙度, 金属度, 反射率, 纹理路径, 法线贴图路径)
    // - 颜色：RGB值(0-1范围)
    // - 粗糙度(roughness)：0=光滑镜面，1=粗糙漫反射
    // - 金属度(metalness)：0=非金属，1=金属
    // ============================================================================

    // 黄色外观：用于AUV外壳，低粗糙度使其看起来光滑
    CreateLook("yellow", sf::Color::RGB(1.f, 0.9f, 0.f), 0.3f, 0.f);
    // 灰色外观：用于支撑杆等结构件
    CreateLook("grey", sf::Color::RGB(0.3f, 0.3f, 0.3f), 0.4f, 0.5f);
    // 金属外观：银灰色，低粗糙度(0.2)使其有光泽，高金属度(0.9)
    CreateLook("metal", sf::Color::RGB(0.8f, 0.8f, 0.85f), 0.2f, 0.9f);
    // 海床外观：带法线贴图实现砂质纹理效果
    CreateLook("seabed", sf::Color::RGB(0.7f, 0.7f, 0.5f), 0.9f, 0.f, 0.f, "", sf::GetDataPath() + "sand_normal.png");
    // 螺旋桨外观：带纹理贴图
    CreateLook("propeller", sf::Color::RGB(1.f, 1.f, 1.f), 0.3f, 0.f, 0.f, sf::GetDataPath() + "propeller_tex.png");
    // 黑色外观：用于推进器导管
    CreateLook("black", sf::Color::RGB(0.1f, 0.1f, 0.1f), 0.4f, 0.5f);
    // 机械臂外观：棕色金属质感
    CreateLook("manipulator", sf::Color::RGB(0.2f, 0.15f, 0.1f), 0.6f, 0.8f);
    // 机械臂关节4外观：带纹理
    CreateLook("link4", sf::Color::RGB(1.f, 1.f, 1.f), 0.6f, 0.8f, 0.f, sf::GetDataPath() + "link4_tex.png");

    // ============================================================================
    // 第三部分：环境设置 (ENVIRONMENT)
    // ============================================================================
    // 配置海洋环境、大气、地理坐标等
    // ============================================================================

    // 启用海洋环境，参数0.0表示无波浪（静水）
    // 若设置>0则会生成FFT波浪仿真
    EnableOcean(1.0);

    // 设置水体类型(Jerlov水型)：0.2表示沿海浑浊水域
    // 影响光在水中的衰减和散射，数值越大水越浑浊
    getOcean()->setWaterType(0.2);

    // 添加水流场：Jet是射流，从(0,0,1)位置沿Y轴方向，半径0.3m，速度5m/s
    getOcean()->AddVelocityField(new sf::Jet(sf::Vector3(0,0,1.0), sf::VY(), 0.3, 5.0));
    // 添加均匀水流：沿X轴正方向1m/s的恒定水流
    getOcean()->AddVelocityField(new sf::Uniform(sf::Vector3(1.0,0.0,0.0)));
    // 启用水流效果（使上述水流场生效）
    getOcean()->EnableCurrents();

    // 设置太阳位置：方位角0度（正北），仰角60度
    // 影响水下光照和阴影效果
    getAtmosphere()->SetSunPosition(0.0, 60.0);

    // 初始化NED(北东下)地理坐标系
    // 参数：纬度41.77737°N，经度3.03376°E（西班牙加泰罗尼亚海岸），海拔0m
    // 用于GPS传感器等需要地理坐标的功能
    getNED()->Init(41.77737, 3.03376, 0.0);

    // ============================================================================
    // 第四部分：静态实体 (STATIC ENTITIES)
    // ============================================================================
    // 静态实体是不参与动力学计算的固定物体，如海床、障碍物等
    // ============================================================================

    // 创建海床地形
    // Terrain(名称, 高度图路径, X缩放, Y缩放, Z缩放(高度), 材料, 外观, 纹理平铺)
    // 高度图是灰度图像，白色=高，黑色=低
    sf::Terrain* seabed = new sf::Terrain("Seabed", sf::GetDataPath() + "terrain.png", 2.0, 2.0, 15.0, "Rock", "seabed", 5.f);
    // 将海床放置在深度15m处（NED坐标系Z轴向下为正）
    AddStaticEntity(seabed, sf::Transform(sf::IQ(), sf::Vector3(0,0,15.0)));

    // 创建一个圆柱形障碍物
    // Obstacle(名称, 半径, 高度, 变换矩阵, 材料, 外观)
    sf::Obstacle* cyl = new sf::Obstacle("Cyl", 0.5, 5.0, sf::I4(), "Fiberglass", "seabed");
    // 放置障碍物，旋转90度使其水平放置
    // sf::Quaternion(roll, pitch, yaw) 欧拉角表示的旋转
    AddStaticEntity(cyl, sf::Transform(sf::Quaternion(0,M_PI_2,0), sf::Vector3(6.0,2.0,5.0)));

    // ========== AUV周围的障碍物（金属材质，高声纳反射） ==========
    // 8个金属圆柱围绕AUV放置，半径60米，均匀分布（每45度一个）
    // 用于测试声纳探测效果
    const sf::Scalar obstacleRadius = 60.0;  // 围绕半径（距AUV中心的距离）
    const sf::Scalar cylinderRadius = 1.5;   // 圆柱半径
    const sf::Scalar cylinderHeight = 12.0;  // 圆柱高度
    const sf::Scalar obstacleDepth = 6.0;    // 圆柱中心深度

    for (int i = 0; i < 8; ++i) {
        sf::Scalar angle = i * M_PI / 4.0;  // 每45度放置一个（360°/8=45°）
        // 使用极坐标转换为笛卡尔坐标
        sf::Scalar x = obstacleRadius * cos(angle);
        sf::Scalar y = obstacleRadius * sin(angle);
        std::string name = "Obstacle" + std::to_string(i + 1);
        sf::Obstacle* obstacle = new sf::Obstacle(name, cylinderRadius, cylinderHeight, sf::I4(), "Metal", "metal");
        // sf::IQ() 返回单位四元数（无旋转），障碍物垂直放置
        AddStaticEntity(obstacle, sf::Transform(sf::IQ(), sf::Vector3(x, y, obstacleDepth)));
    }

    // ============================================================================
    // 第五部分：灯光设置 (LIGHTS)
    // ============================================================================
    // 灯光作为执行器添加，可以动态控制开关和亮度
    // ============================================================================

    // 创建聚光灯
    // Light(名称, 光源半径, 光锥角度, 色温颜色, 光通量lumen)
    // BlackBody(5000.0) 生成5000K色温的光（类似日光）
	sf::Light* spot = new sf::Light("Spot", 0.02, 50.0, sf::Color::BlackBody(5000.0), 100.0);
	// 将灯光附加到世界坐标系固定位置
	spot->AttachToWorld(sf::Transform(sf::Quaternion(0,0,M_PI/3.0), sf::Vector3(0.0,0.0,1.0)));
	AddActuator(spot);

    // 创建全向灯（点光源）
    // Light(名称, 光源半径, 色温颜色, 光通量lumen) - 无光锥角度参数则为全向
    sf::Light* omni = new sf::Light("Omni", 0.02, sf::Color::BlackBody(5000.0), 10000.0);
	omni->AttachToWorld(sf::Transform(sf::Quaternion(0,0,M_PI/3.0), sf::Vector3(2.0,2.0,0.5)));
	AddActuator(omni);

    // ============================================================================
    // 第六部分：AUV本体构建 (VEHICLE BODY)
    // ============================================================================
    // GIRONA500 AUV是一个复合体(Compound)，由多个外部零件和内部零件组成
    // 外部零件参与水动力学计算，内部零件只贡献质量和惯性
    // ============================================================================

    // 设置物理属性
    sf::PhysicsSettings phy;
    phy.mode = sf::PhysicsMode::SUBMERGED;  // 完全浸没模式：计算浮力、附加质量、水阻力
    phy.collisions = true;                   // 启用碰撞检测

    // ---------- 外部零件（关闭浮力单独设置，由整体复合体统一计算） ----------
    phy.buoyancy = false;

    // 加载AUV外壳模型（三个相同的hull组成三体结构）
    // Polyhedron(名称, 物理设置, OBJ文件, 缩放, 变换, 材料, 外观, 流体阻力系数)
    // 底部外壳
    sf::Polyhedron* hullB = new sf::Polyhedron("HullBottom", phy, sf::GetDataPath() + "hull_hydro.obj", sf::Scalar(1), sf::I4(), "Fiberglass", "yellow", sf::Scalar(0.003));
    // 左舷外壳
    sf::Polyhedron* hullP = new sf::Polyhedron("HullPort", phy, sf::GetDataPath() + "hull_hydro.obj", sf::Scalar(1), sf::I4(), "Fiberglass", "yellow", sf::Scalar(0.003));
    // 右舷外壳
    sf::Polyhedron* hullS = new sf::Polyhedron("HullStarboard", phy, sf::GetDataPath() + "hull_hydro.obj", sf::Scalar(1), sf::I4(), "Fiberglass", "yellow", sf::Scalar(0.003));
    // 尾部垂直支撑杆
    sf::Polyhedron* vBarStern = new sf::Polyhedron("VBarStern", phy, sf::GetDataPath() + "vbar_hydro.obj", sf::Scalar(1), sf::I4(), "Dummy", "grey", sf::Scalar(0.003));
    // 艏部垂直支撑杆
    sf::Polyhedron* vBarBow = new sf::Polyhedron("VBarBow", phy, sf::GetDataPath() + "vbar_hydro.obj", sf::Scalar(1), sf::I4(), "Dummy", "grey", sf::Scalar(0.003));

    // ---------- 推进器导管（启用浮力计算） ----------
    phy.buoyancy = true;
    // Sway方向推进器导管（横向移动）
    sf::Polyhedron* ductSway = new sf::Polyhedron("DuctSway", phy, sf::GetDataPath() + "duct_hydro.obj", sf::Scalar(1), sf::I4(), "Dummy", "black");
    // Surge方向左舷推进器导管（前进）
    sf::Polyhedron* ductSurgeP = new sf::Polyhedron("DuctSurgePort", phy, sf::GetDataPath() + "duct_hydro.obj", sf::Scalar(1), sf::I4(), "Dummy", "black");
    // Surge方向右舷推进器导管
    sf::Polyhedron* ductSurgeS = new sf::Polyhedron("DuctSurgeStarboard", phy, sf::GetDataPath() + "duct_hydro.obj", sf::Scalar(1), sf::I4(), "Dummy", "black");
    // Heave方向尾部推进器导管（垂向移动）
    sf::Polyhedron* ductHeaveS = new sf::Polyhedron("DuctHeaveStern", phy, sf::GetDataPath() + "duct_hydro.obj", sf::Scalar(1), sf::I4(), "Dummy", "black");
    // Heave方向艏部推进器导管
    sf::Polyhedron* ductHeaveB = new sf::Polyhedron("DuctHeaveBow", phy, sf::GetDataPath() + "duct_hydro.obj", sf::Scalar(1), sf::I4(), "Dummy", "black");

    // ---------- 内部零件（用于调整质量分布和重心） ----------
    // 电池筒：半径0.13m，长度0.6m，质量缩放到70kg
    sf::Cylinder* batteryCyl = new sf::Cylinder("BatteryCylinder", phy, 0.13, 0.6, sf::I4(), "Dummy", "manipulator");
    batteryCyl->ScalePhysicalPropertiesToArbitraryMass(sf::Scalar(70));  // 将质量设为70kg
    // 左舷配重筒：质量20kg
    sf::Cylinder* portCyl = new sf::Cylinder("PortCylinder", phy, 0.13, 1.0, sf::I4(), "Dummy", "manipulator");
    portCyl->ScalePhysicalPropertiesToArbitraryMass(sf::Scalar(20));
    // 右舷配重筒：质量20kg
    sf::Cylinder* starboardCyl = new sf::Cylinder("StarboardCylinder", phy, 0.13, 1.0, sf::I4(), "Dummy", "manipulator");
    starboardCyl->ScalePhysicalPropertiesToArbitraryMass(sf::Scalar(20));

    // ---------- 组装复合体 ----------
    // 以底部外壳为基准创建复合体
    // Compound(名称, 物理设置, 基准零件, 基准零件在复合体中的变换)
    sf::Compound* vehicle = new sf::Compound("Vehicle", phy, hullB, sf::I4());

    // 添加外部零件：AddExternalPart(零件, 相对于基准的位姿变换)
    // 位姿变换 = Transform(旋转四元数, 平移向量)
    // 左舷外壳：Y方向偏移-0.35m，Z方向偏移-0.7m（上方）
    vehicle->AddExternalPart(hullP, sf::Transform(sf::IQ(), sf::Vector3(0,-0.35,-0.7)));
    // 右舷外壳：Y方向偏移+0.35m
    vehicle->AddExternalPart(hullS, sf::Transform(sf::IQ(), sf::Vector3(0,0.35,-0.7)));
    // 尾部支撑杆
    vehicle->AddExternalPart(vBarStern, sf::Transform(sf::Quaternion::getIdentity(), sf::Vector3(-0.25,0.0,-0.15)));
    // 艏部支撑杆
    vehicle->AddExternalPart(vBarBow, sf::Transform(sf::Quaternion::getIdentity(), sf::Vector3(0.30,0.0,-0.15)));
    // 各推进器导管（需要旋转到正确方向）
    vehicle->AddExternalPart(ductSway, sf::Transform(sf::Quaternion(M_PI_2,M_PI,0), sf::Vector3(-0.0137, 0.0307, -0.38)));
    vehicle->AddExternalPart(ductSurgeP, sf::Transform(sf::Quaternion(0,0,M_PI), sf::Vector3(-0.2807,-0.2587,-0.38)));
    vehicle->AddExternalPart(ductSurgeS, sf::Transform(sf::Quaternion(0,0,0), sf::Vector3(-0.2807,0.2587,-0.38)));
    vehicle->AddExternalPart(ductHeaveS, sf::Transform(sf::Quaternion(M_PI_2,-M_PI_2,0), sf::Vector3(-0.5337,0.0,-0.6747)));
    vehicle->AddExternalPart(ductHeaveB, sf::Transform(sf::Quaternion(-M_PI_2,-M_PI_2,0), sf::Vector3(0.5837,0.0,-0.6747)));

    // 添加内部零件：只贡献质量和惯性，不参与水动力计算
    vehicle->AddInternalPart(batteryCyl, sf::Transform(sf::Quaternion(0,M_PI_2,0), sf::Vector3(-0.1,0,0)));
    vehicle->AddInternalPart(portCyl, sf::Transform(sf::Quaternion(0,M_PI_2,0), sf::Vector3(0.0,-0.35,-0.7)));
    vehicle->AddInternalPart(starboardCyl, sf::Transform(sf::Quaternion(0,M_PI_2,0), sf::Vector3(0.0,0.35,-0.7)));

    // 隐藏内部零件的显示
    vehicle->setDisplayInternalParts(false);

    // ============================================================================
    // 第七部分：机械臂构建 (MANIPULATOR)
    // ============================================================================
    // 4自由度机械臂 + 2指夹爪
    // 每个连杆从OBJ文件加载，带水动力学网格(_hydro.obj后缀)
    // ============================================================================

    // 机械臂基座（固定在AUV上）
    sf::Polyhedron* baseLink = new sf::Polyhedron("ArmBaseLink", phy, sf::GetDataPath() + "base_link_uji_hydro.obj", sf::Scalar(1), sf::I4(), "Dummy", "manipulator");
    // 关节1的连杆
    sf::Polyhedron* link1 = new sf::Polyhedron("ArmLink1", phy, sf::GetDataPath() + "link1_hydro.obj", sf::Scalar(1), sf::I4(), "Dummy", "manipulator");
    // 关节2的连杆
    sf::Polyhedron* link2 = new sf::Polyhedron("ArmLink2", phy, sf::GetDataPath() + "link2_hydro.obj", sf::Scalar(1), sf::I4(), "Dummy", "manipulator");
    // 关节3的连杆
    sf::Polyhedron* link3 = new sf::Polyhedron("ArmLink3", phy, sf::GetDataPath() + "link3_hydro.obj", sf::Scalar(1), sf::I4(), "Dummy", "manipulator");
    // 关节4的连杆（带力矩传感器）
    sf::Polyhedron* link4 = new sf::Polyhedron("ArmLink4", phy, sf::GetDataPath() + "link4ft_hydro.obj", sf::Scalar(1), sf::I4(), "Dummy", "link4");
    // 末端执行器（夹爪基座）
    sf::Polyhedron* ee = new sf::Polyhedron("EE", phy, sf::GetDataPath() + "eeprobe_hydro.obj", sf::Scalar(1), sf::I4(), "Neutral", "manipulator");
    // 夹爪手指1
    sf::Polyhedron* finger1 = new sf::Polyhedron("Finger1", phy, sf::GetDataPath() + "fingerA_hydro.obj", sf::Scalar(1), sf::I4(), "Dummy", "manipulator");
    // 夹爪手指2
    sf::Polyhedron* finger2 = new sf::Polyhedron("Finger2", phy, sf::GetDataPath() + "fingerA_hydro.obj", sf::Scalar(1), sf::I4(), "Dummy", "manipulator");

    // 将所有机械臂连杆放入数组，用于后续定义机器人结构
    std::vector<sf::SolidEntity*> arm;
    arm.push_back(baseLink);
    arm.push_back(link1);
    arm.push_back(link2);
    arm.push_back(link3);
    arm.push_back(link4);
    arm.push_back(ee);
    arm.push_back(finger1);
    arm.push_back(finger2);

    // ============================================================================
    // 第八部分：伺服电机创建 (SERVOMOTORS)
    // ============================================================================
    // 伺服电机是位置控制的关节执行器
    // Servo(名称, Kp比例增益, Kd微分增益, 最大扭矩Nm)
    // 使用PD控制器跟踪目标位置
    // ============================================================================

    // 机械臂关节伺服（高扭矩100Nm）
    sf::Servo* srv1 = new sf::Servo("Servo1", 1.0, 1.0, 100.0);  // 关节1：绕Z轴旋转
    sf::Servo* srv2 = new sf::Servo("Servo2", 1.0, 1.0, 100.0);  // 关节2：绕Y轴旋转
    sf::Servo* srv3 = new sf::Servo("Servo3", 1.0, 1.0, 100.0);  // 关节3：绕Y轴旋转
    sf::Servo* srv4 = new sf::Servo("Servo4", 1.0, 1.0, 100.0);  // 关节4：绕Z轴旋转
    // 夹爪手指伺服（低扭矩10Nm）
    sf::Servo* srv5 = new sf::Servo("FServo1", 1.0, 1.0, 10.0);  // 手指1
    sf::Servo* srv6 = new sf::Servo("FServo2", 1.0, 1.0, 10.0);  // 手指2

    // ============================================================================
    // 第九部分：推进器创建 (THRUSTERS)
    // ============================================================================
    // GIRONA500有5个推进器：
    // - 1个Sway方向（横向移动）
    // - 2个Surge方向（前后移动，左右对称）
    // - 2个Heave方向（上下移动，前后分布）
    // ============================================================================

    // 推进器名称数组
    std::array<std::string, 5> thrusterNames = {"ThrusterSway", "ThrusterSurgePort", "ThrusterSurgeStarboard", "ThrusterHeaveStern", "ThrusterHeaveBow"};
    std::array<sf::Thruster*, 5> thrusters;

    // 螺旋桨模型（所有推进器共用同一个模型）
    std::shared_ptr<sf::Polyhedron> propeller = std::make_shared<sf::Polyhedron>("Propeller", phy, sf::GetDataPath() + "propeller.obj", sf::Scalar(1), sf::I4(), "Dummy", "propeller");

    // 循环创建5个推进器
    for(size_t i=0; i<thrusterNames.size(); ++i)
    {
        // 转子动力学模型：MechanicalPI是机械PI控制器
        // MechanicalPI(惯性, Kp, Ki, 最大转速rad/s)
        std::shared_ptr<sf::MechanicalPI> rotorDynamics;
        rotorDynamics = std::make_shared<sf::MechanicalPI>(1.0, 10.0, 5.0, 5.0);

        // 推力模型：FDThrust是流体动力学推力模型
        // FDThrust(直径, 前进推力系数, 后退推力系数, 扭矩系数, 双向, 水密度)
        std::shared_ptr<sf::FDThrust> thrustModel;
        thrustModel = std::make_shared<sf::FDThrust>(0.18, 0.48, 0.48, 0.05, true, getOcean()->getLiquid().density);

        // 创建推进器
        // Thruster(名称, 螺旋桨模型, 转子动力学, 推力模型, 直径, 右手螺旋, 最大RPM, 显示, 渲染螺旋桨)
        thrusters[i] = new sf::Thruster(thrusterNames[i], propeller, rotorDynamics, thrustModel, 0.18, true, 105.0, true, true);
    }

    // ============================================================================
    // 可变浮力系统（VBS）- 已注释掉
    // ============================================================================
    // VBS通过改变排水体积来调节浮力，用于深度控制
    //std::vector<std::string> vmeshes;
    //vmeshes.push_back(sf::GetDataPath() + "vbs_max.obj");  // 最大体积网格
    //vmeshes.push_back(sf::GetDataPath() + "vbs_min.obj");  // 最小体积网格
    //sf::VariableBuoyancy* vbs = new sf::VariableBuoyancy("VBS", vmeshes, 0.002);

    // ============================================================================
    // 第十部分：传感器创建 (SENSORS)
    // ============================================================================
    // 传感器分为标量传感器和视觉传感器两类
    // ============================================================================

    // ---------- 标量传感器 ----------

    // 里程计：提供绝对位置和速度（理想传感器，用于调试）
    sf::Odometry* odom = new sf::Odometry("Odom");

    // 压力传感器：测量深度
    sf::Pressure* press = new sf::Pressure("Pressure");
    press->setNoise(1.0);  // 设置噪声标准差1Pa

    // DVL多普勒速度仪：测量相对海底的速度和高度
    // DVL(名称, 波束角度, 是否垂直波束)
    sf::DVL* dvl = new sf::DVL("DVL", 30.0, false);
    // setNoise(速度偏置, 速度噪声, 速度百分比噪声, 高度偏置, 高度噪声)
    dvl->setNoise(0.0, 0.02, 0.05, 0.0, 0.02);

    // IMU惯性测量单元：测量加速度和角速度
    sf::IMU* imu = new sf::IMU("IMU");
    // setNoise(加速度偏置, 加速度噪声, 角速度偏置, 角速度噪声)
    imu->setNoise(sf::V0(), sf::Vector3(0.05, 0.05, 0.1), 0.0, sf::Vector3(0.01, 0.01, 0.02));

    // FOG光纤陀螺罗盘：测量航向角
    sf::Compass* fog = new sf::Compass("FOG");
    fog->setNoise(0.01);  // 噪声标准差0.01rad

    // GPS全球定位系统：提供经纬度（仅水面有效）
    sf::GPS* gps = new sf::GPS("GPS");
    gps->setNoise(0.5);  // 噪声标准差0.5m

    // 多波束声纳（已注释）
    //sf::Multibeam2* mb = new sf::Multibeam2("Multibeam", 1000, 300, 50.0, 40.0, 0.1, 10.0, 10.0);
    //mb->setDisplayOnScreen(true);

    // 深度相机（已注释）
    //sf::DepthCamera* dc = new sf::DepthCamera("DepthCam", 1000, 350, 50.0, 0.1, 10.0, 10.0);
    //dc->setDisplayOnScreen(true);

    // ---------- 视觉传感器 ----------

    // FLS前视声纳：用于前方障碍物探测
    // FLS(名称, 波束数, 距离bins, 水平视角, 垂直视角, 最小距离, 最大距离, 色图, 输出格式)
    sf::FLS* fls = new sf::FLS("FLS", 256, 500, 150.0, 30.0, 1.0, 20.0, sf::ColorMap::GREEN_BLUE, sf::SonarOutputFormat::U8);
    fls->setNoise(0.05, 0.05);  // 设置噪声
    fls->setDisplayOnScreen(true, 800, 250, 0.4f);  // 在屏幕显示，位置(800,250)，缩放0.4
    //fls->InstallNewDataHandler(std::bind(&UnderwaterTestManager::FLSDataCallback, this, std::placeholders::_1));

    // MSIS机械扫描成像声纳：360度扫描
    // MSIS(名称, 扫描速度rad/s, bins, 水平分辨率, 垂直视角, 起始角, 终止角, 最小距离, 最大距离, 色图, 输出格式)
    sf::MSIS* msis = new sf::MSIS("MSIS", 1.5, 500, 2.0, 30.0, 60, 120, 1.0, 100.0, sf::ColorMap::GREEN_BLUE, sf::SonarOutputFormat::U8);
    // 设置单向扫描模式
    msis->setUnidirectional(true);
    msis->setDisplayOnScreen(true, 880, 455, 0.6f);
    // msis->InstallNewDataHandler(std::bind(&UnderwaterTestManager::MSISDataCallback, this, std::placeholders::_1));

    // SSS侧扫声纳：用于海底测绘
    // SSS(名称, 线像素数, 显示高度, 水平视角, 垂直视角, 量程, 最小距离, 最大距离, 色图, 输出格式)
    sf::SSS* sss = new sf::SSS("SSS", 800, 400, 70.0, 1.5, 50.0, 1.0, 100.0, sf::ColorMap::GREEN_BLUE, sf::SonarOutputFormat::U8);
    sss->setDisplayOnScreen(true, 710, 5, 0.6f);
    // sss->InstallNewDataHandler(std::bind(&UnderwaterTestManager::SSSDataCallback, this, std::placeholders::_1));

    // ---------- Echo Sounder 测深声纳（水平安装，左右各一个） ----------
    // EchoSounder(名称, 波束宽度, 波束方向(true=+Z), 采样频率, 历史长度)
    // 左侧 Echo Sounder：波束宽度10°，使用+Z方向（安装时旋转使其指向左侧）
    sf::EchoSounder* echoLeft = new sf::EchoSounder("EchoSounderLeft", 10.0, true, 10.0, -1);
    echoLeft->setRange(0.5, 80.0);  // 量程 0.5m - 80m
    echoLeft->setNoise(0.02);       // 噪声标准差 0.02m

    // 右侧 Echo Sounder
    sf::EchoSounder* echoRight = new sf::EchoSounder("EchoSounderRight", 10.0, true, 10.0, -1);
    echoRight->setRange(0.5, 80.0);
    echoRight->setNoise(0.02);

    // 彩色相机（已注释）
    //sf::ColorCamera* cam = new sf::ColorCamera("Cam", 300, 200, 60.0, 10.0);
    //cam->setDisplayOnScreen(true);
    //sf::ColorCamera* cam2 = new sf::ColorCamera("Cam", 300, 200, 60.0);

    // ============================================================================
    // 第十一部分：机器人组装 (ROBOT ASSEMBLY)
    // ============================================================================
    // 使用Featherstone算法的多刚体动力学机器人
    // 将所有零件、关节、执行器、传感器组装成完整的AUV
    // ============================================================================

    // 创建机器人实例
    // FeatherstoneRobot(名称, 是否固定基座)
    // false表示基座可自由移动（水下机器人）
    sf::Robot* auv = new sf::FeatherstoneRobot("GIRONA500", false);

    // ---------- 定义机械结构 ----------
    // DefineLinks(基座链接, 其他链接数组)
    // vehicle是AUV本体，arm是机械臂连杆数组
    auv->DefineLinks(vehicle, arm);

    // 定义固定关节：将机械臂基座固定在AUV前方
    // DefineFixedJoint(关节名, 父链接, 子链接, 子相对父的位姿)
    auv->DefineFixedJoint("VehicleToArm", "Vehicle", "ArmBaseLink", sf::Transform(sf::IQ(), sf::Vector3(0.74,0.0,0.0)));

    // 定义旋转关节：机械臂的4个自由度
    // DefineRevoluteJoint(关节名, 父链接, 子链接, 位姿, 旋转轴, 角度限制(弧度))
    // 关节1：绕Z轴旋转（偏航）
    auv->DefineRevoluteJoint("Joint1", "ArmBaseLink", "ArmLink1", sf::I4(), sf::Vector3(0.0, 0.0, 1.0), std::make_pair(-1.0, 1.0));
    // 关节2：绕Y轴旋转（俯仰），偏移0.1065m
    auv->DefineRevoluteJoint("Joint2", "ArmLink1", "ArmLink2", sf::Transform(sf::IQ(), sf::Vector3(0.1065, 0.0, 0.0)), sf::Vector3(0.0, 1.0, 0.0), std::make_pair(-1.0, 1.0));
    // 关节3：绕Y轴旋转
    auv->DefineRevoluteJoint("Joint3", "ArmLink2", "ArmLink3", sf::Transform(sf::IQ(), sf::Vector3(0.23332, 0.0, 0.0)), sf::Vector3(0.0, 1.0, 0.0), std::make_pair(-1.0, 1.0));
    // 关节4：绕Z轴旋转（手腕旋转）
    auv->DefineRevoluteJoint("Joint4", "ArmLink3", "ArmLink4", sf::Transform(sf::IQ(), sf::Vector3(0.103, 0.0, 0.201)), sf::Vector3(0.0, 0.0, 1.0),  std::make_pair(-1.0, 1.0));
    // 末端执行器固定连接
    auv->DefineFixedJoint("Fix", "ArmLink4", "EE", sf::Transform(sf::IQ(), sf::Vector3(0.0, 0.0, 0.05)));
    // 夹爪手指关节
    auv->DefineRevoluteJoint("Joint5", "EE", "Finger1", sf::Transform(sf::IQ(), sf::Vector3(0.03,0,0.1)), sf::VY(), std::make_pair(0.0, 1.0));
    auv->DefineRevoluteJoint("Joint6", "EE", "Finger2", sf::Transform(sf::Quaternion(M_PI, 0.0, 0.0), sf::Vector3(-0.03,0,0.1)), sf::VY(), std::make_pair(0.0, 1.0));

    // 构建运动学结构（计算雅可比矩阵等）
    auv->BuildKinematicStructure();

    // ---------- 添加关节执行器 ----------
    // AddJointActuator(伺服电机, 关节名)
    auv->AddJointActuator(srv1, "Joint1");
    auv->AddJointActuator(srv2, "Joint2");
    auv->AddJointActuator(srv3, "Joint3");
    auv->AddJointActuator(srv4, "Joint4");
    auv->AddJointActuator(srv5, "Joint5");
    auv->AddJointActuator(srv6, "Joint6");

    // ---------- 添加推进器 ----------
    // AddLinkActuator(推进器, 链接名, 在链接上的位姿)
    // 推进器的位姿决定了推力方向
    // Sway推进器：横向，旋转90度使推力沿Y轴
    auv->AddLinkActuator(thrusters[0], "Vehicle", sf::Transform(sf::Quaternion(M_PI_2,M_PI,0), sf::Vector3(-0.0137, 0.0307, -0.38)));
    // Surge左舷推进器：向后推进
    auv->AddLinkActuator(thrusters[1], "Vehicle", sf::Transform(sf::IQ(), sf::Vector3(-0.2807,-0.2587,-0.38)));
    // Surge右舷推进器
    auv->AddLinkActuator(thrusters[2], "Vehicle", sf::Transform(sf::IQ(), sf::Vector3(-0.2807,0.2587,-0.38)));
    // Heave尾部推进器：向下推进
    auv->AddLinkActuator(thrusters[3], "Vehicle", sf::Transform(sf::Quaternion(0,-M_PI_2,0), sf::Vector3(-0.5337,0.0,-0.6747)));
    // Heave艏部推进器
    auv->AddLinkActuator(thrusters[4], "Vehicle", sf::Transform(sf::Quaternion(0,-M_PI_2,0), sf::Vector3(0.5837,0.0,-0.6747)));
    //auv->AddLinkActuator(vbs, "Vehicle", sf::Transform(sf::IQ(), sf::Vector3(-0.5,0.0,0.0)));

    // ---------- 添加传感器 ----------
    // AddLinkSensor(传感器, 链接名, 在链接上的位姿)
    // 里程计：位于车体中心
    auv->AddLinkSensor(odom, "Vehicle", sf::Transform(sf::IQ(), sf::Vector3(0,0,0)));
    // 压力传感器：位于车体前上方
    auv->AddLinkSensor(press, "Vehicle", sf::Transform(sf::IQ(), sf::Vector3(0.6,0,-0.7)));
    // DVL：位于车体后下方，旋转使波束向下
    auv->AddLinkSensor(dvl, "Vehicle", sf::Transform(sf::Quaternion(-M_PI_4,0,M_PI), sf::Vector3(-0.5,0,0.1)));
    // IMU：位于车体顶部
    auv->AddLinkSensor(imu, "Vehicle", sf::Transform(sf::IQ(), sf::Vector3(0,0,-0.7)));
    // FOG罗盘：位于车体前上方
    auv->AddLinkSensor(fog, "Vehicle", sf::Transform(sf::IQ(), sf::Vector3(0.3,0,-0.7)));
    // GPS：位于车体后顶部（需露出水面才有效）
    auv->AddLinkSensor(gps, "Vehicle", sf::Transform(sf::IQ(), sf::Vector3(-0.5,0,-0.9)));

    // 添加视觉传感器（FLS和SSS已注释）
    // auv->AddVisionSensor(fls, "Vehicle", sf::Transform(sf::Quaternion(1.57, 0.0, 0.8), sf::Vector3(0.0,0.0,1.0)));
    // auv->AddVisionSensor(sss, "Vehicle", sf::Transform(sf::Quaternion(1.57, 0.0, 0.0), sf::Vector3(0.0,0.0,0.0)));
    // MSIS声纳：位于车体下方，旋转使其水平扫描
    auv->AddVisionSensor(msis, "Vehicle", sf::Transform(sf::Quaternion(0.0, 0.0, 1.57), sf::Vector3(0.0,0.0,1.0)));

    // 添加水平安装的 Echo Sounder（左右各一个）
    // 使用轴角旋转来精确控制方向
    // 左侧：波束指向Y轴负方向（左舷）- 绕X轴旋转+90度
    sf::Quaternion rotLeft(sf::Vector3(1, 0, 0), M_PI_2);  // 绕X轴旋转+90度，使+Z轴指向-Y
    auv->AddLinkSensor(echoLeft, "Vehicle", sf::Transform(rotLeft, sf::Vector3(0.0, -0.4, -0.35)));
    // 右侧：波束指向Y轴正方向（右舷）- 绕X轴旋转-90度
    sf::Quaternion rotRight(sf::Vector3(1, 0, 0), -M_PI_2);  // 绕X轴旋转-90度，使+Z轴指向+Y
    auv->AddLinkSensor(echoRight, "Vehicle", sf::Transform(rotRight, sf::Vector3(0.0, 0.4, -0.35)));
    //auv->AddVisionSensor(cam, "Vehicle", sf::Transform(sf::Quaternion(1.57, 0.0, 1.57), sf::Vector3(0.0,0.0,1.0)));
    //auv->AddVisionSensor(cam2, "Vehicle", sf::Transform(sf::Quaternion(1.57, 0.0, 1.57), sf::Vector3(0.0,0.0,2.0)));

    // ============================================================================
    // 第十二部分：将机器人添加到仿真世界
    // ============================================================================
    // AddRobot(机器人, 初始位姿)
    // 位姿 = Transform(旋转四元数, 位置向量)
    // Quaternion(roll, pitch, yaw) - 这里yaw=-90度，使AUV初始朝向Y轴负方向
    // 位置(0,0,2)表示深度2米
    // ============================================================================
    AddRobot(auv, sf::Transform(sf::Quaternion(0, 0, -M_PI_2), sf::Vector3(0.0, 0.0, 2.0)));
#endif
} 

void UnderwaterTestManager::SimulationStepCompleted(sf::Scalar timeStep)
{
    // 实时打印 Echo Sounder 测量数据
    static sf::Scalar printTimer = 0;
    printTimer += timeStep;
    if(printTimer >= 0.5)  // 每0.5秒打印一次
    {
        printTimer = 0;
#ifndef PARSED_SCENARIO
        sf::EchoSounder* echoLeft = dynamic_cast<sf::EchoSounder*>(getRobot("GIRONA500")->getSensor("EchoSounderLeft"));
        sf::EchoSounder* echoRight = dynamic_cast<sf::EchoSounder*>(getRobot("GIRONA500")->getSensor("EchoSounderRight"));

        if(echoLeft && echoRight)
        {
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "[EchoSounder] Left: ";
            if(echoLeft->hasDetection())
                std::cout << echoLeft->getDepth() << "m";
            else
                std::cout << "---";
            std::cout << " | Right: ";
            if(echoRight->hasDetection())
                std::cout << echoRight->getDepth() << "m";
            else
                std::cout << "---";
            std::cout << std::endl;
        }
#endif
    }

    if(false)
    {
#ifdef PARSED_SCENARIO
        sf::Thruster* th = dynamic_cast<sf::Thruster*>(getRobot("GIRONA500")->getActuator("GIRONA500/ThrusterSurgePort"));
#else
        sf::Thruster* th = dynamic_cast<sf::Thruster*>(getRobot("GIRONA500")->getActuator("ThrusterSurgePort"));
#endif
        if (th)
        {
            double rpm = th->getOmega() * sf::Scalar(60.0) / sf::Scalar(2.0 * M_PI);
            std::cout << std::setprecision(3) << "[" << th->getName() << "] RPM: " << rpm << ", Thrust: " << th->getThrust() << std::endl;
        }
    }
}

void UnderwaterTestManager::FLSDataCallback(sf::FLS* fls)
{
    auto format = fls->getOutputFormat();
    switch (format)
    {
        case sf::SonarOutputFormat::U8:
            PrintData<GLubyte>(fls->getImageDataPointer(), 10);
            break;

        case sf::SonarOutputFormat::U16:
            PrintData<GLushort>(fls->getImageDataPointer(), 10);
            break;

        case sf::SonarOutputFormat::U32:
            PrintData<GLuint>(fls->getImageDataPointer(), 10);
            break;

        case sf::SonarOutputFormat::F32:
            PrintData<GLfloat>(fls->getImageDataPointer(), 10);
            break;
    }
    std::cout << std::endl;
}

void UnderwaterTestManager::MSISDataCallback(sf::MSIS* msis)
{
    auto format = msis->getOutputFormat();
    switch (format)
    {
        case sf::SonarOutputFormat::U8:
            PrintData<GLubyte>(msis->getImageDataPointer(), 1000);
            break;

        case sf::SonarOutputFormat::U16:
            PrintData<GLushort>(msis->getImageDataPointer(), 1000);
            break;

        case sf::SonarOutputFormat::U32:
            PrintData<GLuint>(msis->getImageDataPointer(), 1000);
            break;

        case sf::SonarOutputFormat::F32:
            PrintData<GLfloat>(msis->getImageDataPointer(), 1000);
            break;
    }
    std::cout << std::endl;
}

void UnderwaterTestManager::SSSDataCallback(sf::SSS* sss)
{
    auto format = sss->getOutputFormat();
    switch (format)
    {
        case sf::SonarOutputFormat::U8:
            PrintData<GLubyte>(sss->getImageDataPointer(), 100);
            break;

        case sf::SonarOutputFormat::U16:
            PrintData<GLushort>(sss->getImageDataPointer(), 100);
            break;

        case sf::SonarOutputFormat::U32:
            PrintData<GLuint>(sss->getImageDataPointer(), 100);
            break;

        case sf::SonarOutputFormat::F32:
            PrintData<GLfloat>(sss->getImageDataPointer(), 100);
            break;
    }
    std::cout << std::endl;
}
