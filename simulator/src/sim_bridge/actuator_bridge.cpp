#include "sim_bridge/actuator_bridge.hpp"
#include "sim_bridge/message_converter.hpp"
#include <nng_comm/nng_comm.hpp>
#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/actuators/Actuator.h>
#include <Stonefish/actuators/VariableBuoyancy.h>
#include <Stonefish/actuators/Thruster.h>
#include <Stonefish/actuators/Servo.h>
#include <iostream>
#include <algorithm>
#include <cmath>

namespace sim_bridge {

ActuatorBridge::ActuatorBridge(sf::SimulationManager* sim_manager)
    : sim_manager_(sim_manager),
      vbs_actuator_(nullptr),
      lcg_actuator_(nullptr),
      thruster1_actuator_(nullptr),
      thruster2_actuator_(nullptr),
      servo1_actuator_(nullptr),
      servo2_actuator_(nullptr),
      robot_name_("sam"),
      target_thruster1_rpm_(0.0),
      target_thruster2_rpm_(0.0),
      target_servo1_angle_(0.0),
      target_servo2_angle_(0.0),
      current_thruster1_rpm_(0.0),
      current_thruster2_rpm_(0.0),
      current_servo1_angle_(0.0),
      current_servo2_angle_(0.0) {
}

ActuatorBridge::~ActuatorBridge() {
    stop();
}

bool ActuatorBridge::init(const nlohmann::json& config) {
    if (!config.contains("actuators")) {
        std::cerr << "Network config missing 'actuators' section" << std::endl;
        return false;
    }

    auto actuators_cfg = config["actuators"];

    vbs_sub_ = std::make_unique<nng_comm::Subscriber>();
    if (!vbs_sub_->init(actuators_cfg["vbs_command"].get<std::string>())) {
        std::cerr << "Warning: Failed to initialize VBS subscriber" << std::endl;
        vbs_sub_.reset();
    } else {
        vbs_sub_->set_callback([this](const nlohmann::json& msg) { on_vbs_command(msg); });
    }

    lcg_sub_ = std::make_unique<nng_comm::Subscriber>();
    if (!lcg_sub_->init(actuators_cfg["lcg_command"].get<std::string>())) {
        std::cerr << "Failed to initialize LCG subscriber" << std::endl;
        return false;
    }
    lcg_sub_->set_callback([this](const nlohmann::json& msg) { on_lcg_command(msg); });

    thruster1_sub_ = std::make_unique<nng_comm::Subscriber>();
    if (!thruster1_sub_->init(actuators_cfg["thruster1_command"].get<std::string>())) {
        std::cerr << "Failed to initialize Thruster1 subscriber" << std::endl;
        return false;
    }
    thruster1_sub_->set_callback([this](const nlohmann::json& msg) { on_thruster1_command(msg); });

    thruster2_sub_ = std::make_unique<nng_comm::Subscriber>();
    if (!thruster2_sub_->init(actuators_cfg["thruster2_command"].get<std::string>())) {
        std::cerr << "Failed to initialize Thruster2 subscriber" << std::endl;
        return false;
    }
    thruster2_sub_->set_callback([this](const nlohmann::json& msg) { on_thruster2_command(msg); });

    thrust_angle_sub_ = std::make_unique<nng_comm::Subscriber>();
    if (!thrust_angle_sub_->init(actuators_cfg["thrust_angle_command"].get<std::string>())) {
        std::cerr << "Failed to initialize thrust angle subscriber" << std::endl;
        return false;
    }
    thrust_angle_sub_->set_callback([this](const nlohmann::json& msg) { on_thrust_angle_command(msg); });

    std::cout << "Actuator bridge initialized" << std::endl;
    return true;
}

void ActuatorBridge::start() {
    if (vbs_sub_) vbs_sub_->start_async();
    if (lcg_sub_) lcg_sub_->start_async();
    if (thruster1_sub_) thruster1_sub_->start_async();
    if (thruster2_sub_) thruster2_sub_->start_async();
    if (thrust_angle_sub_) thrust_angle_sub_->start_async();

    std::cout << "Actuator bridge started" << std::endl;
}

void ActuatorBridge::stop() {
    if (vbs_sub_) vbs_sub_->stop_async();
    if (lcg_sub_) lcg_sub_->stop_async();
    if (thruster1_sub_) thruster1_sub_->stop_async();
    if (thruster2_sub_) thruster2_sub_->stop_async();
    if (thrust_angle_sub_) thrust_angle_sub_->stop_async();
}

void ActuatorBridge::reset() {
    {
        std::lock_guard<std::mutex> lock(target_mutex_);
        target_thruster1_rpm_ = 0.0;
        target_thruster2_rpm_ = 0.0;
        target_servo1_angle_ = 0.0;
        target_servo2_angle_ = 0.0;
    }
    current_thruster1_rpm_ = 0.0;
    current_thruster2_rpm_ = 0.0;
    current_servo1_angle_ = 0.0;
    current_servo2_angle_ = 0.0;

    if (thruster1_actuator_) {
        sf::Thruster* thruster = dynamic_cast<sf::Thruster*>(thruster1_actuator_);
        if (thruster) thruster->setSetpoint(0.0);
    }
    if (thruster2_actuator_) {
        sf::Thruster* thruster = dynamic_cast<sf::Thruster*>(thruster2_actuator_);
        if (thruster) thruster->setSetpoint(0.0);
    }
    if (servo1_actuator_) {
        sf::Servo* servo = dynamic_cast<sf::Servo*>(servo1_actuator_);
        if (servo) servo->setDesiredPosition(0.0);
    }
    if (servo2_actuator_) {
        sf::Servo* servo = dynamic_cast<sf::Servo*>(servo2_actuator_);
        if (servo) servo->setDesiredPosition(0.0);
    }
    if (vbs_actuator_) {
        sf::VariableBuoyancy* vbs = dynamic_cast<sf::VariableBuoyancy*>(vbs_actuator_);
        if (vbs) vbs->setFlowRate(0.0);
    }
    if (lcg_actuator_) {
        sf::Servo* servo = dynamic_cast<sf::Servo*>(lcg_actuator_);
        if (servo) servo->setDesiredPosition(0.0);
    }

    std::cout << "ActuatorBridge: All actuators reset to zero" << std::endl;
}

sf::Actuator* ActuatorBridge::find_actuator(const std::string& name) {
    if (!sim_manager_) {
        return nullptr;
    }
    return sim_manager_->getActuator(name);
}

void ActuatorBridge::on_vbs_command(const nlohmann::json& msg) {
    try {
        double percentage = MessageConverter::vbs_command_from_json(msg);

        if (!vbs_actuator_ && sim_manager_) {
            vbs_actuator_ = find_actuator(robot_name_ + "/VBS");
            if (!vbs_actuator_) {
                vbs_actuator_ = find_actuator("VBS");
            }
            if (vbs_actuator_) {
                std::cout << "Found VBS actuator" << std::endl;
            }
        }

        if (vbs_actuator_) {
            sf::VariableBuoyancy* vbs = dynamic_cast<sf::VariableBuoyancy*>(vbs_actuator_);
            if (vbs) {
                double flow_rate = (percentage - 50.0) / 50.0 * 0.003;
                vbs->setFlowRate(flow_rate);
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "Error processing VBS command: " << e.what() << std::endl;
    }
}

void ActuatorBridge::on_lcg_command(const nlohmann::json& msg) {
    try {
        double percentage = MessageConverter::lcg_command_from_json(msg);

        if (!lcg_actuator_ && sim_manager_) {
            lcg_actuator_ = find_actuator(robot_name_ + "/ServoLCG");
            if (!lcg_actuator_) {
                lcg_actuator_ = find_actuator("ServoLCG");
            }
            if (lcg_actuator_) {
                std::cout << "Found LCG actuator" << std::endl;
            }
        }

        if (lcg_actuator_) {
            sf::Servo* servo = dynamic_cast<sf::Servo*>(lcg_actuator_);
            if (servo) {
                double position_m = (percentage / 100.0 - 0.5) * 0.02;
                servo->setDesiredPosition(position_m);
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "Error processing LCG command: " << e.what() << std::endl;
    }
}

void ActuatorBridge::on_thruster1_command(const nlohmann::json& msg) {
    try {
        double rpm = MessageConverter::thruster_command_from_json(msg);

        {
            std::lock_guard<std::mutex> lock(target_mutex_);
            target_thruster1_rpm_ = rpm;
        }

        if (!thruster1_actuator_ && sim_manager_) {
            thruster1_actuator_ = find_actuator(robot_name_ + "/ThrusterSurge1");
            if (!thruster1_actuator_) {
                thruster1_actuator_ = find_actuator("ThrusterSurge1");
            }
            if (thruster1_actuator_) {
                std::cout << "Found ThrusterSurge1 actuator" << std::endl;
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "Error processing Thruster1 command: " << e.what() << std::endl;
    }
}

void ActuatorBridge::on_thruster2_command(const nlohmann::json& msg) {
    try {
        double rpm = MessageConverter::thruster_command_from_json(msg);

        {
            std::lock_guard<std::mutex> lock(target_mutex_);
            target_thruster2_rpm_ = rpm;
        }

        if (!thruster2_actuator_ && sim_manager_) {
            thruster2_actuator_ = find_actuator(robot_name_ + "/ThrusterSurge2");
            if (!thruster2_actuator_) {
                thruster2_actuator_ = find_actuator("ThrusterSurge2");
            }
            if (thruster2_actuator_) {
                std::cout << "Found ThrusterSurge2 actuator" << std::endl;
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "Error processing Thruster2 command: " << e.what() << std::endl;
    }
}

void ActuatorBridge::on_thrust_angle_command(const nlohmann::json& msg) {
    try {
        auto [horizontal, vertical] = MessageConverter::thrust_angle_from_json(msg);

        {
            std::lock_guard<std::mutex> lock(target_mutex_);
            target_servo1_angle_ = std::clamp(horizontal, -0.12, 0.12);
            target_servo2_angle_ = std::clamp(vertical, -0.12, 0.12);
        }

        if (!servo1_actuator_ && sim_manager_) {
            servo1_actuator_ = find_actuator(robot_name_ + "/Servo1");
            if (!servo1_actuator_) {
                servo1_actuator_ = find_actuator("Servo1");
            }
            if (servo1_actuator_) {
                std::cout << "Found Servo1 actuator (yaw)" << std::endl;
            }
        }

        if (!servo2_actuator_ && sim_manager_) {
            servo2_actuator_ = find_actuator(robot_name_ + "/Servo2");
            if (!servo2_actuator_) {
                servo2_actuator_ = find_actuator("Servo2");
            }
            if (servo2_actuator_) {
                std::cout << "Found Servo2 actuator (pitch)" << std::endl;
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "Error processing thrust angle command: " << e.what() << std::endl;
    }
}

double ActuatorBridge::smooth_value(double current, double target, double rate, double dt) {
    if (current < target) {
        return std::min(current + rate * dt, target);
    } else if (current > target) {
        return std::max(current - rate * dt, target);
    }
    return current;
}

void ActuatorBridge::update(double dt) {
    double target_rpm1, target_rpm2, target_angle1, target_angle2;
    {
        std::lock_guard<std::mutex> lock(target_mutex_);
        target_rpm1 = target_thruster1_rpm_;
        target_rpm2 = target_thruster2_rpm_;
        target_angle1 = target_servo1_angle_;
        target_angle2 = target_servo2_angle_;
    }

    current_thruster1_rpm_ = smooth_value(current_thruster1_rpm_, target_rpm1, RPM_RAMP_RATE, dt);
    current_thruster2_rpm_ = smooth_value(current_thruster2_rpm_, target_rpm2, RPM_RAMP_RATE, dt);
    current_servo1_angle_ = smooth_value(current_servo1_angle_, target_angle1, ANGLE_RAMP_RATE, dt);
    current_servo2_angle_ = smooth_value(current_servo2_angle_, target_angle2, ANGLE_RAMP_RATE, dt);

    if (thruster1_actuator_) {
        sf::Thruster* thruster = dynamic_cast<sf::Thruster*>(thruster1_actuator_);
        if (thruster) {
            thruster->setSetpoint(current_thruster1_rpm_);
        }
    }

    if (thruster2_actuator_) {
        sf::Thruster* thruster = dynamic_cast<sf::Thruster*>(thruster2_actuator_);
        if (thruster) {
            thruster->setSetpoint(current_thruster2_rpm_);
        }
    }

    if (servo1_actuator_) {
        sf::Servo* servo = dynamic_cast<sf::Servo*>(servo1_actuator_);
        if (servo) {
            servo->setDesiredPosition(current_servo1_angle_);
        }
    }

    if (servo2_actuator_) {
        sf::Servo* servo = dynamic_cast<sf::Servo*>(servo2_actuator_);
        if (servo) {
            servo->setDesiredPosition(current_servo2_angle_);
        }
    }
}

} // namespace sim_bridge
