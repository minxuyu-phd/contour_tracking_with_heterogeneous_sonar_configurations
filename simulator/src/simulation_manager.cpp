#include "simulation_manager.hpp"
#include "sim_bridge/sensor_bridge.hpp"
#include "sim_bridge/actuator_bridge.hpp"
#include <Stonefish/entities/statics/Plane.h>
#include <Stonefish/sensors/Sensor.h>
#include <Stonefish/sensors/vision/MSIS.h>
#include <Stonefish/core/Robot.h>
#include <Stonefish/graphics/OpenGLTrackball.h>
#include <iostream>

SimulationManager::SimulationManager(sf::Scalar stepsPerSecond, const std::string& scenario_file)
    : sf::SimulationManager(stepsPerSecond, sf::Solver::LEMKE, sf::CollisionFilter::INCLUSIVE),
      scenario_file_(scenario_file) {
}

SimulationManager::~SimulationManager() {
}

void SimulationManager::BuildScenario() {
    std::cout << "Loading scenario from: " << scenario_file_ << std::endl;
    std::cout << "Data path: " << sf::GetDataPath() << std::endl;

    sf::ScenarioParser parser(this);
    std::string full_path = sf::GetDataPath() + scenario_file_;
    std::cout << "Full scenario path: " << full_path << std::endl;

    bool success = parser.Parse(full_path);

    if (!success) {
        std::cerr << "Failed to parse scenario file!" << std::endl;
        parser.SaveLog("scenario_parse.log");
        auto log = parser.getLog();
        for (const auto& msg : log) {
            std::cerr << msg.text << std::endl;
        }
    } else {
        std::cout << "Scenario loaded successfully" << std::endl;

        sf::OpenGLTrackball* trackball = getTrackball();
        if (trackball) {
            trackball->MoveCenter(glm::vec3(-130.0f, 0.0f, 15.0f));
            std::cout << "Camera view set to AUV position" << std::endl;
        }

        sf::Robot* robot = getRobot("sam");
        if (robot && !initial_pose_saved_) {
            initial_auv_pose_ = robot->getTransform();
            initial_pose_saved_ = true;
            std::cout << "SimulationManager: Saved initial AUV pose from scenario" << std::endl;
        }

        if (robot) {
            std::vector<std::string> sensors_to_hide = {
                "sam/dynamics",
                "sam/pressure_link",
                "sam/dvl_link",
                "sam/imu_link_ned",
                "sam/sbg_link_ned",
                "sam/gps_link"
            };

            for (const auto& name : sensors_to_hide) {
                sf::Sensor* sensor = getSensor(name);
                if (sensor) {
                    sensor->setRenderable(false);
                }
            }

            sf::Sensor* msis_sensor = getSensor("sam/msis_link");
            if (msis_sensor) {
                sf::MSIS* msis = dynamic_cast<sf::MSIS*>(msis_sensor);
                if (msis) {
                    msis->setUnidirectional(true);
                    std::cout << "MSIS set to unidirectional scan mode" << std::endl;
                }
            }
        }
    }
}

void SimulationManager::SimulationStepCompleted(sf::Scalar timeStep) {
    if (actuator_bridge_) {
        actuator_bridge_->update(timeStep);
    }

    if (sensor_bridge_ && ++publish_counter_ >= publish_prescaler_) {
        publish_counter_ = 0;
        sensor_bridge_->publish_all();
    }
}

void SimulationManager::setSensorBridge(sim_bridge::SensorBridge* bridge) {
    sensor_bridge_ = bridge;
}

void SimulationManager::setActuatorBridge(sim_bridge::ActuatorBridge* bridge) {
    actuator_bridge_ = bridge;
}

void SimulationManager::resetAUV() {
    sf::Robot* robot = getRobot("sam");
    if (!robot) {
        std::cerr << "SimulationManager: Cannot reset - robot 'sam' not found" << std::endl;
        return;
    }

    if (!initial_pose_saved_) {
        initial_auv_pose_ = robot->getTransform();
        initial_pose_saved_ = true;
        std::cerr << "SimulationManager: WARNING - initial pose not saved during BuildScenario, "
                  << "using current pose (may not be the true initial pose)" << std::endl;
    }

    robot->Respawn(this, initial_auv_pose_);

    if (actuator_bridge_) {
        actuator_bridge_->reset();
    }

    std::cout << "SimulationManager: AUV reset to initial pose" << std::endl;
}
