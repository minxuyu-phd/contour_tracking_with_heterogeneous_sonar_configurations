#ifndef SIMULATION_MANAGER_HPP
#define SIMULATION_MANAGER_HPP

#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/core/ScenarioParser.h>
#include <Stonefish/utils/SystemUtil.hpp>
#include <nlohmann/json.hpp>
#include <string>

namespace sim_bridge {
    class SensorBridge;
    class ActuatorBridge;
}

class SimulationManager : public sf::SimulationManager {
public:
    SimulationManager(sf::Scalar stepsPerSecond, const std::string& scenario_file);
    virtual ~SimulationManager();

    void BuildScenario() override;
    void SimulationStepCompleted(sf::Scalar timeStep) override;

    void setSensorBridge(sim_bridge::SensorBridge* bridge);
    void setActuatorBridge(sim_bridge::ActuatorBridge* bridge);

    void resetAUV();

private:
    std::string scenario_file_;
    sim_bridge::SensorBridge* sensor_bridge_ = nullptr;
    sim_bridge::ActuatorBridge* actuator_bridge_ = nullptr;
    unsigned int publish_counter_ = 0;
    unsigned int publish_prescaler_ = 10;  // Publish every 10 steps (300Hz -> 30Hz)

    sf::Transform initial_auv_pose_;
    bool initial_pose_saved_ = false;
};

#endif // SIMULATION_MANAGER_HPP
