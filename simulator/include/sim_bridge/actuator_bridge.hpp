#pragma once

#include <nlohmann/json.hpp>
#include <memory>
#include <string>
#include <mutex>

namespace sf {
    class SimulationManager;
    class Actuator;
}

namespace nng_comm {
    class Subscriber;
}

namespace sim_bridge {

class ActuatorBridge {
public:
    explicit ActuatorBridge(sf::SimulationManager* sim_manager);
    ~ActuatorBridge();

    // Initialize all actuator subscribers
    bool init(const nlohmann::json& config);

    // Start async subscription threads
    void start();

    // Stop subscription threads
    void stop();

    // Reset all actuator states to zero
    void reset();

    // Update smoothed actuator values (call from simulation loop)
    void update(double dt);

private:
    sf::SimulationManager* sim_manager_;

    // Subscribers for each actuator
    std::unique_ptr<nng_comm::Subscriber> vbs_sub_;
    std::unique_ptr<nng_comm::Subscriber> lcg_sub_;
    std::unique_ptr<nng_comm::Subscriber> thruster1_sub_;
    std::unique_ptr<nng_comm::Subscriber> thruster2_sub_;
    std::unique_ptr<nng_comm::Subscriber> thrust_angle_sub_;

    // Actuator pointers
    sf::Actuator* vbs_actuator_;
    sf::Actuator* lcg_actuator_;
    sf::Actuator* thruster1_actuator_;
    sf::Actuator* thruster2_actuator_;
    sf::Actuator* servo1_actuator_;  // yaw
    sf::Actuator* servo2_actuator_;  // pitch

    // Robot name for actuator lookup
    std::string robot_name_;

    // Target values (set by callbacks)
    std::mutex target_mutex_;
    double target_thruster1_rpm_ = 0.0;
    double target_thruster2_rpm_ = 0.0;
    double target_servo1_angle_ = 0.0;
    double target_servo2_angle_ = 0.0;

    // Current smoothed values
    double current_thruster1_rpm_ = 0.0;
    double current_thruster2_rpm_ = 0.0;
    double current_servo1_angle_ = 0.0;
    double current_servo2_angle_ = 0.0;

    // Smoothing parameters (rates per second)
    static constexpr double RPM_RAMP_RATE = 600.0;
    static constexpr double ANGLE_RAMP_RATE = 0.5;

    // Subscription callbacks
    void on_vbs_command(const nlohmann::json& msg);
    void on_lcg_command(const nlohmann::json& msg);
    void on_thruster1_command(const nlohmann::json& msg);
    void on_thruster2_command(const nlohmann::json& msg);
    void on_thrust_angle_command(const nlohmann::json& msg);

    // Helper to find actuator
    sf::Actuator* find_actuator(const std::string& name);

    // Helper for smooth value transition
    static double smooth_value(double current, double target, double rate, double dt);
};

} // namespace sim_bridge
