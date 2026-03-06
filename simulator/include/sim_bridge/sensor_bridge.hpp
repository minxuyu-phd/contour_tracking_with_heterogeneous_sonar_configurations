#pragma once

#include <nlohmann/json.hpp>
#include <memory>
#include <string>

// Forward declarations
namespace sf {
    class SimulationManager;
    class Sensor;
    class ScalarSensor;
    class Odometry;
    class Pressure;
    class DVL;
    class IMU;
    class GPS;
    class MSIS;
    class EchoSounder;
}

namespace nng_comm {
    class Publisher;
}

namespace sim_bridge {

class SensorBridge {
public:
    explicit SensorBridge(sf::SimulationManager* sim_manager);
    ~SensorBridge();

    // Initialize all sensor publishers
    bool init(const nlohmann::json& config);

    // Publish all sensor data (called in simulation loop)
    void publish_all();

    // Individual sensor publishers
    void publish_odometry();
    void publish_pressure();
    void publish_dvl();
    void publish_imu();
    void publish_gps();
    void publish_msis();
    void publish_echosounder_left();
    void publish_echosounder_right();

private:
    sf::SimulationManager* sim_manager_;

    // Publishers for each sensor
    std::unique_ptr<nng_comm::Publisher> odom_pub_;
    std::unique_ptr<nng_comm::Publisher> pressure_pub_;
    std::unique_ptr<nng_comm::Publisher> dvl_pub_;
    std::unique_ptr<nng_comm::Publisher> imu_pub_;
    std::unique_ptr<nng_comm::Publisher> gps_pub_;
    std::unique_ptr<nng_comm::Publisher> msis_pub_;
    std::unique_ptr<nng_comm::Publisher> echosounder_left_pub_;
    std::unique_ptr<nng_comm::Publisher> echosounder_right_pub_;

    // Sensor pointers
    sf::Odometry* odom_sensor_;
    sf::Pressure* pressure_sensor_;
    sf::DVL* dvl_sensor_;
    sf::IMU* imu_sensor_;
    sf::GPS* gps_sensor_;
    sf::MSIS* msis_sensor_;
    sf::EchoSounder* echosounder_left_sensor_;
    sf::EchoSounder* echosounder_right_sensor_;

    // Robot name for sensor lookup
    std::string robot_name_;

    // Initialization status
    bool initialized_ = false;

    // MSIS data buffer (captured via callback)
    std::vector<uint8_t> msis_data_buffer_;
    int msis_last_step_ = -999;
    double msis_last_angle_ = 0.0;
    bool msis_data_valid_ = false;
    bool msis_callback_installed_ = false;

    // Helper to find sensor
    sf::Sensor* find_sensor(const std::string& name);

    // MSIS callback handler
    void onMsisDataReady(sf::MSIS* msis);
};

} // namespace sim_bridge
