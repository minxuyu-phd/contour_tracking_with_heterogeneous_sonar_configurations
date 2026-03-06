#include "sim_bridge/message_converter.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace sim_bridge;

void test_odometry_conversion() {
    std::cout << "\n=== Testing Odometry Conversion ===" << std::endl;

    double timestamp = 1234567.89;
    double position[3] = {1.0, 2.0, -5.0};
    double orientation[3] = {0.1, 0.2, 0.3};
    double linear_vel[3] = {0.5, 0.0, 0.0};
    double angular_vel[3] = {0.0, 0.0, 0.05};

    nlohmann::json msg = MessageConverter::odometry_to_json(
        timestamp, position, orientation, linear_vel, angular_vel);

    std::cout << "Odometry JSON: " << msg.dump(2) << std::endl;

    assert(msg["timestamp"] == timestamp);
    assert(msg["position"]["x"] == 1.0);
    assert(msg["position"]["y"] == 2.0);
    assert(msg["position"]["z"] == -5.0);
    assert(msg["orientation"]["roll"] == 0.1);
    assert(msg["orientation"]["pitch"] == 0.2);
    assert(msg["orientation"]["yaw"] == 0.3);

    std::cout << "Odometry conversion test PASSED" << std::endl;
}

void test_pressure_conversion() {
    std::cout << "\n=== Testing Pressure Conversion ===" << std::endl;

    double timestamp = 1234567.89;
    double pressure_pa = 151325.0;

    nlohmann::json msg = MessageConverter::pressure_to_json(timestamp, pressure_pa);

    std::cout << "Pressure JSON: " << msg.dump(2) << std::endl;

    assert(msg["timestamp"] == timestamp);
    assert(msg["pressure_pa"] == pressure_pa);

    std::cout << "Pressure conversion test PASSED" << std::endl;
}

void test_dvl_conversion() {
    std::cout << "\n=== Testing DVL Conversion ===" << std::endl;

    double timestamp = 1234567.89;
    double velocity[3] = {0.5, 0.1, -0.05};
    double altitude = 2.5;
    bool valid = true;

    nlohmann::json msg = MessageConverter::dvl_to_json(timestamp, velocity, altitude, valid);

    std::cout << "DVL JSON: " << msg.dump(2) << std::endl;

    assert(msg["timestamp"] == timestamp);
    assert(msg["velocity"]["x"] == 0.5);
    assert(msg["velocity"]["y"] == 0.1);
    assert(msg["velocity"]["z"] == -0.05);
    assert(msg["altitude"] == 2.5);
    assert(msg["valid"] == true);

    std::cout << "DVL conversion test PASSED" << std::endl;
}

void test_imu_conversion() {
    std::cout << "\n=== Testing IMU Conversion ===" << std::endl;

    double timestamp = 1234567.89;
    double angular_velocity[3] = {0.01, 0.02, 0.05};
    double linear_acceleration[3] = {0.1, 0.2, 9.81};

    nlohmann::json msg = MessageConverter::imu_to_json(
        timestamp, angular_velocity, linear_acceleration);

    std::cout << "IMU JSON: " << msg.dump(2) << std::endl;

    assert(msg["timestamp"] == timestamp);
    assert(msg["angular_velocity"]["x"] == 0.01);
    assert(std::abs(msg["linear_acceleration"]["z"].get<double>() - 9.81) < 0.001);

    std::cout << "IMU conversion test PASSED" << std::endl;
}

void test_gps_conversion() {
    std::cout << "\n=== Testing GPS Conversion ===" << std::endl;

    double timestamp = 1234567.89;
    double latitude = 43.93183;
    double longitude = 15.44264;
    double altitude = 0.0;
    int fix_type = 3;

    nlohmann::json msg = MessageConverter::gps_to_json(
        timestamp, latitude, longitude, altitude, fix_type);

    std::cout << "GPS JSON: " << msg.dump(2) << std::endl;

    assert(msg["timestamp"] == timestamp);
    assert(std::abs(msg["latitude"].get<double>() - 43.93183) < 0.00001);
    assert(std::abs(msg["longitude"].get<double>() - 15.44264) < 0.00001);
    assert(msg["fix_type"] == 3);

    std::cout << "GPS conversion test PASSED" << std::endl;
}

void test_vbs_command() {
    std::cout << "\n=== Testing VBS Command Conversion ===" << std::endl;

    nlohmann::json cmd = {
        {"timestamp", 1234567.89},
        {"percentage", 75.5}
    };

    double percentage = MessageConverter::vbs_command_from_json(cmd);
    assert(percentage == 75.5);

    std::cout << "VBS command conversion test PASSED" << std::endl;
}

void test_thruster_command() {
    std::cout << "\n=== Testing Thruster Command Conversion ===" << std::endl;

    nlohmann::json cmd = {
        {"timestamp", 1234567.89},
        {"rpm", 500.0}
    };

    double rpm = MessageConverter::thruster_command_from_json(cmd);
    assert(rpm == 500.0);

    std::cout << "Thruster command conversion test PASSED" << std::endl;
}

void test_thrust_angle_command() {
    std::cout << "\n=== Testing Thrust Angle Command Conversion ===" << std::endl;

    nlohmann::json cmd = {
        {"timestamp", 1234567.89},
        {"horizontal_radians", 0.1},
        {"vertical_radians", -0.05}
    };

    auto [horizontal, vertical] = MessageConverter::thrust_angle_from_json(cmd);
    assert(horizontal == 0.1);
    assert(vertical == -0.05);

    std::cout << "Thrust angle command conversion test PASSED" << std::endl;
}

int main() {
    std::cout << "=== Message Converter Tests ===" << std::endl;

    try {
        test_odometry_conversion();
        test_pressure_conversion();
        test_dvl_conversion();
        test_imu_conversion();
        test_gps_conversion();
        test_vbs_command();
        test_thruster_command();
        test_thrust_angle_command();

        std::cout << "\n=== All tests PASSED ===" << std::endl;
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
}
