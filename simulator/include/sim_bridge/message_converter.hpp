#pragma once

#include <nlohmann/json.hpp>
#include <utility>

namespace sim_bridge {

class MessageConverter {
public:
    // Stonefish -> JSON conversions
    static nlohmann::json odometry_to_json(double timestamp,
                                           const double* position,
                                           const double* orientation,
                                           const double* linear_vel,
                                           const double* angular_vel);

    static nlohmann::json pressure_to_json(double timestamp, double pressure_pa);

    static nlohmann::json dvl_to_json(double timestamp,
                                      const double* velocity,
                                      double altitude,
                                      bool valid);

    static nlohmann::json imu_to_json(double timestamp,
                                      const double* angular_velocity,
                                      const double* linear_acceleration);

    static nlohmann::json gps_to_json(double timestamp,
                                      double latitude,
                                      double longitude,
                                      double altitude,
                                      int fix_type);

    // MSIS sonar data conversion
    static nlohmann::json msis_to_json(double timestamp,
                                        int current_step,
                                        double current_angle_deg,
                                        double range_min,
                                        double range_max,
                                        double rotation_min,
                                        double rotation_max,
                                        int num_bins,
                                        const uint8_t* beam_data,
                                        int data_size);

    // EchoSounder data conversion
    static nlohmann::json echosounder_to_json(double timestamp,
                                               double distance,
                                               bool detected);

    // JSON -> Stonefish command conversions
    static double vbs_command_from_json(const nlohmann::json& msg);
    static double lcg_command_from_json(const nlohmann::json& msg);
    static double thruster_command_from_json(const nlohmann::json& msg);
    static std::pair<double, double> thrust_angle_from_json(const nlohmann::json& msg);
};

} // namespace sim_bridge
