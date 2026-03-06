#include "sim_bridge/message_converter.hpp"
#include <stdexcept>

namespace sim_bridge {

nlohmann::json MessageConverter::odometry_to_json(double timestamp,
                                                   const double* position,
                                                   const double* orientation,
                                                   const double* linear_vel,
                                                   const double* angular_vel) {
    nlohmann::json msg;
    msg["timestamp"] = timestamp;

    msg["position"] = {
        {"x", position[0]},
        {"y", position[1]},
        {"z", position[2]}
    };

    msg["orientation"] = {
        {"roll", orientation[0]},
        {"pitch", orientation[1]},
        {"yaw", orientation[2]}
    };

    msg["linear_velocity"] = {
        {"x", linear_vel[0]},
        {"y", linear_vel[1]},
        {"z", linear_vel[2]}
    };

    msg["angular_velocity"] = {
        {"x", angular_vel[0]},
        {"y", angular_vel[1]},
        {"z", angular_vel[2]}
    };

    return msg;
}

nlohmann::json MessageConverter::pressure_to_json(double timestamp, double pressure_pa) {
    nlohmann::json msg;
    msg["timestamp"] = timestamp;
    msg["pressure_pa"] = pressure_pa;
    return msg;
}

nlohmann::json MessageConverter::dvl_to_json(double timestamp,
                                              const double* velocity,
                                              double altitude,
                                              bool valid) {
    nlohmann::json msg;
    msg["timestamp"] = timestamp;

    msg["velocity"] = {
        {"x", velocity[0]},
        {"y", velocity[1]},
        {"z", velocity[2]}
    };

    msg["altitude"] = altitude;
    msg["valid"] = valid;

    return msg;
}

nlohmann::json MessageConverter::imu_to_json(double timestamp,
                                              const double* angular_velocity,
                                              const double* linear_acceleration) {
    nlohmann::json msg;
    msg["timestamp"] = timestamp;

    msg["angular_velocity"] = {
        {"x", angular_velocity[0]},
        {"y", angular_velocity[1]},
        {"z", angular_velocity[2]}
    };

    msg["linear_acceleration"] = {
        {"x", linear_acceleration[0]},
        {"y", linear_acceleration[1]},
        {"z", linear_acceleration[2]}
    };

    return msg;
}

nlohmann::json MessageConverter::gps_to_json(double timestamp,
                                              double latitude,
                                              double longitude,
                                              double altitude,
                                              int fix_type) {
    nlohmann::json msg;
    msg["timestamp"] = timestamp;
    msg["latitude"] = latitude;
    msg["longitude"] = longitude;
    msg["altitude"] = altitude;
    msg["fix_type"] = fix_type;
    return msg;
}

nlohmann::json MessageConverter::msis_to_json(double timestamp,
                                               int current_step,
                                               double current_angle_deg,
                                               double range_min,
                                               double range_max,
                                               double rotation_min,
                                               double rotation_max,
                                               int num_bins,
                                               const uint8_t* beam_data,
                                               int data_size) {
    nlohmann::json msg;
    msg["timestamp"] = timestamp;
    msg["current_step"] = current_step;
    msg["current_angle_deg"] = current_angle_deg;
    msg["range_min"] = range_min;
    msg["range_max"] = range_max;
    msg["rotation_min"] = rotation_min;
    msg["rotation_max"] = rotation_max;
    msg["num_bins"] = num_bins;

    if (beam_data != nullptr && data_size > 0) {
        std::vector<uint8_t> data_vec(beam_data, beam_data + data_size);
        msg["beam_data"] = nlohmann::json::binary(data_vec);
    } else {
        msg["beam_data"] = nullptr;
    }

    return msg;
}

nlohmann::json MessageConverter::echosounder_to_json(double timestamp,
                                                       double distance,
                                                       bool detected) {
    nlohmann::json msg;
    msg["timestamp"] = timestamp;
    msg["distance"] = distance;
    msg["detected"] = detected;
    return msg;
}

double MessageConverter::vbs_command_from_json(const nlohmann::json& msg) {
    if (!msg.contains("percentage")) {
        throw std::runtime_error("VBS command missing 'percentage' field");
    }
    return msg["percentage"].get<double>();
}

double MessageConverter::lcg_command_from_json(const nlohmann::json& msg) {
    if (!msg.contains("percentage")) {
        throw std::runtime_error("LCG command missing 'percentage' field");
    }
    return msg["percentage"].get<double>();
}

double MessageConverter::thruster_command_from_json(const nlohmann::json& msg) {
    if (!msg.contains("rpm")) {
        throw std::runtime_error("Thruster command missing 'rpm' field");
    }
    return msg["rpm"].get<double>();
}

std::pair<double, double> MessageConverter::thrust_angle_from_json(const nlohmann::json& msg) {
    if (!msg.contains("horizontal_radians") || !msg.contains("vertical_radians")) {
        throw std::runtime_error("Thrust angle command missing angle fields");
    }
    double horizontal = msg["horizontal_radians"].get<double>();
    double vertical = msg["vertical_radians"].get<double>();
    return {horizontal, vertical};
}

} // namespace sim_bridge
