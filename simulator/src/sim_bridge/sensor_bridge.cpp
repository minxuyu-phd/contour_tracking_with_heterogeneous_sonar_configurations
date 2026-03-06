#include "sim_bridge/sensor_bridge.hpp"
#include "sim_bridge/message_converter.hpp"
#include <nng_comm/nng_comm.hpp>
#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/sensors/Sample.h>
#include <Stonefish/sensors/scalar/Odometry.h>
#include <Stonefish/sensors/scalar/Pressure.h>
#include <Stonefish/sensors/scalar/DVL.h>
#include <Stonefish/sensors/scalar/IMU.h>
#include <Stonefish/sensors/scalar/GPS.h>
#include <Stonefish/sensors/vision/MSIS.h>
#include <Stonefish/graphics/OpenGLSonar.h>
#include <Stonefish/sensors/scalar/EchoSounder.h>
#include <iostream>
#include <cmath>
#include <functional>

namespace sim_bridge {

SensorBridge::SensorBridge(sf::SimulationManager* sim_manager)
    : sim_manager_(sim_manager),
      odom_sensor_(nullptr),
      pressure_sensor_(nullptr),
      dvl_sensor_(nullptr),
      imu_sensor_(nullptr),
      gps_sensor_(nullptr),
      msis_sensor_(nullptr),
      echosounder_left_sensor_(nullptr),
      echosounder_right_sensor_(nullptr),
      robot_name_("sam") {
}

SensorBridge::~SensorBridge() = default;

bool SensorBridge::init(const nlohmann::json& config) {
    if (!config.contains("sensors")) {
        std::cerr << "Network config missing 'sensors' section" << std::endl;
        return false;
    }

    auto sensors_cfg = config["sensors"];
    bool all_success = true;

    auto init_publisher = [&](std::unique_ptr<nng_comm::Publisher>& pub, const std::string& key) {
        pub = std::make_unique<nng_comm::Publisher>();
        if (!pub->init(sensors_cfg[key].get<std::string>())) {
            std::cerr << "Failed to initialize " << key << " publisher" << std::endl;
            pub.reset();
            all_success = false;
        }
    };

    init_publisher(odom_pub_, "odometry");
    init_publisher(pressure_pub_, "pressure");
    init_publisher(dvl_pub_, "dvl");
    init_publisher(imu_pub_, "imu");
    init_publisher(gps_pub_, "gps");
    init_publisher(msis_pub_, "msis");
    init_publisher(echosounder_left_pub_, "echosounder_left");
    init_publisher(echosounder_right_pub_, "echosounder_right");

    initialized_ = all_success;

    if (all_success) {
        std::cout << "Sensor bridge initialized successfully" << std::endl;
    } else {
        std::cerr << "Sensor bridge initialized with errors (some publishers failed)" << std::endl;
    }

    return all_success;
}

sf::Sensor* SensorBridge::find_sensor(const std::string& name) {
    if (!sim_manager_) {
        return nullptr;
    }
    return sim_manager_->getSensor(name);
}

void SensorBridge::publish_all() {
    publish_odometry();
    publish_pressure();
    publish_dvl();
    publish_imu();
    publish_gps();
    publish_msis();
    publish_echosounder_left();
    publish_echosounder_right();
}

void SensorBridge::publish_odometry() {
    if (!odom_pub_) return;

    if (!odom_sensor_ && sim_manager_) {
        odom_sensor_ = dynamic_cast<sf::Odometry*>(find_sensor(robot_name_ + "/dynamics"));
        if (!odom_sensor_) {
            odom_sensor_ = dynamic_cast<sf::Odometry*>(find_sensor("dynamics"));
        }
    }

    double timestamp = 0.0;
    double position[3] = {0.0, 0.0, 0.0};
    double orientation[3] = {0.0, 0.0, 0.0};
    double linear_vel[3] = {0.0, 0.0, 0.0};
    double angular_vel[3] = {0.0, 0.0, 0.0};

    if (odom_sensor_ && sim_manager_) {
        timestamp = sim_manager_->getSimulationTime();

        sf::Sample sample = odom_sensor_->getLastSample();

        if (sample.getNumOfDimensions() >= 13) {
            position[0] = sample.getValue(0);
            position[1] = sample.getValue(1);
            position[2] = sample.getValue(2);

            linear_vel[0] = sample.getValue(3);
            linear_vel[1] = sample.getValue(4);
            linear_vel[2] = sample.getValue(5);

            double qx = sample.getValue(6);
            double qy = sample.getValue(7);
            double qz = sample.getValue(8);
            double qw = sample.getValue(9);

            double sinr_cosp = 2.0 * (qw * qx + qy * qz);
            double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
            orientation[0] = std::atan2(sinr_cosp, cosr_cosp);

            double sinp = 2.0 * (qw * qy - qz * qx);
            if (std::abs(sinp) >= 1.0)
                orientation[1] = std::copysign(M_PI / 2, sinp);
            else
                orientation[1] = std::asin(sinp);

            double siny_cosp = 2.0 * (qw * qz + qx * qy);
            double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
            orientation[2] = std::atan2(siny_cosp, cosy_cosp);

            angular_vel[0] = sample.getValue(10);
            angular_vel[1] = sample.getValue(11);
            angular_vel[2] = sample.getValue(12);
        }
    }

    nlohmann::json msg = MessageConverter::odometry_to_json(
        timestamp, position, orientation, linear_vel, angular_vel);

    odom_pub_->publish(msg);
}

void SensorBridge::publish_pressure() {
    if (!pressure_pub_) return;

    if (!pressure_sensor_ && sim_manager_) {
        pressure_sensor_ = dynamic_cast<sf::Pressure*>(find_sensor(robot_name_ + "/pressure_link"));
        if (!pressure_sensor_) {
            pressure_sensor_ = dynamic_cast<sf::Pressure*>(find_sensor("pressure"));
        }
    }

    double timestamp = 0.0;
    double pressure_pa = 101325.0;

    if (pressure_sensor_ && sim_manager_) {
        timestamp = sim_manager_->getSimulationTime();

        sf::Sample sample = pressure_sensor_->getLastSample();
        if (sample.getNumOfDimensions() >= 1) {
            pressure_pa = sample.getValue(0);
        }
    }

    nlohmann::json msg = MessageConverter::pressure_to_json(timestamp, pressure_pa);
    pressure_pub_->publish(msg);
}

void SensorBridge::publish_dvl() {
    if (!dvl_pub_) return;

    if (!dvl_sensor_ && sim_manager_) {
        dvl_sensor_ = dynamic_cast<sf::DVL*>(find_sensor(robot_name_ + "/dvl_link"));
        if (!dvl_sensor_) {
            dvl_sensor_ = dynamic_cast<sf::DVL*>(find_sensor("dvl"));
        }
    }

    double timestamp = 0.0;
    double velocity[3] = {0.0, 0.0, 0.0};
    double altitude = 0.0;
    bool valid = false;

    if (dvl_sensor_ && sim_manager_) {
        timestamp = sim_manager_->getSimulationTime();

        sf::Sample sample = dvl_sensor_->getLastSample();
        if (sample.getNumOfDimensions() >= 4) {
            velocity[0] = sample.getValue(0);
            velocity[1] = sample.getValue(1);
            velocity[2] = sample.getValue(2);
            altitude = sample.getValue(3);
            valid = (altitude > 0.0);
        }
    }

    nlohmann::json msg = MessageConverter::dvl_to_json(timestamp, velocity, altitude, valid);
    dvl_pub_->publish(msg);
}

void SensorBridge::publish_imu() {
    if (!imu_pub_) return;

    if (!imu_sensor_ && sim_manager_) {
        imu_sensor_ = dynamic_cast<sf::IMU*>(find_sensor(robot_name_ + "/imu_link_ned"));
        if (!imu_sensor_) {
            imu_sensor_ = dynamic_cast<sf::IMU*>(find_sensor("imu"));
        }
    }

    double timestamp = 0.0;
    double angular_velocity[3] = {0.0, 0.0, 0.0};
    double linear_acceleration[3] = {0.0, 0.0, 9.81};

    if (imu_sensor_ && sim_manager_) {
        timestamp = sim_manager_->getSimulationTime();

        sf::Sample sample = imu_sensor_->getLastSample();
        if (sample.getNumOfDimensions() >= 6) {
            angular_velocity[0] = sample.getValue(0);
            angular_velocity[1] = sample.getValue(1);
            angular_velocity[2] = sample.getValue(2);
            linear_acceleration[0] = sample.getValue(3);
            linear_acceleration[1] = sample.getValue(4);
            linear_acceleration[2] = sample.getValue(5);
        }
    }

    nlohmann::json msg = MessageConverter::imu_to_json(
        timestamp, angular_velocity, linear_acceleration);
    imu_pub_->publish(msg);
}

void SensorBridge::publish_gps() {
    if (!gps_pub_) return;

    if (!gps_sensor_ && sim_manager_) {
        gps_sensor_ = dynamic_cast<sf::GPS*>(find_sensor(robot_name_ + "/gps_link"));
        if (!gps_sensor_) {
            gps_sensor_ = dynamic_cast<sf::GPS*>(find_sensor("gps"));
        }
    }

    double timestamp = 0.0;
    double latitude = 43.93183;
    double longitude = 15.44264;
    double altitude = 0.0;
    int fix_type = 0;

    if (gps_sensor_ && sim_manager_) {
        timestamp = sim_manager_->getSimulationTime();

        sf::Sample sample = gps_sensor_->getLastSample();
        if (sample.getNumOfDimensions() >= 3) {
            latitude = sample.getValue(0);
            longitude = sample.getValue(1);
            altitude = sample.getValue(2);
            fix_type = 3;

            if (sample.getNumOfDimensions() >= 4) {
                fix_type = static_cast<int>(sample.getValue(3));
            }
        }
    }

    nlohmann::json msg = MessageConverter::gps_to_json(
        timestamp, latitude, longitude, altitude, fix_type);
    gps_pub_->publish(msg);
}

void SensorBridge::onMsisDataReady(sf::MSIS* msis) {
    if (!msis) return;

    unsigned int resX, resY;
    msis->getResolution(resX, resY);
    int num_bins = static_cast<int>(resY);
    int num_steps = static_cast<int>(resX);

    sf::SonarOutputFormat format = msis->getOutputFormat();
    unsigned int beam_index = msis->getCurrentBeamIndex();

    void* data_ptr = msis->getImageDataPointer();
    if (data_ptr != nullptr && num_bins > 0 && beam_index < static_cast<unsigned int>(num_steps)) {
        msis_last_step_ = msis->getCurrentRotationStep();
        msis_last_angle_ = msis_last_step_ * msis->getRotationStepAngle();

        msis_data_buffer_.resize(num_bins);

        if (format == sf::SonarOutputFormat::U8) {
            const uint8_t* full_data = static_cast<const uint8_t*>(data_ptr);
            for (int bin = 0; bin < num_bins; ++bin) {
                msis_data_buffer_[bin] = full_data[bin * num_steps + beam_index];
            }
        } else if (format == sf::SonarOutputFormat::F32) {
            const float* full_data = static_cast<const float*>(data_ptr);
            for (int bin = 0; bin < num_bins; ++bin) {
                float val = full_data[bin * num_steps + beam_index] * 255.0f;
                msis_data_buffer_[bin] = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, val)));
            }
        } else if (format == sf::SonarOutputFormat::U16) {
            const uint16_t* full_data = static_cast<const uint16_t*>(data_ptr);
            for (int bin = 0; bin < num_bins; ++bin) {
                msis_data_buffer_[bin] = static_cast<uint8_t>(full_data[bin * num_steps + beam_index] >> 8);
            }
        } else {  // U32
            const uint32_t* full_data = static_cast<const uint32_t*>(data_ptr);
            for (int bin = 0; bin < num_bins; ++bin) {
                msis_data_buffer_[bin] = static_cast<uint8_t>(full_data[bin * num_steps + beam_index] >> 24);
            }
        }

        msis_data_valid_ = true;
    }
}

void SensorBridge::publish_msis() {
    if (!msis_pub_) return;

    if (!msis_sensor_ && sim_manager_) {
        msis_sensor_ = dynamic_cast<sf::MSIS*>(find_sensor(robot_name_ + "/msis_link"));
        if (!msis_sensor_) {
            msis_sensor_ = dynamic_cast<sf::MSIS*>(find_sensor("msis_link"));
        }
    }

    if (!msis_sensor_ || !sim_manager_) return;

    if (!msis_callback_installed_) {
        msis_sensor_->InstallNewDataHandler(
            std::bind(&SensorBridge::onMsisDataReady, this, std::placeholders::_1));
        msis_callback_installed_ = true;
    }

    double timestamp = sim_manager_->getSimulationTime();

    int current_step = msis_sensor_->getCurrentRotationStep();
    double step_angle = msis_sensor_->getRotationStepAngle();
    double current_angle_deg = current_step * step_angle;
    double range_min = msis_sensor_->getRangeMin();
    double range_max = msis_sensor_->getRangeMax();

    sf::Scalar rot_min, rot_max;
    msis_sensor_->getRotationLimits(rot_min, rot_max);

    unsigned int resX, resY;
    msis_sensor_->getResolution(resX, resY);
    int num_bins = static_cast<int>(resY);

    const uint8_t* beam_data = nullptr;
    int data_size = 0;

    if (msis_data_valid_ && !msis_data_buffer_.empty()) {
        beam_data = msis_data_buffer_.data();
        data_size = static_cast<int>(msis_data_buffer_.size());
        current_step = msis_last_step_;
        current_angle_deg = msis_last_angle_;
    }

    nlohmann::json msg = MessageConverter::msis_to_json(
        timestamp, current_step, current_angle_deg, range_min, range_max, rot_min, rot_max, num_bins, beam_data, data_size);
    msis_pub_->publish(msg);
}

void SensorBridge::publish_echosounder_left() {
    if (!echosounder_left_pub_) return;

    if (!echosounder_left_sensor_ && sim_manager_) {
        echosounder_left_sensor_ = dynamic_cast<sf::EchoSounder*>(find_sensor(robot_name_ + "/echosounder_left_link"));
        if (!echosounder_left_sensor_) {
            echosounder_left_sensor_ = dynamic_cast<sf::EchoSounder*>(find_sensor("echosounder_left_link"));
        }
    }

    double timestamp = 0.0;
    double distance = 0.0;
    bool detected = false;

    if (echosounder_left_sensor_ && sim_manager_) {
        timestamp = sim_manager_->getSimulationTime();
        distance = echosounder_left_sensor_->getDepth();
        detected = echosounder_left_sensor_->hasDetection();
    }

    nlohmann::json msg = MessageConverter::echosounder_to_json(timestamp, distance, detected);
    echosounder_left_pub_->publish(msg);
}

void SensorBridge::publish_echosounder_right() {
    if (!echosounder_right_pub_) return;

    if (!echosounder_right_sensor_ && sim_manager_) {
        echosounder_right_sensor_ = dynamic_cast<sf::EchoSounder*>(find_sensor(robot_name_ + "/echosounder_right_link"));
        if (!echosounder_right_sensor_) {
            echosounder_right_sensor_ = dynamic_cast<sf::EchoSounder*>(find_sensor("echosounder_right_link"));
        }
    }

    double timestamp = 0.0;
    double distance = 0.0;
    bool detected = false;

    if (echosounder_right_sensor_ && sim_manager_) {
        timestamp = sim_manager_->getSimulationTime();
        distance = echosounder_right_sensor_->getDepth();
        detected = echosounder_right_sensor_->hasDetection();
    }

    nlohmann::json msg = MessageConverter::echosounder_to_json(timestamp, distance, detected);
    echosounder_right_pub_->publish(msg);
}

} // namespace sim_bridge
