// Copyright 2024 Michael's ROS2 Development Guide
// Licensed under Apache License 2.0

#ifndef BASIC_PUB_SUB__SENSOR_FUSION_CORE_HPP_
#define BASIC_PUB_SUB__SENSOR_FUSION_CORE_HPP_

#include <vector>
#include <array>
#include <cmath>
#include <mutex>
#include <memory>

namespace basic_pub_sub {

/**
 * @struct IMUData
 * @brief IMU sensor data structure
 */
struct IMUData {
    double timestamp;  // Seconds since epoch
    std::array<double, 3> linear_acceleration;  // m/s²
    std::array<double, 3> angular_velocity;     // rad/s
    std::array<double, 4> orientation;          // Quaternion (x,y,z,w)
};

/**
 * @struct MagData
 * @brief Magnetometer sensor data structure
 */
struct MagData {
    double timestamp;  // Seconds since epoch
    std::array<double, 3> magnetic_field;  // μT (microtesla)
};

/**
 * @struct WheelData
 * @brief Wheel encoder data structure
 */
struct WheelData {
    double timestamp;  // Seconds since epoch
    double linear_velocity;   // m/s
    double angular_velocity;  // rad/s
};

/**
 * @struct FusedState
 * @brief Fused state estimate
 */
struct FusedState {
    bool valid = false;
    double timestamp;
    
    std::array<double, 3> position;          // meters
    std::array<double, 4> orientation;       // Quaternion (x,y,z,w)
    std::array<double, 3> linear_velocity;   // m/s
    std::array<double, 3> angular_velocity;  // rad/s
    
    std::array<double, 6> position_covariance;    // Diagonal only
    std::array<double, 6> orientation_covariance; // Diagonal only
};

/**
 * @class SensorFusionCore
 * @brief Core sensor fusion algorithm (ROS-agnostic)
 * 
 * This class implements a simplified Kalman filter for sensor fusion.
 * In a real application, you would use a more sophisticated filter
 * like an Extended Kalman Filter (EKF) or Unscented Kalman Filter (UKF).
 */
class SensorFusionCore {
public:
    SensorFusionCore();
    
    /**
     * @brief Update IMU data
     * @param data IMU data to process
     */
    void update_imu_data(const IMUData& data);
    
    /**
     * @brief Update magnetometer data
     * @param data Magnetometer data to process
     */
    void update_mag_data(const MagData& data);
    
    /**
     * @brief Update wheel encoder data
     * @param data Wheel data to process
     */
    void update_wheel_data(const WheelData& data);
    
    /**
     * @brief Get current fused state estimate
     * @return Current state estimate
     */
    FusedState get_fused_state();
    
    /**
     * @brief Configure sensor fusion parameters
     * @param enable_mag Enable magnetometer fusion
     * @param enable_wheel Enable wheel encoder fusion
     * @param imu_trust IMU trust factor (0-1)
     * @param wheel_trust Wheel trust factor (0-1)
     * @param mag_trust Magnetometer trust factor (0-1)
     */
    void set_sensor_config(bool enable_mag, bool enable_wheel,
                          double imu_trust, double wheel_trust, double mag_trust);
    
    /**
     * @brief Reset fusion filter
     */
    void reset();

private:
    // State variables
    FusedState current_state_;
    
    // Sensor configuration
    bool enable_mag_;
    bool enable_wheel_;
    double imu_trust_;
    double wheel_trust_;
    double mag_trust_;
    
    // Timing
    double last_fusion_time_;
    
    // Sensor data buffers
    IMUData last_imu_data_;
    MagData last_mag_data_;
    WheelData last_wheel_data_;
    
    // Filter parameters
    double process_noise_;
    double measurement_noise_;
    
    // Mutex for thread safety
    std::mutex state_mutex_;
    
    // Helper methods
    void predict_state(double dt);
    void update_with_imu();
    void update_with_mag();
    void update_with_wheel();
    void normalize_quaternion(std::array<double, 4>& q);
    void quaternion_to_euler(const std::array<double, 4>& q, 
                            std::array<double, 3>& euler);
    void euler_to_quaternion(const std::array<double, 3>& euler,
                            std::array<double, 4>& q);
    void fuse_orientations(const std::array<double, 4>& imu_orient,
                          const std::array<double, 4>& mag_orient,
                          std::array<double, 4>& fused_orient);
};

} // namespace basic_pub_sub

#endif // BASIC_PUB_SUB__SENSOR_FUSION_CORE_HPP_