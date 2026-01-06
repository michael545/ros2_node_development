// Copyright 2024 Michael's ROS2 Development Guide
// Licensed under Apache License 2.0

#include "basic_pub_sub/sensor_fusion_core.hpp"
#include <algorithm>
#include <cmath>

namespace basic_pub_sub {

SensorFusionCore::SensorFusionCore()
    : enable_mag_(true),
      enable_wheel_(true),
      imu_trust_(0.8),
      wheel_trust_(0.6),
      mag_trust_(0.4),
      last_fusion_time_(0.0),
      process_noise_(0.1),
      measurement_noise_(0.5) {
    
    // Initialize state
    reset();
}

void SensorFusionCore::update_imu_data(const IMUData& data) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // Store latest IMU data
    last_imu_data_ = data;
    
    // If we have recent data from other sensors, perform fusion
    double current_time = data.timestamp;
    if (last_fusion_time_ > 0) {
        double dt = current_time - last_fusion_time_;
        if (dt > 0 && dt < 0.1) { // Reasonable time step
            predict_state(dt);
            update_with_imu();
            
            if (enable_mag_ && last_mag_data_.timestamp > last_fusion_time_) {
                update_with_mag();
            }
            
            if (enable_wheel_ && last_wheel_data_.timestamp > last_fusion_time_) {
                update_with_wheel();
            }
            
            last_fusion_time_ = current_time;
            current_state_.timestamp = current_time;
            current_state_.valid = true;
        }
    } else {
        // First IMU message - initialize state
        current_state_.orientation = data.orientation;
        current_state_.angular_velocity = data.angular_velocity;
        current_state_.linear_velocity = {0.0, 0.0, 0.0};
        current_state_.position = {0.0, 0.0, 0.0};
        current_state_.timestamp = data.timestamp;
        current_state_.valid = true;
        last_fusion_time_ = data.timestamp;
    }
}

void SensorFusionCore::update_mag_data(const MagData& data) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (enable_mag_) {
        last_mag_data_ = data;
        
        // If we have recent IMU data, update orientation
        if (last_imu_data_.timestamp > 0 && 
            std::abs(data.timestamp - last_imu_data_.timestamp) < 0.05) {
            update_with_mag();
        }
    }
}

void SensorFusionCore::update_wheel_data(const WheelData& data) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (enable_wheel_) {
        last_wheel_data_ = data;
        
        // If we have recent state, update position and velocity
        if (current_state_.valid && 
            std::abs(data.timestamp - current_state_.timestamp) < 0.05) {
            update_with_wheel();
        }
    }
}

FusedState SensorFusionCore::get_fused_state() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_state_;
}

void SensorFusionCore::set_sensor_config(bool enable_mag, bool enable_wheel,
                                        double imu_trust, double wheel_trust, double mag_trust) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    enable_mag_ = enable_mag;
    enable_wheel_ = enable_wheel;
    imu_trust_ = std::max(0.0, std::min(1.0, imu_trust));
    wheel_trust_ = std::max(0.0, std::min(1.0, wheel_trust));
    mag_trust_ = std::max(0.0, std::min(1.0, mag_trust));
    
    // Normalize trust factors
    double sum = imu_trust_ + wheel_trust_ + mag_trust_;
    if (sum > 0) {
        imu_trust_ /= sum;
        wheel_trust_ /= sum;
        mag_trust_ /= sum;
    }
}

void SensorFusionCore::reset() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    current_state_ = FusedState();
    current_state_.position = {0.0, 0.0, 0.0};
    current_state_.orientation = {0.0, 0.0, 0.0, 1.0}; // Identity quaternion
    current_state_.linear_velocity = {0.0, 0.0, 0.0};
    current_state_.angular_velocity = {0.0, 0.0, 0.0};
    current_state_.valid = false;
    
    last_fusion_time_ = 0.0;
    last_imu_data_ = IMUData();
    last_mag_data_ = MagData();
    last_wheel_data_ = WheelData();
}

void SensorFusionCore::predict_state(double dt) {
    // Simple prediction step - integrate velocities
    // Position prediction: p = p + v * dt
    current_state_.position[0] += current_state_.linear_velocity[0] * dt;
    current_state_.position[1] += current_state_.linear_velocity[1] * dt;
    current_state_.position[2] += current_state_.linear_velocity[2] * dt;
    
    // Orientation prediction using angular velocity
    // Convert angular velocity to quaternion derivative
    std::array<double, 4> orient_dot;
    orient_dot[0] = 0.5 * ( 
        current_state_.angular_velocity[0] * current_state_.orientation[3] +
        current_state_.angular_velocity[1] * current_state_.orientation[2] -
        current_state_.angular_velocity[2] * current_state_.orientation[1]);
    
    orient_dot[1] = 0.5 * (
        current_state_.angular_velocity[1] * current_state_.orientation[3] +
        current_state_.angular_velocity[2] * current_state_.orientation[0] -
        current_state_.angular_velocity[0] * current_state_.orientation[2]);
    
    orient_dot[2] = 0.5 * (
        current_state_.angular_velocity[2] * current_state_.orientation[3] +
        current_state_.angular_velocity[0] * current_state_.orientation[1] -
        current_state_.angular_velocity[1] * current_state_.orientation[0]);
    
    orient_dot[3] = -0.5 * (
        current_state_.angular_velocity[0] * current_state_.orientation[0] +
        current_state_.angular_velocity[1] * current_state_.orientation[1] +
        current_state_.angular_velocity[2] * current_state_.orientation[2]);
    
    // Update orientation: q = q + q_dot * dt
    current_state_.orientation[0] += orient_dot[0] * dt;
    current_state_.orientation[1] += orient_dot[1] * dt;
    current_state_.orientation[2] += orient_dot[2] * dt;
    current_state_.orientation[3] += orient_dot[3] * dt;
    
    // Normalize quaternion
    normalize_quaternion(current_state_.orientation);
    
    // Add process noise (simplified)
    current_state_.position_covariance[0] += process_noise_ * dt; // x
    current_state_.position_covariance[1] += process_noise_ * dt; // y
    current_state_.position_covariance[2] += process_noise_ * dt; // z
}

void SensorFusionCore::update_with_imu() {
    // Update angular velocity directly from IMU
    current_state_.angular_velocity = last_imu_data_.angular_velocity;
    
    // Fuse IMU orientation with current state
    std::array<double, 4> fused_orient;
    fuse_orientations(last_imu_data_.orientation, current_state_.orientation, fused_orient);
    current_state_.orientation = fused_orient;
    
    // Update linear velocity from IMU acceleration (simplified)
    // In a real implementation, you would account for gravity and proper integration
    current_state_.linear_velocity[0] += last_imu_data_.linear_acceleration[0] * 
                                         (1.0 / 100.0); // Simple low-pass filter
    current_state_.linear_velocity[1] += last_imu_data_.linear_acceleration[1] * 
                                         (1.0 / 100.0);
    current_state_.linear_velocity[2] += (last_imu_data_.linear_acceleration[2] - 9.81) * 
                                         (1.0 / 100.0); // Subtract gravity
    
    // Update covariance based on measurement noise
    current_state_.position_covariance[0] = std::max(0.01, 
        current_state_.position_covariance[0] - measurement_noise_ * imu_trust_);
    current_state_.position_covariance[1] = std::max(0.01,
        current_state_.position_covariance[1] - measurement_noise_ * imu_trust_);
    current_state_.position_covariance[2] = std::max(0.01,
        current_state_.position_covariance[2] - measurement_noise_ * imu_trust_);
}

void SensorFusionCore::update_with_mag() {
    if (!enable_mag_) return;
    
    // Magnetometer provides heading correction
    // Calculate heading from magnetometer (simplified)
    double heading = std::atan2(last_mag_data_.magnetic_field[1], 
                               last_mag_data_.magnetic_field[0]);
    
    // Convert current orientation to Euler angles
    std::array<double, 3> euler;
    quaternion_to_euler(current_state_.orientation, euler);
    
    // Fuse magnetometer heading with current yaw
    // Simple complementary filter
    double fused_yaw = euler[2] * (1.0 - mag_trust_) + heading * mag_trust_;
    
    // Convert back to quaternion
    euler[2] = fused_yaw;
    euler_to_quaternion(euler, current_state_.orientation);
    
    // Update covariance
    current_state_.orientation_covariance[5] = std::max(0.01, // yaw covariance
        current_state_.orientation_covariance[5] - measurement_noise_ * mag_trust_);
}

void SensorFusionCore::update_with_wheel() {
    if (!enable_wheel_) return;
    
    // Wheel encoders provide velocity information
    // Fuse with IMU-based velocity estimate
    current_state_.linear_velocity[0] = 
        current_state_.linear_velocity[0] * (1.0 - wheel_trust_) +
        last_wheel_data_.linear_velocity * wheel_trust_;
    
    current_state_.angular_velocity[2] = 
        current_state_.angular_velocity[2] * (1.0 - wheel_trust_) +
        last_wheel_data_.angular_velocity * wheel_trust_;
    
    // Update position based on wheel velocity
    double dt = last_wheel_data_.timestamp - last_fusion_time_;
    if (dt > 0 && dt < 0.1) {
        current_state_.position[0] += current_state_.linear_velocity[0] * dt;
        current_state_.position[1] += current_state_.linear_velocity[1] * dt;
        
        // Update orientation from angular velocity
        std::array<double, 3> euler;
        quaternion_to_euler(current_state_.orientation, euler);
        euler[2] += current_state_.angular_velocity[2] * dt;
        euler_to_quaternion(euler, current_state_.orientation);
    }
    
    // Update covariance
    current_state_.position_covariance[0] = std::max(0.01,
        current_state_.position_covariance[0] - measurement_noise_ * wheel_trust_);
    current_state_.position_covariance[1] = std::max(0.01,
        current_state_.position_covariance[1] - measurement_noise_ * wheel_trust_);
}

void SensorFusionCore::normalize_quaternion(std::array<double, 4>& q) {
    double norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 0) {
        q[0] /= norm;
        q[1] /= norm;
        q[2] /= norm;
        q[3] /= norm;
    } else {
        // Reset to identity if norm is zero
        q = {0.0, 0.0, 0.0, 1.0};
    }
}

void SensorFusionCore::quaternion_to_euler(const std::array<double, 4>& q,
                                          std::array<double, 3>& euler) {
    // Roll (x-axis rotation)
    double sinr_cosp = 2 * (q[3] * q[0] + q[1] * q[2]);
    double cosr_cosp = 1 - 2 * (q[0] * q[0] + q[1] * q[1]);
    euler[0] = std::atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    double sinp = 2 * (q[3] * q[1] - q[2] * q[0]);
    if (std::abs(sinp) >= 1) {
        euler[1] = std::copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
    } else {
        euler[1] = std::asin(sinp);
    }
    
    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (q[3] * q[2] + q[0] * q[1]);
    double cosy_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
    euler[2] = std::atan2(siny_cosp, cosy_cosp);
}

void SensorFusionCore::euler_to_quaternion(const std::array<double, 3>& euler,
                                          std::array<double, 4>& q) {
    // Abbreviations for the various angular functions
    double cy = std::cos(euler[2] * 0.5);
    double sy = std::sin(euler[2] * 0.5);
    double cp = std::cos(euler[1] * 0.5);
    double sp = std::sin(euler[1] * 0.5);
    double cr = std::cos(euler[0] * 0.5);
    double sr = std::sin(euler[0] * 0.5);
    
    q[3] = cr * cp * cy + sr * sp * sy;
    q[0] = sr * cp * cy - cr * sp * sy;
    q[1] = cr * sp * cy + sr * cp * sy;
    q[2] = cr * cp * sy - sr * sp * cy;
}

void SensorFusionCore::fuse_orientations(const std::array<double, 4>& imu_orient,
                                        const std::array<double, 4>& current_orient,
                                        std::array<double, 4>& fused_orient) {
    // Simple complementary filter for orientation fusion
    // Weighted average based on trust factors
    
    // Convert to Euler angles for easier fusion
    std::array<double, 3> imu_euler, current_euler, fused_euler;
    quaternion_to_euler(imu_orient, imu_euler);
    quaternion_to_euler(current_orient, current_euler);
    
    // Fuse each angle component
    fused_euler[0] = imu_euler[0] * imu_trust_ + current_euler[0] * (1.0 - imu_trust_);
    fused_euler[1] = imu_euler[1] * imu_trust_ + current_euler[1] * (1.0 - imu_trust_);
    fused_euler[2] = imu_euler[2] * imu_trust_ + current_euler[2] * (1.0 - imu_trust_);
    
    // Convert back to quaternion
    euler_to_quaternion(fused_euler, fused_orient);
    normalize_quaternion(fused_orient);
}

} // namespace basic_pub_sub