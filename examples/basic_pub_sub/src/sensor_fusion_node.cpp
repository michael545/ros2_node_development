// Copyright 2024 Michael's ROS2 Development Guide
// Licensed under Apache License 2.0

#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "basic_pub_sub/sensor_fusion_core.hpp"

/**
 * @class SensorFusionNode
 * @brief Advanced sensor fusion node demonstrating real-world robotics scenarios
 * 
 * This node demonstrates professional ROS2 patterns for sensor fusion:
 * - Multi-sensor integration (IMU, Magnetometer, Wheel Encoders)
 * - Kalman filtering for state estimation
 * - Real-time performance considerations
 * - Error handling and data validation
 */
class SensorFusionNode : public rclcpp::Node {
public:
    SensorFusionNode()
        : Node("sensor_fusion_node"),
          fusion_core_(),
          imu_sequence_(0),
          mag_sequence_(0),
          wheel_sequence_(0) {
        
        // Configure QoS profiles for different sensor requirements
        auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(10));
        imu_qos.reliability(RCL_BEST_EFFORT);
        imu_qos.durability(RCL_VOLATILE);
        
        auto mag_qos = rclcpp::QoS(rclcpp::KeepLast(5));
        mag_qos.reliability(RCL_BEST_EFFORT);
        
        auto wheel_qos = rclcpp::QoS(rclcpp::KeepLast(1));
        wheel_qos.reliability(RCL_RELIABLE);
        
        auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(1));
        odom_qos.reliability(RCL_RELIABLE);
        odom_qos.durability(RCL_TRANSIENT_LOCAL);
        
        // Create subscriptions
        imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", imu_qos,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                handle_imu_data(msg);
            });
        
        mag_subscription_ = create_subscription<sensor_msgs::msg::MagneticField>(
            "imu/mag", mag_qos,
            [this](const sensor_msgs::msg::MagneticField::SharedPtr msg) {
                handle_mag_data(msg);
            });
        
        wheel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
            "wheel_velocity", wheel_qos,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                handle_wheel_data(msg);
            });
        
        // Create publisher
        odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
            "odom", odom_qos);
        
        // Create timer for periodic fusion updates
        fusion_timer_ = create_wall_timer(
            std::chrono::milliseconds(10),  // 100Hz fusion rate
            [this]() {
                publish_fused_odometry();
            });
        
        // Set up parameter callbacks
        setup_parameters();
        
        RCLCPP_INFO(get_logger(), "Sensor Fusion Node initialized");
        RCLCPP_INFO(get_logger(), "Fusion rate: 100Hz");
        RCLCPP_INFO(get_logger(), "Using Kalman filter for state estimation");
    }

private:
    /**
     * @brief Handle IMU data with validation and preprocessing
     */
    void handle_imu_data(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // Validate timestamp
        if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                "Received IMU message with zero timestamp");
            return;
        }
        
        // Check for reasonable values
        if (std::abs(msg->linear_acceleration.x) > 50.0 ||
            std::abs(msg->linear_acceleration.y) > 50.0 ||
            std::abs(msg->linear_acceleration.z) > 50.0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                "IMU acceleration out of range: [%.2f, %.2f, %.2f] m/s²",
                                msg->linear_acceleration.x,
                                msg->linear_acceleration.y,
                                msg->linear_acceleration.z);
            return;
        }
        
        // Convert to fusion core format
        basic_pub_sub::IMUData imu_data;
        imu_data.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        imu_data.linear_acceleration = {
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z
        };
        imu_data.angular_velocity = {
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z
        };
        imu_data.orientation = {
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        };
        
        // Update fusion core
        fusion_core_.update_imu_data(imu_data);
        imu_sequence_++;
        
        // Log data rate
        if (imu_sequence_ % 100 == 0) {
            RCLCPP_DEBUG(get_logger(), "IMU data rate: %.1f Hz",
                        100.0 / (msg->header.stamp - last_imu_time_).seconds());
        }
        last_imu_time_ = msg->header.stamp;
    }
    
    /**
     * @brief Handle magnetometer data
     */
    void handle_mag_data(const sensor_msgs::msg::MagneticField::SharedPtr msg) {
        // Validate magnetic field strength
        double field_strength = std::sqrt(
            msg->magnetic_field.x * msg->magnetic_field.x +
            msg->magnetic_field.y * msg->magnetic_field.y +
            msg->magnetic_field.z * msg->magnetic_field.z);
        
        if (field_strength < 20.0 || field_strength > 80.0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                "Magnetic field strength %.1f μT out of expected range [20-80 μT]",
                                field_strength);
        }
        
        basic_pub_sub::MagData mag_data;
        mag_data.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        mag_data.magnetic_field = {
            msg->magnetic_field.x,
            msg->magnetic_field.y,
            msg->magnetic_field.z
        };
        
        fusion_core_.update_mag_data(mag_data);
        mag_sequence_++;
    }
    
    /**
     * @brief Handle wheel encoder data
     */
    void handle_wheel_data(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Validate wheel velocities
        if (std::abs(msg->linear.x) > 10.0 || std::abs(msg->angular.z) > 10.0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                "Unreasonable wheel velocities: linear=%.2f, angular=%.2f",
                                msg->linear.x, msg->angular.z);
            return;
        }
        
        basic_pub_sub::WheelData wheel_data;
        wheel_data.timestamp = now().seconds();
        wheel_data.linear_velocity = msg->linear.x;
        wheel_data.angular_velocity = msg->angular.z;
        
        fusion_core_.update_wheel_data(wheel_data);
        wheel_sequence_++;
    }
    
    /**
     * @brief Publish fused odometry at fixed rate
     */
    void publish_fused_odometry() {
        // Get fused state from core
        auto state = fusion_core_.get_fused_state();
        
        if (!state.valid) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                "Fusion state invalid - not publishing odometry");
            return;
        }
        
        // Create odometry message
        auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
        odom_msg->header.stamp = now();
        odom_msg->header.frame_id = "odom";
        odom_msg->child_frame_id = "base_link";
        
        // Position
        odom_msg->pose.pose.position.x = state.position[0];
        odom_msg->pose.pose.position.y = state.position[1];
        odom_msg->pose.pose.position.z = state.position[2];
        
        // Orientation
        odom_msg->pose.pose.orientation.x = state.orientation[0];
        odom_msg->pose.pose.orientation.y = state.orientation[1];
        odom_msg->pose.pose.orientation.z = state.orientation[2];
        odom_msg->pose.pose.orientation.w = state.orientation[3];
        
        // Velocity
        odom_msg->twist.twist.linear.x = state.linear_velocity[0];
        odom_msg->twist.twist.linear.y = state.linear_velocity[1];
        odom_msg->twist.twist.linear.z = state.linear_velocity[2];
        
        odom_msg->twist.twist.angular.x = state.angular_velocity[0];
        odom_msg->twist.twist.angular.y = state.angular_velocity[1];
        odom_msg->twist.twist.angular.z = state.angular_velocity[2];
        
        // Covariance (simplified - real implementation would calculate proper covariance)
        odom_msg->pose.covariance = {
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,  // x
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,  // y
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,  // z
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,  // roll
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,  // pitch
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01    // yaw
        };
        
        odom_msg->twist.covariance = {
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,  // vx
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,  // vy
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,  // vz
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,  // vroll
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,  // vpitch
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01    // vyaw
        };
        
        // Publish odometry
        odometry_publisher_->publish(std::move(odom_msg));
        
        // Log fusion status periodically
        fusion_cycle_count_++;
        if (fusion_cycle_count_ % 100 == 0) {
            RCLCPP_INFO(get_logger(), 
                       "Fusion Status - IMU: %zu, Mag: %zu, Wheel: %zu messages processed",
                       imu_sequence_, mag_sequence_, wheel_sequence_);
        }
    }
    
    /**
     * @brief Set up dynamic parameters
     */
    void setup_parameters() {
        // Declare parameters with defaults
        declare_parameter("fusion.enable_mag", true);
        declare_parameter("fusion.enable_wheel", true);
        declare_parameter("fusion.imu_trust_factor", 0.8);
        declare_parameter("fusion.wheel_trust_factor", 0.6);
        declare_parameter("fusion.mag_trust_factor", 0.4);
        
        // Set parameter callback
        param_callback_handle_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& params) {
                return parameter_callback(params);
            });
        
        // Apply initial parameters
        apply_parameters();
    }
    
    /**
     * @brief Handle parameter changes
     */
    rcl_interfaces::msg::SetParametersResult parameter_callback(
        const std::vector<rclcpp::Parameter>& params) {
        
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "Parameters updated successfully";
        
        try {
            apply_parameters();
        } catch (const std::exception& e) {
            result.successful = false;
            result.reason = std::string("Parameter update failed: ") + e.what();
        }
        
        return result;
    }
    
    /**
     * @brief Apply current parameters to fusion core
     */
    void apply_parameters() {
        bool enable_mag = get_parameter("fusion.enable_mag").as_bool();
        bool enable_wheel = get_parameter("fusion.enable_wheel").as_bool();
        double imu_trust = get_parameter("fusion.imu_trust_factor").as_double();
        double wheel_trust = get_parameter("fusion.wheel_trust_factor").as_double();
        double mag_trust = get_parameter("fusion.mag_trust_factor").as_double();
        
        // Validate parameters
        if (imu_trust < 0.0 || imu_trust > 1.0 ||
            wheel_trust < 0.0 || wheel_trust > 1.0 ||
            mag_trust < 0.0 || mag_trust > 1.0) {
            throw std::runtime_error("Trust factors must be between 0.0 and 1.0");
        }
        
        fusion_core_.set_sensor_config(enable_mag, enable_wheel, imu_trust, wheel_trust, mag_trust);
        
        RCLCPP_INFO(get_logger(), "Fusion parameters updated:");
        RCLCPP_INFO(get_logger(), "  Magnetometer: %s (trust=%.2f)",
                   enable_mag ? "enabled" : "disabled", mag_trust);
        RCLCPP_INFO(get_logger(), "  Wheel encoders: %s (trust=%.2f)",
                   enable_wheel ? "enabled" : "disabled", wheel_trust);
        RCLCPP_INFO(get_logger(), "  IMU trust factor: %.2f", imu_trust);
    }

    // Core fusion logic
    basic_pub_sub::SensorFusionCore fusion_core_;
    
    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr wheel_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    rclcpp::TimerBase::SharedPtr fusion_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    
    // State tracking
    size_t imu_sequence_;
    size_t mag_sequence_;
    size_t wheel_sequence_;
    size_t fusion_cycle_count_;
    rclcpp::Time last_imu_time_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    
    // Use multi-threaded executor for better sensor data handling
    rclcpp::executors::MultiThreadedExecutor executor;
    auto sensor_fusion_node = std::make_shared<SensorFusionNode>();
    executor.add_node(sensor_fusion_node);
    
    RCLCPP_INFO(rclcpp::get_logger("sensor_fusion_main"), 
               "Starting Sensor Fusion Node with multi-threaded executor");
    
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}