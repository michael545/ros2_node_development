#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <gnss_navigation/imu_gnss_tracker.h>

#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <std_srvs/srv/empty.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <memory>
#include <mutex>
#include <optional>

namespace gnss_navigation {

class IMUGNSSTrackerROS : public rclcpp::Node {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    IMUGNSSTrackerROS();
    ~IMUGNSSTrackerROS() override = default;

private:
    void onSyncMsgs(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gnss_msg,
                    const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg);
    void updateTF(const rclcpp::Time& stamp);
    void handleResetOrigin(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                           std::shared_ptr<std_srvs::srv::Empty::Response> response);
    void handleRepublishOrigin(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                               std::shared_ptr<std_srvs::srv::Empty::Response> response);
    // ROS components
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;

    // Subscribers
    using NavSatFixSub = message_filters::Subscriber<sensor_msgs::msg::NavSatFix>;
    using ImuSub = message_filters::Subscriber<sensor_msgs::msg::Imu>;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::NavSatFix, sensor_msgs::msg::Imu>;
    using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

    std::shared_ptr<NavSatFixSub> gnss_sub_;
    std::shared_ptr<ImuSub> imu_sub_;
    std::shared_ptr<Synchronizer> sync_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr origin_pub_;

    // Services
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_origin_srv_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr republish_origin_srv_;

    // Configuration
    std::string global_frame_id_;
    std::string odom_frame_id_;
    std::string base_frame_id_;
    std::string gnss_topic_;
    std::string imu_topic_;
    std::string origin_fix_topic_;
    int min_fix_status_;
    double initial_pos_x_{0.0};
    double initial_pos_y_{0.0};
    double initial_pos_a_{0.0};
    double yaw_offset_{0.0};

    // Tracker
    std::unique_ptr<IMUGNSSTracker> tracker_;
    IMUGNSSTracker::Options tracker_options_;

    // Latest messages
    sensor_msgs::msg::NavSatFix origin_navsat_;
    bool origin_set_{false};
    bool first_pose_published_{false};

    rclcpp::Duration transform_tolerance_{0, 0};
    const double default_yaw_std_dev_{0.5};
    rclcpp::TimerBase::SharedPtr param_timer_;
};

} // namespace gnss_navigation
