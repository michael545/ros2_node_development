#include <gnss_navigation_ros/imu_gnss_tracker_ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <array>
#include <cmath>
#include <chrono>

namespace gnss_navigation {

IMUGNSSTrackerROS::IMUGNSSTrackerROS()
    : Node("imu_gnss_tracker_ros")
{
    // Declare parameters
    declare_parameter("global_frame_id", "map");
    declare_parameter("odom_frame_id", "odom");
    declare_parameter("base_frame_id", "base_link");
    declare_parameter("gnss_topic", "/gnss/fix");
    declare_parameter("imu_topic", "/gnss/imu");
    declare_parameter("initial_pos_x", 0.0);
    declare_parameter("initial_pos_y", 0.0);
    declare_parameter("initial_pos_a", 0.0);
    declare_parameter("yaw_offset", 0.0);
    declare_parameter("transform_tolerance", 0.1);
    declare_parameter("origin_fix_topic", "/gnss/origin_fix");
    declare_parameter("min_fix_status", static_cast<int>(sensor_msgs::msg::NavSatStatus::STATUS_FIX));
    
    // Tracker options
    declare_parameter("gnss_min_var", 0.025);
    declare_parameter("smoothing_factor", 0.1);
    declare_parameter("use_kalman_filter", false);
    declare_parameter("process_noise_pos", 0.1);
    declare_parameter("process_noise_yaw", 0.1);

    // Get parameters
    global_frame_id_ = get_parameter("global_frame_id").as_string();
    odom_frame_id_ = get_parameter("odom_frame_id").as_string();
    base_frame_id_ = get_parameter("base_frame_id").as_string();
    gnss_topic_ = get_parameter("gnss_topic").as_string();
    imu_topic_ = get_parameter("imu_topic").as_string();
    origin_fix_topic_ = get_parameter("origin_fix_topic").as_string();
    min_fix_status_ = static_cast<int>(get_parameter("min_fix_status").as_int());
    
    initial_pos_x_ = get_parameter("initial_pos_x").as_double();
    initial_pos_y_ = get_parameter("initial_pos_y").as_double();
    initial_pos_a_ = get_parameter("initial_pos_a").as_double();
    yaw_offset_ = get_parameter("yaw_offset").as_double();
    
    transform_tolerance_ = rclcpp::Duration::from_seconds(
        get_parameter("transform_tolerance").as_double());

    // Tracker options
    tracker_options_.gnss_min_var = get_parameter("gnss_min_var").as_double();
    tracker_options_.smoothing_factor = get_parameter("smoothing_factor").as_double();
    tracker_options_.use_kalman_filter = get_parameter("use_kalman_filter").as_bool();
    tracker_options_.process_noise_pos = get_parameter("process_noise_pos").as_double();
    tracker_options_.process_noise_yaw = get_parameter("process_noise_yaw").as_double();

    // Create tracker
    tracker_ = std::make_unique<IMUGNSSTracker>(tracker_options_);
    tracker_->setPose(Pose2D(initial_pos_x_, initial_pos_y_, initial_pos_a_));

    // Periodic parameter logging (every 3s) so you can see launch params and yaw_offset
    param_timer_ = this->create_wall_timer(
        std::chrono::seconds(3),
        [this]() {
            RCLCPP_INFO(this->get_logger(),
                        "Node params: global_frame='%s' odom_frame='%s' base_frame='%s' gnss_topic='%s' imu_topic='%s' initial_pos=(%.3f, %.3f, %.3f) yaw_offset=%.6f smoothing=%.3f use_kalman=%s process_noise_pos=%.3f process_noise_yaw=%.3f gnss_min_var=%.6f",
                        global_frame_id_.c_str(),
                        odom_frame_id_.c_str(),
                        base_frame_id_.c_str(),
                        gnss_topic_.c_str(),
                        imu_topic_.c_str(),
                        initial_pos_x_, initial_pos_y_, initial_pos_a_,
                        yaw_offset_, tracker_options_.smoothing_factor,
                        tracker_options_.use_kalman_filter ? "true" : "false",
                        tracker_options_.process_noise_pos,
                        tracker_options_.process_noise_yaw,
                        tracker_options_.gnss_min_var);
        });

    // Setup TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Subscribers
    auto sensor_qos = rclcpp::SensorDataQoS();
    gnss_sub_ = std::make_shared<NavSatFixSub>(this, gnss_topic_, sensor_qos.get_rmw_qos_profile());
    imu_sub_ = std::make_shared<ImuSub>(this, imu_topic_, sensor_qos.get_rmw_qos_profile());

    sync_ = std::make_shared<Synchronizer>(SyncPolicy(10), *gnss_sub_, *imu_sub_);
    sync_->registerCallback(std::bind(&IMUGNSSTrackerROS::onSyncMsgs, this, std::placeholders::_1, std::placeholders::_2));

    // Publishers
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "pose", rclcpp::QoS(2));

    auto origin_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    origin_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(origin_fix_topic_, origin_qos);

    reset_origin_srv_ = this->create_service<std_srvs::srv::Empty>(
        "reset_origin",
        std::bind(&IMUGNSSTrackerROS::handleResetOrigin, this, std::placeholders::_1, std::placeholders::_2));

    republish_origin_srv_ = this->create_service<std_srvs::srv::Empty>(
        "republish_origin_fix",
        std::bind(&IMUGNSSTrackerROS::handleRepublishOrigin, this, std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(),
                "IMU+GNSS Tracker ROS wrapper initialized. GNSS topic: '%s', IMU topic: '%s', pose topic: '%s'.",
                gnss_topic_.c_str(),
                imu_topic_.c_str(),
                pose_pub_->get_topic_name());
}

void IMUGNSSTrackerROS::onSyncMsgs(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gnss_msg,
                                   const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg)
{
    if (gnss_msg->status.status < min_fix_status_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "GNSS status %d is below configured minimum %d; skipping update.",
                             gnss_msg->status.status, min_fix_status_);
        return;
    }

    if (!origin_set_) {
        origin_navsat_ = *gnss_msg;
        origin_pub_->publish(origin_navsat_);
        origin_set_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "Origin fix established (lat: %.8f, lon: %.8f)",
                    origin_navsat_.latitude, origin_navsat_.longitude);
    }

    // Create a combined IMU+GNSS message for the tracker
    IMUGNSS imu_gnss;
    imu_gnss.status = gnss_msg->status.status;
    imu_gnss.latitude = gnss_msg->latitude;
    imu_gnss.longitude = gnss_msg->longitude;
    
    // Position covariance
    imu_gnss.position_covar(0,0) = gnss_msg->position_covariance[0];
    imu_gnss.position_covar(0,1) = gnss_msg->position_covariance[1];
    imu_gnss.position_covar(1,0) = gnss_msg->position_covariance[3];
    imu_gnss.position_covar(1,1) = gnss_msg->position_covariance[4];

    // Orientation from IMU
    tf2::Quaternion q;
    tf2::fromMsg(imu_msg->orientation, q);
    q.normalize();
    tf2::Matrix3x3 m(q);
    m.getRPY(imu_gnss.roll, imu_gnss.pitch, imu_gnss.yaw);
    
    // CONSTANT: IMU Mounting Offset
    // The IMU on this robot is mounted such that its "0" heading points towards the back of the robot (-X).
    // We add PI to align it with the base_link forward axis (+X).
    constexpr double IMU_MOUNTING_OFFSET_YAW = M_PI;
    imu_gnss.yaw += IMU_MOUNTING_OFFSET_YAW;

    // Apply yaw offset (e.g. for ENU vs Compass alignment)
    imu_gnss.yaw += yaw_offset_;

    // Normalize yaw to be within [-PI, PI]
    imu_gnss.yaw = std::atan2(std::sin(imu_gnss.yaw), std::cos(imu_gnss.yaw));

    // Orientation uncertainty

    // Orientation uncertainty
    double yaw_variance = imu_msg->orientation_covariance[8];
    bool yaw_variance_valid = std::isfinite(yaw_variance) && yaw_variance >= 0.0;
    if (!yaw_variance_valid) {
        yaw_variance = default_yaw_std_dev_ * default_yaw_std_dev_;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "IMU yaw covariance invalid; substituting default std dev %.3f rad.",
                             default_yaw_std_dev_);
    }
    imu_gnss.yaw_std_dev = std::sqrt(yaw_variance);

    // Update the core tracker
    double timestamp = rclcpp::Time(imu_msg->header.stamp).seconds();
    if (tracker_->update(imu_gnss, timestamp)) {
        // Get the latest pose from the tracker
        Pose2D pose = tracker_->getPose();

        // Publish pose message
        geometry_msgs::msg::PoseWithCovarianceStamped p;
        p.header.stamp = imu_msg->header.stamp;
        p.header.frame_id = global_frame_id_;
        p.pose.pose.position.x = pose.x();
        p.pose.pose.position.y = pose.y();

        // Use fused tracker rotation for published orientation (was using raw IMU quaternion)
        tf2::Quaternion pose_q;
        pose_q.setRPY(0, 0, pose.rotation());
        pose_q.normalize();
        p.pose.pose.orientation = tf2::toMsg(pose_q);

        const auto& covar = tracker_->getCovar();
        std::array<double, 36> pose_covariance{};
        pose_covariance.fill(0.0);

        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                pose_covariance[i * 6 + j] = covar(i, j);
            }
        }

        double yaw_var_tracker = covar(2, 2);
        pose_covariance[5 * 6 + 5] = yaw_var_tracker;

        const auto& imu_orientation_cov = imu_msg->orientation_covariance;
        bool orientation_cov_valid = true;
        for (int idx = 0; idx < 9; ++idx) {
            if (!std::isfinite(imu_orientation_cov[idx])) {
                orientation_cov_valid = false;
                break;
            }
            if ((idx % 4 == 0) && imu_orientation_cov[idx] < 0.0) {
                orientation_cov_valid = false;
                break;
            }
        }

        if (orientation_cov_valid) {
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    pose_covariance[(i + 3) * 6 + (j + 3)] = imu_orientation_cov[i * 3 + j];
                }
            }

            if (pose_covariance[5 * 6 + 5] < yaw_var_tracker) {
                pose_covariance[5 * 6 + 5] = yaw_var_tracker;
            }
        } else {
            double default_var = default_yaw_std_dev_ * default_yaw_std_dev_;
            pose_covariance[3 * 6 + 3] = default_var;
            pose_covariance[4 * 6 + 4] = default_var;
            pose_covariance[5 * 6 + 5] = yaw_var_tracker;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "IMU orientation covariance invalid; using tracker yaw variance and default roll/pitch variance.");
        }

        p.pose.covariance = pose_covariance;

        pose_pub_->publish(p);

        if (!first_pose_published_) {
            RCLCPP_INFO(this->get_logger(),
                        "First fused pose published at (%.3f, %.3f) heading %.3f rad.",
                        pose.x(), pose.y(), pose.rotation());
            first_pose_published_ = true;
        }

        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "Fused pose (x=%.3f, y=%.3f, yaw=%.3f rad).",
                              pose.x(), pose.y(), pose.rotation());

        // tf transfom
        updateTF(imu_msg->header.stamp);
    }
}

void IMUGNSSTrackerROS::handleResetOrigin(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    (void)request;
    (void)response;

    origin_set_ = false;
    origin_navsat_ = sensor_msgs::msg::NavSatFix();

    tracker_ = std::make_unique<IMUGNSSTracker>(tracker_options_);
    tracker_->setPose(Pose2D(initial_pos_x_, initial_pos_y_, initial_pos_a_));

    RCLCPP_WARN(this->get_logger(), "Origin reset requested; awaiting next valid GNSS fix.");
}

void IMUGNSSTrackerROS::handleRepublishOrigin(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    (void)request;
    (void)response;

    if (origin_set_) {
        origin_pub_->publish(origin_navsat_);
        RCLCPP_INFO(this->get_logger(), "Re-published stored origin NavSatFix.");
    } else {
        RCLCPP_WARN(this->get_logger(), "No origin fix stored; nothing to republish.");
    }
}

void IMUGNSSTrackerROS::updateTF(const rclcpp::Time& stamp)
{
    //= `map` -> `base_link` transform.
    
    Pose2D pose = tracker_->getPose();

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp + transform_tolerance_;
    transform.header.frame_id = global_frame_id_;
    transform.child_frame_id = base_frame_id_;
    
    // The tracker's pose (x, y, yaw) is now aligned in the core (via gnss_ref_pose_).
    // We publish the pose directly without additional rotation.

    transform.transform.translation.x = pose.x();
    transform.transform.translation.y = pose.y();
    transform.transform.translation.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, pose.rotation());
    transform.transform.rotation = tf2::toMsg(q);

    tfb_->sendTransform(transform);
}

} 

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gnss_navigation::IMUGNSSTrackerROS>());
    rclcpp::shutdown();
    return 0;
}
