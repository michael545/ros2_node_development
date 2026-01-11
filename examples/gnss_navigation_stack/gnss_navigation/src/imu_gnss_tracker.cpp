#include "gnss_navigation/imu_gnss_tracker.h"

namespace gnss_navigation {

// Internal GNSS struct for UTMtoLL conversion
struct GNSS {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    double latitude;
    double longitude;
    inline void fromUTM(double x, double y, const std::string& zone)
    { UTMtoLL(y, x, zone, latitude, longitude); }
};

IMUGNSSTracker::IMUGNSSTracker(const Options& options)
    : options_(options)
{
    pose_ = Pose2D(0, 0, 0);
    covar_ = Matrix3d::Identity() * 999;  // High initial uncertainty
    vel_ = Vector2d(0, 0);

    has_first_update_ = false;
    has_first_gnss_ = false;
    last_timestamp_ = 0.0;
}

IMUGNSSTracker::~IMUGNSSTracker()
{}

bool IMUGNSSTracker::update(const IMUGNSS& imu_gnss, double timestamp)
{
    if (imu_gnss.status == -1) {// No fix, bad data 
        return false; 
    }

    if (!has_first_gnss_) { //koord izhodisce v  (0 0)
        initializeGNSS(imu_gnss);
        has_first_gnss_ = true;
        last_timestamp_ = timestamp;
        return true; // Don't process first update, just initialize
    }
    
    double dt = timestamp - last_timestamp_;
    if (dt <= 0) {
        return false;
    }

    if (options_.use_kalman_filter) {
        kalmanUpdate(imu_gnss, dt);
    } else {
        simpleUpdate(imu_gnss);
    }

    has_first_update_ = true;
    last_timestamp_ = timestamp;

    return true;
}

void IMUGNSSTracker::initializeGNSS(const IMUGNSS& imu_gnss)
{
    // Convert GNSS to UTM
    double gx, gy;
    imu_gnss.toUTM(gx, gy, gnss_zone_);

    // Set reference pose
    // We use 0 (0 degrees) as the reference yaw.
    // This ensures that the local coordinate system is rotated 180 degrees relative to UTM.
    // This aligns the position output with the desired map frame (correcting the (5,5) -> (-5,-5) inversion).
    gnss_ref_pose_ = Pose2D(gx, gy, 0);
    gnss_pose_ = gnss_ref_pose_;

    // Set offset to current pose (if pose was set manually)
    gnss_offset_ = pose_;

    // Update pose to match GNSS
    Pose2D gnss_local = gnss_offset_ + (gnss_ref_pose_ - gnss_pose_);
    pose_ = Pose2D(gnss_local.x(), gnss_local.y(), imu_gnss.getYaw());
}

void IMUGNSSTracker::simpleUpdate(const IMUGNSS& imu_gnss)
{
    // Convert GNSS to UTM
    double gx, gy;
    imu_gnss.toUTM(gx, gy, gnss_zone_);

    // Create GNSS pose in global coordinates
    Pose2D gnss_global(gx, gy, imu_gnss.getYaw());
    gnss_pose_ = gnss_global;

    // Convert to local coordinates
    Pose2D gnss_local = gnss_offset_ + (gnss_ref_pose_ - gnss_global);

    // Update position with smoothing
    double pos_weight = 1.0 - options_.smoothing_factor;
    pose_.state.translation().x() = pos_weight * gnss_local.x() + 
                                    (1.0 - pos_weight) * pose_.state.translation().x();
    pose_.state.translation().y() = pos_weight * gnss_local.y() + 
                                    (1.0 - pos_weight) * pose_.state.translation().y();

    // Update orientation (use IMU directly, no smoothing needed usually)
    pose_.state.so2() = SO2d(imu_gnss.getYaw());

    // Update covariance from GNSS uncertainty
    Matrix2d pos_covar = imu_gnss.position_covar;
    pos_covar(0,0) += options_.gnss_min_var;
    pos_covar(1,1) += options_.gnss_min_var;

    covar_.block<2,2>(0,0) = pos_covar;
    
    // Set orientation variance from IMU uncertainty
    double yaw_var = imu_gnss.yaw_std_dev * imu_gnss.yaw_std_dev;
    covar_(2,2) = yaw_var;
}

void IMUGNSSTracker::kalmanUpdate(const IMUGNSS& imu_gnss, double dt)
{
    // 1. Prediction Step
    // Predict state using a simple constant velocity model
    pose_.state.translation().x() += vel_.x() * dt;
    pose_.state.translation().y() += vel_.y() * dt;
    // Yaw is predicted to be the same (it's corrected by IMU measurement later)

    // Predict covariance
    Matrix3d F = Matrix3d::Identity();
    F(0, 2) = -vel_.y() * dt; // Simplified Jacobian
    F(1, 2) = vel_.x() * dt;
    
    Matrix3d Q = Matrix3d::Identity(); // Process noise
    Q(0,0) = options_.process_noise_pos * dt;
    Q(1,1) = options_.process_noise_pos * dt;
    Q(2,2) = options_.process_noise_yaw * dt;

    covar_ = F * covar_ * F.transpose() + Q;

    // 2. Measurement Update Step
    double gx, gy;
    imu_gnss.toUTM(gx, gy, gnss_zone_);

    // Create GNSS pose in global coordinates
    Pose2D gnss_global(gx, gy, imu_gnss.getYaw());
    gnss_pose_ = gnss_global;
    
    Pose2D gnss_local = gnss_offset_ + (gnss_ref_pose_ - gnss_global);

    // Estimate velocity from position change
    if (dt > 1e-6) {
        vel_ = (gnss_local.xy() - pose_.xy()) / dt;
    }

    // Measurement covariance
    Matrix3d R = Matrix3d::Identity();
    R.block<2,2>(0,0) = imu_gnss.position_covar;
    R(0,0) += options_.gnss_min_var;
    R(1,1) += options_.gnss_min_var;
    R(2,2) = imu_gnss.yaw_std_dev * imu_gnss.yaw_std_dev;

    // Kalman gain
    Matrix3d S = covar_ + R;
    Matrix3d K = covar_ * S.inverse();

    // Update state with measurement
    Vector3d innovation;
    innovation.head<2>() = gnss_local.xy() - pose_.xy();
    innovation(2) = SO2d(imu_gnss.getYaw()).log() - pose_.state.so2().log();
    
    Vector3d correction = K * innovation;
    pose_.state.translation() += correction.head<2>();
    pose_.state.so2() = pose_.state.so2() * SO2d(correction(2));


    // Update covariance
    Matrix3d I = Matrix3d::Identity();
    covar_ = (I - K) * covar_;
}

bool IMUGNSSTracker::UTMtoLL(double x, double y, double& latitude, double& longitude)
{
    if (!has_first_gnss_)
        return false;

    Pose2D local(x, y, 0.0);
    Pose2D global = gnss_ref_pose_ + (gnss_offset_ - local);

    GNSS gnss;
    gnss.fromUTM(global.x(), global.y(), gnss_zone_);

    latitude = gnss.latitude;
    longitude = gnss.longitude;

    return true;
}

} 
