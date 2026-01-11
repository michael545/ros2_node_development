#pragma once

#include "gnss_navigation/types.h"
#include "gnss_navigation/pose2d.h"
#include "gnss_navigation/utm.h"

namespace gnss_navigation {

// Combined IMU+GNSS data structure
struct IMUGNSS {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // GNSS data
    int16_t status = -1;  // -1 = no fix
    double latitude;
    double longitude;
    Matrix2d position_covar;  // Position uncertainty (2x2 covariance matrix)

    // IMU data (3D orientation)
    double roll;
    double pitch;
    double yaw;  // Heading (most important for 2D)
    double yaw_std_dev;  // Yaw uncertainty (standard deviation in radians)

    // Convenience methods
    inline void toUTM(double& x, double& y, std::string& zone) const
    { LLtoUTM(latitude, longitude, y, x, zone); }

    inline double getYaw() const { return yaw; }
};

class IMUGNSSTracker {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct Options {
        Options() {}

        // Minimum GNSS variance (adds to covariance to prevent singularity)
        double gnss_min_var = 0.025;

        // Smoothing factor (0 = no smoothing, 1 = full smoothing)
        // Higher = smoother but more lag
        double smoothing_factor = 0.1;

        // Whether to use simple averaging or Kalman-like filtering
        bool use_kalman_filter = false;

        // Process noise covariance (for Kalman filter)
        // Represents uncertainty in the motion model
        double process_noise_pos = 0.1;
        double process_noise_yaw = 0.1;
    };

    IMUGNSSTracker(const Options& options = Options());

    virtual ~IMUGNSSTracker();

    /**
     * Update tracker with new IMU+GNSS measurement
     * 
     * @param imu_gnss Combined IMU+GNSS data
     * @param timestamp Current timestamp
     * @return true if update was successful
     */
    bool update(const IMUGNSS& imu_gnss, double timestamp);

    /**
     * Set the initial pose
     */
    inline void setPose(const Pose2D& pose)
    { pose_ = pose; has_first_update_ = false; }

    /**
     * Get current pose estimate
     */
    inline const Pose2D& getPose() const
    { return pose_; }

    /**
     * Get current pose covariance
     */
    inline const Matrix3d& getCovar() const
    { return covar_; }

    /**
     * Get GNSS reference information
     */
    inline const Pose2D& getGNSSRef() const
    { return gnss_ref_pose_; }

    inline const Pose2D& getGNSSOffset() const
    { return gnss_offset_; }

    inline const std::string& getGNSSZone() const
    { return gnss_zone_; }

    /**
     * Set GNSS reference information (for coordinate conversion)
     */
    inline void setGNSSInfo(const Pose2D& ref, const Pose2D& offset, const std::string& zone)
    {
        gnss_ref_pose_ = ref;
        gnss_offset_ = offset;
        gnss_zone_ = zone;
        has_first_gnss_ = true;
    }

    /**
     * Convert UTM coordinates to lat/lon
     */
    bool UTMtoLL(double x, double y, double& latitude, double& longitude);

private:
    void initializeGNSS(const IMUGNSS& imu_gnss);

    // Simple weighted average update
    void simpleUpdate(const IMUGNSS& imu_gnss);

    // Kalman-like filter update
    void kalmanUpdate(const IMUGNSS& imu_gnss, double dt);

private:
    Options options_;

    // Current pose estimate
    Pose2D pose_;
    Matrix3d covar_;
    Vector2d vel_; // Estimated velocity

    // GNSS reference frame (for coordinate conversion)
    Pose2D gnss_ref_pose_;
    Pose2D gnss_offset_;
    Pose2D gnss_pose_;  // Last GNSS pose in global coordinates
    std::string gnss_zone_;

    // State tracking
    bool has_first_update_;
    bool has_first_gnss_;
    double last_timestamp_;
};

} // namespace gnss_navigation
