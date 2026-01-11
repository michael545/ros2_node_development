#pragma once

#include <Eigen/Geometry>

#include "gnss_navigation/types.h"
#include "gnss_navigation/lie.h"

namespace gnss_navigation  {

struct Pose2D {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Pose2D();
    Pose2D(const double& x, const double& y, const double& rotation);
    Pose2D(const Vector2d& xy, const double& rotation);
    Pose2D(const Vector3d& xyr);
    Pose2D(const Affine2d& transformation);
    Pose2D(const Pose2D& other);
    Pose2D(const SE2d& se2);

    virtual ~Pose2D();

    Pose2D operator+(const Pose2D& other) const;
    Pose2D operator-(const Pose2D& other) const;

    Vector2d operator*(const Vector2d& point) const;

    Pose2D& operator+=(const Pose2D& other);
    Pose2D& operator-=(const Pose2D& other);

    Pose2D& operator=(const Pose2D& other);

    double x() const;
    double y() const;
    Vector2d xy() const;

    double rotation() const;

    const Vector3d xyr() const;

    // The pose is represented by a special euclidean group.
    SE2d state;
};

} // namespace gnss_navigation
