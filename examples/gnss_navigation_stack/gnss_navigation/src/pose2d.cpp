#include "gnss_navigation/pose2d.h"

namespace gnss_navigation {

Pose2D::Pose2D()
    : state()
{}

Pose2D::Pose2D(const double& x, const double& y, const double& rotation)
    : state(rotation, Vector2d(x,y))
{}

Pose2D::Pose2D(const Vector2d& xy, const double& rotation)
    : state(rotation, xy)
{}

Pose2D::Pose2D(const Vector3d& xyr)
    : state(xyr[2], xyr.head<2>())
{}

Pose2D::Pose2D(const Affine2d& transformation)
{
    state.translation() = transformation.translation();
    state.setRotationMatrix(transformation.linear());
}

Pose2D::Pose2D(const Pose2D& other)
{
    state = other.state;
}

Pose2D::Pose2D(const SE2d& se2)
    : state(se2)
{}

Pose2D::~Pose2D()
{}

Pose2D& Pose2D::operator=(const Pose2D& other)
{
    state = other.state;
    return *this;
}

Pose2D Pose2D::operator+(const Pose2D& other) const
{
    return Pose2D(state * other.state);
}

Pose2D Pose2D::operator-(const Pose2D& other) const
{
    return Pose2D(state.inverse() * other.state);
}

Pose2D& Pose2D::operator+=(const Pose2D& other)
{
    state *= other.state;
    return *this;
}

Pose2D& Pose2D::operator-=(const Pose2D& other)
{
    state = state.inverse() * other.state;
    return *this;
}

Vector2d Pose2D::operator*(const Vector2d& point) const
{
    return state*point;
}

double Pose2D::x() const
{
    return state.translation().x();
}

double Pose2D::y() const
{
    return state.translation().y();
}

Eigen::Vector2d Pose2D::xy() const
{
    return state.translation();
}

double Pose2D::rotation() const
{
    return state.so2().log();
}

const Eigen::Vector3d Pose2D::xyr() const
{
    Vector3d tmp;
    tmp.x() = state.translation().x();
    tmp.y() = state.translation().y();
    tmp.z() = state.so2().log();
    return tmp;
}

} // namespace gnss_navigation
