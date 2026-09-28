#include "robotics_cpp_foundations/pose2d.hpp"
#include "robotics_cpp_foundations/angle_utils.hpp"

#include <cmath>

namespace robotics
{
    Pose2D::Pose2D()
        : x_(0.0), y_(0.0), theta_(0.0)
    {
    }

    Pose2D::Pose2D(double x, double y, double theta)
        : x_(x), y_(y), theta_(theta)
    {
    }

    double Pose2D::x() const
    {
        return x_;
    }

    double Pose2D::y() const
    { 
        return y_;
    }

    double Pose2D::theta() const
    {
        return theta_;
    }

    void Pose2D::setX(double x)
    {
        x_ = x;
    }

    void Pose2D::setY(double y)
    {
        y_ = y;
    }

    void Pose2D::setTheta(double theta)
    {
        theta_ = theta;
    }

    Vector2D Pose2D::position() const
    {
        return Vector2D(x_, y_);
    }

    double Pose2D::distanceTo(const Pose2D &other) const
    {
        return position().distanceTo(other.position());
    }

    Pose2D Pose2D::relativeTo(const Pose2D &reference) const
    {
        const double dx = x_ - reference.x_;
        const double dy = y_ - reference.y_;

        const double cos_theta = std::cos(reference.theta_);
        const double sin_theta = std::sin(reference.theta_);

        const double relative_x = cos_theta * dx + sin_theta * dy;
        const double relative_y = -sin_theta * dx + cos_theta * dy;

        const double relative_theta = normalizeAngle(theta_ - reference.theta_);

        return Pose2D(
            relative_x,
            relative_y,
            relative_theta);
    }

    Pose2D Pose2D::transformBy(const Pose2D &relative) const
    {
        const double cos_theta = std::cos(theta_);
        const double sin_theta = std::sin(theta_);

        const double transformed_x = x_ + cos_theta * relative.x_ - sin_theta * relative.y_;
        const double transformed_y = y_ + sin_theta * relative.x_ + cos_theta * relative.y_;

        const double transformed_theta = normalizeAngle(theta_ + relative.theta_);

        return Pose2D(
            transformed_x,
            transformed_y,
            transformed_theta);
    }
} // namespace robotics
