#include "robotics_cpp_foundations/vector2d.hpp"

#include <stdexcept>

namespace robotics
{

    Vector2D::Vector2D()
        : x_(0.0), y_(0.0)
    {
    }

    Vector2D::Vector2D(double x, double y)
        : x_(x), y_(y)
    {
    }

    double Vector2D::x() const
    {
        return x_;
    }

    double Vector2D::y() const
    {
        return y_;
    }

    void Vector2D::setX(double x)
    {
        x_ = x;
    }

    void Vector2D::setY(double y)
    {
        y_ = y;
    }

    double Vector2D::magnitude() const
    {
        return std::sqrt(x_ * x_ + y_ * y_);
    }

    Vector2D Vector2D::normalized() const
    {
        const double mag = magnitude();

        if (mag == 0.0)
        {
            throw std::runtime_error("Can't notmalize a zero-length vector.");
        }

        return Vector2D(x_ / mag, y_ / mag);
    }

    double Vector2D::dot(const Vector2D &other) const
    {
        return x_ * other.x_ + y_ * other.y_;
    }

    double Vector2D::distanceTo(const Vector2D &other) const
    {
        const double dx = other.x_ - x_;
        const double dy = other.y_ - y_;

        return std::sqrt(dx * dx + dy * dy);
    }

    Vector2D Vector2D::operator+(const Vector2D &other) const
    {
        return Vector2D(x_ + other.x_, y_ + other.y_);
    }

    Vector2D Vector2D::operator-(const Vector2D &other) const
    {
        return Vector2D(x_ - other.x_, y_ - other.y_);
    }

    Vector2D Vector2D::operator*(double scalar) const
    {
        return Vector2D(x_ * scalar, y_ * scalar);
    }

} // namespace robotics
