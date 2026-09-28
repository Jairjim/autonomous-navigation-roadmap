#ifndef ROBOTICS_CPP_FOUNDATIONS__VECTOR2D_HPP_
#define ROBOTICS_CPP_FOUNDATIONS__VECTOR2D_HPP_

#include <cmath>

namespace robotics
{

class Vector2D
{
public:
    Vector2D();
    Vector2D(double x, double y);

    double x() const;
    double y() const;

    void setX(double x);
    void setY(double y);

    double magnitude() const;
    Vector2D normalized() const;

    double dot(const Vector2D & other) const;
    double distanceTo(const Vector2D & other) const;

    Vector2D operator+(const Vector2D & other) const;
    Vector2D operator-(const Vector2D & other) const;
    Vector2D operator*(double scalar) const;

private:
    double x_;
    double y_;
};

}  // namespace robotics

#endif  // ROBOTICS_CPP_FOUNDATIONS__VECTOR2D_HPP_