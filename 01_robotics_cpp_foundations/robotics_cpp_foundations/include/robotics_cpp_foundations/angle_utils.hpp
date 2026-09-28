#ifndef ROBOTICS_CPP_FOUNDATIONS__ANGLE_UTILS_HPP_
#define ROBOTICS_CPP_FOUNDATIONS__ANGLE_UTILS_HPP_

#include <cmath>

namespace robotics
{
    constexpr double PI = 3.14159265358979323846;

    double degreesToRadians(double degrees);

    double radiansToDegrees(double radians);

    double normalizeAngle(double angle);
} // namespace robotics

#endif