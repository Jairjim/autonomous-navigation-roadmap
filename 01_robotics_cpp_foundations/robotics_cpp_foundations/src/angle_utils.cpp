#include "robotics_cpp_foundations/angle_utils.hpp"

namespace robotics
{

    double degreesToRadians(double degrees)
    {
        return degrees * PI / 180.0;
    }

    double radiansToDegrees(double radians)
    {
        return radians * 180.0 / PI;
    }

    double normalizeAngle(double angle)
    {
        while (angle > PI)
        {
            angle -= 2.0 * PI;
        }

        while (angle <= -PI)
        {
            angle += 2.0 * PI;
        }

        return angle;
    }
} // namespace robotics