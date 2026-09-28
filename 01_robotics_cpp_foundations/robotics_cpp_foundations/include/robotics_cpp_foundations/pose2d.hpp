#ifndef ROBOTICS_CPP_FOUNDATIONS__POSE2D_HPP_
#define ROBOTICS_CPP_FOUNDATIONS__POSE2D_HPP_

#include "robotics_cpp_foundations/vector2d.hpp"

namespace robotics
{

    class Pose2D
    {
    public:
        Pose2D();
        Pose2D(double x, double y, double theta);

        double x() const;
        double y() const;
        double theta() const;

        void setX(double x);
        void setY(double y);
        void setTheta(double theta);

        Vector2D position() const;

        double distanceTo(const Pose2D &other) const;

        Pose2D relativeTo(const Pose2D &reference) const;

        Pose2D transformBy(const Pose2D &relative) const;

    private:
        double x_;
        double y_;
        double theta_;
    };
} // namespace robotics

#endif