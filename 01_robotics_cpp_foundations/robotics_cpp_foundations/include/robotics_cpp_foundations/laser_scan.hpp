#ifndef ROBOTICS_CPP_FOUNDATIONS__LASER_SCAN_HPP_
#define ROBOTICS_CPP_FOUNDATIONS__LASER_SCAN_HPP_

#include <vector>

namespace robotics
{

    struct LaserMeasurement
    {
        double angle;
        double range;
    };

    class LaserScan2D
    {
    public:
        LaserScan2D(
            double range_min,
            double range_max);

        void addMeasurement(
            double angle,
            double range);

        const std::vector<LaserMeasurement> &
        measurements() const;

        std::size_t size() const;

        double rangeMin() const;
        double rangeMax() const;

    private:
        double range_min_;
        double range_max_;
        std::vector<LaserMeasurement> measurements_;
    };

} // namespace robotics

#endif