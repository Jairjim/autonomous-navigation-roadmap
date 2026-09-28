#include "robotics_cpp_foundations/laser_scan.hpp"

#include <stdexcept>

namespace robotics
{

    LaserScan2D::LaserScan2D(
        double range_min,
        double range_max)
        : range_min_(range_min),
          range_max_(range_max)
    {
        if (range_min < 0.0)
        {
            throw std::invalid_argument("Minimum range cannot be negative!!");
        }

        if (range_max <= range_min)
        {
            throw std::invalid_argument(
                "Maximum range must be greater than minimum range.");
        }
    }

    void LaserScan2D::addMeasurement(
        double angle,
        double range)
    {
        measurements_.push_back({angle,
                                 range});
    }

    const std::vector<LaserMeasurement> &
    LaserScan2D::measurements() const
    {
        return measurements_;
    }

    std::size_t LaserScan2D::size() const
    {
        return measurements_.size();
    }

    double LaserScan2D::rangeMin() const
    {
        return range_min_;
    }

    double LaserScan2D::rangeMax() const
    {
        return range_max_;
    }

} // namespace robotics