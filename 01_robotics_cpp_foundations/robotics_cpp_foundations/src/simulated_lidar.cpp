#include "robotics_cpp_foundations/simulated_lidar.hpp"

#include <cmath>
#include <stdexcept>

#include "robotics_cpp_foundations/angle_utils.hpp"

namespace robotics
{

    SimulatedLidar::SimulatedLidar(
        double range_min,
        double range_max,
        std::size_t beam_count)
        : range_min_(range_min),
          range_max_(range_max),
          beam_count_(beam_count)
    {
        if (range_min < 0.0)
        {
            throw std::invalid_argument(
                "Minimum range cannot be negative.");
        }

        if (range_max <= range_min)
        {
            throw std::invalid_argument(
                "Maximum range must be greater than minimum range.");
        }

        if (beam_count == 0)
        {
            throw std::invalid_argument(
                "LiDAR must contain at least one beam.");
        }
    }

    double SimulatedLidar::castRay(
        const OccupancyGrid2D &world,
        const Pose2D &sensor_pose,
        double relative_angle) const
    {
        const double global_angle =
            sensor_pose.theta() + relative_angle;

        const double step =
            world.resolution() * 0.5;

        for (double distance = range_min_;
             distance < range_max_;
             distance += step)
        {
            const double x =
                sensor_pose.x() +
                distance * std::cos(global_angle);

            const double y =
                sensor_pose.y() +
                distance * std::sin(global_angle);

            try
            {
                const GridCoordinate cell =
                    world.worldToGrid(x, y);

                if (world.getCell(
                        cell.column,
                        cell.row) == CellState::Occupied)
                {
                    return distance;
                }
            }
            catch (const std::out_of_range &)
            {
                return range_max_;
            }
        }

        return range_max_;
    }

    LaserScan2D SimulatedLidar::scan(
        const OccupancyGrid2D &world,
        const Pose2D &sensor_pose) const
    {
        LaserScan2D scan(
            range_min_,
            range_max_);

        const double angle_increment =
            (2.0 * PI) /
            static_cast<double>(beam_count_);

        for (std::size_t i = 0;
             i < beam_count_;
             ++i)
        {
            const double angle =
                -PI +
                static_cast<double>(i) *
                    angle_increment;

            const double range =
                castRay(
                    world,
                    sensor_pose,
                    angle);

            scan.addMeasurement(
                angle,
                range);
        }

        return scan;
    }
} // namespace robotics