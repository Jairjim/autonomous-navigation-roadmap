#ifndef ROBOTICS_CPP_FOUNDATIONS__SIMULATED_LIDAR_HPP_
#define ROBOTICS_CPP_FOUNDATIONS__SIMULATED_LIDAR_HPP_

#include <cstddef>

#include "robotics_cpp_foundations/laser_scan.hpp"
#include "robotics_cpp_foundations/occupancy_grid.hpp"
#include "robotics_cpp_foundations/pose2d.hpp"

namespace robotics
{

    class SimulatedLidar
    {
    public:
        SimulatedLidar(
            double range_min,
            double range_max,
            std::size_t beam_count);

        LaserScan2D scan(
            const OccupancyGrid2D &world,
            const Pose2D &sensor_pose) const;

    private:
        double castRay(
            const OccupancyGrid2D &world,
            const Pose2D &sensor_pose,
            double relative_angle) const;

        double range_min_;
        double range_max_;
        std::size_t beam_count_;
    };
} // namespace robotics

#endif