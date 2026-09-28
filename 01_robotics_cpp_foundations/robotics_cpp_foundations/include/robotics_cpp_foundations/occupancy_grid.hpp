#ifndef ROBOTICS_CPP_FOUNDATIONS__OCCUPANCY_GRID_HPP_
#define ROBOTICS_CPP_FOUNDATIONS__OCCUPANCY_GRID_HPP_

#include "robotics_cpp_foundations/vector2d.hpp"
#include "robotics_cpp_foundations/pose2d.hpp"
#include "robotics_cpp_foundations/laser_scan.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace robotics
{
    enum class CellState : std::int8_t
    {
        Unknown = -1,
        Free = 0,
        Occupied = 100
    };

    struct GridCoordinate
    {
        int column;
        int row;
    };

    class OccupancyGrid2D
    {
    public:
        OccupancyGrid2D(
            std::size_t width,
            std::size_t height,
            double resolution,
            double origin_x = 0.0,
            double origin_y = 0.0);

        std::size_t width() const;
        std::size_t height() const;

        double resolution() const;

        bool isInside(
            int column,
            int row) const;

        CellState getCell(
            int column,
            int row) const;

        void setCell(
            int column,
            int row,
            CellState state);

        GridCoordinate worldToGrid(
            double world_x,
            double world_y) const;

        Vector2D gridToWorld(
            int column,
            int row) const;

        void print() const;

        std::vector<GridCoordinate> rayTrace(
            int start_column,
            int start_row,
            int end_column,
            int end_row) const;

        void insertRay(
            double sensor_x,
            double sensor_y,
            double end_x,
            double end_y,
            bool endpoint_occupied);

        void insertScan(
            const Pose2D &sensor_pose,
            const LaserScan2D &scan);

    private:
        std::size_t index(
            int column,
            int row) const;

        std::size_t width_;
        std::size_t height_;

        double resolution_;

        double origin_x_;
        double origin_y_;

        std::vector<CellState> cells_;

        GridCoordinate worldToGridUnchecked(
            double world_x,
            double world_y) const;
    };
} // namespace robotics

#endif