#include "robotics_cpp_foundations/occupancy_grid.hpp"
#include "robotics_cpp_foundations/vector2d.hpp"

#include <stdexcept>
#include <cmath>
#include <iostream>
#include <cstdlib>

namespace robotics
{

    OccupancyGrid2D::OccupancyGrid2D(
        std::size_t width,
        std::size_t height,
        double resolution,
        double origin_x,
        double origin_y) : width_(width),
                           height_(height),
                           resolution_(resolution),
                           origin_x_(origin_x),
                           origin_y_(origin_y),
                           cells_(width * height, CellState::Unknown)
    {
        if (width == 0 || height == 0)
        {
            throw std::invalid_argument(
                "Grid dimensions must be greater than zero.");
        }

        if (resolution <= 0.0)
        {
            throw std::invalid_argument(
                "Grid resolution must be greater than zero");
        }
    }

    std::size_t OccupancyGrid2D::width() const
    {
        return width_;
    }

    std::size_t OccupancyGrid2D::height() const
    {
        return height_;
    }

    double OccupancyGrid2D::resolution() const
    {
        return resolution_;
    }

    bool OccupancyGrid2D::isInside(
        int column,
        int row) const
    {
        return column >= 0 &&
               row >= 0 &&
               column < static_cast<int>(width_) &&
               row < static_cast<int>(height_);
    }

    std::size_t OccupancyGrid2D::index(int column, int row) const
    {
        if (!isInside(column, row))
        {
            throw std::out_of_range(
                "Grid coordinates are outside the map");
        }

        return static_cast<std::size_t>(row) * width_ + static_cast<std::size_t>(column);
    }

    CellState OccupancyGrid2D::getCell(
        int column,
        int row) const
    {
        return cells_.at(index(column, row));
    }

    void OccupancyGrid2D::setCell(
        int column,
        int row,
        CellState state)
    {
        cells_.at(index(column, row)) = state;
    }

    GridCoordinate OccupancyGrid2D::worldToGrid(
        double world_x,
        double world_y) const
    {

        const GridCoordinate coordinate = worldToGridUnchecked(world_x, world_y);

        if (!isInside(
                coordinate.column,
                coordinate.row))
        {
            throw std::out_of_range(
                "World coordinates are outside the map.");
        }

        return coordinate;
    }

    Vector2D OccupancyGrid2D::gridToWorld(
        int column,
        int row) const
    {
        if (!isInside(column, row))
        {
            throw std::out_of_range(
                "Grid coordinates are outside the map.");
        }

        double world_x = origin_x_ + (column + 0.5) * resolution_;
        double world_y = origin_y_ + (row + 0.5) * resolution_;

        return Vector2D(
            world_x, world_y);
    }

    void OccupancyGrid2D::print() const
    {
        for (int row = static_cast<int>(height_) - 1;
             row >= 0;
             --row)
        {
            for (std::size_t column = 0;
                 column < width_;
                 ++column)
            {
                const CellState state =
                    getCell(
                        static_cast<int>(column),
                        row);

                switch (state)
                {
                case CellState::Unknown:
                    std::cout << "? ";
                    break;

                case CellState::Free:
                    std::cout << ". ";
                    break;

                case CellState::Occupied:
                    std::cout << "# ";
                    break;
                }
            }

            std::cout << '\n';
        }
    }

    std::vector<GridCoordinate> OccupancyGrid2D::rayTrace(
        int start_column,
        int start_row,
        int end_column,
        int end_row) const
    {
        if (!isInside(start_column, start_row) ||
            !isInside(end_column, end_row))
        {
            throw std::out_of_range(
                "Ray coordinates are outside the map.");
        }

        std::vector<GridCoordinate> cells;

        int x = start_column;
        int y = start_row;

        const int dx = std::abs(end_column - start_column);
        const int dy = std::abs(end_row - start_row);

        const int step_x =
            start_column < end_column ? 1 : -1;

        const int step_y =
            start_row < end_row ? 1 : -1;

        int error = dx - dy;

        while (true)
        {
            cells.push_back({x, y});

            if (x == end_column &&
                y == end_row)
            {
                break;
            }

            const int error2 = 2 * error;

            if (error2 > -dy)
            {
                error -= dy;
                x += step_x;
            }

            if (error2 < dx)
            {
                error += dx;
                y += step_y;
            }
        }

        return cells;
    }

    void OccupancyGrid2D::insertRay(
        double sensor_x,
        double sensor_y,
        double end_x,
        double end_y,
        bool endpoint_occupied)
    {
        const GridCoordinate start =
            worldToGrid(sensor_x, sensor_y);

        const GridCoordinate original_end =
            worldToGridUnchecked(
                end_x,
                end_y);

        const bool endpoint_inside =
            isInside(
                original_end.column,
                original_end.row);

        const double dx =
            end_x - sensor_x;

        const double dy =
            end_y - sensor_y;

        const double distance =
            std::sqrt(
                dx * dx +
                dy * dy);

        if (distance <= 0.0)
        {
            return;
        }

        const double direction_x =
            dx / distance;

        const double direction_y =
            dy / distance;

        double last_valid_x = sensor_x;
        double last_valid_y = sensor_y;

        const double step =
            resolution_ * 0.5;

        for (double travelled = 0.0;
             travelled <= distance;
             travelled += step)
        {
            const double current_x =
                sensor_x +
                travelled * direction_x;

            const double current_y =
                sensor_y +
                travelled * direction_y;

            const GridCoordinate coordinate =
                worldToGridUnchecked(
                    current_x,
                    current_y);

            if (!isInside(
                    coordinate.column,
                    coordinate.row))
            {
                break;
            }

            last_valid_x = current_x;
            last_valid_y = current_y;
        }

        const GridCoordinate end =
            worldToGrid(
                last_valid_x,
                last_valid_y);

        const std::vector<GridCoordinate> ray =
            rayTrace(
                start.column,
                start.row,
                end.column,
                end.row);

        if (ray.empty())
        {
            return;
        }

        for (std::size_t i = 0;
             i + 1 < ray.size();
             ++i)
        {
            const auto &cell = ray[i];

            setCell(
                cell.column,
                cell.row,
                CellState::Free);
        }

        const auto &endpoint =
            ray.back();

        const bool mark_endpoint_occupied =
            endpoint_occupied &&
            endpoint_inside;

        setCell(
            endpoint.column,
            endpoint.row,
            mark_endpoint_occupied
                ? CellState::Occupied
                : CellState::Free);
    }

    void OccupancyGrid2D::insertScan(
        const Pose2D &sensor_pose,
        const LaserScan2D &scan)
    {

        for (const auto &measurement :
             scan.measurements())
        {

            if (measurement.range < scan.rangeMin())
            {
                continue;
            }

            const bool hit = measurement.range < scan.rangeMax();

            const double usable_range = hit ? measurement.range : scan.rangeMax();

            const double global_angle =
                sensor_pose.theta() +
                measurement.angle;

            const double end_x = sensor_pose.x() + usable_range * std::cos(global_angle);

            const double end_y = sensor_pose.y() + usable_range * std::sin(global_angle);

            insertRay(
                sensor_pose.x(),
                sensor_pose.y(),
                end_x,
                end_y,
                hit);
        }
    }

    GridCoordinate OccupancyGrid2D::worldToGridUnchecked(
        double world_x,
        double world_y) const
    {
        const int column = static_cast<int>(
            std::floor(
                (world_x - origin_x_) / resolution_));

        const int row = static_cast<int>(
            std::floor(
                (world_y - origin_y_) / resolution_));

        return GridCoordinate{
            column,
            row};
    }
} // namespace robotics