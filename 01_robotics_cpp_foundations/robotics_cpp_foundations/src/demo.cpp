#include <iostream>
#include <vector>

#include "robotics_cpp_foundations/occupancy_grid.hpp"
#include "robotics_cpp_foundations/pose2d.hpp"
#include "robotics_cpp_foundations/simulated_lidar.hpp"

int main()
{
    robotics::OccupancyGrid2D world(
        10,
        10,
        0.1);

    // Outer walls
    for (int column = 0; column < 10; ++column)
    {
        world.setCell(
            column,
            0,
            robotics::CellState::Occupied);

        world.setCell(
            column,
            9,
            robotics::CellState::Occupied);
    }

    for (int row = 0; row < 10; ++row)
    {
        world.setCell(
            0,
            row,
            robotics::CellState::Occupied);

        world.setCell(
            9,
            row,
            robotics::CellState::Occupied);
    }

    // Internal wall
    for (int row = 4; row < 8; ++row)
    {
        world.setCell(
            7,
            row,
            robotics::CellState::Occupied);
    }

    robotics::Pose2D robot(
        0.5,
        0.5,
        0.0);

    robotics::SimulatedLidar lidar(
        0.1,
        0.3,
        360);

    robotics::LaserScan2D scan =
        lidar.scan(
            world,
            robot);

    std::cout
        << "Generated "
        << scan.size()
        << " LiDAR measurements.\n";

    robotics::OccupancyGrid2D robot_map(
        10,
        10,
        0.1);

    const std::vector<robotics::Pose2D> trajectory = {
        robotics::Pose2D(0.2, 0.2, 0.0),
        robotics::Pose2D(0.4, 0.2, 0.0),
        robotics::Pose2D(0.5, 0.4, 0.0),
        robotics::Pose2D(0.5, 0.6, 0.0),
        robotics::Pose2D(0.3, 0.7, 0.0)};

    for (const auto &pose : trajectory)
    {
        const robotics::LaserScan2D current_scan =
            lidar.scan(
                world,
                pose);

        robot_map.insertScan(
            pose,
            current_scan);
    }

    std::cout << "\nGROUND TRUTH:\n\n";
    world.print();

    std::cout << "\nROBOT MAP:\n\n";
    robot_map.print();

    return 0;
}