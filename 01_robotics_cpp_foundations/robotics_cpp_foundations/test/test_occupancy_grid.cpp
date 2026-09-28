#include <gtest/gtest.h>

#include "robotics_cpp_foundations/occupancy_grid.hpp"

TEST(OccupancyGridTest, InitializesCellsAsUnknown)
{
    const robotics::OccupancyGrid2D grid(
        10,
        5,
        0.1);

    EXPECT_EQ(
        grid.getCell(3, 2),
        robotics::CellState::Unknown);
}

TEST(OccupancyGridTest, SetsAndGetsOccupiedCell)
{
    robotics::OccupancyGrid2D grid(
        10,
        5,
        0.1);

    grid.setCell(
        3,
        2,
        robotics::CellState::Occupied);

    EXPECT_EQ(
        grid.getCell(3, 2),
        robotics::CellState::Occupied);
}

TEST(OccupancyGridTest, ConvertsWorldToGrid)
{
    const robotics::OccupancyGrid2D grid(
        10,
        5,
        0.1);

    const auto coordinate =
        grid.worldToGrid(
            0.35,
            0.24);

    EXPECT_EQ(coordinate.column, 3);
    EXPECT_EQ(coordinate.row, 2);
}

TEST(OccupancyGridTest, ConvertsGridToWorldCenter)
{
    const robotics::OccupancyGrid2D grid(
        10,
        5,
        0.1);

    const robotics::Vector2D world =
        grid.gridToWorld(
            3,
            2);

    EXPECT_NEAR(
        world.x(),
        0.35,
        1e-9);

    EXPECT_NEAR(
        world.y(),
        0.25,
        1e-9);
}

TEST(OccupancyGridTest, GridWorldRoundTrip)
{
    const robotics::OccupancyGrid2D grid(
        10,
        5,
        0.1);

    const robotics::Vector2D world =
        grid.gridToWorld(
            3,
            2);

    const auto recovered =
        grid.worldToGrid(
            world.x(),
            world.y());

    EXPECT_EQ(recovered.column, 3);
    EXPECT_EQ(recovered.row, 2);
}

TEST(OccupancyGridTest, ThrowsForOutsideCell)
{
    const robotics::OccupancyGrid2D grid(
        10,
        5,
        0.1);

    EXPECT_THROW(
        grid.getCell(20, 20),
        std::out_of_range);
}

TEST(OccupancyGridTest, DetectsOutsideCoordinates)
{
    const robotics::OccupancyGrid2D grid(
        10,
        5,
        0.1);

    EXPECT_FALSE(
        grid.isInside(-1, 0));

    EXPECT_FALSE(
        grid.isInside(10, 0));

    EXPECT_TRUE(
        grid.isInside(9, 4));
}

TEST(OccupancyGridTest, RayTraceIncludesEndpoints)
{
    const robotics::OccupancyGrid2D grid(
        10,
        6,
        0.1);

    const auto ray =
        grid.rayTrace(
            1,
            1,
            8,
            4);

    ASSERT_FALSE(ray.empty());

    EXPECT_EQ(ray.front().column, 1);
    EXPECT_EQ(ray.front().row, 1);

    EXPECT_EQ(ray.back().column, 8);
    EXPECT_EQ(ray.back().row, 4);
}