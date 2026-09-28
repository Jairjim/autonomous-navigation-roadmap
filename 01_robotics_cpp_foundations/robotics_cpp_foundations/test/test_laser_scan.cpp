#include <gtest/gtest.h>

#include "robotics_cpp_foundations/laser_scan.hpp"

TEST(LaserScanTest, RejectsInvalidRangeConfiguration)
{
    EXPECT_THROW(
        robotics::LaserScan2D(5.0, 1.0),
        std::invalid_argument);
}

TEST(LaserScanTest, StoresMeasurements)
{
    robotics::LaserScan2D scan(
        0.1,
        5.0);

    scan.addMeasurement(
        0.0,
        2.5);

    EXPECT_EQ(
        scan.size(),
        1u);

    ASSERT_FALSE(
        scan.measurements().empty());

    EXPECT_NEAR(
        scan.measurements().front().range,
        2.5,
        1e-9);
}