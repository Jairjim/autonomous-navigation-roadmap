#include <gtest/gtest.h>

#include "robotics_cpp_foundations/angle_utils.hpp"

TEST(AngleUtilsTest, ConvertsDegreesToRadians)
{
    EXPECT_NEAR(
        robotics::degreesToRadians(180.0),
        robotics::PI,
        1e-9);
}

TEST(AngleUtilsTest, ConvertsRadiansToDegrees)
{
    EXPECT_NEAR(
        robotics::radiansToDegrees(robotics::PI),
        180.0,
        1e-9);
}

TEST(AngleUtilsTest, NormalizesPositiveAngle)
{
    const double angle =
        robotics::degreesToRadians(450.0);

    const double normalized =
        robotics::normalizeAngle(angle);

    EXPECT_NEAR(
        robotics::radiansToDegrees(normalized),
        90.0,
        1e-9);
}

TEST(AngleUtilsTest, NormalizesNegativeAngle)
{
    const double angle =
        robotics::degreesToRadians(-450.0);

    const double normalized =
        robotics::normalizeAngle(angle);

    EXPECT_NEAR(
        robotics::radiansToDegrees(normalized),
        -90.0,
        1e-9);
}