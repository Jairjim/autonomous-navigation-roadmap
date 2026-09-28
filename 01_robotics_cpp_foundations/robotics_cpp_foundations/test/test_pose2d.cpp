#include <gtest/gtest.h>

#include "robotics_cpp_foundations/angle_utils.hpp"
#include "robotics_cpp_foundations/pose2d.hpp"

TEST(Pose2DTest, CalculatesDistance)
{
    const robotics::Pose2D a(
        0.0,
        0.0,
        0.0);

    const robotics::Pose2D b(
        3.0,
        4.0,
        0.0);

    EXPECT_NEAR(
        a.distanceTo(b),
        5.0,
        1e-9);
}

TEST(Pose2DTest, TransformsAndRecoversPose)
{
    const robotics::Pose2D robot(
        2.0,
        3.0,
        robotics::degreesToRadians(90.0));

    const robotics::Pose2D local(
        2.0,
        0.0,
        0.0);

    const robotics::Pose2D world =
        robot.transformBy(local);

    EXPECT_NEAR(world.x(), 2.0, 1e-9);
    EXPECT_NEAR(world.y(), 5.0, 1e-9);

    const robotics::Pose2D recovered =
        world.relativeTo(robot);

    EXPECT_NEAR(recovered.x(), 2.0, 1e-9);
    EXPECT_NEAR(recovered.y(), 0.0, 1e-9);
}