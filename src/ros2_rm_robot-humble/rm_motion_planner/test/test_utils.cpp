#include <gtest/gtest.h>
#include "rm_motion_planner/utils.hpp"
#include <Eigen/Core>

using namespace rm_motion_planner;

TEST(UtilsTest, TcpPoseToMatrixIdentity)
{
    Eigen::Matrix<double,6,1> pose;
    pose << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    Eigen::Matrix4d T = tcpPose2HomogenMatrix(pose);
    EXPECT_NEAR(T(0,3), 0.0, 1e-9);
    EXPECT_NEAR(T(1,3), 0.0, 1e-9);
    EXPECT_NEAR(T(2,3), 0.0, 1e-9);
    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    EXPECT_TRUE((T - I).norm() < 1e-9);
}
