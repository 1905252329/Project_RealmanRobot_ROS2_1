#include "rm_motion_planner/utils.hpp"
#include <cmath>

namespace rm_motion_planner {

Eigen::Matrix3d eulerToRotationMatrixZYX(double rx, double ry, double rz)
{
    // ZYX intrinsic: R = Rz * Ry * Rx
    Eigen::AngleAxisd Rz(rz, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd Ry(ry, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd Rx(rx, Eigen::Vector3d::UnitX());
    return (Rz * Ry * Rx).toRotationMatrix();
}

Eigen::Vector3d rotationMatrixToEulerZYX(const Eigen::Matrix3d& R)
{
    double sy = std::sqrt(R(0,0)*R(0,0) + R(1,0)*R(1,0));
    bool singular = sy < 1e-6;
    double x, y, z;
    if (!singular) {
        x = std::atan2(R(2,1), R(2,2));
        y = std::atan2(-R(2,0), sy);
        z = std::atan2(R(1,0), R(0,0));
    } else {
        x = std::atan2(-R(1,2), R(1,1));
        y = std::atan2(-R(2,0), sy);
        z = 0;
    }
    return Eigen::Vector3d(x, y, z);
}

Eigen::Matrix4d tcpPose2HomogenMatrix(const Eigen::Matrix<double,6,1>& tcpPose)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    Eigen::Vector3d pos = tcpPose.template segment<3>(0);
    double rx = tcpPose(3), ry = tcpPose(4), rz = tcpPose(5);
    T.block<3,3>(0,0) = eulerToRotationMatrixZYX(rx, ry, rz);
    T(0,3) = pos.x();
    T(1,3) = pos.y();
    T(2,3) = pos.z();
    return T;
}

Eigen::Matrix4d tcpPose2HomogenMatrix(const Eigen::Quaterniond& q, const Eigen::Vector3d& pos)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = q.toRotationMatrix();
    T.block<3,1>(0,3) = pos;
    return T;
}

} // namespace rm_motion_planner
