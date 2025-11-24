#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace rm_motion_planner {

// Convert 6x1 vector [x,y,z,rx,ry,rz] where rx,ry,rz are ZYX intrinsic Euler angles (radians) to 4x4 homogeneous matrix
Eigen::Matrix4d tcpPose2HomogenMatrix(const Eigen::Matrix<double,6,1>& tcpPose);

// Convert quaternion+position to homogeneous matrix
Eigen::Matrix4d tcpPose2HomogenMatrix(const Eigen::Quaterniond& q, const Eigen::Vector3d& pos);

// Convert ZYX intrinsic Euler angles to rotation matrix (Rx, Ry, Rz are in radians)
Eigen::Matrix3d eulerToRotationMatrixZYX(double rx, double ry, double rz);

// Rotation matrix to ZYX Euler angles
Eigen::Vector3d rotationMatrixToEulerZYX(const Eigen::Matrix3d& R);

} // namespace rm_motion_planner
