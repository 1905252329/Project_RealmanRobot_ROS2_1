#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <trajectory_msgs/msg/joint_trajectory.hpp>


Eigen::Matrix4d EulerToTransformation(double x, double y, double z, double roll, double pitch, double yaw)
{
    Eigen::Matrix3d Rx, Ry, Rz;

    // Rotation about X-axis (roll)
    Rx << 1, 0, 0,
        0, cos(roll), -sin(roll),
        0, sin(roll), cos(roll);

    // Rotation about Y-axis (pitch)
    Ry << cos(pitch), 0, sin(pitch),
        0, 1, 0,
        -sin(pitch), 0, cos(pitch);

    // Rotation about Z-axis (yaw)
    Rz << cos(yaw), -sin(yaw), 0,
        sin(yaw), cos(yaw), 0,
        0, 0, 1;

    // Combine rotations (XYZ extrinsic)
    Eigen::Matrix3d R = Rz * Ry * Rx; // Reverse order for extrinsic

    // Create the 4x4 transformation matrix
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;                        // Set rotation
    T.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z); // Set translation

    return T;
}

Eigen::Matrix4d TransWithFixedRz(const Eigen::Matrix4d& T, double x, double y, double z) {
    // Extract the rotation matrix (top-left 3x3) and translation (last column)
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);  // Extract 3x3 rotation part
    // Eigen::Vector3d translation = T.block<3, 1>(0, 3);  // Extract translation part (unused)

    // Extract the yaw angle (theta) from the rotation matrix (rotation around Z-axis)
    double theta = std::atan2(R(1, 0), R(0, 0));

    // Reconstruct the rotation matrix with only Rz (Z-axis rotation)
    Eigen::Matrix3d Rz, Rx;
    Rz << cos(theta), -sin(theta), 0,
          sin(theta),  cos(theta), 0,
          0, 0, 1;

    Rx << 1, 0, 0,
          0, cos(M_PI), -sin(M_PI),
          0, sin(M_PI), cos(M_PI);
    // Create the new 4x4 transformation matrix
    Eigen::Matrix4d newT;
    newT.setIdentity();  // Initialize as identity matrix
    newT.block<3, 3>(0, 0) = Rz * Rx;  // Set the rotation part (top-left 3x3)
    newT(0, 3) = x;      // Set translation along X
    newT(1, 3) = y;      // Set translation along Y
    newT(2, 3) = z;      // Set translation along Z

    return newT;
}

void Convert_to_2dVector(const trajectory_msgs::msg::JointTrajectory& motion_plan,
                         std::vector<std::vector<double>>& matrix)
{
    if (motion_plan.points.empty()) {
        return;
    }

    matrix.clear();

    for (const auto& point : motion_plan.points)
    {
        std::vector<double> joints(6);

        for (size_t j = 0; j < 6; ++j)
        {
            joints[j] = point.positions[j];
        }

        matrix.push_back(joints);
    }
}