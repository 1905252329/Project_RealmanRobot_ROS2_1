/*
 * Util functions for collision avoidance
 */

#pragma once

#include <memory>
#include <vector>
#include <string>
#include <optional>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <fcl/fcl.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "kinlib.h"

namespace CollisionUtils {
    using kinlib::ErrorCodes;

class ObstacleBase {
public:
    virtual ~ObstacleBase() = default;

    virtual const fcl::CollisionObjectd &getCollisionObject() const = 0;

    virtual void setTransform(const Eigen::Vector3d &position, const Eigen::Matrix3d &orientation) = 0;
};

class BoxObstacle : public ObstacleBase {
    // 定义一个指向fcl::Boxd的共享指针
    std::shared_ptr<fcl::Boxd> geometry;
    // 定义一个fcl::CollisionObjectd对象
    fcl::CollisionObjectd collision_object;

public:
    BoxObstacle(const Eigen::Vector3d &dimensions, 
                const Eigen::Vector3d &position,
                const Eigen::Matrix3d &orientation = Eigen::Matrix3d::Identity())
        : geometry(std::make_shared<fcl::Boxd>(dimensions[0], dimensions[1], dimensions[2])),
          collision_object(geometry, fcl::Transform3d::Identity()) {
        collision_object.setTranslation(position);
        collision_object.setRotation(orientation);
    }

    const fcl::CollisionObjectd &getCollisionObject() const override {
        return collision_object;
    }

    void setTransform(const Eigen::Vector3d &position, const Eigen::Matrix3d &orientation) override {
        collision_object.setTranslation(position);
        collision_object.setRotation(orientation);
    }
};

class SphereObstacle : public ObstacleBase {
    std::shared_ptr<fcl::Sphered> geometry;
    fcl::CollisionObjectd collision_object;

public:
    SphereObstacle(double radius, 
                   const Eigen::Vector3d &position, 
                   const Eigen::Matrix3d &orientation = Eigen::Matrix3d::Identity())
        : geometry(std::make_shared<fcl::Sphered>(radius)),
          collision_object(geometry, fcl::Transform3d::Identity()) {
        collision_object.setTranslation(position);
        collision_object.setRotation(orientation);
    }

    const fcl::CollisionObjectd &getCollisionObject() const override {
        return collision_object;
    }

    void setTransform(const Eigen::Vector3d &position, const Eigen::Matrix3d &orientation) override {
    collision_object.setTranslation(position);
    collision_object.setRotation(orientation);
    }
};

class CylinderObstacle : public ObstacleBase {
    std::shared_ptr<fcl::Cylinderd> geometry;
    fcl::CollisionObjectd collision_object;

public:
    CylinderObstacle(double radius, double height, 
                     const Eigen::Vector3d &position, 
                     const Eigen::Matrix3d &orientation = Eigen::Matrix3d::Identity())
        : geometry(std::make_shared<fcl::Cylinderd>(radius, height)),
          collision_object(geometry, fcl::Transform3d::Identity()) {
        collision_object.setTranslation(position);
        collision_object.setRotation(orientation);
    }

    const fcl::CollisionObjectd &getCollisionObject() const override {
        return collision_object;
    }
    void setTransform(const Eigen::Vector3d &position, const Eigen::Matrix3d &orientation) override {
    collision_object.setTranslation(position);
    collision_object.setRotation(orientation);
    }
};

class MeshObstacle : public ObstacleBase {
    std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<double>>> geometry;
    fcl::CollisionObjectd collision_object;

public:
    MeshObstacle(const std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<double>>>& bvh_model,
                 const Eigen::Vector3d &position, 
                 const Eigen::Matrix3d &orientation)
        : geometry(bvh_model),
          collision_object(geometry, fcl::Transform3d::Identity()) {
        setTransform(position, orientation);
    }

    const fcl::CollisionObjectd &getCollisionObject() const override {
        return collision_object;
    }

    void setTransform(const Eigen::Vector3d &position, const Eigen::Matrix3d &orientation) override {
        collision_object.setTranslation(position);
        collision_object.setRotation(orientation);
    }
};


// Struct for link cylinder information (aborted since offset links are ignored automaticlly for now)
// struct LinkCylinderInfo : public ObstacleBase {
//     std::shared_ptr<ObstacleBase> obstacle; 
//     bool is_offset = false; // Indicates if this is an offset link

//     LinkCylinderInfo() = default;

//     LinkCylinderInfo(const std::shared_ptr<ObstacleBase> &obs, bool offset = false)
//         : obstacle(obs), is_offset(offset) {}

//     const fcl::CollisionObjectd &getCollisionObject() const override {
//         return obstacle->getCollisionObject();
//     }

//     void setTransform(const Eigen::Vector3d &position, const Eigen::Matrix3d &orientation) override {
//         obstacle->setTransform(position, orientation);
//     }
// };


// Utility functions for obstacle creation and collision checking
/*!
    \brief Creates a box obstacle.

    \param dimensions The dimensions of the box (x, y, z).
    \param position   The position of the box's center.
    \param orientation Optional orientation as a rotation matrix.
    \return           A shared pointer to a `BoxObstacle` object.
*/
std::shared_ptr<ObstacleBase> createBox(const Eigen::Vector3d &dimensions,
                                        const Eigen::Vector3d &position,
                                        const Eigen::Matrix3d &orientation = Eigen::Matrix3d::Identity());

/*!
    \brief Creates a sphere obstacle.

    \param radius      The radius of the sphere.
    \param position    The position of the sphere's center.
    \param orientation Optional orientation as a rotation matrix.
    \return            A shared pointer to a `SphereObstacle` object.
*/
std::shared_ptr<ObstacleBase> createSphere(double radius,
                                           const Eigen::Vector3d &position,
                                           const Eigen::Matrix3d &orientation = Eigen::Matrix3d::Identity());

/*!
    \brief Creates a cylinder obstacle.

    \param radius      The radius of the cylinder.
    \param height      The height of the cylinder.
    \param position    The position of the cylinder's center.
    \param orientation Optional orientation as a rotation matrix.
    \return            A shared pointer to a `CylinderObstacle` object.
*/
std::shared_ptr<ObstacleBase> createCylinder(double radius,
                                             double height,
                                             const Eigen::Vector3d &position,
                                             const Eigen::Matrix3d &orientation = Eigen::Matrix3d::Identity());

/*!
    \brief Removes an obstacle from a collection by index.

    \param obstacles The collection of shared pointers to obstacles.
    \param index     The index of the obstacle to remove.

    \return   Operation status as an ErrorCode.
*/
ErrorCodes removeObstacle(std::vector<std::shared_ptr<ObstacleBase>> &obstacles, size_t index);


/*!
    \brief Creates a grasp object.

    \param type         The type of the object ("box", "sphere", "cylinder").
    \param size         The dimensions of the object:
                        - For "box": Vector of size 3 (width, height, depth).
                        - For "sphere": Vector of size 1 (radius).
                        - For "cylinder": Vector of size 2 (radius, height).
    \param g_base_tool  Transformation matrix representing the object's position and orientation.

    \return             A shared pointer to the created object.
    \throws             std::invalid_argument if the type is unknown or the size is invalid.
*/
std::shared_ptr<ObstacleBase> createGraspObject(
    const std::string &type,
    const Eigen::Vector3d &size,
    const Eigen::Matrix4d &g_base_tool);

/*!
    \brief Precomputes link cylinders for the manipulator.

    \param num_links_ignore     Number of manipulator links to ignore (exclude base_link).
    \param radius_array         radius of each link (exclude base_link, so its size should match the joint count).
    \param g_intermediate       Intermediate transformations for the manipulator.
    \param rotation_adjustments Using intermediate transformations(in which are actually joints' orientations) 
                                as links' orientations. Considering that link cylinders' z axis should always point
                                to next joint(for fcl to generate correct models), directly using joints' orientation 
                                could be wrong, therefore introducing this param to rotate certain link to the right
                                orientation, this include a 3 by 1 vector representing the aixe, and a double 
                                representing how many degrees it should rotate(in radians).

    \return               A vector of shared pointers to `LinkCylinderInfo` objects.
*/
std::vector<std::shared_ptr<ObstacleBase>> armCylinderModel(
    const int num_links_ignore, 
    const std::vector<double> radius_array,
    const std::vector<Eigen::Matrix4d> &g_intermediate, 
    const std::vector<std::pair<Eigen::Vector3d, double>> &rotation_adjustments);

std::shared_ptr<ObstacleBase> createMeshFromSTL(
    const std::string& stl_path, 
    const Eigen::Matrix4d& transform); 

std::vector<std::shared_ptr<ObstacleBase>> armMeshModel(
    const int num_links_ignore,
    const std::vector<std::string>& stl_files, 
    const std::vector<Eigen::Matrix4d>& g_intermediate);

std::vector<std::shared_ptr<ObstacleBase>> buildLinkMeshes(const std::vector<std::string>& stl_files);

void updateLinkMeshTransforms(std::vector<std::shared_ptr<ObstacleBase>>& link_meshes,
                              const std::vector<Eigen::Matrix4d>& g_intermediate,
                              const std::vector<Eigen::Matrix4d>& mesh_offset_transforms);
                               
/*!
    \brief Checks collision between two obstacles.

    \param obj1               The first obstacle.
    \param obj2               The second obstacle.
    \param min_dist           Output: Minimum distance.
    \param contact_point_obj1 Output: Contact point on the first obstacle.
    \param contact_point_obj2 Output: Contact point on the second obstacle.
    
    \return   Operation status as an ErrorCode.
*/
ErrorCodes checkCollision(const ObstacleBase &obj1,
                    const ObstacleBase &obj2,
                    double &min_dist,
                    Eigen::Vector3d &contact_point_obj1,
                    Eigen::Vector3d &contact_point_obj2);

/*!
  \brief    Compute contact Jacobian for a non-grasped link
            If compute the contact Jacobian of a grasped object, 
            link_index should be the index of the last link 

  \details  Computes the contact Jacobian for the specified link
            using the contact point and spatial Jacobian.

  \param    link_index       Index of the link.
  \param    contact_point    Contact point on the link.
  \param    spatial_jacobian Spatial Jacobian of the manipulator.
  \param    contact_jacobian Output variable for the contact Jacobian.

  \return   Operation status as an ErrorCode.
*/
ErrorCodes getContactJacobian(int link_index,
                          const Eigen::Vector3d &contact_point,
                          const Eigen::MatrixXd &spatial_jacobian,
                          Eigen::MatrixXd &contact_jacobian);

/*!
    \brief Computes collision information for the manipulator links and obstacles.

    \param link_cylinders      Joint values for computing link cylinders and Jacobians
    \param obstacles           Collection of obstacles in the environment
    \param grasped_object      Grasped object (if any)
    \param spatial_jacobian    spatial Jacobian at the moment
    \param check_self_collision Do self collision check or not
    \param num_links_ignore    Number of manipulator links to ignore
    \param dof                 Degrees of freedom of the manipulator
    \param contact_normal_array Output: Contact normals for each link
    \param dist_array          Output: Distances for each link
    \param contact_points_array Output: Contact points for each link
    \param j_contact_array     Output: Contact Jacobians for each link

    \return   Operation status as an ErrorCode.
*/
ErrorCodes getCollisionInfo(
    const std::vector<std::shared_ptr<ObstacleBase>> &link_cylinders,
    const std::vector<std::shared_ptr<ObstacleBase>> &obstacles,
    const std::shared_ptr<ObstacleBase> &grasped_object,
    const Eigen::MatrixXd &spatial_jacobian,
    const bool check_self_collision,
    const int num_links_ignore,
    const int dof,
    Eigen::MatrixXd &contact_normal_array,
    std::vector<double> &dist_array,
    std::vector<Eigen::MatrixXd> &contact_points_array,
    std::vector<Eigen::MatrixXd> &j_contact_array);

} // namespace CollisionUtils
