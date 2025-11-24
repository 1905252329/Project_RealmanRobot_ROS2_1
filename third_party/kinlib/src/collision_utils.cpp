/*
 * Util functions for collision avoidance
 */

#include "kinlib/collision_utils.h"
#include "kinlib/kinlib_kinematics.h" 


namespace CollisionUtils {

using kinlib::ErrorCodes;

std::shared_ptr<ObstacleBase> createBox(const Eigen::Vector3d &dimensions, 
                                        const Eigen::Vector3d &position,
                                        const Eigen::Matrix3d &orientation) {
    return std::make_shared<BoxObstacle>(dimensions, position, orientation);
}

std::shared_ptr<ObstacleBase> createSphere(double radius, 
                                           const Eigen::Vector3d &position,
                                           const Eigen::Matrix3d &orientation) {
    return std::make_shared<SphereObstacle>(radius, position, orientation);
}

std::shared_ptr<ObstacleBase> createCylinder(double radius, double height, 
                                             const Eigen::Vector3d &position,
                                             const Eigen::Matrix3d &orientation) {
    return std::make_shared<CylinderObstacle>(radius, height, position, orientation);
}

ErrorCodes removeObstacle(std::vector<std::shared_ptr<ObstacleBase>> &obstacles, size_t index) {
    if (index >= obstacles.size()) {
        return ErrorCodes::PARAMETER_ERROR;
    }

    obstacles.erase(obstacles.begin() + index);
    return ErrorCodes::OPERATION_SUCCESS;
}


std::shared_ptr<ObstacleBase> createGraspObject(const std::string &type,                  
                                                const Eigen::Vector3d &size,          
                                                const Eigen::Matrix4d &g_base_tool) {

    // Extract position and orientation from g_base_tool
    Eigen::Vector3d position = g_base_tool.block<3, 1>(0, 3);
    Eigen::Matrix3d orientation = g_base_tool.block<3, 3>(0, 0);

    // Use the appropriate utility function to create the object
    if (type == "box") {
        if (size.size() != 3) {
            throw std::invalid_argument("Box requires 3 dimensions (width, height, depth)");
        }
        return createBox(size, position, orientation);
    } else if (type == "sphere") {
        if (size.size() != 1) {
            throw std::invalid_argument("Sphere requires 1 dimension (radius)");
        }
        return createSphere(size[0], position, orientation);
    } else if (type == "cylinder") {
        if (size.size() != 2) {
            throw std::invalid_argument("Cylinder requires 2 dimensions (radius, height)");
        }
        return createCylinder(size[0], size[1], position, orientation);
    } else {
        throw std::invalid_argument("Unknown grasp object type: " + type);
    }
}

std::vector<std::shared_ptr<ObstacleBase>> armCylinderModel(
    const int num_links_ignore, 
    const std::vector<double> radius_array,
    const std::vector<Eigen::Matrix4d> &g_intermediate, 
    const std::vector<std::pair<Eigen::Vector3d, double>> &rotation_adjustments) {

    std::vector<std::shared_ptr<ObstacleBase>> link_cylinders;

    // Iterate over the joints, skipping the base and tool links
    for (size_t i = 1; i < g_intermediate.size() - 1; ++i) {
        // Skip the links that are to be ignored
        if (static_cast<int>(i - 1) < num_links_ignore) {
            link_cylinders.push_back(nullptr);  // Placeholder for ignored links
            continue;
        }

        // Compute cylinder parameters
        double radius = radius_array[i - 1];  // Set the radius of the link
        double height = (g_intermediate[i + 1].block<3, 1>(0, 3) - g_intermediate[i].block<3, 1>(0, 3)).norm();

        // Compute the position and orientation of the cylinder
        Eigen::Vector3d position = g_intermediate[i].block<3, 1>(0, 3); // Translation vector
        Eigen::Matrix3d orientation = g_intermediate[i].block<3, 3>(0, 0); // Rotation matrix

        // Apply the rotation adjustment for this link
        const auto &[rotation_axis, rotation_angle] = rotation_adjustments[i - 1];
        Eigen::AngleAxisd rotation_adjustment(rotation_angle, rotation_axis.normalized());
        orientation = orientation * rotation_adjustment.toRotationMatrix();

        // Adjust the position to the center of the cylinder
        Eigen::Vector3d cylinder_axis_world = orientation * Eigen::Vector3d(0, 0, 1); // Z-axis in world frame
        position += 0.5 * height * cylinder_axis_world;

        // Create the cylinder obstacle using the utility function
        auto cylinder = createCylinder(radius, height, position, orientation);

        // Add the cylinder to the list of obstacles
        link_cylinders.push_back(cylinder);
    }

    return link_cylinders;
}

std::shared_ptr<ObstacleBase> createMeshFromSTL(
    const std::string& stl_path, 
    const Eigen::Matrix4d& transform) {
    
    // Load STL file using Assimp
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(stl_path, aiProcess_Triangulate);
    if (!scene || !scene->mMeshes) {
        throw std::runtime_error("Failed to load STL file: " + stl_path);
    }

    // Get mesh from file
    aiMesh* mesh = scene->mMeshes[0]; // Assuming one mesh per file
    auto bvh_model = std::make_shared<fcl::BVHModel<fcl::OBBRSS<double>>>();

    std::vector<fcl::Vector3d> vertices;
    std::vector<fcl::Triangle> triangles;

    // Convert vertices
    for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
        vertices.emplace_back(mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z);
    }

    // Convert faces
    for (unsigned int i = 0; i < mesh->mNumFaces; ++i) {
        aiFace face = mesh->mFaces[i];
        if (face.mNumIndices == 3) { // Only triangles
            triangles.emplace_back(face.mIndices[0], face.mIndices[1], face.mIndices[2]);
        }
    }

    // Load data into BVH model
    bvh_model->beginModel();
    bvh_model->addSubModel(vertices, triangles);
    bvh_model->endModel();

    // Extract position and orientation for consistency
    Eigen::Vector3d position = transform.block<3,1>(0,3);
    Eigen::Matrix3d orientation = transform.block<3,3>(0,0);

    return std::make_shared<MeshObstacle>(bvh_model, position, orientation);
}

std::vector<std::shared_ptr<ObstacleBase>> buildLinkMeshes(const std::vector<std::string>& stl_files) {
    std::vector<std::shared_ptr<ObstacleBase>> link_meshes;

    for (size_t i = 0; i < stl_files.size(); ++i) {
        const std::string& mesh_path = stl_files[i];
        auto mesh_obj = createMeshFromSTL(mesh_path, Eigen::Matrix4d::Identity());
        link_meshes.push_back(mesh_obj);
    }

    return link_meshes;
}

void updateLinkMeshTransforms(std::vector<std::shared_ptr<ObstacleBase>>& link_meshes,
                              const std::vector<Eigen::Matrix4d>& g_intermediate,
                              const std::vector<Eigen::Matrix4d>& mesh_offset_transforms) {
    for (size_t i = 1; i < link_meshes.size(); ++i) {
        if (link_meshes[i]) {
            // Apply the mesh offset transform
            Eigen::Matrix4d T_world_to_mesh = g_intermediate[i] * mesh_offset_transforms[i - 1];

            Eigen::Vector3d position = T_world_to_mesh.block<3,1>(0, 3);
            Eigen::Matrix3d rotation = T_world_to_mesh.block<3,3>(0, 0);

            link_meshes[i]->setTransform(position, rotation);
        }
    }
}

ErrorCodes checkCollision(const ObstacleBase &obj1,
                         const ObstacleBase &obj2,
                         double &min_dist,
                         Eigen::Vector3d &contact_point_obj1,
                         Eigen::Vector3d &contact_point_obj2) {

    // // Extract and print geometry dimensions
    // auto printGeometryInfo = [](const fcl::CollisionObjectd &obj) {
    //     const auto &geom = obj.collisionGeometry();
    //     if (auto box = dynamic_cast<const fcl::Boxd *>(geom.get())) {
    //         std::cout << "Geometry Type: Box" << std::endl;
    //         std::cout << "Dimensions: " << box->side[0] << " x " << box->side[1] << " x " << box->side[2] << std::endl;
    //     } else if (auto cylinder = dynamic_cast<const fcl::Cylinderd *>(geom.get())) {
    //         std::cout << "Geometry Type: Cylinder" << std::endl;
    //         std::cout << "Radius: " << cylinder->radius << ", Height: " << cylinder->lz << std::endl;
    //     } else if (auto sphere = dynamic_cast<const fcl::Sphered *>(geom.get())) {
    //         std::cout << "Geometry Type: Sphere" << std::endl;
    //         std::cout << "Radius: " << sphere->radius << std::endl;
    //     } else {
    //         std::cout << "Unknown Geometry Type" << std::endl;
    //     }
    // };

    // // Print pose and geometry information of obj1
    // std::cout << "Object 1 Position: " << obj1.getCollisionObject().getTranslation().transpose() << std::endl;
    // std::cout << "Object 1 Orientation:\n" << obj1.getCollisionObject().getRotation() << std::endl;
    // printGeometryInfo(obj1.getCollisionObject());

    // // Print pose and geometry information of obj2
    // std::cout << "Object 2 Position: " << obj2.getCollisionObject().getTranslation().transpose() << std::endl;
    // std::cout << "Object 2 Orientation:\n" << obj2.getCollisionObject().getRotation() << std::endl;
    // printGeometryInfo(obj2.getCollisionObject());

    // Perform the distance computation
    fcl::DistanceRequestd request;
    fcl::DistanceResultd result;

    request.enable_nearest_points = true;
    request.gjk_solver_type = fcl::GJKSolverType::GST_INDEP; // default solver/GST_LIBCCD doesn't work
    request.distance_tolerance = 1e-6;

    min_dist = fcl::distance(&obj1.getCollisionObject(), &obj2.getCollisionObject(), request, result);

    if (std::isnan(min_dist)) {
        std::cerr << "Error: min_dist is NaN!" << std::endl;
        return ErrorCodes::OPERATION_FAILURE;
    }

    if (min_dist >= 0) {
        contact_point_obj1 = result.nearest_points[0];
        contact_point_obj2 = result.nearest_points[1];
        return ErrorCodes::OPERATION_SUCCESS;
    }
    
    std::cerr << "[Collision Check] min_dist < 0, there is a penetration." << std::endl;
    return ErrorCodes::OPERATION_FAILURE;
}

ErrorCodes getContactJacobian(int link_index,
                              const Eigen::Vector3d &contact_point,
                              const Eigen::MatrixXd &spatial_jacobian,
                              Eigen::MatrixXd &contact_jacobian) {
    
    int col_num = spatial_jacobian.cols();

    if (link_index < 0 || link_index >= col_num) {
      return ErrorCodes::PARAMETER_ERROR;
    }

    Eigen::Matrix3d P_hat = kinlib::getSkewMatrix(contact_point);

    Eigen::MatrixXd Js = Eigen::MatrixXd::Zero(6, col_num); 
    Js.leftCols(link_index) = spatial_jacobian.leftCols(link_index);

    Eigen::MatrixXd temp = (Eigen::MatrixXd(3, 6) << Eigen::Matrix3d::Identity(), -P_hat).finished() * Js;

    contact_jacobian.resize(6, temp.cols());
    contact_jacobian << temp, Eigen::MatrixXd::Zero(3, temp.cols());

    return ErrorCodes::OPERATION_SUCCESS;
}

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
    std::vector<Eigen::MatrixXd> &j_contact_array) {
    
    int grasped_obj_con = (grasped_object != nullptr) ? 1 : 0;
    int total_links = dof - num_links_ignore;

    // Initialize output arrays
    dist_array.assign(total_links + grasped_obj_con, 1000.0); // Large initial distance
    contact_normal_array = Eigen::MatrixXd::Zero(6, total_links + grasped_obj_con);
    contact_points_array.resize(total_links + grasped_obj_con, Eigen::MatrixXd(3, 2));
    j_contact_array.resize(total_links + grasped_obj_con);

    for (const auto &obstacle : obstacles) {
        for (int itr_index = 1; itr_index <= total_links; ++itr_index) {
            int num_link = num_links_ignore + itr_index;
    
            if (static_cast<size_t>(num_link) >= link_cylinders.size()) {
                throw std::out_of_range("Link cylinder index exceeds available cylinders.");
            }
    
            const auto &current_cylinder = link_cylinders[num_link];
            if (!current_cylinder) {
                throw std::runtime_error("Link cylinder is empty. Check link cylinder model creation or num_links_ignore.");
            }
    
            Eigen::Vector3d cp_obj, cp_cylinder;
            double min_d;
    
            auto result = checkCollision(*obstacle, *current_cylinder, min_d, cp_obj, cp_cylinder);
    
            if (result != ErrorCodes::OPERATION_SUCCESS) {
                std::cerr << "[KinematicsSolver] Failed to do collision check with environment.\n";
                return ErrorCodes::OPERATION_FAILURE;
            }
    
            if (min_d < dist_array[itr_index - 1]) {
                dist_array[itr_index - 1] = min_d;
                contact_normal_array.block<3, 1>(0, itr_index - 1) = (cp_cylinder - cp_obj).normalized();
                contact_points_array[itr_index - 1] << cp_obj, cp_cylinder;
            }
        }
    }
    

    // Process grasped object separately
    if (grasped_object) {
        for (const auto &obstacle : obstacles) {
            Eigen::Vector3d cp_obstacle, cp_grasped;
            double min_d;
    
            auto result = checkCollision(*obstacle, *grasped_object, min_d, cp_obstacle, cp_grasped);
    
            if (result != ErrorCodes::OPERATION_SUCCESS) {
                std::cerr << "[KinematicsSolver] Failed to do collision check with grasped object.\n";
                return ErrorCodes::OPERATION_FAILURE;
            }
    
            if (min_d < dist_array[total_links]) {
                dist_array[total_links] = min_d;
                contact_normal_array.block<3, 1>(0, total_links) = (cp_grasped - cp_obstacle).normalized();
                contact_points_array[total_links] << cp_obstacle, cp_grasped;
            }
        }
    }

    // Self-collision check: include base link as environment obstacle in self-collision checks
    if (check_self_collision) {
        for (int i = 1; i < total_links; ++i) {  // Start from 1 to skip base link
            for (int j = 0; j < total_links; ++j) {  // Start from 0 to total_links, to include all links as obstacles
                // Skip adjacent links and the link itself
                if (j == i || j == i - 1 || j == i + 1) { 
                    continue;
                }
    
                const auto &link1 = link_cylinders[i];  // Link i (moving link)
                const auto &link2 = link_cylinders[j];  // Link j (obstacle link)
                Eigen::Vector3d cp_link1, cp_link2;
                double min_d;
    
                // Perform the self-collision check
                auto result = checkCollision(*link2, *link1, min_d, cp_link2, cp_link1);

                if (result != ErrorCodes::OPERATION_SUCCESS) {
                    std::cerr << "[KinematicsSolver] checkCollision() failed for link " << i << std::endl;
                    return ErrorCodes::OPERATION_FAILURE;
                }
                
                if (min_d < dist_array[i - 1]) {
                    dist_array[i - 1] = min_d;
                    contact_normal_array.block<3, 1>(0, i - 1) = (cp_link1 - cp_link2).normalized();
                    contact_points_array[i - 1] << cp_link2, cp_link1;
                }   
            }
        }
    }

    // Compute the contact Jacobians
    for (int i = 0; i < total_links + grasped_obj_con; ++i) { 
        Eigen::Vector3d contact_point = contact_points_array[i].col(1);

        // If this is the last iteration (grasped object), adjust the index
        if (grasped_object && i == total_links) {
            // Use the index of the last link
            if (getContactJacobian(i + num_links_ignore - 1, contact_point, spatial_jacobian, j_contact_array[i]) != ErrorCodes::OPERATION_SUCCESS) {
                return ErrorCodes::OPERATION_FAILURE;
            }
        } 
        else {
            // Regular case for all other links
            if (getContactJacobian(i + num_links_ignore, contact_point, spatial_jacobian, j_contact_array[i]) != ErrorCodes::OPERATION_SUCCESS) {
                return ErrorCodes::OPERATION_FAILURE;
            }
        }
    }

    return ErrorCodes::OPERATION_SUCCESS;
}

} // namespace CollisionUtils
