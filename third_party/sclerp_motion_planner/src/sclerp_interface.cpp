#include "sclerp_motion_planner/sclerp_interface.h"
#include "sclerp_motion_planner/interpolator.h"

#if DEBUG

#include "eigen_matrix_formatting.h"
#include <fstream>
#include <signal.h>
#include <ctime>
#include <sys/stat.h>
#include <errno.h>

#include <rosbag/bag.h>

namespace sclerp_interface
{

Eigen::IOFormat CSVFormat(Eigen::FullPrecision,0,",","\n","","","","");

std::string log_folder_name = "sclerp_interface";
std::string home_dir(getenv("HOME"));
std::string log_dir = home_dir + "/logs/" + log_folder_name;

std::string timestamp_str = "Y%YM%mD%d_T%H%M%S";

bool log_folder_error = false;
bool log_file_error = false;

rosbag::Bag bag;

}

#endif

namespace sclerp_interface
{

ScLERPInterface::ScLERPInterface( const std::string base_link,
                                  const std::string tip_link,
                                  const std::shared_ptr<rclcpp::Node> &nh,
                                  const std::string &urdf_path,
                                  const std::vector<std::string> &stl_paths)
                                  
  : nh_(nh), name_("ScLERPInterface"), base_link_name(base_link), tip_link_name(tip_link), local_urdf_file(urdf_path), stl_paths_(stl_paths)
{

  init_failed = false;
  urdf::Model urdf_model;
  KDL::Tree robot_tree;
  KDL::Chain manip_chain;
  std::map<std::string, urdf::JointSharedPtr> jnt_list;

  if (!local_urdf_file.empty())
  {
    RCLCPP_INFO(nh_->get_logger(), "Reading URDF from file: %s", local_urdf_file.c_str());

    if (!urdf_model.initFile(local_urdf_file))
    {
      RCLCPP_ERROR(nh_->get_logger(), "Error loading URDF file!");
      init_failed = true;
    }

    if (!kdl_parser::treeFromUrdfModel(urdf_model, robot_tree))
    {
      RCLCPP_ERROR(nh_->get_logger(), "Error constructing KDL Tree!");
      init_failed = true;
    }
  }
  else
  {
    RCLCPP_INFO(nh_->get_logger(), "Reading URDF from Parameter Server");

    if (!urdf_model.initString("/move_group/robot_description")) {
      RCLCPP_ERROR(nh_->get_logger(), "Failed to load URDF from parameter server!");
      init_failed = true;
    }

    nh_->declare_parameter<std::string>("/move_group/robot_description", "");
    nh_->get_parameter("/move_group/robot_description", robot_desc_string);

    if (!kdl_parser::treeFromString(robot_desc_string, robot_tree))
    {
      RCLCPP_ERROR(nh_->get_logger(), "Failed to construct KDL Tree from Parameter Server!");
      init_failed = true;
    }
  }

  jnt_list = urdf_model.joints_;
  if (!robot_tree.getChain(base_link_name, tip_link_name, manip_chain))
  {
    RCLCPP_ERROR(nh_->get_logger(), "getChain() failed from '%s' to '%s'", base_link_name.c_str(), tip_link_name.c_str());
    init_failed = true;
  }

  if (manip_chain.getNrOfSegments() == 0)
  {
    RCLCPP_ERROR(nh_->get_logger(), "KDL Chain is empty!");
    init_failed = true;
  }

  if(init_failed)
  { 
    return;  
  }

  RCLCPP_WARN(nh_->get_logger(), "Chain has %u segments", manip_chain.getNrOfSegments());

  Eigen::Matrix4d t_ref;
  Eigen::Matrix4d t_jnt;
  Eigen::Matrix4d t_tip;

  Eigen::Vector4d p_jnt;
  Eigen::Vector4d v_jnt;

  t_ref <<  1, 0, 0, 0, 
            0, 1, 0, 0, 
            0, 0, 1, 0, 
            0, 0, 0, 1;

  t_tip <<  1, 0, 0, 0, 
            0, 1, 0, 0, 
            0, 0, 1, 0, 
            0, 0, 0, 1;

  p_jnt << 0, 0, 0, 1;

  v_jnt << 0, 0, 0, 0;

  KDL::Frame frame_to_tip;
  
  kinlib::Manipulator manip;
  kinlib::JointType jnt_type;
  kinlib::JointLimits jnt_lim;
  
  for(int itr = 0; itr < manip_chain.getNrOfSegments(); itr++)
  {
    KDL::Segment chain_seg = manip_chain.getSegment(itr);
    std::string seg_name = chain_seg.getName();

    // Get corresponding URDF link
    urdf::LinkConstSharedPtr urdf_link = urdf_model.getLink(seg_name);
    Eigen::Matrix4d mesh_offset = getMeshOffsetFromURDF(urdf_link);

    if (chain_seg.getJoint().getType() != KDL::Joint::JointType::Fixed) {
      mesh_offset_transforms_.push_back(mesh_offset);
    }

    KDL::Joint jnt = chain_seg.getJoint();
    KDL::RigidBodyInertia inertia_prop = chain_seg.getInertia();
    KDL::RotationalInertia rot_inertia = inertia_prop.getRotationalInertia();

    Eigen::Matrix3d rot_inertia_mat;

    frame_to_tip = chain_seg.getFrameToTip();

    urdf::JointSharedPtr jnt_ptr(jnt_list[jnt.getName()]);

    for(int r_itr = 0; r_itr < 3; r_itr++)
    {
      for(int c_itr = 0; c_itr < 3; c_itr++)
      {
        t_tip(r_itr, c_itr) = frame_to_tip.M.data[(r_itr * 3) + c_itr];
        rot_inertia_mat(r_itr, c_itr) = rot_inertia.data[(r_itr * 3) + c_itr];
      }

      t_tip(r_itr, 3) = frame_to_tip.p.data[r_itr];

      p_jnt(r_itr) = jnt.JointOrigin().data[r_itr];
      v_jnt(r_itr) = jnt.JointAxis().data[r_itr];
    }

    Eigen::Vector4d w_p_jnt = t_ref * p_jnt;
    Eigen::Vector4d w_v_jnt = t_ref * v_jnt;

    t_ref = t_ref * t_tip;

    if(jnt.getType() == KDL::Joint::JointType::RotAxis)
    { 
      jnt_lim.upper_limit_ = jnt_ptr->limits->upper;
      jnt_lim.lower_limit_ = jnt_ptr->limits->lower;

      jnt_type = kinlib::JointType::Revolute;
      manip.addJoint(jnt_type, jnt.getName(), w_v_jnt, w_p_jnt, jnt_lim, t_ref);
    }
    else
    {
      if(itr < (manip_chain.getNrOfSegments()))
      {
        manip.modifyEndJointTipPose(t_ref);
      }
    }
    
  }
  
  kinlib_solver_ = kinlib::KinematicsSolver(manip, stl_paths, mesh_offset_transforms_);
}
  
bool ScLERPInterface::solve(const Eigen::VectorXd &init_jnt_values,
                            const Eigen::Matrix4d &g_f,
                            trajectory_msgs::msg::JointTrajectory &jnt_trajectory)
{
  // 定义一个4x4矩阵g_i
  Eigen::Matrix4d g_i;
  
  // 调用kinlib_solver_的getFK方法，计算初始关节值对应的末端执行器位姿，并存储在g_i中
  kinlib_solver_.getFK(init_jnt_values, g_i);

  // 🧪 打印初始和目标位姿
  std::stringstream ss_init;
  ss_init << "\n初始位姿矩阵:\n" << g_i << std::endl;
  RCLCPP_INFO(nh_->get_logger(), "%s", ss_init.str().c_str());
  
  std::stringstream ss_target;
  ss_target << "\n目标位姿矩阵:\n" << g_f << std::endl;
  RCLCPP_INFO(nh_->get_logger(), "%s", ss_target.str().c_str());

  // 调用kinlib_solver_的getMotionPlan方法，计算从初始关节值到目标末端执行器位姿g_f的运动规划，并将结果存储在jnt_trajectory中
  kinlib::ErrorCodes plan_result = kinlib_solver_.getMotionPlan(
                                      init_jnt_values,
                                      g_i,
                                      g_f,
                                      jnt_trajectory);
        
  // 📝 🧪 Start(调试用，用完注释或删除)————>
  // 如果运动规划成功，则进行轨迹终点验证
  if(plan_result == kinlib::ErrorCodes::OPERATION_SUCCESS)
  {
    // 验证轨迹终点是否与目标位姿一致
    if (!jnt_trajectory.points.empty()) {
      // 获取轨迹最后一个点的关节值
      Eigen::VectorXd final_joints(7);
      for (int i = 0; i < 7 && i < static_cast<int>(jnt_trajectory.points.back().positions.size()); i++) {
        final_joints(i) = jnt_trajectory.points.back().positions[i];
      }
      
      // 计算FK验证
      Eigen::Matrix4d final_pose;
      kinlib_solver_.getFK(final_joints, final_pose);
      
      // 计算与目标位姿的差异
      double position_diff = (final_pose.block<3,1>(0,3) - g_f.block<3,1>(0,3)).norm();
      Eigen::Quaterniond final_quat(final_pose.block<3,3>(0,0));
      Eigen::Quaterniond target_quat(g_f.block<3,3>(0,0));
      double rotation_diff = final_quat.angularDistance(target_quat);
      
      // 输出验证信息
      std::stringstream ss;
      ss << "\n目标位姿矩阵:\n" << g_f << std::endl;
      ss << "\n轨迹终点FK计算结果:\n" << final_pose << std::endl;
      ss << "位置差异: " << position_diff << ", 旋转差异: " << rotation_diff << std::endl;
      RCLCPP_INFO(nh_->get_logger(), "%s", ss.str().c_str());
      
      // 如果差异过大，输出警告
      if (position_diff > 0.01 || rotation_diff > 0.1) {
        RCLCPP_WARN(nh_->get_logger(), "轨迹终点与目标位姿差异较大，请检查坐标系一致性或规划器参数");
      }
      
    }
    // End <———— 📝 🧪
    
    return true;
  }
  else
  {
    RCLCPP_ERROR(nh_->get_logger(), "ScLERP规划失败，错误码: %d", plan_result);
    return false;
  }
                                      
}

/**
* @brief 计算并返回关节轨迹，同时考虑碰撞避免
*
* 使用提供的初始关节值、目标位姿、是否检查自碰撞、忽略的连杆数量、障碍物和抓取的物体，计算关节轨迹。
*
* @param init_jnt_values 初始关节值，类型为Eigen::VectorXd
* @param g_f 目标位姿，类型为Eigen::Matrix4d
* @param check_self_collision 是否检查自碰撞，类型为bool
* @param num_links_ignore 需要忽略的连杆数量，类型为int
* @param obstacles 障碍物列表，类型为std::vector<std::shared_ptr<CollisionUtils::ObstacleBase>>
* @param grasped_object 抓取的物体，类型为std::shared_ptr<CollisionUtils::ObstacleBase>
* @param jnt_trajectory 存储计算出的关节轨迹，类型为trajectory_msgs::msg::JointTrajectory
*
* @return 如果计算成功返回true，否则返回false
*/
bool ScLERPInterface::solve(const Eigen::VectorXd &init_jnt_values,
                            const Eigen::Matrix4d &g_f,
                            const bool check_self_collision,
                            const int num_links_ignore,
                            const std::vector<std::shared_ptr<CollisionUtils::ObstacleBase>> &obstacles,
                            const std::shared_ptr<CollisionUtils::ObstacleBase> &grasped_object,
                            trajectory_msgs::msg::JointTrajectory &jnt_trajectory)
{ 
  if (stl_paths_.empty())
  {
    std::cerr << "[Sclerp] You should not use collision avoidance planner without providing link meshes.\n";
    return false;
  }
  Eigen::Matrix4d g_i;
  
  kinlib_solver_.getFK(init_jnt_values, g_i);

  kinlib::ErrorCodes plan_result = kinlib_solver_.getMotionPlanWithCollisionAvoidance(
                                      init_jnt_values,
                                      g_i,
                                      g_f,
                                      check_self_collision,
                                      num_links_ignore,
                                      obstacles,
                                      grasped_object,
                                      jnt_trajectory);
                                      
  if(plan_result == kinlib::ErrorCodes::OPERATION_SUCCESS)
  {
    return true;
  }
  else
  {
    return false;  
  }
                                      
}

bool ScLERPInterface::planMultiStageTrajectory(const Eigen::VectorXd& init_jnt_values,
                                               const std::vector<Eigen::Matrix4d>& ee_targets,
                                               std::vector<std::vector<double>>& final_trajectory,
                                               const std::vector<double>& max_velocity,
                                               const std::vector<double>& max_acceleration,
                                               double frequency,
                                               bool resample)
{
    Eigen::VectorXd jnt_values = init_jnt_values;
    trajectory_msgs::msg::JointTrajectory full_plan;
    std::set<size_t> key_indices = {0};
    size_t point_offset = 0;

    for (size_t i = 0; i < ee_targets.size(); ++i) {
        trajectory_msgs::msg::JointTrajectory stage_plan;

        if (!solve(jnt_values, ee_targets[i], stage_plan)) {
            std::cerr << "[Stage " << i << "] Planning failed.\n";
            return false;
        }

        if (stage_plan.points.size() == 10000) {
            std::cerr << "[Stage " << i << "] Target not reachable.\n";
            return false;
        }

        // Update joint state
        for (size_t j = 0; j < jnt_values.size(); ++j)
            jnt_values[j] = stage_plan.points.back().positions[j];

        point_offset += stage_plan.points.size();
        key_indices.insert(point_offset - 1);

        full_plan.points.insert(full_plan.points.end(),
                                stage_plan.points.begin(), stage_plan.points.end());
    }

    std::vector<std::vector<double>> path;
    convertToMatrix(full_plan, path);

    QuinticInterpolator interpolator(path, max_velocity, max_acceleration, frequency);
    if (resample)
        interpolator.enableResampling(true, key_indices, 5.0);

    interpolator.computeTrajectory();
    final_trajectory = interpolator.getTrajectory();
    return true;
}


Eigen::Matrix4d ScLERPInterface::getMeshOffsetFromURDF(const urdf::LinkConstSharedPtr& link)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    if (link && link->visual)
    {
        const urdf::Rotation& r = link->visual->origin.rotation;
        const urdf::Vector3& p = link->visual->origin.position;

        Eigen::Quaterniond q(r.w, r.x, r.y, r.z);
        Eigen::Matrix3d R = q.toRotationMatrix();

        T.block<3,3>(0,0) = R;
        T.block<3,1>(0,3) = Eigen::Vector3d(p.x, p.y, p.z);
    }

    return T;
}

void ScLERPInterface::convertToMatrix(const trajectory_msgs::msg::JointTrajectory& motion_plan,
                                          std::vector<std::vector<double>>& matrix)
{
    matrix.clear();
    if (motion_plan.points.empty()) return;

    for (const auto& point : motion_plan.points) {
        std::vector<double> joints(point.positions.size());
        for (size_t j = 0; j < point.positions.size(); ++j) {
            joints[j] = point.positions[j];
        }
        matrix.push_back(joints);
    }
}

}

