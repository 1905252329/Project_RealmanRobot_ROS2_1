/*
   Kinemtic Solver definitions for kinlib
 */

#include <iostream>
#include "kinlib/kinlib_kinematics.h"

Eigen::IOFormat PrintFormat(4,0,", ","\n");

#define TAU_MAX_VAL     0.1
#define TAU_BRAKE_VAL   0.9

#if DEBUG

#include "eigen_matrix_formatting.h"
#include <signal.h>
#include <fstream>
#include <ctime>
#include <sys/stat.h>
#include <errno.h>

namespace kinlib
{

Eigen::IOFormat CSVFormat(Eigen::FullPrecision,0,",","\n","","","","");

std::string log_folder_name = "kinlib_kinematics";
std::string home_dir(getenv("HOME"));
std::string log_dir = home_dir + "/logs/" + log_folder_name;

std::string timestamp_str = "Y%YM%mD%d_T%H%M%S";

bool log_folder_error = false;
bool log_file_error = false;
bool log_plan_request_error = false;


}

#endif

namespace kinlib
{
class Polynomial
{
  private:
    double brake_start;
    double max_amp;
    
    Eigen::Matrix3d mat;
    Eigen::Vector3d coeff;
    Eigen::Vector3d r_val;
  
  public:
    Polynomial()
    {
      brake_start = TAU_BRAKE_VAL;
      max_amp = TAU_MAX_VAL;
      
      mat(0,0) = std::pow(brake_start, 3);
      mat(0,1) = std::pow(brake_start, 4);
      mat(0,2) = std::pow(brake_start, 5);

      mat(1,0) = 3 * std::pow(brake_start, 2);
      mat(1,1) = 4 * std::pow(brake_start, 3);
      mat(1,2) = 5 * std::pow(brake_start, 4);

      mat(2,0) = 6 * brake_start;
      mat(2,1) = 12 * std::pow(brake_start, 2);
      mat(2,2) = 20 * std::pow(brake_start, 3);
      
      r_val(0) = max_amp;
      r_val(1) = 0;
      r_val(2) = 0;
      
      coeff = mat.inverse() * r_val;
    }

    Polynomial(double tau_brake, double tau_max)
    {
      brake_start = tau_brake;
      max_amp = tau_max;
      
      mat(0,0) = std::pow(brake_start, 3);
      mat(0,1) = std::pow(brake_start, 4);
      mat(0,2) = std::pow(brake_start, 5);

      mat(1,0) = 3 * std::pow(brake_start, 2);
      mat(1,1) = 4 * std::pow(brake_start, 3);
      mat(1,2) = 5 * std::pow(brake_start, 4);

      mat(2,0) = 6 * brake_start;
      mat(2,1) = 12 * std::pow(brake_start, 2);
      mat(2,2) = 20 * std::pow(brake_start, 3);
      
      r_val(0) = max_amp;
      r_val(1) = 0;
      r_val(2) = 0;
      
      coeff = mat.inverse() * r_val;
    }
    
    double getValue(double t)
    {
      if(t >= brake_start)
      {
        return max_amp;
      }
      
      return ((coeff(0) * std::pow(t, 3)) +
              (coeff(1) * std::pow(t, 4)) +
              (coeff(2) * std::pow(t, 5)));
    }
};

double positionDistance(const Eigen::Matrix4d &t1, const Eigen::Matrix4d &t2)
{
  Eigen::Vector4d p_delta = t1.col(3) - t2.col(3);
  return p_delta.norm();
}

double positionDistance(const Eigen::VectorXd &p1, const Eigen::VectorXd &p2)
{
  Eigen::Vector3d p_delta = p1.head<3>() - p2.head<3>();
  return p_delta.norm();
}

double rotationDistance(const Eigen::Matrix4d &t1, const Eigen::Matrix4d &t2)
{
  Eigen::Quaterniond q1(t1.block<3,3>(0,0));
  Eigen::Quaterniond q2(t2.block<3,3>(0,0));

  return rotationDistance(q1, q2);
}

double rotationDistance(const Eigen::Quaterniond &q1, 
                        const Eigen::Quaterniond &q2)
{
  Eigen::Vector4d q1_coeff = q1.coeffs();
  Eigen::Vector4d q2_coeff = q2.coeffs();

  double d1 = (q1_coeff - q2_coeff).norm();
  double d2 = (q1_coeff + q2_coeff).norm();

  return (d1 > d2 ? d2 : d1);
}

double rotationDistance(eigen_ext::DualQuat &dq_1,
                        eigen_ext::DualQuat &dq_2)
{
  Eigen::Vector4d q1_coeff = dq_1.getRealPartVec();
  Eigen::Vector4d q2_coeff = dq_2.getRealPartVec();

  double d1 = (q1_coeff - q2_coeff).norm();
  double d2 = (q1_coeff + q2_coeff).norm();

  return (d1 > d2 ? d2 : d1);
}

Eigen::Matrix3d getSkewMatrix(const Eigen::VectorXd &vec)
{
  Eigen::Matrix3d skew_mat = Eigen::Matrix3d::Zero();

  skew_mat(0,1) = -vec(2);
  skew_mat(0,2) = vec(1);
  skew_mat(1,0) = vec(2);
  skew_mat(1,2) = -vec(0);
  skew_mat(2,0) = -vec(1);
  skew_mat(2,1) = vec(0);

  return skew_mat;
}

Eigen::Matrix4d getTransformationInv(const Eigen::Matrix4d &g)
{
  Eigen::Matrix4d g_inv = Eigen::Matrix4d::Identity();

  g_inv.block<3,3>(0,0) = g.block<3,3>(0,0).transpose();
  g_inv.block<3,1>(0,3) = -g_inv.block<3,3>(0,0) * g.block<3,1>(0,3);

  return g_inv;
}

Eigen::Matrix<double,6,6> getAdjoint(const Eigen::Matrix4d &g)
{
  Eigen::Matrix<double,6,6> adj = Eigen::Matrix<double,6,6>::Zero();

  //std::cout << "adj :\n" << adj << "\n\n";

  Eigen::Vector4d p = g.col(3);
  adj.block<3,3>(0,0) = g.block<3,3>(0,0);
  adj.block<3,3>(3,3) = g.block<3,3>(0,0);
  adj.block<3,3>(0,3) = getSkewMatrix(p) * g.block<3,3>(0,0);

  //std::cout << "adj :\n" << adj << "\n\n";

  return adj;
}

Eigen::MatrixXd svdPseudoInverse(const Eigen::MatrixXd &matrix, double tolerance) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double threshold = tolerance * std::max(matrix.cols(), matrix.rows()) * svd.singularValues().array().abs().maxCoeff();
    
    // Materialize singular values inversion with thresholding
    Eigen::VectorXd singularValuesInv = svd.singularValues();
    for (int i = 0; i < singularValuesInv.size(); ++i) {
        singularValuesInv(i) = (singularValuesInv(i) > threshold) ? (1.0 / singularValuesInv(i)) : 0.0;
    }

    // Compute pseudo-inverse
    return svd.matrixV() * singularValuesInv.asDiagonal() * svd.matrixU().transpose();
}

ErrorCodes getScrewParameters(  const Eigen::Matrix4d &g_i,
                                const Eigen::Matrix4d &g_f,
                                Eigen::Vector3d &omega,
                                double &theta,
                                double &h,
                                Eigen::Vector3d &l,
                                ScrewMotionType &screw_motion_type)
{
  eigen_ext::DualQuat dq_i(g_i);
  eigen_ext::DualQuat dq_f(g_f);

  Eigen::Matrix4d g = g_f * getTransformationInv(g_i);

  Eigen::Matrix3d R = g.block<3,3>(0,0);
  Eigen::Vector3d p = g.block<3,1>(0,3);

  Eigen::AngleAxisd angle_axis(R);

  Eigen::Vector3d v = Eigen::Vector3d::Zero();

  // Assume that the motion is a general screw motion
  // Check for special cases is included later
  screw_motion_type = ScrewMotionType::GENERAL_SCREW;

  // Handle special case (Pure Translation)
  if(fabs(angle_axis.angle()) <= PURE_TRANSLATION_ROT_ANGLE_THRESHOLD)
  {
    // Pitch is infinity for pure translation
    // Set as 0 because infinity cannot be represented
    // Ignore pitch if screw_motion_type = ScrewMotionType::PURE_TRANSLATION
    h = 0;

    // Set screw_motion_type as ScrewMotionType::PURE_TRANSLATION
    screw_motion_type = ScrewMotionType::PURE_TRANSLATION;

    // Magnitude of screw
    theta = p.norm();

    // Handle special case where there is no motion
    // (i.e) g_i = g_f
    if(theta <= NO_MOTION_MAGNITUDE_THRESHOLD)
    {
      screw_motion_type = ScrewMotionType::NO_MOTION;

      omega(0) = 0; omega(1) = 0; omega(2) = 0;
      l(0) = 0; l(1) = 0; l(2) = 0;

      return ErrorCodes::OPERATION_SUCCESS;
    }

    // Screw axis
    omega = p.normalized();

    // Point on the screw axis (origin)
    l(0) = 0; l(1) = 0; l(2) = 0;

    return ErrorCodes::OPERATION_SUCCESS;
  }

  // Magnitude of screw
  theta = angle_axis.angle();

  // Screw axis
  omega = angle_axis.axis();

  Eigen::Matrix3d A = ((Eigen::Matrix3d::Identity() - R) * getSkewMatrix(omega))
      + (theta * (omega * omega.transpose()));

  v = A.inverse() * p;

  // Pitch of screw
  h = omega.transpose() * v;

  // Check if motion is pure rotation
  if(fabs(h) < PURE_ROTATION_PITCH_THRESHOLD)
  {
    screw_motion_type = ScrewMotionType::PURE_ROTATION;
  }

  // Point on the screw axis
  l = omega.cross(v);

  return ErrorCodes::OPERATION_SUCCESS;
}

ErrorCodes getNearestPoseOnScrew( const Eigen::Matrix4d &g_i,
                                  const Eigen::Matrix4d &g_f,
                                  const Eigen::Matrix4d &g_t,
                                  double &t,
                                  double &d_pos,
                                  double &d_rot)
{
  eigen_ext::DualQuat dq_i(g_i);
  eigen_ext::DualQuat dq_f(g_f);

  std::vector<double> d_pos_array(3,0);
  std::vector<double> t_inter(3,0);

  std::vector<Eigen::Matrix4d> g_inter;
  g_inter.resize(3);

  double t_i = 0;
  double t_f = 1;
  double L = t_f - t_i;

  std::vector<eigen_ext::DualQuat> dq_t_inter;
  dq_t_inter.resize(3);

  int min_d_idx = 0;
  double min_d = 10000000;

  bool init_flag = false;

  std::vector<double> coarse_d(10,0);

  // Do a initial coarse search
  for(int i = 0; i <= 10; i++)
  {
    double t_coarse = i * 0.1;
  
    dq_t_inter[0] = eigen_ext::DualQuat::dualQuatInterpolation(
        dq_i, dq_f, t_coarse);
    g_inter[0] = dq_t_inter[0].getTransform();
    d_pos_array[0] = positionDistance(g_inter[0], g_t);

    if(d_pos_array[0] < min_d)
    {
      min_d = d_pos_array[0];
      min_d_idx = i;
    }
  }

  if(min_d < 1.0e-3)
  {
    t = min_d_idx * 0.1;
    goto compute_distances;
  }

  if(min_d_idx == 0)
  {
    t = 0;
    t_i = 0;
    t_f = 0.1;
    L = 0.1;
  }
  else if(min_d_idx == 10)
  {
    t = 1;
    t_i = 0.9;
    t_f = 1;
    L = 0.1;
  }
  else
  {
    t = min_d_idx * 0.1;
    t_i = (min_d_idx * 0.1) - 0.1;
    t_f = (min_d_idx * 0.1) + 0.1;
    L = 0.1;
  }

  while(min_d > 0.001)
  {
    L = t_f - t_i;

    if(L < 0.0001)
    {
      t = (t_i + t_f) / 2.0;
      break;
    }

    t_inter[0] = t_i + (L/4.0);
    t_inter[1] = t_i + (L/2.0);
    t_inter[2] = t_f - (L/4.0);
    
    for(int i = 0; i < 3; i++)
    {
      dq_t_inter[i] = eigen_ext::DualQuat::dualQuatInterpolation(
          dq_i, dq_f, t_inter[i]);
      g_inter[i] = dq_t_inter[i].getTransform();
      d_pos_array[i] = positionDistance(g_inter[i], g_t);
    }

    if(d_pos_array[0] < d_pos_array[1])
    {
      t_f = t_inter[1];
      t = t_inter[0];
      min_d = d_pos_array[0];
    }
    else if(d_pos_array[2] < d_pos_array[1])
    {
      t_i = t_inter[1];
      t = t_inter[2];
      min_d = d_pos_array[2];
    }
    else
    {
      t_i = t_inter[0];
      t_f = t_inter[2];
      t = t_inter[1];
      min_d = d_pos_array[1];
    }
  }

compute_distances:
  dq_t_inter[0] = eigen_ext::DualQuat::dualQuatInterpolation(dq_i, dq_f, t);
  g_inter[0] = dq_t_inter[0].getTransform();

  d_pos = positionDistance(g_inter[0], g_t);
  d_rot = rotationDistance(g_inter[0], g_t);

  return ErrorCodes::OPERATION_SUCCESS;
}

ErrorCodes getScrewSegments(const std::vector<Eigen::Matrix4d> &g_seq,
                            std::vector<unsigned int> &segs,
                            double max_pos_d,
                            double max_rot_d)
{
  segs.clear();

  unsigned int start_idx = 0;

  unsigned int i,j,k;

  unsigned int end_idx;
  unsigned int itr;

  double pos_d_diff = 0;
  double rot_d_diff = 0;
  double nearest_t;

  while(start_idx < g_seq.size())
  {
    for(end_idx = start_idx + 1; end_idx < g_seq.size(); end_idx++)
    {
      double prev_nearest_t = -1;

      for(itr = start_idx + 1; itr <= end_idx; itr++)
      {
        getNearestPoseOnScrew(g_seq[start_idx], g_seq[end_idx], g_seq[itr],
                              nearest_t, pos_d_diff, rot_d_diff);

        if((pos_d_diff > max_pos_d) || (rot_d_diff > max_rot_d) || (std::abs(nearest_t - prev_nearest_t) < 1e-3))
        {
          segs.push_back(end_idx);
          start_idx = end_idx;

          if(end_idx == (g_seq.size() - 1))
          {
            return ErrorCodes::OPERATION_SUCCESS;
          }

          break;
        }

        prev_nearest_t = nearest_t;
      }

      if(end_idx == (g_seq.size() - 1))
      {
        segs.push_back(end_idx);
        return ErrorCodes::OPERATION_SUCCESS;
      }
    }
  }

  return ErrorCodes::OPERATION_SUCCESS;
}

KinematicsSolver::KinematicsSolver()
{

}

KinematicsSolver::KinematicsSolver(Manipulator manip, 
                                   const std::vector<std::string>& link_meshes_path,
                                   const std::vector<Eigen::Matrix4d> mesh_offset_transforms) :
  manipulator_(manip)
{
  if (!link_meshes_path.empty()) {
      std::cerr << "[KinematicsSolver] Loading link meshes...\n";
      link_meshes_ = CollisionUtils::buildLinkMeshes(link_meshes_path);
  } else {
      std::cerr << "[KinematicsSolver] No mesh paths provided, skipping mesh loading.\n";
      link_meshes_.clear();
  }
  mesh_offset_transforms_ = mesh_offset_transforms;
}


bool KinematicsSolver::loadManipulator( std::string robot_desc_file,
                                        std::string base_link_name,
                                        std::string tip_link_name)
{
  urdf::Model urdf_model;
  if(!urdf_model.initFile(robot_desc_file))
  {
    std::cout << "Error loading URDF file!";
    std::cout.flush();
    return false;
  }

  KDL::Tree robot_tree;
  if(!kdl_parser::treeFromFile(robot_desc_file, robot_tree))
  {
    std::cout << "Error constructing KDL Tree!";
    std::cout.flush();
    return false;
  }

  std::map<std::string, urdf::JointSharedPtr> jnt_list = urdf_model.joints_;

  KDL::Chain manip_chain;
  if(!robot_tree.getChain(base_link_name, tip_link_name, manip_chain))
  {
    std::cout << "Error constructing KDL Chain!";
    std::cout.flush();
    return false;
  }
  
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
  
  manipulator_ = Manipulator();
  JointType jnt_type;
  JointLimits jnt_lim;
  
  for(int itr = 0; itr < manip_chain.getNrOfSegments(); itr++)
  {
    KDL::Segment chain_seg = manip_chain.getSegment(itr);
    std::string seg_name = chain_seg.getName();
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

      jnt_type = JointType::Revolute;
      manipulator_.addJoint(
          jnt_type, jnt.getName(), w_v_jnt, w_p_jnt, jnt_lim, t_ref);
    }
    else
    {
      if(itr < (manip_chain.getNrOfSegments()))
      {
        manipulator_.modifyEndJointTipPose(t_ref);
      }
    }
  }

  return true;
}

Manipulator KinematicsSolver::getManipulator(void)
{
  return manipulator_;
}

/*!
  \brief    Forward Kinematics of manipulator

  \details  This function solves the forward kinematics of the manipulator
            for the given joint values and returns the end-effector pose.
            It computes the transformation from base to tool frame by 
            multiplying successive joint transformations.

  \param    jnt_values    Joint values of manipulator
  \param    g_base_tool   Variable to store FK Results (end-effector pose as a 4x4 transformation matrix)

  \return   Operation status
*/
ErrorCodes KinematicsSolver::getFK(const Eigen::VectorXd &jnt_values, 
                                  Eigen::Matrix4d &g_base_tool)
{
  Eigen::Matrix4d g;

  // 初始化末端执行器变换矩阵为单位矩阵
  g_base_tool.setIdentity();

  // 遍历所有关节
  for(unsigned int itr = 0; itr < manipulator_.joint_count_; itr++)
  {
    // 初始化当前关节的变换矩阵为单位矩阵
    g.setIdentity();

    // 判断当前关节类型是否为旋转关节
    if(manipulator_.joint_types_[itr] == JointType::Revolute)
    {
      // 计算旋转关节的变换矩阵
      // 计算旋转矩阵
      Eigen::Matrix3d rot_mat;
      Eigen::Vector3d rot_axis(manipulator_.joint_axes_[itr].head<3>());
      rot_mat = Eigen::AngleAxisd(jnt_values(itr), rot_axis);
      g.block<3,3>(0,0) = rot_mat;

      // 计算平移部分
      g.block<3,1>(0,3) = (Eigen::Matrix3d::Identity() - rot_mat) *
                          manipulator_.joint_q_[itr].head<3>();
    }
    // 判断当前关节类型是否为平移关节
    else if(manipulator_.joint_types_[itr] == JointType::Prismatic)
    {
      // 计算平移关节的变换矩阵
      Eigen::Vector3d transl_axis(manipulator_.joint_axes_[itr].head<3>());
      g.block<3,1>(0,3) = jnt_values(itr) * transl_axis;
    }

    // 累积变换以获得末端执行器配置
    g_base_tool = g_base_tool * g;
  }

  // 将基坐标系的变换矩阵应用到末端执行器的变换矩阵上
  g_base_tool = g_base_tool * manipulator_.gst0_;

  return ErrorCodes::OPERATION_SUCCESS;
}

ErrorCodes KinematicsSolver::getFK(const Eigen::VectorXd &jnt_values,
                                   std::vector<Eigen::Matrix4d> &intermediate_transforms) {
    Eigen::Matrix4d g;
    Eigen::Matrix4d g_cumulative;

    // 设置g_cumulative为单位矩阵
    g_cumulative.setIdentity();
    // 清空intermediate_transforms
    intermediate_transforms.clear();

    // 将g_cumulative推入intermediate_transforms
    intermediate_transforms.push_back(g_cumulative);

    for (unsigned int itr = 0; itr < manipulator_.joint_count_; itr++) {
        g.setIdentity();

        if (manipulator_.joint_types_[itr] == JointType::Revolute) {
            // 确定旋转关节的指数映射
            // Determine exponential for revolute joint
            Eigen::Matrix3d rot_mat;
            Eigen::Vector3d rot_axis(manipulator_.joint_axes_[itr].head<3>());
            rot_mat = Eigen::AngleAxisd(jnt_values(itr), rot_axis);
            g.block<3, 3>(0, 0) = rot_mat;

            g.block<3, 1>(0, 3) = (Eigen::Matrix3d::Identity() - rot_mat) *
                                  manipulator_.joint_q_[itr].head<3>();
        } else if (manipulator_.joint_types_[itr] == JointType::Prismatic) {
            // 确定平移关节的指数映射
            // Determine exponential for prismatic joint
            Eigen::Vector3d transl_axis(manipulator_.joint_axes_[itr].head<3>());
            g.block<3, 1>(0, 3) = jnt_values(itr) * transl_axis;
        }

        // 获取当前关节的静态变换（gst0_i）
        // Retrieve the static transformation for this joint (gst0_i)
        Eigen::Matrix4d gst0_i = manipulator_.joint_tip_pose_[itr];

        // 更新累积变换（从基座到当前关节）
        // Update the cumulative transformation (base to current joint)
        g_cumulative = g_cumulative * g;

        // 存储累积变换
        // Store the cumulative transformation
        intermediate_transforms.push_back(g_cumulative * gst0_i);
    }

    // 应用静态工具变换（gst0_）
    // Apply the static tool transformation (gst0_)
    g_cumulative = g_cumulative * manipulator_.gst0_;

    // 存储最终变换（从基座到工具坐标系）
    // Store the final transformation (base to tool frame)
    intermediate_transforms.push_back(g_cumulative);

    return ErrorCodes::OPERATION_SUCCESS;
}



ErrorCodes KinematicsSolver::getSpatialJacobian(
    const Eigen::VectorXd &jnt_values,
    Eigen::MatrixXd &manip_jac)
{
  //const unsigned int jnt_count = manipulator_.joint_count_;
  //manip_jac = Eigen::Matrix<double,6,jnt_count>::Zero();
  manip_jac.resize(6, manipulator_.joint_count_);
  manip_jac.Zero(6, manipulator_.joint_count_);
  
  Eigen::MatrixXd joint_twists;
  //joint_twists = Eigen::Matrix<double,6,jnt_count>::Zero();
  joint_twists.resize(6, manipulator_.joint_count_);
  joint_twists = Eigen::MatrixXd::Zero(6, manipulator_.joint_count_);

  //std::cout << joint_twists << "\n\n";

  std::vector<Eigen::Matrix4d> g;

  for(unsigned int itr = 0; itr < manipulator_.joint_count_ - 1; itr++)
  {
    g.push_back(Eigen::Matrix<double,4,4>::Identity());
  }

  // Determine joint twists
  for(unsigned int itr = 0; itr < manipulator_.joint_count_; itr++)
  {
    Eigen::Vector3d jnt_axis = manipulator_.joint_axes_[itr].head<3>();
    Eigen::Vector3d jnt_q = manipulator_.joint_q_[itr].head<3>();

    if(manipulator_.joint_types_[itr] == JointType::Revolute)
    {
      //std::cout << "jnt_axis\n" << jnt_axis << "\n\n";
      //std::cout << "jnt_q\n" << jnt_q << "\n\n";
      //std::cout << "jnt_axis_cross\n" << jnt_axis.cross(jnt_q) << "\n\n";
      joint_twists.block<3,1>(0,itr) = -jnt_axis.cross(jnt_q);
      joint_twists.block<3,1>(3,itr) = jnt_axis;
    }
    else if(manipulator_.joint_types_[itr] == JointType::Prismatic)
    {
      joint_twists.block<3,1>(0,itr) = jnt_axis;
    }

    //std::cout << joint_twists << "\n\n";
  }

  // Determine joint twists exponential
  for(unsigned int i = 1; i < manipulator_.joint_count_; i++)
  {
    for(unsigned int j = 0; j < i; j++)
    {
      Eigen::Matrix4d temp_g = Eigen::Matrix4d::Identity();

      if(manipulator_.joint_types_[j] == JointType::Revolute)
      {
        // Determine exponential for revolute joint
        Eigen::Matrix3d rot_mat;
        Eigen::Vector3d rot_axis(manipulator_.joint_axes_[j].head<3>());
        rot_mat = Eigen::AngleAxisd(jnt_values(j), rot_axis);
        temp_g.block<3,3>(0,0) = rot_mat;

        temp_g.block<3,1>(0,3) = (Eigen::Matrix3d::Identity() - rot_mat) *
                            manipulator_.joint_q_[j].head<3>();
      }
      else if(manipulator_.joint_types_[j] == JointType::Prismatic)
      {
        // Determine exponential for prismatic joint
        Eigen::Vector3d transl_axis(manipulator_.joint_axes_[j].head<3>());
        temp_g.block<3,1>(0,3) = jnt_values(j) * transl_axis;
      }

      g[i-1] = g[i-1] * temp_g;
    }
  }

  manip_jac.col(0) = joint_twists.col(0);

  for(unsigned int itr = 1; itr < manipulator_.joint_count_; itr++)
  {
    //std::cout << "g[itr-1] :\n" << g[itr-1] << "\n\n";
    //std::cout << "Adjoint :\n" << getAdjoint(g[itr-1]) << "\n\n";
    //std::cout << "joint_twists.col(itr) :\n" << joint_twists.col(itr) << "\n\n";
    manip_jac.col(itr) = getAdjoint(g[itr-1]) * joint_twists.col(itr);
  }

  return ErrorCodes::OPERATION_SUCCESS;
}

ErrorCodes KinematicsSolver::getResolvedMotionRateControlStep(
    eigen_ext::DualQuat &dq_i,
    eigen_ext::DualQuat &dq_f,
    const Eigen::VectorXd &jnt_values,
    Eigen::VectorXd &jnt_values_increment)
{
  Eigen::Matrix4d g(Eigen::Matrix4d::Identity());
  Eigen::Vector3d p;
  Eigen::Matrix<double,7,1> gamma_i, gamma_f;

  Eigen::Matrix3d I3(Eigen::Matrix3d::Identity());

  Eigen::Vector3d p_i;
  Eigen::Vector4d q_i;

  g = dq_i.getTransform();
  p = g.block<3,1>(0,3);
  gamma_i.head<3>() = p;
  gamma_i.tail<4>() = dq_i.getRealPartVec();

  p_i = gamma_i.head<3>();
  q_i = gamma_i.tail<4>();

  g = dq_f.getTransform();
  p = g.block<3,1>(0,3);
  gamma_f.head<3>() = p;
  gamma_f.tail<4>() = dq_f.getRealPartVec();

  Eigen::MatrixXd s_jac;

  getSpatialJacobian(jnt_values, s_jac);

  Eigen::Matrix<double,3,4> J_1(Eigen::Matrix<double,3,4>::Zero());

  J_1.block<3,1>(0,0) = -q_i.tail<3>();
  J_1(0,1) =  q_i(0);
  J_1(0,2) = -q_i(3);
  J_1(0,3) =  q_i(2);
  J_1(1,1) =  q_i(3);
  J_1(1,2) =  q_i(0);
  J_1(1,3) = -q_i(1);
  J_1(2,1) = -q_i(2);
  J_1(2,2) =  q_i(1);
  J_1(2,3) =  q_i(0);

  Eigen::Matrix<double,6,7> J_2(Eigen::Matrix<double,6,7>::Zero());

  J_2.block<3,3>(0,0) = I3;
  J_2.block<3,4>(0,3) = 2 * getSkewMatrix(p_i) * J_1;
  J_2.block<3,4>(3,3) = 2 * J_1;

  Eigen::MatrixXd jac_pseudo_inv;

  Eigen::MatrixXd temp_mat;
  temp_mat = s_jac * s_jac.transpose();

  // TODO Include invertible check

  jac_pseudo_inv = s_jac.transpose() * temp_mat.inverse();

  Eigen::MatrixXd B;
  B = jac_pseudo_inv * J_2;

  jnt_values_increment = B * (gamma_f - gamma_i);

  return ErrorCodes::OPERATION_SUCCESS;
}

Eigen::VectorXd KinematicsSolver::getAdjustedJoints(
  double h,
  const std::vector<double> &dist_array,
  const Eigen::MatrixXd &contact_normal_array,
  double safe_dist,
  const Eigen::VectorXd &current_joint_values,
  const Eigen::VectorXd &next_joint_values,
  const std::vector<Eigen::MatrixXd> &j_contact_array) {

  // Identify contacts violating safe distance
  // If no violating contacts, return next_joint_values as is
  std::vector<int> violating_contacts;
  for (size_t i = 0; i < dist_array.size(); ++i) {
      if (dist_array[i] < safe_dist) {
          violating_contacts.push_back(i);
      }
  }

  if (violating_contacts.empty()) {
      return next_joint_values;
  }
  // Define q and M matrices
  int nc = contact_normal_array.cols();
  Eigen::VectorXd q(nc);
  Eigen::MatrixXd M(nc, nc);
  q.setZero();
  M.setZero();
  for (int n = 0; n < nc; ++n) {
      q[n] = dist_array[n] - safe_dist +
              h * contact_normal_array.col(n).transpose() *
                  j_contact_array[n] * (next_joint_values - current_joint_values);

      for (int m = 0; m < nc; ++m) {
      M(n, m) = h * contact_normal_array.col(n).transpose() *
                j_contact_array[n] *
                svdPseudoInverse(j_contact_array[m]) *
                contact_normal_array.col(m);
      }
  }

  // Static scaling for q and M (more stable than dynamic scaling)
  q *= 1000;
  M *= 1000;
  
  // Solve the LCP using Lemke's solver
  Eigen::VectorXd z;
  LemkeResult lemke_result = Lemke(q, M);
  z = lemke_result.z;

  // Rescale z based on the scaling factor
  z /= 1000;

  // Calculate compensating displacement
  Eigen::VectorXd comp_joint_values = Eigen::VectorXd::Zero(current_joint_values.size());
  for (int i = 0; i < nc; ++i) {
      comp_joint_values += svdPseudoInverse(j_contact_array[i]) * contact_normal_array.col(i) * z[i];
  }

  // Update the next joint values
  Eigen::VectorXd adjusted_joint_values = next_joint_values + comp_joint_values;

  return adjusted_joint_values;
}

/*!
  \brief    获取机械臂运动规划

  \details  该函数基于给定的初始关节角度和末端执行器目标位姿，
            使用螺旋线性插值(Screw Linear Interpolation)来求解运动规划。
            它通过迭代计算关节空间中的小步长移动，逐步将末端执行器移向目标位置，
            并生成一条关节轨迹。

  \param    init_jnt_values     机械臂初始关节值
  \param    g_i                 初始末端执行器位姿
  \param    g_f                 目标末端执行器位姿
  \param    jnt_trajectory      用于存储运动规划的关节轨迹

  \return   操作状态 - OPERATION_SUCCESS表示成功，
            OPERATION_FAILURE表示关节增量不是有限值，
            JOINT_LIMIT_ERROR表示达到关节限制
*/
ErrorCodes KinematicsSolver::getMotionPlan(
    const Eigen::VectorXd &init_jnt_values,
    const Eigen::Matrix4d &g_i,
    const Eigen::Matrix4d &g_f,
    trajectory_msgs::msg::JointTrajectory &jnt_trajectory)
    //std::vector<Eigen::VectorXd> &jnt_values_seq)
{
  // 用于存储不同步骤的关节值变量
  Eigen::VectorXd joint_values_inc;
  Eigen::VectorXd current_joint_values;
  Eigen::VectorXd next_joint_values;
  
#if DEBUG

  std::time_t now = std::time(0);
  std::tm* timestamp = std::localtime(&now);

  char timestamp_char[100];
  strftime(timestamp_char, 100, timestamp_str.c_str(), timestamp);

  std::string current_log_folder(timestamp_char);
  std::string log_dir_path = log_dir + "/" + current_log_folder;

  std::string timestamp_str_val(timestamp_char);
  std::string log_file_path = log_dir_path + "/" + timestamp_str_val 
                            + "_motion_plan.csv";

  std::ofstream log_file;

  if(mkdir(log_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH))
  {
    if(errno != EEXIST)
    {
      std::cout << "Error : Cannot create log folders\n";
      std::cout << "Error " << errno << ": " << strerror(errno);
      std::cout.flush();
      log_folder_error = true;
    }
  }

  if(!log_folder_error)
  {
    if(mkdir(log_dir_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH))
    {
      if(errno != EEXIST)
      {
        std::cout << "Error : Cannot create log folders\n";
        std::cout << "Error " << errno << ": " << strerror(errno);
        std::cout.flush();
        log_folder_error = true;
      }
    }
    
    if(!log_folder_error)
    {
      log_file.open(log_file_path, std::ofstream::out | std::ofstream::app);

      if(!log_file.is_open())
      {
        log_file_error = true;

        std::cout << "Error : Cannot open log file\n";
        std::cout.flush();
      }
    }
  }

#endif

  // 距离变量，用于跟踪向目标的进度
  double pos_dist, rot_dist;

  current_joint_values = init_jnt_values;

  unsigned long int itr_cnt = 0;

  // 将初始和最终变换转换为双四元数以便插值
  eigen_ext::DualQuat dq_i(g_i);
  eigen_ext::DualQuat dq_f(g_f);

  Eigen::Matrix4d g_current;

  eigen_ext::DualQuat dq_current = dq_i;
  eigen_ext::DualQuat dq_next;

  // 计算初始位置和旋转距离到目标
  pos_dist = positionDistance(g_i, g_f);
  rot_dist = rotationDistance(dq_i, dq_f);

  // 从机械臂中提取关节限制到矩阵形式
  Eigen::MatrixXd joint_limits;
  joint_limits.resize(manipulator_.joint_count_, 2);
  for(int i = 0; i < manipulator_.joint_count_; i++)
  {
    joint_limits(i,0) = manipulator_.joint_limits_[i].lower_limit_;
    joint_limits(i,1) = manipulator_.joint_limits_[i].upper_limit_;
  }

  // 初始化步长和插值参数
  double beta = 0.5;
  double step_size = beta;

  double tau = 0.001;
  double tau_i = 0.01;

  double tau_max = 0.1;
  double tau_break = 0.9;

  // 用于自适应步长调整的多项式函数
  Polynomial tau_f(tau_break, tau_max);

  Eigen::VectorXd joint_values_delta;

  // 初始化轨迹数据结构
  trajectory_msgs::msg::JointTrajectoryPoint jnt_trajectory_point;
  jnt_trajectory.joint_names = manipulator_.joint_names_;
  jnt_trajectory.points.clear();

  std::vector<double> jnt_values(manipulator_.joint_count_,0);

  // 初始到最终位姿的螺旋运动参数
  Eigen::Vector3d omega;
  double theta;
  double h;
  Eigen::Vector3d l;
  ScrewMotionType screw_type;
  
  // 当前到最终位姿的螺旋运动参数
  Eigen::Vector3d curr_omega;
  double curr_theta;
  double curr_h;
  Eigen::Vector3d curr_l;
  ScrewMotionType curr_screw_type;

  // 获取整体运动的螺旋参数
  kinlib::getScrewParameters(g_i, g_f, omega, theta, h, l, screw_type);

  // 主迭代循环 - 继续直到达到目标或超过最大迭代次数
  while((!(pos_dist < 0.0005 && rot_dist < 0.005)) && (itr_cnt < 10000))
  {

    itr_cnt++;

    step_size = beta;

    // 在当前和最终双四元数之间插值
    dq_next = eigen_ext::DualQuat::dualQuatInterpolation(
        dq_current, dq_f, tau);

    // 基于剩余距离的自适应步长调整
    if(tau < tau_max)
    {
      Eigen::Matrix4d g_next_temp = dq_next.getTransform();

      kinlib::getScrewParameters(
          g_next_temp, g_f, curr_omega, curr_theta, curr_h, curr_l, curr_screw_type);

      if(theta != 0)
      {
        double motion_dist = std::abs(curr_theta / theta);
        
        tau = tau_f.getValue(motion_dist) + tau_i;
      }
    }

    // 使用解析运动率控制计算关节增量
    getResolvedMotionRateControlStep(
        dq_current, dq_next, current_joint_values, joint_values_inc);

    // 检查关节增量是否为有效数字
    if(joint_values_inc.hasNaN())
    {
      //jnt_values_seq.clear();
      std::cout << "\n关节增量不是有限值！\n";
      std::cout.flush();

#if DEBUG
      if(!(log_folder_error || log_file_error))
      {
        for(int temp_itr = 0; temp_itr < joint_values_inc.rows(); temp_itr++)
        {
          log_file << 1000 << ',';
        }
        log_file << '\n';

        log_file << joint_values_inc << '\n';
        
        log_file.flush();
      }
#endif

      return ErrorCodes::OPERATION_FAILURE;
    }

    int joint_limit_id = 0;

determine_next_angles:

    joint_limit_id = 0;

    // 使用当前步长计算关节值变化
    joint_values_delta = joint_values_inc * step_size;

    // 计算下一个关节值
    next_joint_values = current_joint_values + joint_values_delta;

    // 检查是否满足关节限制
    for(int i = 0; i < manipulator_.joint_count_; i++)
    {

      jnt_values[i] = next_joint_values(i);
      //std::cout << i <<"th @Joint limits are" << joint_limits(i,0) << " and " << joint_limits(i,1) << std::endl;

      if(((next_joint_values(i) > joint_limits(i,0)) && (next_joint_values(i) < joint_limits(i,1)))) 
      {
        continue;
      }
      else
      {
        joint_limit_id = i + 1;

        // 查找最大关节变化以确定是否接近限制
        double max_val = fabs(joint_values_delta(0));

        for(int j = 1; j < manipulator_.joint_count_; j++)
        {
          if(fabs(joint_values_delta(j)) > max_val)
          {
            max_val = fabs(joint_values_delta(j));
          }
        }

        // 如果变化很小，说明已达到关节限制
        if(max_val <= 0.0001)
        {
          std::cout << "\n关节 " << manipulator_.joint_names_[i] << " 达到限制！\n";
          std::cout << next_joint_values.transpose().format(PrintFormat)<<'\n';
          std::cout.flush();

#if DEBUG
          if(!(log_folder_error || log_file_error))
          {
            for(int temp_itr = 0; temp_itr < manipulator_.joint_count_; temp_itr++)
            {
              log_file << -1000 << ',';
            }
            log_file << '\n';

            log_file.flush();
          }
#endif
          return ErrorCodes::JOINT_LIMIT_ERROR;
        }

        // 减小步长并重试
        step_size = step_size / 10;
        goto determine_next_angles;
      }

    }

    // 将有效的关节值添加到轨迹中
    jnt_trajectory_point.positions = jnt_values;
    jnt_trajectory.points.push_back(jnt_trajectory_point);

    //jnt_values_seq.push_back(next_joint_values);

#if DEBUG
    if(!(log_folder_error || log_file_error))
    {
      log_file << next_joint_values.transpose().format(CSVFormat) << '\n';
      log_file.flush();
    }
#endif

    // 使用新关节值更新正向运动学
    getFK(next_joint_values, g_current);

    // 将当前变换转换为双四元数
    dq_current = eigen_ext::DualQuat::transformationToDualQuat(g_current);

    // 更新当前关节值
    current_joint_values = next_joint_values;

    // 计算到目标的新距离
    pos_dist = positionDistance(g_current, g_f);
    rot_dist = rotationDistance(dq_current, dq_f);
  }

  return ErrorCodes::OPERATION_SUCCESS;
}


/**
* @brief 获取具有避障功能的运动规划。
*
* 根据初始关节值、初始和目标位姿矩阵、是否检查自碰撞、忽略的连杆数量、障碍物列表、抓取物体和关节轨迹消息，
* 计算并返回关节轨迹，同时避免与障碍物和自身的碰撞。
*
* @param init_jnt_values 初始关节值
* @param g_i 初始位姿矩阵
* @param g_f 目标位姿矩阵
* @param check_self_collision 是否检查自碰撞
* @param num_links_ignore 忽略的连杆数量
* @param obstacles 障碍物列表
* @param grasped_object 抓取物体
* @param jnt_trajectory 关节轨迹消息
*
* @return 错误代码，表示操作是否成功。
*/
ErrorCodes KinematicsSolver::getMotionPlanWithCollisionAvoidance(
    const Eigen::VectorXd &init_jnt_values,
    const Eigen::Matrix4d &g_i,
    const Eigen::Matrix4d &g_f,
    const bool check_self_collision,
    const int num_links_ignore,
    const std::vector<std::shared_ptr<CollisionUtils::ObstacleBase>> &obstacles,
    const std::shared_ptr<CollisionUtils::ObstacleBase> &grasped_object,
    trajectory_msgs::msg::JointTrajectory &jnt_trajectory)
{
  Eigen::VectorXd joint_values_inc;
  Eigen::VectorXd current_joint_values;
  Eigen::VectorXd next_joint_values;
  
#if DEBUG

  std::time_t now = std::time(0);
  std::tm* timestamp = std::localtime(&now);

  char timestamp_char[100];
  strftime(timestamp_char, 100, timestamp_str.c_str(), timestamp);

  std::string current_log_folder(timestamp_char);
  std::string log_dir_path = log_dir + "/" + current_log_folder;

  std::string timestamp_str_val(timestamp_char);
  std::string log_file_path = log_dir_path + "/" + timestamp_str_val 
                            + "_motion_plan.csv";

  std::ofstream log_file;

  if(mkdir(log_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH))
  {
    if(errno != EEXIST)
    {
      std::cout << "Error : Cannot create log folders\n";
      std::cout << "Error " << errno << ": " << strerror(errno);
      std::cout.flush();
      log_folder_error = true;
    }
  }

  if(!log_folder_error)
  {
    if(mkdir(log_dir_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH))
    {
      if(errno != EEXIST)
      {
        std::cout << "Error : Cannot create log folders\n";
        std::cout << "Error " << errno << ": " << strerror(errno);
        std::cout.flush();
        log_folder_error = true;
      }
    }
    
    if(!log_folder_error)
    {
      log_file.open(log_file_path, std::ofstream::out | std::ofstream::app);

      if(!log_file.is_open())
      {
        log_file_error = true;

        std::cout << "Error : Cannot open log file\n";
        std::cout.flush();
      }
    }
  }

#endif

  double pos_dist, rot_dist;

  current_joint_values = init_jnt_values;

  unsigned long int itr_cnt = 0;

  eigen_ext::DualQuat dq_i(g_i);
  eigen_ext::DualQuat dq_f(g_f);

  Eigen::Matrix4d g_current;

  eigen_ext::DualQuat dq_current = dq_i;
  eigen_ext::DualQuat dq_next;

  pos_dist = positionDistance(g_i, g_f);
  rot_dist = rotationDistance(dq_i, dq_f);

  Eigen::MatrixXd joint_limits;
  joint_limits.resize(manipulator_.joint_count_, 2);
  for(int i = 0; i < manipulator_.joint_count_; i++)
  {
    joint_limits(i,0) = manipulator_.joint_limits_[i].lower_limit_;
    joint_limits(i,1) = manipulator_.joint_limits_[i].upper_limit_;
  }

  double beta = 0.5;
  double step_size = beta;

  double tau = 0.001;
  double tau_i = 0.01;

  double tau_max = 0.1;
  double tau_break = 0.9;

  Polynomial tau_f(tau_break, tau_max);

  Eigen::VectorXd joint_values_delta;

  trajectory_msgs::msg::JointTrajectoryPoint jnt_trajectory_point;
  jnt_trajectory.joint_names = manipulator_.joint_names_;
  jnt_trajectory.points.clear();

  std::vector<double> jnt_values(manipulator_.joint_count_, 0);

  Eigen::Vector3d omega;
  double theta;
  double h;
  Eigen::Vector3d l;
  ScrewMotionType screw_type;
  
  Eigen::Vector3d curr_omega;
  double curr_theta;
  double curr_h;
  Eigen::Vector3d curr_l;
  ScrewMotionType curr_screw_type;

  kinlib::getScrewParameters(g_i, g_f, omega, theta, h, l, screw_type);

  while((!(pos_dist < 0.0005 && rot_dist < 0.005)) && (itr_cnt < 10000))
  {
    itr_cnt++;

    step_size = beta;

    dq_next = eigen_ext::DualQuat::dualQuatInterpolation(dq_current, dq_f, tau);

    if(tau < tau_max)
    {
      Eigen::Matrix4d g_next_temp = dq_next.getTransform();

      kinlib::getScrewParameters(
          g_next_temp, g_f, curr_omega, curr_theta, curr_h, curr_l, curr_screw_type);

      if(theta != 0)
      {
        double motion_dist = std::abs(curr_theta / theta);
        
        tau = tau_f.getValue(motion_dist) + tau_i;
      }
    }

    getResolvedMotionRateControlStep(
        dq_current, dq_next, current_joint_values, joint_values_inc);

    if(joint_values_inc.hasNaN())
    {
      //jnt_values_seq.clear();
      std::cout << "\nJoint increments are not finite!\n";
      std::cout.flush();

#if DEBUG
      if(!(log_folder_error || log_file_error))
      {
        for(int temp_itr = 0; temp_itr < joint_values_inc.rows(); temp_itr++)
        {
          log_file << 1000 << ',';
        }
        log_file << '\n';

        log_file << joint_values_inc << '\n';
        
        log_file.flush();
      }
#endif

      return ErrorCodes::OPERATION_FAILURE;
    }

    int joint_limit_id = 0;

determine_next_angles:

    joint_limit_id = 0;

    joint_values_delta = joint_values_inc * step_size;

    next_joint_values = current_joint_values + joint_values_delta;

    // @TODO: add collision check here

    Eigen::MatrixXd contact_normal_array;
    Eigen::MatrixXd spatial_jacobian;
    std::vector<double> dist_array;
    std::vector<double> radius_array;
    std::vector<Eigen::MatrixXd> contact_points_array;
    std::vector<Eigen::MatrixXd> j_contact_array;
    std::vector<Eigen::Matrix4d> intermediate_transforms;

    getFK(next_joint_values, intermediate_transforms);

    updateLinkMeshTransforms(link_meshes_, intermediate_transforms, mesh_offset_transforms_);

    getSpatialJacobian(next_joint_values, spatial_jacobian);

    if (grasped_object) 
    { // the best part of this is that you dont need to dive into the te
      grasped_object->setTransform(intermediate_transforms.back().block<3, 1>(0, 3), intermediate_transforms.back().block<3, 3>(0, 0));
    }

    if (CollisionUtils::getCollisionInfo(link_meshes_, obstacles, grasped_object, spatial_jacobian, check_self_collision,
                                        num_links_ignore, manipulator_.joint_count_, contact_normal_array,
                                        dist_array, contact_points_array, j_contact_array) != ErrorCodes::OPERATION_SUCCESS) 
    {
      std::cerr << "[KinematicsSolver] Failed to get collision info.\n";
      return ErrorCodes::OPERATION_FAILURE;
    }
    
    double h = 0.001;
    double safe_dist = 0.010;
    Eigen::VectorXd adjusted_joint_values;

    adjusted_joint_values = KinematicsSolver::getAdjustedJoints(h, dist_array, contact_normal_array,
                                                                safe_dist, current_joint_values, 
                                                                next_joint_values, j_contact_array);
    
    next_joint_values = adjusted_joint_values;

    // Check if joint limits are satisfied
    for(int i = 0; i < manipulator_.joint_count_; i++)
    {

      jnt_values[i] = next_joint_values(i);
      //std::cout << i <<"th @Joint limits are" << joint_limits(i,0) << " and " << joint_limits(i,1) << std::endl;

      if( ((next_joint_values(i) > joint_limits(i,0)) && (next_joint_values(i) < joint_limits(i,1)))) 
      {
        continue;
      }
      else
      {
        joint_limit_id = i + 1;

        double max_val = fabs(joint_values_delta(0));

        for(int j = 1; j < manipulator_.joint_count_; j++)
        {
          if(fabs(joint_values_delta(j)) > max_val)
          {
            max_val = fabs(joint_values_delta(j));
          }
        }

        if(max_val <= 0.0001)
        {
          std::cout << "\nJoint " << manipulator_.joint_names_[i] << " limits reached!\n";
          std::cout << next_joint_values.transpose().format(PrintFormat)<<'\n';
          std::cout.flush();

#if DEBUG
          if(!(log_folder_error || log_file_error))
          {
            for(int temp_itr = 0; temp_itr < manipulator_.joint_count_; temp_itr++)
            {
              log_file << -1000 << ',';
            }
            log_file << '\n';

            log_file.flush();
          }
#endif
          return ErrorCodes::JOINT_LIMIT_ERROR;
        }

        step_size = step_size / 10;
        goto determine_next_angles;
      }

    }

    jnt_trajectory_point.positions = jnt_values;
    jnt_trajectory.points.push_back(jnt_trajectory_point);

    //jnt_values_seq.push_back(next_joint_values);

#if DEBUG
    if(!(log_folder_error || log_file_error))
    {
      log_file << next_joint_values.transpose().format(CSVFormat) << '\n';
      log_file.flush();
    }
#endif

    getFK(next_joint_values, g_current);

    dq_current = eigen_ext::DualQuat::transformationToDualQuat(g_current);

    current_joint_values = next_joint_values;

    pos_dist = positionDistance(g_current, g_f);
    rot_dist = rotationDistance(dq_current, dq_f);
  }

  return ErrorCodes::OPERATION_SUCCESS;
}

} // namespace kinlib
