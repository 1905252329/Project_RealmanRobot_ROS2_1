/*
 * kinlib::Manipulator class declaration
 */

#pragma once

#include <vector>
#include <string>
#include <map>
#include <iterator>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "kinlib.h"

namespace kinlib
{
/*!
  \brief    %Manipulator joint type enum

  \details  Enum to represent the different joint types
            -# Revolute\tRevolute Joint
            -# Prismatic\tPrismatic Joint
*/
typedef enum{Revolute, Prismatic, None} JointType;

/*!
  \brief    Joint limits

  \details  Object to represent joint limits
*/
struct JointLimits
{
  /*!
    \brief  Lower joint limit
  */
  double lower_limit_;

  /*!
    \brief  Upper joint limit
  */
  double upper_limit_;
};

/*!
  \brief    Defines a serial manipulator

  \details  This class is used to represent a serial manipulator and its
            properties such as\n
            -# Joint types (Revolute/Prismatic)
            -# Joint axis
            -# Points present on the joint axes
*/
class Manipulator
{
  public:
    /*!
      \brief    %Manipulator constructor

      \details  Constructor to initialize class object
    */
    Manipulator(void);

    /*!
      \brief    Get type of the manipulator joints

      \details  Returns the type of all the joints present in the manipulator

      \return   Type of the joint
    */
    std::vector<JointType> getJointTypes(void);

    /*!
      \brief    Get joint name and id

      \details  Returns a map of joint names and their corresponding id

      \return   Joint name and id mapping
    */
    std::map<std::string, unsigned int> getJointNameAndIDMapping(void);

    /*!
      \brief    Get joint axis

      \details  Returns the axes of all the joints present in the manipulator
      
      \return   Joint axes
    */
    std::vector<Eigen::Vector4d> getJointAxes(void);

    /*!
      \brief    Get joint positions

      \details  Returns the position of all the joints present in the
                manipulator

      \return   Joint positioins
    */
    std::vector<Eigen::Vector4d> getJointPositions(void);

    /*!
      \brief    Get number of joints

      \details  Returns the number of joints present in the manipulator

      \return   Number of joints
    */
    unsigned int getNumberOfJoints(void);

    /*!
      \brief    Adds a joint to the manipulator

      \details  This function adds a joint based on the given specifications
                to the end of the serial chain of the manipulator

      \param    jnt_type      Type of the joint
      \param    jnt_name      Name of the joint
      \param    jnt_axis      Direction of the joint axis
      \param    jnt_q         Position of the joint
      \param    jnt_limits    Joint limits
      \param    jnt_tip_pose  Pose of joint tip

      \return   Status of operation
    */ 
    ErrorCodes addJoint(const JointType & jnt_type,
                        const std::string & jnt_name,
                        const Eigen::Vector4d & jnt_axis,
                        const Eigen::Vector4d & jnt_q,
                        const JointLimits & jnt_limits,
                        const Eigen::Matrix4d & jnt_tip_pose);

    /*!
      \brief    Modify end joint tip pose

      \details  This function is used to modify the tip pose of the last joint
                in the serial chain of the manipulator

      \param    g_tip     New tip pose of the last joint

      \return   Status of operation
    */
    ErrorCodes modifyEndJointTipPose(const Eigen::Matrix4d & g_tip);

    /*!
      \brief    Set reference configuration of the manipulator

      \details  This function sets the reference configuration of the 
                manipulator

      \param    g_ref     Reference configuration of the manipulator

      \return   Status of operation
    */
    ErrorCodes  setReferenceConfiguration(const Eigen::Matrix4d & g_ref); 

    /*!
      \brief    Get reference configuration of the manipulator

      \details  This function returns the reference configuration (gst0_) of the 
                manipulator

      \return   Reference configuration of the manipulator
    */
    Eigen::Matrix4d getReferenceConfiguration(void);

  //  private:

    /*!
      \brief    %Manipulator reference configuration

      \details  This variable stores the reference configuration of the
                end-effector
    */
    Eigen::Matrix4d gst0_;

    /*!
      \brief    %Manipulator static transformations for joints

      \details  This variable stores the static transformations (gst0_i) for
                all the joints relative to the base frame
    */
    std::vector<Eigen::Matrix4d> gst0_i_list_;

    /*!
      \brief    %Manipulator joint types

      \details  This variable stores the types of joints present in the 
                manipulator
    */
    std::vector<JointType> joint_types_;

    /*!
      \brief    %Manipulator joint names

      \details  This variable stores the names of the joints present in the
                manipulator
    */
    std::vector<std::string> joint_names_;

    /*!
      \brief    %Manipulator joint limits

      \details  This variable stores the joint limits of the manipulator
    */
    std::vector<JointLimits> joint_limits_;

    /*!
      \brief    %Manipulator joint names, joint id mapping

      \details  Map of joint name and its corresponding id
    */
    std::map<std::string, unsigned int> joint_name_id_map_;

    /*!
      \brief    %Manipulator joint axes

      \details  This variable stores the directions of all the manipulator
                joint axes
    */
    std::vector<Eigen::Vector4d> joint_axes_;

    /*!
      \brief    %Manipulator joint positions

      \details  This variable stores the positions of all the joints in the
                manipulator with respect to the base frame of the manipulator 
    */
    std::vector<Eigen::Vector4d> joint_q_;

    /*!
      \brief    Joint configuration with respect to base frame

      \details  This variable stores the configuration of each joint with
                respect to the base frame of the manipulator
    */
    std::vector<Eigen::Matrix4d> joint_tip_pose_;

    /*!
      \brief    %Manipulator joint count

      \details  Number of joints present in the manipulator
    */
    unsigned int joint_count_;
};
}