/* Author: Dasharadhan Mahalingam */

#include <ros/ros.h>

#include "sclerp_motion_planner/sclerp_interface.h"

#include <armadillo>

#include <fstream>

int main(int argc, char** argv)
{
  ros::init (argc, argv, "sclerp_motion_planner_test");

  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(3);

  ROS_INFO("Node Starting!");

  spinner.start();

  ros::Duration(2).sleep();

  sclerp_interface::ScLERPInterface planner_interface("base", "left_gripper_base", nh);
  
  Eigen::VectorXd jnt_values(7);
  jnt_values << 0, 0, 0, 0, 0, 0, 0;
  
  Eigen::Matrix4d g_base_tool;
  
  std::string rand_jnt_angles_file = 
    "/home/byd/Documents/rand_jnt_angles.csv";
  
  Eigen::IOFormat CSVFormat(Eigen::FullPrecision,0,",","\n","","","","");
  std::string g_log_file = 
    "/home/byd/Documents/rand_jnt_angles_g.csv";
  std::ofstream log_file;
  
  log_file.open(g_log_file, std::ofstream::out | std::ofstream::app);
  
  arma::Mat<double> rand_jnt_angles;
  rand_jnt_angles.load(rand_jnt_angles_file, arma::csv_ascii);
  
  for(int i = 0; i < rand_jnt_angles.n_rows; i++)
  {
    for(int j = 0; j < 7; j++)
    {
      jnt_values(j) = rand_jnt_angles(i,j);
    }
    
    planner_interface.kinlib_solver_.getFK(jnt_values, g_base_tool);
    
    /*
    if(log_file.is_open())
    {
      log_file << g_base_tool.format(CSVFormat) << '\n';
        
      log_file.flush();
    }
    */
     
    ROS_INFO_STREAM("g_base_tool :\n\n" << g_base_tool << '\n');
  }
  
  for(int j = 0; j < 7; j++)
  {
    jnt_values(j) = rand_jnt_angles(2,j);
  }
  
  planner_interface.kinlib_solver_.getFK(jnt_values, g_base_tool);
  
  for(int j = 0; j < 7; j++)
  {
    jnt_values(j) = rand_jnt_angles(1,j);
  }
  
  trajectory_msgs::JointTrajectory motion_plan;
  
  bool res = planner_interface.solve(jnt_values, g_base_tool, motion_plan);
  
  ROS_INFO_STREAM("Motion Plan Result : " << res);
  ROS_INFO_STREAM("Trajectory Length  : " << motion_plan.points.size());
  ROS_INFO_STREAM("Trajectory Point  : " << motion_plan.points[0].positions.size());
  
  std::vector<double> jnt_angle_vect = motion_plan.points[0].positions;
  
  
  
  while(ros::ok())
  {
  
  }
  
  return 0;
}
