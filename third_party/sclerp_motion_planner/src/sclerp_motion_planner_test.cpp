#include <ros/ros.h>

// 包含ScLERP运动规划器接口
#include "sclerp_motion_planner/sclerp_interface.h"

// 包含Armadillo库，用于处理矩阵运算和CSV文件读取
#include <armadillo>

// 包含文件流库，用于文件操作
#include <fstream>

/**
 * \brief ScLERP运动规划器测试程序主函数
 * 
 * 该程序用于测试ScLERP（Spherical Linear Interpolation）运动规划算法，
 * 验证机械臂运动规划功能的正确性和有效性。
 * 
 * \param argc 命令行参数数量
 * \param argv 命令行参数数组
 * \return 程序退出状态码
 */
int main(int argc, char** argv)
{
/*******Part01 ROS初始化和节点设置********/
  // 初始化ROS节点，节点名为"sclerp_motion_planner_test"
  ros::init (argc, argv, "sclerp_motion_planner_test");

  // 创建ROS节点句柄
  ros::NodeHandle nh;

  // 创建异步spinner，使用3个线程处理ROS回调
  ros::AsyncSpinner spinner(3);

  // 打印节点启动信息
  ROS_INFO("Node Starting!");

  // 启动异步spinner
  spinner.start();

  // 等待2秒，确保系统初始化完成
  ros::Duration(2).sleep();

/*******Part02 ScLERP规划器实例化********/

  // 创建ScLERP运动规划器实例
  // 参数："base"为机械臂基座链接名称，"left_gripper_base"为末端执行器链接名称
  sclerp_interface::ScLERPInterface planner_interface("base", "left_gripper_base", nh);
  
  // 初始化7自由度关节角度向量
  Eigen::VectorXd jnt_values(7);
  jnt_values << 0, 0, 0, 0, 0, 0, 0;
  
  // 定义末端执行器位姿矩阵（4x4齐次变换矩阵）
  Eigen::Matrix4d g_base_tool;

 /***Part02_1 离线轨迹规划：基于现有数据验证部分***/ 
  // 定义随机关节角度数据文件路径
  std::string rand_jnt_angles_file = 
    "/home/byd/Documents/rand_jnt_angles.csv";
  
  // 定义CSV格式输出格式
  Eigen::IOFormat CSVFormat(Eigen::FullPrecision,0,",","\n","","","","");
  
  // 定义末端执行器位姿日志文件路径（当前被注释掉未使用）
  std::string g_log_file = 
    "/home/byd/Documents/rand_jnt_angles_g.csv";
  
  // 创建文件输出流对象
  std::ofstream log_file;
  
  // 打开日志文件，以追加模式写入
  log_file.open(g_log_file, std::ofstream::out | std::ofstream::app);
  
  // 使用Armadillo库加载随机关节角度数据
  arma::Mat<double> rand_jnt_angles;
  rand_jnt_angles.load(rand_jnt_angles_file, arma::csv_ascii);
  
  // 遍历所有随机关节角度数据，计算对应的末端执行器位姿
  for(int i = 0; i < rand_jnt_angles.n_rows; i++)
  {
    // 从CSV文件中读取第i行的7个关节角度值
    for(int j = 0; j < 7; j++)
    {
      jnt_values(j) = rand_jnt_angles(i,j);
    }
    
    // 使用正向运动学求解器计算当前关节角度对应的末端执行器位姿
    planner_interface.kinlib_solver_.getFK(jnt_values, g_base_tool);
    
    /*
    // 如果日志文件打开成功，则将末端执行器位姿写入日志文件
    if(log_file.is_open())
    {
      log_file << g_base_tool.format(CSVFormat) << '\n';
        
      log_file.flush();
    }
    */
     
    // 打印当前末端执行器位姿到控制台
    ROS_INFO_STREAM("g_base_tool :\n\n" << g_base_tool << '\n');
  }
  
  /*** Part02_2 实时轨迹规划计算 ***/
  // 设置目标关节角度（使用CSV文件中的第3行数据）
  for(int j = 0; j < 7; j++)
  {
    jnt_values(j) = rand_jnt_angles(2,j);
  }
  
  // 计算目标关节角度对应的末端执行器位姿（作为运动规划的目标位姿）
  planner_interface.kinlib_solver_.getFK(jnt_values, g_base_tool);
  
  // 设置起始关节角度（使用CSV文件中的第2行数据）
  for(int j = 0; j < 7; j++)
  {
    jnt_values(j) = rand_jnt_angles(1,j);
  }
  
  // 创建关节轨迹对象，用于存储运动规划结果
  trajectory_msgs::JointTrajectory motion_plan;
  
  // 调用ScLERP运动规划器进行运动规划
  // 参数：起始关节角度、目标末端执行器位姿、规划结果存储变量
  bool res = planner_interface.solve(jnt_values, g_base_tool, motion_plan);
  
  // 打印运动规划结果信息
  ROS_INFO_STREAM("Motion Plan Result : " << res);                 // 规划是否成功
  ROS_INFO_STREAM("Trajectory Length  : " << motion_plan.points.size());  // 轨迹点数
  ROS_INFO_STREAM("Trajectory Point  : " << motion_plan.points[0].positions.size()); // 关节数
  
  // 提取轨迹中的第一个点的关节角度值
  std::vector<double> jnt_angle_vect = motion_plan.points[0].positions;
  
  // 空循环，保持程序运行直到ROS节点被关闭
  while(ros::ok())
  {
  
  }
  
  // 程序正常退出
  return 0;
}