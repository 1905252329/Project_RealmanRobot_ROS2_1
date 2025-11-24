//
// Created by ubuntu on 23-11-28.
//
#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rm_ros_interfaces/msg/movej.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/****************************************创建类************************************/ 
class MoveJDemo: public rclcpp::Node
{
  public:
    MoveJDemo();          //构造函数
    void movej_demo();    //发布MoveJ规划指令
    void MovejDemo_Callback(const std_msgs::msg::Bool & msg);   //结果回调函数
  
  private:
    rclcpp::Publisher<rm_ros_interfaces::msg::Movej>::SharedPtr publisher_;            //声明发布器
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;   //声明订阅器
    rm_ros_interfaces::msg::Movej movej_way;
    int arm_dof_ = 6;
};


/******************************接收到订阅的机械臂执行状态消息后，会进入消息回调函数**************************/ 
void MoveJDemo::MovejDemo_Callback(const std_msgs::msg::Bool & msg)
{
    // 将接收到的消息打印出来，显示是否执行成功
    if(msg.data)
    {
        RCLCPP_INFO (this->get_logger(),"*******Movej succeeded\n");
    } else {
        RCLCPP_INFO (this->get_logger(),"*******Movej Failed\n");
    }
}   


/*******************************************发布movej指令函数****************************************/
/**
 * @brief 发布MoveJ运动指令
 * @details 根据机械臂自由度设置相应的关节角度目标位置，并发布MoveJ运动指令
 * 
 * 设置内容包括：
 * - 根据arm_dof_值设置6自由度或7自由度机械臂的关节角度目标值
 * - 设置运动速度为20
 * - 设置轨迹连接模式为0（立即规划）
 * - 设置阻塞模式为true（等待执行完成）
 */
void MoveJDemo::movej_demo()
{
    // 设置6自由度机械臂的目标关节角度
    if(arm_dof_ == 6)
    {
      movej_way.joint[0] = -0.360829;
      movej_way.joint[1] = 0.528468;
      movej_way.joint[2] = 1.326293;
      movej_way.joint[3] = -0.000454;
      movej_way.joint[4] = 1.221748;
      movej_way.joint[5] = 0.000052;
      movej_way.speed = 20;
      movej_way.dof = 6;
    }

    // 设置7自由度机械臂的目标关节角度
    if(arm_dof_ == 7)
    {
      movej_way.joint[0] = 0.176278;
      movej_way.joint[1] = 0.0;
      movej_way.joint[2] = 0.3543;
      movej_way.joint[3] = 0.53;
      movej_way.joint[4] = 0.00873;
      movej_way.joint[5] = 0.3595;
      movej_way.joint[6] = 0.3595;
      movej_way.speed = 20;
      movej_way.dof = 7;
    }
    // 设置轨迹连接模式为立即规划，阻塞模式为true，并发布指令
    movej_way.trajectory_connect = 0;
    movej_way.block = true;
    this->publisher_->publish(movej_way);
    
}


/***********************************构造函数，初始化发布器订阅器****************************************/
/**
 * @brief MoveJDemo类的构造函数
 * @details 初始化ROS2节点、参数、发布器和订阅器，并启动MoveJ演示
 * 
 * 功能包括：
 * 1. 初始化ROS2节点，节点名为"Movej_demo"
 * 2. 声明并获取机械臂自由度参数arm_dof
 * 3. 根据自由度调整关节向量大小
 * 4. 创建订阅器接收MoveJ执行结果
 * 5. 创建发布器发送MoveJ指令
 * 6. 启动MoveJ演示流程
 */
MoveJDemo::MoveJDemo():rclcpp::Node("Movej_demo")
{
  // 声明并获取机械臂自由度参数
  this->declare_parameter<int>("arm_dof", arm_dof_);
  this->get_parameter("arm_dof", arm_dof_);
  RCLCPP_INFO (this->get_logger(),"arm_dof is %d\n",arm_dof_);
  
  // 根据自由度设置关节向量大小
  if(arm_dof_ == 6)
  {movej_way.joint.resize(6);}
  else if(arm_dof_ == 7)
  {movej_way.joint.resize(7);}
  
  // 创建订阅器和发布器
  subscription_ = this->create_subscription<std_msgs::msg::Bool>("/rm_driver/movej_result", rclcpp::ParametersQoS(), std::bind(&MoveJDemo::MovejDemo_Callback, this,_1));
  
  publisher_ = this->create_publisher<rm_ros_interfaces::msg::Movej>("/rm_driver/movej_cmd", rclcpp::ParametersQoS());
  
  // 等待2秒确保节点初始化完成，然后启动演示
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  movej_demo();
}


/******************************************************主函数*********************************************/
/**
* @brief 主函数
*
* 初始化ROS 2节点，并启动MoveJDemo节点。
*
* @param argc 命令行参数数量
* @param argv 命令行参数数组
*
* @return 0 表示程序正常退出
*/
int main(int argc, char** argv)
{
    // 初始化 ROS 2 节点
  rclcpp::init(argc, argv);

    // 创建一个 MoveJDemo 类的共享指针，并启动节点
  rclcpp::spin(std::make_shared<MoveJDemo>());

    // 关闭 ROS 2 节点
  rclcpp::shutdown();

    // 返回 0 表示程序正常结束
  return 0;
}
