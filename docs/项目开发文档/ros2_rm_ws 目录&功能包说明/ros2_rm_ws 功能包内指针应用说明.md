# 功能包内指针应用说明

目的：列出工作区 `src/` 下手写源文件中出现的智能指针（std::unique_ptr / std::shared_ptr / rclcpp::...::SharedPtr 等），按功能包聚合，包含文件路径、使用类型和 1-3 行代码片段作证。

检查项
- [x] 只检索 `src/` 下的手写源/头文件（已排除 build/ 和 install/ 自动生成文件）
- [x] 查找 std::unique_ptr / std::shared_ptr 及常见别名（SharedPtr 等）
- [x] 按功能包聚合，列出文件、指针类型与少量上下文代码片段

汇总（按功能包）

## rm_motion_planner
- src/ros2_rm_robot-humble/rm_motion_planner/src/rm_dual_arm_sclerp_planner_node.cpp
  - 指针类型：std::unique_ptr
  - 示例（附近行）：
    - 行 ~90：
      std::unique_ptr<sclerp_interface::ScLERPInterface> left_arm_planner_;
      std::unique_ptr<sclerp_interface::ScLERPInterface> right_arm_planner_;

- src/ros2_rm_robot-humble/rm_motion_planner/src/rm_sclerp_planner_node_1.cpp
  - 指针类型：std::unique_ptr, rclcpp::Publisher::SharedPtr, rclcpp::Subscription::SharedPtr 等
  - 示例（附近行）：
    - 行 ~454：
      std::unique_ptr<sclerp_interface::ScLERPInterface> planner_interface_;
      rclcpp::Publisher<rm_ros_interfaces::msg::Jointpos>::SharedPtr joint_pos_publisher_;
      std::unique_ptr<RmInterface::ParseURDF> parseURDF_;
      std::unique_ptr<RmInterface::RmRobotAPI> rmRobotAPI_;

- src/ros2_rm_robot-humble/rm_motion_planner/src/rm_sclerp_planner_node.cpp
  - 指针类型：std::unique_ptr, rclcpp_action::Client::SharedPtr, rclcpp::TimerBase::SharedPtr
  - 示例（附近行）：
    - 行 ~426：
      std::unique_ptr<sclerp_interface::ScLERPInterface> planner_interface_;
      rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_ptr_;

- src/ros2_rm_robot-humble/rm_motion_planner/src/rm_sclerp_planner_test.cpp
  - 指针类型：std::unique_ptr, rclcpp_action::Client::SharedPtr
  - 示例（附近行）：
    - 行 ~141：
      std::unique_ptr<sclerp_interface::ScLERPInterface> planner_interface_;

- 其它说明文档中（仅供参考）：
  - src/ros2_rm_robot-humble/rm_motion_planner/代码变更记录.md
    - 使用：const std::shared_ptr<rm_ros_interfaces::srv::GetDH::Request> request


## rm_jodell_evs08_control
- src/ros2_rm_robot-humble/rm_jodell_evs08_control/src/jodell_evs08_control_node.cpp
  - 指针类型：std::unique_ptr
  - 示例（附近行）：
    - 行 ~98：
      std::unique_ptr<JodellEvs08Driver> gripper_driver_;


## rm_control
- src/ros2_rm_robot-humble/rm_control/src/rm_control.cpp
  - 指针类型：std::shared_ptr（action 回调/GoalHandle 等签名中）
  - 示例（附近行）：
    - 行 ~291：
      rclcpp_action::GoalResponse Rm_Control::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal)

- src/ros2_rm_robot-humble/rm_control/include/rm_control.h
  - 指针类型：std::shared_ptr（函数声明中）
  - 示例（附近行）：
    - 行 ~45：
      rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal);


## rm_driver
- src/ros2_rm_robot-humble/rm_driver/src/rm_driver.cpp
  - 指针类型：std::shared_ptr（服务回调签名、lambda 回调中）
  - 示例（附近行）：
    - 行 ~1680：
      void RmArm::modbusControlCallback(
          const std::shared_ptr<rm_ros_interfaces::srv::ModbusControl::Request> request,
          std::shared_ptr<rm_ros_interfaces::srv::ModbusControl::Response> response)
    - 行 ~1706：
      void RmArm::getDHCallback(const std::shared_ptr<rm_ros_interfaces::srv::GetDH::Request> request,
                                std::shared_ptr<rm_ros_interfaces::srv::GetDH::Response> response)

- src/ros2_rm_robot-humble/rm_driver/include/rm_driver/rm_driver.h
  - 指针类型：std::shared_ptr（服务回调声明、subscription 回调等）
  - 示例（附近行）：
    - 行 ~300：
      void modbusControlCallback(const std::shared_ptr<rm_ros_interfaces::srv::ModbusControl::Request> request,
                                 std::shared_ptr<rm_ros_interfaces::srv::ModbusControl::Response> response);


## third_party/sclerp_motion_planner
- src/third_party/sclerp_motion_planner/src/example_aubo_server.cpp
  - 指针类型：std::shared_ptr、std::vector<std::shared_ptr<...>>
  - 示例（附近行）：
    - 行 ~311：
      int getVisionTote(std::shared_ptr<comm_api::GetPoseClient> get_pose_client, Eigen::Matrix4d &server_poses, const std::string &request_type, unsigned &flag)
    - 使用：std::vector<std::shared_ptr<CollisionUtils::ObstacleBase>> collision_objects;

- src/third_party/sclerp_motion_planner/src/sclerp_interface.cpp
  - 指针类型：std::shared_ptr<rclcpp::Node>, std::shared_ptr<CollisionUtils::ObstacleBase>, std::vector<std::shared_ptr<...>>
  - 示例（附近行）：
    - 行 ~36：
      ScLERPInterface::ScLERPInterface(..., const std::shared_ptr<rclcpp::Node> &nh, ...)
    - 使用：const std::vector<std::shared_ptr<CollisionUtils::ObstacleBase>> &obstacles


## third_party/kinlib
- src/third_party/kinlib/src/collision_utils.cpp
  - 指针类型：std::shared_ptr（工厂函数返回、容器元素）
  - 示例（附近行）：
    - 行 ~1：
      std::shared_ptr<ObstacleBase> createBox(...){ return std::make_shared<BoxObstacle>(...); }
    - 使用：std::vector<std::shared_ptr<ObstacleBase>> armCylinderModel(...)

- src/third_party/kinlib/src/kinlib_kinematics.cpp
  - 指针类型：const std::vector<std::shared_ptr<CollisionUtils::ObstacleBase>> &obstacles, const std::shared_ptr<CollisionUtils::ObstacleBase> &grasped_object
  - 示例（附近行）：
    - 行 ~1287：
      const std::vector<std::shared_ptr<CollisionUtils::ObstacleBase>> &obstacles,
      const std::shared_ptr<CollisionUtils::ObstacleBase> &grasped_object,


---

说明与下一步建议：
- 文档已保存为项目根目录下的 `功能包内指针应用说明.md`。
- 如需更细化的输出，我可以：
  1) 为每个匹配项导出具体行号与 3 行上下文（全部文件），并写入同一文档；
  2) 过滤掉 third_party，只保留自研包 `rm_*` 的清单；
  3) 生成一个 CSV（包名, 文件路径, 指针类型, 行号）供进一步处理。

如果你选择一个选项，我会继续执行并更新文档。

## 匹配项详细上下文（行号 + 3 行片段）

### rm_motion_planner（详细）

- 说明：下面代码片段展示 rm_motion_planner 包中使用智能指针的位置，通常用来管理规划器实例或 RM API 封装对象，避免手动释放资源。

- src/ros2_rm_robot-humble/rm_motion_planner/src/rm_dual_arm_sclerp_planner_node.cpp: 行 93-95

```cpp
// 该成员为左臂规划器接口，使用 unique_ptr 管理生命周期（RAII）
  // 左臂规划器接口
  std::unique_ptr<sclerp_interface::ScLERPInterface> left_arm_planner_;

```

- src/ros2_rm_robot-humble/rm_motion_planner/src/rm_dual_arm_sclerp_planner_node.cpp: 行 96-98

```cpp
// 该成员为右臂规划器接口，使用 unique_ptr 管理生命周期，确保析构时自动释放
  // 右臂规划器接口
  std::unique_ptr<sclerp_interface::ScLERPInterface> right_arm_planner_;

```

- src/ros2_rm_robot-humble/rm_motion_planner/src/rm_sclerp_planner_node_1.cpp: 行 456-458

```cpp
// ScLERP 规划器的实例，使用 unique_ptr 明确表示独占所有权
  // ScLERP规划器接口实例
  std::unique_ptr<sclerp_interface::ScLERPInterface> planner_interface_;

```

- src/ros2_rm_robot-humble/rm_motion_planner/src/rm_sclerp_planner_node_1.cpp: 行 499-501

```cpp
// ParseURDF 与 RmRobotAPI 使用 unique_ptr 持有实现细节，便于在析构时自动清理
  // 实例化自定义封装的RM Robot API 接口
  std::unique_ptr<RmInterface::ParseURDF> parseURDF_;
  std::unique_ptr<RmInterface::RmRobotAPI> rmRobotAPI_;

```

- src/ros2_rm_robot-humble/rm_motion_planner/src/rm_sclerp_planner_node.cpp: 行 428-430

```cpp
// 规划器实例与 action 客户端（SharedPtr 为 rclcpp 的共享指针类型，用于回调/跨对象共享）

  std::unique_ptr<sclerp_interface::ScLERPInterface> planner_interface_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_ptr_;

```

- src/ros2_rm_robot-humble/rm_motion_planner/src/rm_sclerp_planner_test.cpp: 行 143-145

```cpp
// 测试代码中同样使用 unique_ptr 持有规划器实例，和 action 客户端的 SharedPtr

  std::unique_ptr<sclerp_interface::ScLERPInterface> planner_interface_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_ptr_;

```

- src/ros2_rm_robot-humble/rm_motion_planner/src/rm_sclerp_planner_node_1_backup(2).cpp: 行 411-413

```cpp
// 备份文件中也保留了 planner_interface_ 的 unique_ptr 定义，表明设计一致性
  // ScLERP规划器接口实例
  std::unique_ptr<sclerp_interface::ScLERPInterface> planner_interface_;

```

- src/ros2_rm_robot-humble/rm_motion_planner/src/rm_sclerp_planner_node_1_backup.cpp: 行 411-413

```cpp
// 另一个备份/历史文件，展示相同的智能指针用法
  // ScLERP规划器接口实例
  std::unique_ptr<sclerp_interface::ScLERPInterface> planner_interface_;

```

- src/ros2_rm_robot-humble/rm_motion_planner/代码变更记录.md: 行 479-483

```markdown
<!-- 说明：示例为文档注释，展示服务回调中使用 std::shared_ptr 的正确命名空间 -->
// 错误的命名空间使用
const std::shared_ptr<rm_msgs::srv::GetDH::Request> request,

// 正确的命名空间使用
const std::shared_ptr<rm_ros_interfaces::srv::GetDH::Request> request,

```


### rm_jodell_evs08_control（详细）

- 说明：驱动节点通过 unique_ptr 管理驱动器对象，保证在节点析构时关闭/释放设备资源。

- src/ros2_rm_robot-humble/rm_jodell_evs08_control/src/jodell_evs08_control_node.cpp: 行 99-101

```cpp
// gripper_driver_ 使用 unique_ptr 管理底层驱动实例的生命周期
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::unique_ptr<JodellEvs08Driver> gripper_driver_;
};

```


### rm_control（详细）

- 说明：action 回调签名使用 std::shared_ptr 传递 Goal/GoalHandle，符合 rclcpp action API 要求，用于跨线程安全共享。

- src/ros2_rm_robot-humble/rm_control/include/rm_control.h: 行 49-53

```cpp
// action 回调声明，参数使用 std::shared_ptr 或 rclcpp 的 SharedPtr 以支持回调中的共享访问

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFJT> goal_handle);
  void execute_move(const std::shared_ptr<GoalHandleFJT> goal_handle);

```

- src/ros2_rm_robot-humble/rm_control/src/rm_control.cpp: 行 292-294

```cpp
// action 接收实现示例，使用 std::shared_ptr<const FollowJointTrajectory::Goal>
}

rclcpp_action::GoalResponse Rm_Control::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal)

```


### rm_driver（详细）

- 说明：服务回调采用 std::shared_ptr 作为请求/响应参数类型（ROS2 风格），并在 lambda 回调中交接给成员函数。

- src/ros2_rm_robot-humble/rm_driver/src/rm_driver.cpp: 行 1686-1690

```cpp
// modbus 控制服务回调，request/response 使用 std::shared_ptr 以便 rclcpp 管理内存
void RmArm::modbusControlCallback(
  const std::shared_ptr<rm_ros_interfaces::srv::ModbusControl::Request> request,
  std::shared_ptr<rm_ros_interfaces::srv::ModbusControl::Response> response)
{

```

- src/ros2_rm_robot-humble/rm_driver/src/rm_driver.cpp: 行 1704-1708

```cpp
// GetDH 服务回调示例，使用 std::shared_ptr 处理请求/响应
}

void RmArm::getDHCallback(const std::shared_ptr<rm_ros_interfaces::srv::GetDH::Request> request,
              std::shared_ptr<rm_ros_interfaces::srv::GetDH::Response> response)
{

```

- src/ros2_rm_robot-humble/rm_driver/src/rm_driver.cpp: 行 2366-2369

```cpp
// 使用 lambda 创建服务时，仍然以 shared_ptr 作为参数类型并转发到成员函数
  modbus_control_service_ = this->create_service<rm_ros_interfaces::srv::ModbusControl>(
    "modbus_control",
    [this](const std::shared_ptr<rm_ros_interfaces::srv::ModbusControl::Request> request,
       std::shared_ptr<rm_ros_interfaces::srv::ModbusControl::Response> response) {
    this->modbusControlCallback(request, response);
    }

```

- src/ros2_rm_robot-humble/rm_driver/include/rm_driver/rm_driver.h: 行 303-309

```cpp
// 头文件中声明服务回调，使用 std::shared_ptr 作为签名的一部分

  // Service callback functions
  void modbusControlCallback(const std::shared_ptr<rm_ros_interfaces::srv::ModbusControl::Request> request,
                 std::shared_ptr<rm_ros_interfaces::srv::ModbusControl::Response> response);
                                
  void getDHCallback(const std::shared_ptr<rm_ros_interfaces::srv::GetDH::Request> request,
             std::shared_ptr<rm_ros_interfaces::srv::GetDH::Response> response);


```


### third_party/sclerp_motion_planner（详细）

- 说明：第三方规划库中大量使用 shared_ptr 管理复杂对象（如 RPC 客户端、障碍物对象等），便于在多个组件间共享引用。

- src/third_party/sclerp_motion_planner/src/example_aubo_server.cpp: 行 309-313

```cpp
// 从视觉服务获取托盘位姿时，get_pose_client 使用 shared_ptr 传递
int getVisionTote(std::shared_ptr<comm_api::GetPoseClient> get_pose_client, Eigen::Matrix4d &server_poses, const std::string &request_type, unsigned &flag) {
  try {
    std::string raw_poses;

```

- src/third_party/sclerp_motion_planner/src/example_aubo_server.cpp: 行 337-341

```cpp
// 类似的视觉请求封装，使用 shared_ptr 以便在多个函数间安全传递客户端对象
int getVisionTray(std::shared_ptr<comm_api::GetPoseClient> get_pose_client, Eigen::Matrix4d &server_poses, const std::string &request_type, unsigned &flag) {
  try {
    std::string raw_poses;

```

- src/third_party/sclerp_motion_planner/src/example_aubo_server.cpp: 行 401-403

```cpp
// 障碍物列表以 vector<shared_ptr<ObstacleBase>> 表示，便于多处引用同一障碍物对象
  std::vector<std::shared_ptr<CollisionUtils::ObstacleBase>> collision_objects;

  Eigen::Vector3d dimension_l, dimension_s;

```

- src/third_party/sclerp_motion_planner/src/example_aubo_server.cpp: 行 589-591

```cpp
// 重复使用的片段，展示相同的障碍物容器用法
  std::vector<std::shared_ptr<CollisionUtils::ObstacleBase>> collision_objects;

  Eigen::Vector3d dimension_l, dimension_s;

```

- src/third_party/sclerp_motion_planner/src/example_aubo_server.cpp: 行 1392-1394

```cpp
// handle_command 中将 get_pose_client 以 shared_ptr 传入，便于并发/回调中使用
// logic of full workflow
bool handle_command(int64_t command, RpcClientPtr rpc_cli, sclerp_interface::ScLERPInterface &planner_interface, GripperController &gripper, std::shared_ptr<comm_api::GetPoseClient> get_pose_client) {
    

```


### third_party/kinlib（详细）

- 说明：kinlib 使用 shared_ptr 作为工厂函数返回和容器元素类型，便于统一管理碰撞对象的生命周期与共享。

- src/third_party/kinlib/src/collision_utils.cpp: 行 11-15

```cpp
// 工厂函数返回 shared_ptr，便于调用方直接使用而无需额外管理内存
using kinlib::ErrorCodes;

std::shared_ptr<ObstacleBase> createBox(const Eigen::Vector3d &dimensions, 
                    const Eigen::Vector3d &position,
```

- src/third_party/kinlib/src/collision_utils.cpp: 行 67-71

```cpp
// armCylinderModel 返回 vector<shared_ptr<ObstacleBase>>，表示链节的圆柱模型集合
std::vector<std::shared_ptr<ObstacleBase>> armCylinderModel(
  const int num_links_ignore, 
  const std::vector<double> radius_array,

```

- src/third_party/kinlib/src/collision_utils.cpp: 行 111-113

```cpp
// createMeshFromSTL 返回 shared_ptr 的 MeshObstacle，供碰撞检测使用
std::shared_ptr<ObstacleBase> createMeshFromSTL(
  const std::string& stl_path, 
  const Eigen::Matrix4d& transform) {

```

- src/third_party/kinlib/src/collision_utils.cpp: 行 154-156

```cpp
// buildLinkMeshes 构建并返回一组 shared_ptr 的网格对象
std::vector<std::shared_ptr<ObstacleBase>> buildLinkMeshes(const std::vector<std::string>& stl_files) {
  std::vector<std::shared_ptr<ObstacleBase>> link_meshes;


```

- src/third_party/kinlib/src/collision_utils.cpp: 行 264-268

```cpp
// getCollisionInfo 接口接受 obstacles 和 grasped_object 的 shared_ptr 引用以便检查碰撞
ErrorCodes getCollisionInfo(
  const std::vector<std::shared_ptr<ObstacleBase>> &link_cylinders,
  const std::vector<std::shared_ptr<ObstacleBase>> &obstacles,
  const std::shared_ptr<ObstacleBase> &grasped_object,

```
