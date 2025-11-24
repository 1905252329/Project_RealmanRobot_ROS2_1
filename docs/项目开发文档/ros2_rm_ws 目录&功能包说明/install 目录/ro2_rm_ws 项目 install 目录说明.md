## 专用接口包 rm_ros_interfaces

下面是 `install/rm_ros_interfaces` 目录的结构大纲与各模块说明，便于快速理解该安装包在工程中的作用与使用注意。

### 结构大纲（简化树）

install/rm_ros_interfaces
- include/
  - rm_ros_interfaces/
    - rm_ros_interfaces/
      - msg/
        - Armstate.h / Armstate.hpp
        - Movej.h / Movej.hpp
        - Gripperset.h / Gripperset.hpp
        - ... (大量由 rosidl 生成的消息头)
        - detail/ (自动生成的 struct / functions / type_support)
      - srv/
        - GetDH.h / GetDH.hpp
        - ModbusControl.h / ModbusControl.hpp
        - jodell_evs08/ (ActivateGripper, CloseModbus, ...)
        - detail/ (自动生成的 srv 结构与类型支持实现)
- lib/
  - librm_ros_interfaces__rosidl_generator_c.so
  - librm_ros_interfaces__rosidl_generator_py.so
  - librm_ros_interfaces__rosidl_typesupport_*.so
- local/
  - (本地安装/配置脚本目录)
- share/
  - rm_ros_interfaces/
    - msg/  (*.msg / *.idl 原始接口定义)
    - srv/  (*.srv / *.idl 原始接口定义)
    - cmake/ / environment/ / local_setup.* / package.*

### 可视化图

为便于快速查看目录结构，工程内提供两种可视化文件：

- Graphviz (.dot)：`docs/install_rm_ros_interfaces_visual/structure.dot`，可用 `dot -Tpng` 渲染为图片。
- ASCII 树：`docs/install_rm_ros_interfaces_visual/structure.txt`，适合终端快速查看。

示例：在项目根目录下将 dot 渲染为 PNG（可选）：

```bash
dot -Tpng docs/install_rm_ros_interfaces_visual/structure.dot -o docs/install_rm_ros_interfaces_visual/structure.png
```

文件位置（相对路径）：

 - `docs/install_rm_ros_interfaces_visual/structure.dot`
 - `docs/install_rm_ros_interfaces_visual/structure.txt`


### 各模块作用（要点）

- `include/.../msg` 与 `include/.../srv`（.h/.hpp）
  - 为 C/C++ 节点提供已生成的消息/服务类型定义（结构体、序列化/反序列化接口、构造器等），供发布/订阅/服务调用时 include 使用。

- `include/.../detail` 目录
  - 包含更低层的函数/struct 与类型支持实现（序列化、内存管理、构建器等），通常由生成代码或 typesupport 调用，不直接面向用户。

- `lib/*.so`
  - 运行时的类型支持与语言绑定库（rosidl_generator、typesupport_fastrtps、introspection 等），支持跨语言通信（C/C++/Python）。ROS2 在运行时会加载这些库来完成消息序列化/反序列化。

- `share/.../msg` 与 `share/.../srv`（原始 .msg/.srv/.idl）
  - 存放包的接口定义源文件（人可读），是生成头文件和类型支持的来源，便于查看字段与注释。

- `share/.../cmake` `environment` `local_setup.*` 等
  - 安装时生成的配置与环境装配脚本，source 这些脚本可以把该包加入运行时环境，注册 ament index 条目等。

总体说明：`install/rm_ros_interfaces` 是接口包的“已安装”副本，包含消息/服务定义、类型支持库与环境装配脚本，供项目内其它包在编译与运行时引用。

### 代表性 message / service 说明（示例字段与用途）

- Movej.msg（关节运动命令）
  - float32[] joint：关节目标角度数组
  - uint8 speed：速度等级或索引
  - bool block：是否阻塞直到运动完成
  - uint8 trajectory_connect：轨迹连接标志（0: 立即执行，1: 与下一条无缝连接）
  - uint8 dof：自由度/关节数
  - 用途：向运动控制节点发送关节插补/运动指令，支持阻塞与轨迹衔接。

- Armstate.msg（机器人状态上报）
  - float32[] joint：当前关节位置
  - geometry_msgs/Pose pose：末端位姿
  - uint16 err / uint8 err_len：错误码与描述长度
  - uint8 dof：自由度
  - 用途：状态上报用于监控、UI、故障检测与高层决策。

- Gripperset.msg（夹具/吸盘设定）
  - uint16 position：目标位置或吸力等级（例如 1–1000）
  - bool block：是否阻塞
  - uint16 timeout：阻塞超时（ms）
  - 用途：控制夹具开合或吸力，带阻塞与超时策略。

- ModbusControl.srv（底层 Modbus 控制）
  - Request: int32 port, int32 address, int32 device, int32 data
  - Response: bool success, string message
  - 用途：向 Modbus 驱动发自定义读写命令，常用于驱动调试或特殊控制。

- GetDH.srv（获取 DH 参数）
  - Response 包含 bool success, string message, float64[7] d,a,alpha,offset
  - 用途：查询机器人 DH 参数，用于运动学与标定。

### 契约与边界/注意事项

- 输入/输出：其他包通过 topic 发布消息（Movej/Movel 等）或通过 service 调用（ModbusControl/GetDH）。
- 错误处理：many messages/services 提供 err 或 success+message 字段，调用方应检查并做保护（停止/重试/人工介入）。
- 成功标准：运动命令被控制节点接收并在状态 topic 或服务响应中确认 success。

### 使用建议

- 在 CMake/colcon 中引用该接口包，应通过 `find_package(rm_ros_interfaces REQUIRED)` 或 ament 依赖并 link 对应的 typesupport。
- 运行时务必 source 安装目录下的 `local_setup.sh`（或 workspace 的 setup.bash），以便加载 `lib/*.so` 与 ament index 条目。
- 不要直接修改 `install/` 下的生成文件，修改应在 `src/rm_ros_interfaces` 的原始 `.msg`/`.srv` 文件中完成，然后重新 colcon build。

---

文档来源：基于工程中 `install/rm_ros_interfaces` 目录的已安装文件与 `src/ros2_rm_robot-humble/rm_ros_interfaces/CMakeLists.txt` 中的接口定义生成条目整理而成。
