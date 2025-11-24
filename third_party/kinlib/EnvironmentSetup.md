> 本说明指导如何在 Ubuntu + ROS 2 系统上，一次性构建并使用 `kinlib` 和 `sclerp_motion_planner` 两个包  
> 已在 **Ubuntu 22.04 + ROS 2 Humble** 上测试，其他 ROS 2 发行版仅需微调即可套用。

## 1. 系统依赖

安装所有系统级库：

```bash
sudo apt update && sudo apt install -y \
  build-essential cmake pkg-config \
  libeigen3-dev libboost-system-dev libboost-filesystem-dev \
  libassimp-dev libccd-dev libfcl-dev libmodbus-dev \
  python3-dev python3-pybind11
```

`libmodbus-dev` 仅限 `sclerp_motion_planner` 使用；其余为两共用依赖。

## 2. ROS 2 依赖

### 2.1 添加 ROS 2 软件源

```bash
sudo apt update && sudo apt install -y curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 2.2 安装桌面版 ROS 2

```bash
sudo apt update
sudo apt install ros-humble-desktop
```

> 使用其他发行版时，将 `humble` 替换为 `foxy`、`iron` ……

### 2.3 再安装补充包

```bash
sudo apt install -y \
  ros-humble-ament-cmake \
  ros-humble-rclcpp \
  ros-humble-std-msgs \
  ros-humble-geometry-msgs \
  ros-humble-trajectory-msgs \
  ros-humble-urdf \
  ros-humble-kdl-parser \
  ros-humble-fcl \
  ros-humble-pybind11-vendor \
  ros-humble-rosbag2-cpp
```

------

## 3. 工作区准备

将压缩包复制、解压至工作空间
 (假设目录为 `~/ros2_ws/src`)

------

## 4. 构建包

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select kinlib sclerp_motion_planner
```

> 若想一次性全构建，可直接 `colcon build`.

------

## 5. 环境生效

```bash
source ~/ros2_ws/install/setup.bash
```

------

## 6. 关键依赖速查表

| 依赖          | 系统包                                             | ROS 2 包                      |
| ----------- | ----------------------------------------------- | ---------------------------- |
| Eigen3      | `libeigen3-dev`                                 |                              |
| Boost       | `libboost-system-dev` `libboost-filesystem-dev` |                              |
| Assimp      | `libassimp-dev`                                 |                              |
| FCL         | `libfcl-dev`                                    | `ros-humble-fcl`             |
| CCD         | `libccd-dev`                                    |                              |
| pybind11    | `python3-pybind11`                              | `ros-humble-pybind11-vendor` |
| Modbus      | `libmodbus-dev`                                 |                              |
| ROS 2 核心    |                                                 | `ament-cmake`, `rclcpp`, …   |
| KDL/URDF    |                                                 | `urdf`, `kdl_parser`         |
| rosbag2     |                                                 | `ros-humble-rosbag2-cpp`     |
| kinlib / 本包 | 源码编译                                            | 位于工作空间                       |

------

## 7. 使用提示

- 平台自带的 `libfcl-dev` / `libccd-dev` 过旧时，可考虑源码安装（见第 **10** 节）。
- 如果 `pybind11` 的 CMake 集成报错，请同时确认系统包 `python3-pybind11` 与 ROS 2 包 `pybind11-vendor` 都已安装。
- 构建完毕后，Python 绑定 `kinlib_py`、`sclerp_py` 会出现在 `install/lib` 下；如直接用 Python 导入，请按需更新 `PYTHONPATH`。
- 修改源代码或依赖后，务必 **重新** `colcon build`.

------

## 8. 一键安装（只用一行）

```bash
sudo apt update && sudo apt install -y \
  build-essential cmake pkg-config libeigen3-dev libboost-system-dev \
  libboost-filesystem-dev libassimp-dev libccd-dev libfcl-dev libmodbus-dev \
  python3-dev python3-pybind11 \
  ros-humble-ament-cmake ros-humble-rclcpp ros-humble-std-msgs \
  ros-humble-geometry-msgs ros-humble-trajectory-msgs ros-humble-urdf \
  ros-humble-kdl-parser ros-humble-fcl ros-humble-pybind11-vendor \
  ros-humble-rosbag2-cpp
```

------

## 9. 常见问题排查

| 现象                | 解决思路                                                                       |
| ----------------- | -------------------------------------------------------------------------- |
| Library not found | `dpkg -L <package>` 检查包文件                                                  |
| CMake 报错          | `colcon build --event-handlers console_direct+` 获取详细输出                     |
| Python import 失败  | 将 `~/ros2_ws/install/<pkg>/lib` 加入 `PYTHONPATH`，或可用 `pip install .`        |
| ROS 2 包未找到        | 确认已先后 source `/opt/ros/humble/setup.bash` 与 `~/ros2_ws/install/setup.bash` |

------

## 10. 进阶：源码编译 FCL / CCD（可选）

> 仅当系统包过旧或需要最新特性时使用。

```bash
# FCL
git clone https://github.com/flexible-collision-library/fcl.git
cd fcl && mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install

# CCD
git clone https://github.com/danfis/libccd.git
cd libccd && mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

------

```

```