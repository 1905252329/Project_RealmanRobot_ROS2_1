# rm_sclerp_planner_node_1_backup.cpp

## 各个函数的功能

### 1. CubicSplineInterpolator类函数

rm_sclerp_planner_node_1.cpp



```cpp
// ... existing code ...
class CubicSplineInterpolator {
public:
    // 构造函数：初始化参数
    CubicSplineInterpolator() {
        initParam();
    }

    // 析构函数：释放内存
    ~CubicSplineInterpolator() {
        releaseMem();
    }

    // 初始化参数
    void initParam() {
        x_sample_ = y_sample_ = M_ = NULL;
        sample_count_ = 0;
        bound1_ = bound2_ = 0;
    }

    // 释放内存
    void releaseMem() {
        delete[] x_sample_;
        delete[] y_sample_;
        delete[] M_;
        initParam();
    }

    // 加载数据并进行样条插值计算
    bool loadData(double* x_data, double* y_data, int count, double bound1, double bound2, BoundType type) {
        // ... implementation ...
    }

    // 样条插值核心算法
    bool spline(BoundType type) {
        // ... implementation ...
    }

    // 根据输入x计算对应的y值、速度和加速度
    bool getYbyX(double& x_in, double& y_out, double& vel, double& acc) {
        // ... implementation ...
    }

    // 三次样条插值主函数：对整个轨迹进行插值处理
    bool computeTrajectory(const std::vector<std::vector<double>>& path, 
                          const std::vector<double>& max_velocity,
                          const std::vector<double>& max_acceleration,
                          double frequency,
                          std::vector<std::vector<double>>& trajectory) {
        // ... implementation ...
    }

private:
    // 检查轨迹速度和加速度是否在限制范围内
    bool checkTrajectoryLimits(const std::vector<std::vector<double>>& velocities,
                              const std::vector<std::vector<double>>& accelerations,
                              const std::vector<double>& max_velocity,
                              const std::vector<double>& max_acceleration) 
    {
        // ... implementation ...
    }
    // ... existing code ...
};
// ... existing code ...
```

### 2. RmSclerpPlannerNode1类函数



```cpp
// ... existing code ...
class RmSclerpPlannerNode1 : public rclcpp::Node
{
public:
    // 构造函数：节点初始化
    explicit RmSclerpPlannerNode1(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("rm_sclerp_planner_node_1", options)
    {
        // ... implementation ...
    }

private:
    // 从URDF文件获取链接名称
    bool getLinkNamesFromURDF(const std::string& urdf_path, std::string& base_link, std::string& tip_link) {
        // ... implementation ...
    }
    
    // 关节状态回调函数：接收并更新当前关节位置
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // ... implementation ...
    }
    
    // 初始化规划器
    void initialize_planner() 
    {
        // ... implementation ...
    }
    
    // 定时器回调：定期检查并执行轨迹规划
    void timer_callback()
    {
        // ... implementation ...
    }

    // 初始化定时器回调：延迟初始化规划器
    void init_timer_callback()
    {
        // ... implementation ...
    }

    // 轨迹规划函数（旧版本）
    void plan_trajectory_alpha()
    {
        // ... implementation ...
    }

    // 轨迹规划函数（当前使用版本）
    void plan_trajectory()
    {
        // ... implementation ...
    }
    
    // 解析关节位置字符串参数
    void parse_joint_positions(const std::string& str, Eigen::VectorXd& joints)
    {
        // ... implementation ...
    }

    // 轨迹定时器回调：定期发布轨迹点
    void trajectory_timer_callback()
    {
        // ... implementation ...
    }

    // 获取DH参数
    void getDHparas() {
        // ... implementation ...
    }
    
    // 动态调整插值频率以满足速度和加速度限制
    double adjustFrequencyForLimits(const std::vector<std::vector<double>>& positions,
                                   const std::vector<double>& max_velocities,
                                   const std::vector<double>& max_accelerations,
                                   double initial_frequency) {
        // ... implementation ...
    }
    
    // 裁剪轨迹点以满足速度和加速度限制
    void clipTrajectoryForLimits(std::vector<std::vector<double>>& positions,
                                const std::vector<double>& max_velocities,
                                const std::vector<double>& max_accelerations,
                                double dt) {
        // ... implementation ...
    }
    // ... existing code ...
};
// ... existing code ...
```

## 节点运行顺序

1. **节点启动**：
   - 调用`RmSclerpPlannerNode1`构造函数
   - 声明参数、创建发布者和订阅者
   - 创建各种定时器
   - 启动`init_timer_`定时器
2. **初始化阶段**：
   - `init_timer_callback()`被调用
   - 调用`initialize_planner()`初始化规划器
   - 调用`getDHparas()`获取DH参数
   - 取消初始化定时器
3. **等待DH参数**：
   - `timer_callback()`定期检查DH参数是否获取完成
   - 一旦`dh_params_received_`为true且未规划轨迹，调用`plan_trajectory()`
4. **轨迹规划阶段**：
   - `plan_trajectory()`执行规划逻辑
   - 设置`trajectory_planned_ = true`标记规划完成
5. **轨迹执行阶段**：
   - `trajectory_timer_callback()`每20ms被调用
   - 发布轨迹点直到完成
   - 完成后关闭节点

## 函数调用关系图



```plainText
main()
└── RmSclerpPlannerNode1构造函数
    ├── 声明参数
    ├── 创建发布者/订阅者
    ├── 创建定时器
    │   ├── init_timer_ → init_timer_callback()
    │   │   └── initialize_planner()
    │   │       ├── getLinkNamesFromURDF()
    │   │       ├── 创建sclerp_interface::ScLERPInterface
    │   │       └── getDHparas()
    │   │           ├── 服务调用获取DH参数
    │   │           └── 从URDF解析DH参数
    │   ├── timer_ → timer_callback()
    │   │   └── plan_trajectory() [条件：dh_params_received_ && !trajectory_planned_]
    │   │       ├── parse_joint_positions()
    │   │       ├── planner_interface_->kinlib_solver_.getFK()
    │   │       ├── planner_interface_->solve()
    │   │       ├── CubicSplineInterpolator::computeTrajectory()
    │   │       │   ├── loadData()
    │   │       │   ├── spline()
    │   │       │   ├── getYbyX()
    │   │       │   └── checkTrajectoryLimits()
    │   │       ├── adjustFrequencyForLimits()
    │   │       ├── clipTrajectoryForLimits()
    │   │       └── 转换为JointTrajectory格式
    │   └── trajectory_timer_ → trajectory_timer_callback()
    │       └── 发布轨迹点到/rm_driver/movej_canfd_cmd
    └── joint_state_subscriber_ → joint_state_callback()
        └── 更新current_joint_positions_
```

## 关键流程说明

1. **参数处理流程**：
   - `parse_joint_positions()` → 解析字符串参数为Eigen向量
2. **运动学计算流程**：
   - `getFK()` → 正向运动学计算末端位姿
3. **轨迹规划流程**：
   - `solve()` → ScLERP规划算法生成粗轨迹
   - `CubicSplineInterpolator::computeTrajectory()` → 三次样条插值细化轨迹
   - `adjustFrequencyForLimits()` → 调整频率确保限制条件
   - `clipTrajectoryForLimits()` → 裁剪轨迹确保速度加速度限制
4. **轨迹执行流程**：
   - `trajectory_timer_callback()` → 定期发布轨迹点
   - 发布完成后关闭节点

这个设计确保了节点按正确的顺序初始化各个组件，然后执行规划和控制任务。



## 运行结果





从输出结果来看，就DH参数获取来看，存在两个问题：

1）两种方式获取DH参数的结果输出都出现了两次，存在重复

2） 通过服务调用获取DH参数的结果第二次输出在完成轨迹规划和插值之后

3）通过服务调用获取DH参数的结果输出在从URDF 文件中解析DH参数的结果之后

我希望节点运行的正确输出是，先输出通过服务调用获取DH参数的结果，再输出通过从URDF文件中解析DH参数的结果 ，然后执行轨迹规划和轨迹执行。

## 问题分析与解决

### 问题分析

通过查看代码，我发现问题的根源在于：

1. [initialize_planner()](javascript:void(0))函数中调用了[getDHparas()](javascript:void(0))
2. 在服务回调中也输出了DH参数信息
3. [timer_callback()](javascript:void(0))中可能会重复调用[getDHparas()](javascript:void(0))

### 解决方法

修改timer_callback函数，移除不必要的重试机制

修改initialize_planner函数，移除重复的getDHparas调用

修改init_timer_callback函数，在初始化完成后调用getDHparas

#### 1. 防止重复执行DH参数获取

添加了静态标志位`dh_params_requested`来确保[getDHparas()](javascript:void(0))函数只执行一次：

```
cppstatic bool dh_params_requested = false;
if (dh_params_requested) {
    return;
}
dh_params_requested = true;
```

#### 2. 调整输出顺序

将URDF文件解析DH参数的代码提取到一个独立的函数[parseURDFDHParams()](javascript:void(0))中，并只在服务回调完成后调用它，确保输出顺序为：

1. 通过服务调用获取DH参数
2. 从URDF文件中解析DH参数
3. 轨迹规划和执行

#### 3. 简化代码结构

移除了[timer_callback()](javascript:void(0))中的重试机制，避免重复调用[getDHparas()](javascript:void(0))。

#### 4. 明确初始化流程

在[init_timer_callback()](javascript:void(0))中明确调用[getDHparas()](javascript:void(0))，并移除了[initialize_planner()](javascript:void(0))中的重复调用。

#### 工作原理

修改后的代码工作流程如下：

1. 节点启动后，[init_timer_callback()](javascript:void(0))被调用
2. [initialize_planner()](javascript:void(0))初始化规划器
3. [getDHparas()](javascript:void(0))被调用，开始服务调用获取DH参数
4. 服务调用完成后，在回调函数中输出服务获取的DH参数
5. 回调函数接着调用[parseURDFDHParams()](javascript:void(0))输出URDF解析的DH参数
6. [timer_callback()](javascript:void(0))检测到DH参数已获取，执行轨迹规划