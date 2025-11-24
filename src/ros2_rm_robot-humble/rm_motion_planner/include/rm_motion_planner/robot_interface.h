    /**
     * @file rm_interface.h
     * @author your name (you@domain.com)
     * @brief 
     * @version 0.1
     * @date 2025-08-22
     * @TODO 
     * @copyright Copyright (c) 2025
     * 
     */


    #include <vector>
    #include <string>
    #include <map>
    #include <iterator>
    #include <Eigen/Core>
    #include <Eigen/Geometry>

    // 添加 URDF 相关的头文件
    #include "urdf/model.h"

    #include <urdf/model.h>
    #include "rclcpp/rclcpp.hpp"
    #include <kdl/frames.hpp>

    // 添加 RM API 头文件
    #include "rm_driver/rm_interface.h"
    #include "rm_driver/rm_service.h"
    #include "rm_driver/rm_define.h"

    namespace RmInterface{

    /**
     * @brief DH参数结构体
     */
    struct DHParameters {
        double d;      // link offset
        double a;      // link length
        double alpha;  // link twist
        double theta;  // joint angle
    };


    class ParseURDF {
    public:
        

        /**
         * @brief 构造函数
         */
        ParseURDF() = default;
        

        /**
         * @brief 从URDF关节变换中提取DH参数
         * @param transform URDF中的关节变换
         * @param joint_index 关节索引
         * @return DH参数
         */
        DHParameters extractDHParameters(const urdf::Pose& transform, size_t joint_index);


        /**
         * @brief 更准确的DH参数提取方法
         * @param joint URDF关节
         * @param parent_link 父链接
         * @param child_link 子链接
         * @return DH参数
         */
        DHParameters computeDHParameters(const urdf::JointConstSharedPtr& joint,
                                            const urdf::LinkConstSharedPtr& parent_link,
                                            const urdf::LinkConstSharedPtr& child_link);
        

        /**
         * @brief 从URDF模型中解析所有关节的DH参数
         * @param urdf_model URDF模型
         * @param logger ROS2日志记录器
         */
        void parseDHParams(const urdf::Model& urdf_model, 
                                            rclcpp::Logger logger);
        

        /**
         * @brief 从URDF文件路径加载模型并解析DH参数，作为外部调用的入口
         * @param urdf_path URDF文件路径
         * @param logger ROS2日志记录器
         * @return 是否成功解析
         */
        bool loadDHParams(const std::string& urdf_path, rclcpp::Logger logger);


        bool getLinkNamesFromURDF(const std::string& urdf_path, std::string& base_link, std::string& tip_link);

    }; // 注意这里需要分号


    class RmRobotAPI{
    private:
        rm_robot_handle* robot_handle_;
        bool rm_api_initialized_;

    public:

        RmRobotAPI();

        void initialize_RmRobot(const std::string& robot_ip = "169.254.247.19");
        // void initialize_RmRobot();

        rm_pose_t compute_FK(const Eigen::VectorXd& joint_angles);

        // ✨ MoveJ角度透传函数
        int movej_canfd(const std::vector<double>& joint_positions);

        // 关节空间 MoveJ运动函数   
        int rm_movej(const std::vector<double>& joint_positions, int speed = 20);
    
        // ✨ 笛卡尔空间直线运动函数
        int rm_movel(const std::vector<float>& tcp_pose, int speed = 20);
    
        // ✨ 获取机械臂当前状态函数
        int rm_get_arm_state(rm_current_arm_state_t& state);
    
        // ✨ 切换当前工具坐标系
        int rm_change_tool_frame(const std::string& tool_name);
    
        // ✨ 添加getter方法来访问状态
        bool isInitialized() const { return rm_api_initialized_; }
        rm_robot_handle* getRobotHandle() const { return robot_handle_; }

}; // 注意这里需要分号

} // namespace RmInterface