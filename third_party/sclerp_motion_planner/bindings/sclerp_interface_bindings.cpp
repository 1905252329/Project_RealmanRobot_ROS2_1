#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "sclerp_motion_planner/sclerp_interface.h"

namespace py = pybind11;
using namespace sclerp_interface;

PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void bind_sclerp_interface(py::module_ &m) {
    py::class_<rclcpp::Node, std::shared_ptr<rclcpp::Node>>(m, "Node");

    py::class_<ScLERPInterface>(m, "ScLERPInterface")
        .def(py::init<const std::string,
                      const std::string,
                      const std::shared_ptr<rclcpp::Node>&,
                      const std::string,
                      const std::vector<std::string>>(),
            py::arg("base_link"),
            py::arg("tip_link"),
            py::arg("node"),
            py::arg("urdf_path") = "",
            py::arg("stl_paths") = std::vector<std::string>()) 

        .def("solve_basic", [](ScLERPInterface &self,
            const Eigen::Ref<const Eigen::VectorXd> &init_jnt,
            const Eigen::Matrix4d &g_f) {
            trajectory_msgs::msg::JointTrajectory jtraj;
            bool success = self.solve(init_jnt, g_f, jtraj);

            std::vector<Eigen::VectorXd> trajectory;
            for (const auto& pt : jtraj.points) {
            Eigen::VectorXd v(pt.positions.size());
            for (size_t i = 0; i < pt.positions.size(); ++i) {
                v(i) = pt.positions[i];
            }
            trajectory.push_back(v);
            }

            return py::make_tuple(success, trajectory);
            })

        .def("solve_with_collision", [](ScLERPInterface &self,
            const Eigen::Ref<const Eigen::VectorXd> &init_jnt,
            const Eigen::Matrix4d &g_f,
            const bool check_self_collision,
            const int num_links_ignore,
            const std::vector<std::shared_ptr<CollisionUtils::ObstacleBase>> &obstacles,
            py::object grasped_obj_py) {  // accept raw Python object
        
            std::shared_ptr<CollisionUtils::ObstacleBase> grasped_obj = nullptr;
            if (!grasped_obj_py.is_none()) {
                grasped_obj = grasped_obj_py.cast<std::shared_ptr<CollisionUtils::ObstacleBase>>();
            }
        
            trajectory_msgs::msg::JointTrajectory jtraj;
            bool success = self.solve(init_jnt, g_f, check_self_collision, num_links_ignore, obstacles, grasped_obj, jtraj);
        
            std::vector<Eigen::VectorXd> trajectory;
            for (const auto& pt : jtraj.points) {
                Eigen::VectorXd v(pt.positions.size());
                for (size_t i = 0; i < pt.positions.size(); ++i) {
                    v(i) = pt.positions[i];
                }
                trajectory.push_back(v);
            }
        
                return py::make_tuple(success, trajectory);
            })
            
    

        .def_readwrite("kinlib_solver_", &ScLERPInterface::kinlib_solver_);
}

PYBIND11_MODULE(sclerp_py, m) {
    bind_sclerp_interface(m);

    // Add a helper to construct a C++ rclcpp::Node from Python
    m.def("make_rclcpp_node", [](const std::string &name) {
        static bool initialized = false;
        if (!initialized) {
            int argc = 0;
            char **argv = nullptr;
            rclcpp::init(argc, argv);
            initialized = true;
        }
        return std::make_shared<rclcpp::Node>(name);
    });
}
