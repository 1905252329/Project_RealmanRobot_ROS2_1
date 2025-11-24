// A dummy script for pybind11
// To use the python APIs, do export PYTHONPATH=$(path to your workspace)/install/sclerp_motion_planner/lib:$PYTHONPATH

// Example
// from sclerp_py import ScLERPInterface, make_rclcpp_node

// node = make_rclcpp_node("dummy_node")
// sclerp = ScLERPInterface("world", "ee_link", node, "/path/to.urdf")
// success, trajectory = sclerp.solve_basic(np.zeros(6), np.eye(4))

// A konwn issue:
// Tested with Numpy 1.21.5 and 2.2.4, the former works and the latter doesn't.
// Try using Numpy 1.x if you meet segmentation fault