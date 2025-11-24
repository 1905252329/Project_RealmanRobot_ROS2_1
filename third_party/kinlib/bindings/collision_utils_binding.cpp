#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <memory>

#include "kinlib/collision_utils.h"

namespace py = pybind11;
using namespace CollisionUtils;

void bind_collision_utils(py::module_ &m) {
    // Base class (abstract)
    py::class_<ObstacleBase, std::shared_ptr<ObstacleBase>>(m, "ObstacleBase")
        .def("getCollisionObject", &ObstacleBase::getCollisionObject)
        .def("setTransform", &ObstacleBase::setTransform);

    // BoxObstacle
    py::class_<BoxObstacle, ObstacleBase, std::shared_ptr<BoxObstacle>>(m, "BoxObstacle")
        .def(py::init<const Eigen::Vector3d&, const Eigen::Vector3d&, const Eigen::Matrix3d&>(),
             py::arg("dimensions"),
             py::arg("position"),
             py::arg("orientation") = Eigen::Matrix3d::Identity());

    // SphereObstacle
    py::class_<SphereObstacle, ObstacleBase, std::shared_ptr<SphereObstacle>>(m, "SphereObstacle")
        .def(py::init<double, const Eigen::Vector3d&, const Eigen::Matrix3d&>(),
             py::arg("radius"),
             py::arg("position"),
             py::arg("orientation") = Eigen::Matrix3d::Identity());

    // CylinderObstacle
    py::class_<CylinderObstacle, ObstacleBase, std::shared_ptr<CylinderObstacle>>(m, "CylinderObstacle")
        .def(py::init<double, double, const Eigen::Vector3d&, const Eigen::Matrix3d&>(),
             py::arg("radius"),
             py::arg("height"),
             py::arg("position"),
             py::arg("orientation") = Eigen::Matrix3d::Identity());

    // Optional: MeshObstacle and utility functions can be added in Phase 2
    // Utility creators
    m.def("createBox", &createBox,
        py::arg("dimensions"),
        py::arg("position"),
        py::arg("orientation") = Eigen::Matrix3d::Identity());

    m.def("createSphere", &createSphere,
        py::arg("radius"),
        py::arg("position"),
        py::arg("orientation") = Eigen::Matrix3d::Identity());

    m.def("createCylinder", &createCylinder,
        py::arg("radius"),
        py::arg("height"),
        py::arg("position"),
        py::arg("orientation") = Eigen::Matrix3d::Identity());

    m.def("createGraspObject", &createGraspObject,
        py::arg("type"),
        py::arg("size"),
        py::arg("g_base_tool"));

    m.def("removeObstacle", &removeObstacle,
        py::arg("obstacles"),
        py::arg("index"));

    m.def("createMeshFromSTL", &createMeshFromSTL,
        py::arg("stl_path"),
        py::arg("transform"));
}
