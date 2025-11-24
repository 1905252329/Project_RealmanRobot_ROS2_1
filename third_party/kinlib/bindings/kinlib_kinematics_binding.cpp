#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "kinlib/kinlib.h"
#include "kinlib/kinlib_kinematics.h"

namespace py = pybind11;
using namespace kinlib;

void bind_kinlib_kinematics(py::module_ &m) {
    // Enum for motion type
    py::enum_<ScrewMotionType>(m, "ScrewMotionType")
        .value("NO_MOTION", NO_MOTION)
        .value("GENERAL_SCREW", GENERAL_SCREW)
        .value("PURE_ROTATION", PURE_ROTATION)
        .value("PURE_TRANSLATION", PURE_TRANSLATION)
        .export_values();

    // Enum for error codes
    py::enum_<kinlib::ErrorCodes>(m, "ErrorCodes")
    .value("OPERATION_SUCCESS", kinlib::ErrorCodes::OPERATION_SUCCESS)
    .value("OPERATION_FAILURE", kinlib::ErrorCodes::OPERATION_FAILURE)
    .value("JOINT_LIMIT_ERROR", kinlib::ErrorCodes::JOINT_LIMIT_ERROR)
    .value("PARAMETER_ERROR", kinlib::ErrorCodes::PARAMETER_ERROR)
    .export_values();

    // KinematicsSolver class
    py::class_<KinematicsSolver>(m, "KinematicsSolver")
        .def(py::init<>())
        .def(py::init<kinlib::Manipulator,
              const std::vector<std::string>&,
              const std::vector<Eigen::Matrix4d>&>())

        .def("getManipulator", &KinematicsSolver::getManipulator)
        .def("getFK", [](KinematicsSolver &self, const Eigen::Ref<const Eigen::VectorXd> &jnt_values) {
            Eigen::Matrix4d g_base_tool;
            ErrorCodes code = self.getFK(jnt_values, g_base_tool);
            return py::make_tuple(code, g_base_tool);
        })
        .def("getFK_intermediate", [](KinematicsSolver &self, const Eigen::Ref<const Eigen::VectorXd> &jnt_values) {
            std::vector<Eigen::Matrix4d> intermediate_transforms;
            ErrorCodes code = self.getFK(jnt_values, intermediate_transforms);
            return py::make_tuple(code, intermediate_transforms);
        })
        .def("getSpatialJacobian", [](KinematicsSolver &self, const Eigen::Ref<const Eigen::VectorXd> &jnt_values) {
            Eigen::MatrixXd manip_jac;
            ErrorCodes code = self.getSpatialJacobian(jnt_values, manip_jac);
            return py::make_tuple(code, manip_jac);
        })
        .def("getResolvedMotionRateControlStep", &KinematicsSolver::getResolvedMotionRateControlStep)
        .def("getAdjustedJoints", &KinematicsSolver::getAdjustedJoints)
        ;

    // Utility functions
    m.def("positionDistance", py::overload_cast<const Eigen::Matrix4d&, const Eigen::Matrix4d&>(&positionDistance));
    m.def("positionDistance_vec", py::overload_cast<const Eigen::VectorXd&, const Eigen::VectorXd&>(&positionDistance));

    m.def("rotationDistance", py::overload_cast<const Eigen::Matrix4d&, const Eigen::Matrix4d&>(&rotationDistance));
    m.def("getSkewMatrix", &getSkewMatrix);
    m.def("getTransformationInv", &getTransformationInv);
    m.def("getAdjoint", &getAdjoint);
    m.def("svdPseudoInverse", &svdPseudoInverse);
}
