#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "kinlib/manipulator.h"

namespace py = pybind11;
using namespace kinlib;

void bind_manipulator(py::module_ &m) {
    // Enum
    py::enum_<JointType>(m, "JointType")
        .value("Revolute", JointType::Revolute)
        .value("Prismatic", JointType::Prismatic)
        .value("None", JointType::None)
        .export_values();

    // JointLimits
    py::class_<JointLimits>(m, "JointLimits")
        .def(py::init<>())
        .def_readwrite("lower_limit_", &JointLimits::lower_limit_)
        .def_readwrite("upper_limit_", &JointLimits::upper_limit_);

    // Manipulator class
    py::class_<Manipulator>(m, "Manipulator")
        .def(py::init<>())

        // Getters
        .def("getJointTypes", &Manipulator::getJointTypes)
        .def("getJointNameAndIDMapping", &Manipulator::getJointNameAndIDMapping)
        .def("getJointAxes", &Manipulator::getJointAxes)
        .def("getJointPositions", &Manipulator::getJointPositions)
        .def("getNumberOfJoints", &Manipulator::getNumberOfJoints)

        // Setters / mutators
        .def("addJoint", &Manipulator::addJoint,
             py::arg("jnt_type"),
             py::arg("jnt_name"),
             py::arg("jnt_axis"),
             py::arg("jnt_q"),
             py::arg("jnt_limits"),
             py::arg("jnt_tip_pose"))
        .def("modifyEndJointTipPose", &Manipulator::modifyEndJointTipPose)
        .def("setReferenceConfiguration", &Manipulator::setReferenceConfiguration);
}
