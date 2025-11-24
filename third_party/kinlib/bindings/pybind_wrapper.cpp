#include <pybind11/pybind11.h>

namespace py = pybind11;

void bind_manipulator(py::module_ &);
void bind_kinlib_kinematics(py::module_ &);
void bind_collision_utils(py::module_ &);

PYBIND11_MODULE(kinlib_py, m) {
    bind_manipulator(m);
    bind_kinlib_kinematics(m);
    bind_collision_utils(m);
}