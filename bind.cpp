#include <pybind11/pybind11.h>

#include "kinematics_model.h"

namespace py = pybind11;

PYBIND11_MODULE(KiCarModule, m) {
    py::class_<KiCar>(m, "KiCar")
        // .def(py::init<float, float>(), py::arg("TS"), py::arg("L"))
        .def(py::init<float, float, double, double, double, double, double>(),
             py::arg("TS"), py::arg("L"), py::arg("x"), py::arg("y"), py::arg("yaw"),
             py::arg("delta_f"), py::arg("vx"))
        .def_property_readonly("GetTs", &KiCar::GetTs)
        .def_property_readonly("GetL", &KiCar::GetL)
        .def_property_readonly("GetX", &KiCar::GetX)
        .def_property_readonly("GetY", &KiCar::GetY)
        .def_property_readonly("GetYaw", &KiCar::GetYaw)
        .def_property_readonly("GetDeltaF", &KiCar::GetDeltaF)
        .def_property_readonly("GetVx", &KiCar::GetVx)
        .def("GetPosition", &KiCar::GetPosition)
        .def("UpdateState", (void (KiCar::*)(double)) &KiCar::UpdateState)
        .def("UpdateState", (void (KiCar::*)(double, double)) &KiCar::UpdateState)
        .def("PrintState", &KiCar::PrintState);
}

