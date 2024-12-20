#include <pybind11/pybind11.h>

#include "kinematics_model.h"
#include "tools.h"

namespace py = pybind11;

PYBIND11_MODULE(Tools, m){
    m.doc() = "some tools";
    m.def("NormalizeAngle", &NormalizeAngle, "Normalize radians angle to (-pi, pi]");
    // m.def("add", &add, "A function which adds two numbers");
    // m.def("my_minus", &my_minus, "A function which minus the second number from the first one");
}

PYBIND11_MODULE(KiCarModule, m) {
    py::class_<KiCar>(m, "KiCar")
        // .def(py::init<float, float>(), py::arg("TS"), py::arg("L"))
        .def(py::init<float, float, double, double, double, double, double>(),
             py::arg("TS"), py::arg("L"), py::arg("x"), py::arg("y"), py::arg("phi"),
             py::arg("delta_f"), py::arg("v"))
        .def_property_readonly("GetTs", &KiCar::GetTs)
        .def_property_readonly("GetL", &KiCar::GetL)
        .def_property_readonly("GetX", &KiCar::GetX)
        .def_property_readonly("GetY", &KiCar::GetY)
        .def_property_readonly("GetYaw", &KiCar::GetYaw)
        .def_property_readonly("GetDeltaF", &KiCar::GetDeltaF)
        .def_property_readonly("GetVx", &KiCar::GetVx)
        .def("GetPosition", &KiCar::GetPosition)
        .def("UpdateState_ForwardEuler", (void (KiCar::*)(double)) &KiCar::UpdateState_ForwardEuler)
        .def("UpdateState_ForwardEuler", (void (KiCar::*)(double, double)) &KiCar::UpdateState_ForwardEuler)
        .def("UpdateState_BackwardEuler", (void (KiCar::*)(double)) &KiCar::UpdateState_BackwardEuler)
        .def("UpdateState_BackwardEuler", (void (KiCar::*)(double, double)) &KiCar::UpdateState_BackwardEuler)
        .def("UpdateState_RK4", (void (KiCar::*)(double)) &KiCar::UpdateState_RK4)
        .def("UpdateState_RK4", (void (KiCar::*)(double, double)) &KiCar::UpdateState_RK4)
        .def("PrintState", &KiCar::PrintState);
}

