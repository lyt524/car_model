#include <pybind11/pybind11.h>

#include "../models/kinematics_model.h"
#include "../referencepath/reference_path.h"
#include "../control/stanley.h"

namespace py = pybind11;


PYBIND11_MODULE(CppModule, m) {
    py::class_<KiCar>(m, "KiCar")
        .def(py::init<float, float, double, double, double, double, double>(),
             py::arg("TS"), py::arg("L"), py::arg("x"), py::arg("y"), py::arg("phi"),
             py::arg("delta_f"), py::arg("v"))
        .def_property_readonly("getTs", &KiCar::getTs)
        .def_property_readonly("getL", &KiCar::getL)
        .def_property_readonly("getX", &KiCar::getX)
        .def_property_readonly("getY", &KiCar::getY)
        .def_property_readonly("getYaw", &KiCar::getYaw)
        .def_property_readonly("getDeltaF", &KiCar::getDeltaF)
        .def_property_readonly("getV", &KiCar::getV)
        .def("GetPosition", &KiCar::GetPosition)
        .def("updateState_ForwardEuler", (void (KiCar::*)(double)) &KiCar::updateState_ForwardEuler)
        .def("updateState_ForwardEuler", (void (KiCar::*)(double, double)) &KiCar::updateState_ForwardEuler)
        .def("updateState_BackwardEuler", (void (KiCar::*)(double)) &KiCar::updateState_BackwardEuler)
        .def("updateState_BackwardEuler", (void (KiCar::*)(double, double)) &KiCar::updateState_BackwardEuler)
        .def("updateState_RK4", (void (KiCar::*)(double)) &KiCar::updateState_RK4)
        .def("updateState_RK4", (void (KiCar::*)(double, double)) &KiCar::updateState_RK4)
        .def("printState", &KiCar::printState);
    
    py::class_<RefPath>(m, "RefPath")
        .def(py::init<int, int>(), py::arg("point_num"), py::arg("row_num"))
        .def("getPoint", &RefPath::getPoint, py::arg("index"), py::arg("x"), py::arg("y"), py::arg("phi"))
        .def("getPointX", &RefPath::getPointX, py::arg("index"))
        .def("getPointY", &RefPath::getPointY, py::arg("index"))
        .def("getPointPhi", &RefPath::getPointPhi, py::arg("index"))
        .def("showPath", &RefPath::showPath)
        .def_readonly("ref_path", &RefPath::ref_path)
        .def_readonly("point_num", &RefPath::point_num)
        .def_readonly("lastNearestPointIndex", &RefPath::lastNearestPointIndex);

    py::class_<SineInfo>(m, "SineInfo")
        .def(py::init<double, double>(), py::arg("amp"), py::arg("freq"))
        .def_readonly("amplitude", &SineInfo::amplitude)
        .def_readonly("frequency", &SineInfo::frequency);

    m.def("generateSinewavePath", &generateSinewavePath, py::arg("path_length"), py::arg("_ref_path"), py::arg("_sine_info"));

    py::class_<Stanley>(m, "Stanley")
        .def(py::init<>())
        .def("findNearestIndex", &Stanley::findNearestIndex, py::arg("ki_car"), py::arg("ref_path"))
        .def("calHeadingError", &Stanley::calHeadingError, py::arg("ki_car"), py::arg("ref_path"))
        .def("calLateralError", &Stanley::calLateralError, py::arg("ki_car"), py::arg("ref_path"))
        .def("stanleyControl", &Stanley::stanleyControl, py::arg("ki_car"), py::arg("ref_path"))
        .def_property_readonly("getLateralError", &Stanley::getLateralError)
        .def_property_readonly("getHeadingError", &Stanley::getHeadingError)
        .def_readwrite("stanley_K_", &Stanley::stanley_K_)
        .def_readonly("delta_f_", &Stanley::delta_f_)
        .def_readonly("heading_error_", &Stanley::heading_error_)
        .def_readonly("lateral_error_", &Stanley::lateral_error_);
}

