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
        .def_property_readonly("GetTs", &KiCar::GetTs)
        .def_property_readonly("GetL", &KiCar::GetL)
        .def_property_readonly("GetX", &KiCar::GetX)
        .def_property_readonly("GetY", &KiCar::GetY)
        .def_property_readonly("GetYaw", &KiCar::GetYaw)
        .def_property_readonly("GetDeltaF", &KiCar::GetDeltaF)
        .def_property_readonly("GetV", &KiCar::GetV)
        .def("GetPosition", &KiCar::GetPosition)
        .def("UpdateState_ForwardEuler", (void (KiCar::*)(double)) &KiCar::UpdateState_ForwardEuler)
        .def("UpdateState_ForwardEuler", (void (KiCar::*)(double, double)) &KiCar::UpdateState_ForwardEuler)
        .def("UpdateState_BackwardEuler", (void (KiCar::*)(double)) &KiCar::UpdateState_BackwardEuler)
        .def("UpdateState_BackwardEuler", (void (KiCar::*)(double, double)) &KiCar::UpdateState_BackwardEuler)
        .def("UpdateState_RK4", (void (KiCar::*)(double)) &KiCar::UpdateState_RK4)
        .def("UpdateState_RK4", (void (KiCar::*)(double, double)) &KiCar::UpdateState_RK4)
        .def("PrintState", &KiCar::PrintState);
    
    py::class_<RefPath>(m, "RefPath")
        .def(py::init<int, int>(), py::arg("point_num"), py::arg("row_num"))
        .def("GetPoint", &RefPath::GetPoint, py::arg("index"), py::arg("x"), py::arg("y"), py::arg("phi"))
        .def("GetPointX", &RefPath::GetPointX, py::arg("index"))
        .def("GetPointY", &RefPath::GetPointY, py::arg("index"))
        .def("GetPointPhi", &RefPath::GetPointPhi, py::arg("index"))
        .def("ShowPath", &RefPath::ShowPath)
        .def_readonly("ref_path", &RefPath::ref_path)
        .def_readonly("point_num", &RefPath::point_num)
        .def_readonly("lastNearestPointIndex", &RefPath::lastNearestPointIndex);

    py::class_<SineInfo>(m, "SineInfo")
        .def(py::init<double, double>(), py::arg("amp"), py::arg("freq"))
        .def_readonly("amplitude", &SineInfo::amplitude)
        .def_readonly("frequency", &SineInfo::frequency);

    m.def("GenerateSinewavePath", &GenerateSinewavePath, py::arg("path_length"), py::arg("_ref_path"), py::arg("_sine_info"));

    py::class_<Stanley>(m, "Stanley")
        .def(py::init<>())
        .def("FindNearestIndex", &Stanley::FindNearestIndex, py::arg("ki_car"), py::arg("ref_path"))
        .def("CalHeadingError", &Stanley::CalHeadingError, py::arg("ki_car"), py::arg("ref_path"))
        .def("CalLateralError", &Stanley::CalLateralError, py::arg("ki_car"), py::arg("ref_path"))
        .def("StanleyControl", &Stanley::StanleyControl, py::arg("ki_car"), py::arg("ref_path"))
        .def_property_readonly("GetLateralError", &Stanley::GetLateralError)
        .def_property_readonly("GetHeadingError", &Stanley::GetHeadingError)
        .def_readwrite("stanleyK", &Stanley::stanleyK)
        .def_readonly("deltaF", &Stanley::deltaF)
        .def_readonly("headingError", &Stanley::headingError)
        .def_readonly("lateralError", &Stanley::lateralError);
}

