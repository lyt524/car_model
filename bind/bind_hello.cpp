#include <pybind11/pybind11.h>

#include "../tools/hellotest.h"

namespace py = pybind11;


PYBIND11_MODULE(HelloTest, m){
    m.doc() = "print Hello";
    m.def("PrintHello", &PrintHello, "print Hello");
}

