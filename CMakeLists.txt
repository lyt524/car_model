cmake_minimum_required(VERSION 3.10)

project(control_simulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(PYTHON_EXECUTABLE "/usr/bin/python3")
set(PYTHON_INCLUDE_DIRECTORY "/usr/include/python3.10")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(OsqpEigen_DIR "/usr/local/lib/cmake/OsqpEigen/OsqpEigenConfig.cmake")  # 替换为你本地的 osqp-eigen 安装路径
find_package(OsqpEigen REQUIRED)

include_directories(/usr/include/eigen3)
# add_executable(CppTest1226 
# test_1226.cpp
# tools/mathtools.cpp
# referencepath/reference_path.cpp
# )

# add_executable(CppTest 
# test.cpp
# models/kinematics_model.cpp
# tools/mathtools.cpp
# referencepath/reference_path.cpp
# control/stanley.cpp
# )

# add_executable(mpcTest 
# test_mpc_1231.cpp
# models/kinematics_model.cpp
# models/bicycle_model_mpc.cpp
# tools/mathtools.cpp
# referencepath/reference_path.cpp
# control/mpc_coupling.cpp
# )

# add_executable(mpcTest 
# test_new_mpc.cpp
# models/kinematics_model.cpp
# tools/mathtools.cpp
# referencepath/reference_path.cpp
# control/mpc_new.cpp
# )

add_executable(test_osqp 
test/test_mpc_couple_0110.cpp
controller/mpc_coupling.cpp
models/kinematics_model.cpp
tools/mathtools.cpp
referencepath/reference_path.cpp
)

# add_executable(test_osqp 
# test0110.cpp
# )

# add_subdirectory(extern/pybind11)
# pybind11_add_module(CppModule
# bind/bind.cpp
# models/kinematics_model.cpp
# referencepath/reference_path.cpp
# control/stanley.cpp
# tools/mathtools.cpp
# )

# target_link_libraries(mpcTest Eigen3::Eigen OsqpEigen::OsqpEigen)

target_link_libraries(test_osqp Eigen3::Eigen OsqpEigen::OsqpEigen)
