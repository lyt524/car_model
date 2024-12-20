# 设置CMake的最小版本和项目名称
cmake_minimum_required(VERSION 3.10)

project(KinematicsModelTest)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)


set(PYTHON_EXECUTABLE "/usr/bin/python3")
set(PYTHON_INCLUDE_DIRECTORY "/usr/include/python3.10")


# # 添加头文件路径（可选，根据需要添加）
# include_directories(${CMAKE_SOURCE_DIR})

# 创建可执行文件（test.cpp）
add_executable(KinematicsModelTest test.cpp models/kinematics_model.cpp tools/mathtools.cpp)

add_subdirectory(extern/pybind11)
pybind11_add_module(KiCarModule bind/bind.cpp models/kinematics_model.cpp tools/mathtools.cpp)
pybind11_add_module(HelloTest bind/bind_hello.cpp tools/hellotest.cpp)
# pybind11_add_module(Tools bind.cpp kinematics_model.cpp tools.cpp)

# 如果你使用的是外部库（例如，链接到其他库），可以在这里添加链接库
# target_link_libraries(KinematicsModelTest <库名>)
