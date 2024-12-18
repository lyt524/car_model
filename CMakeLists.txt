# 设置CMake的最小版本和项目名称
cmake_minimum_required(VERSION 3.10)

project(KinematicsModelTest)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# # 添加头文件路径（可选，根据需要添加）
# include_directories(${CMAKE_SOURCE_DIR})

# 创建可执行文件（test.cpp）
add_executable(KinematicsModelTest test.cpp kinematics_model.cpp)

# 如果你使用的是外部库（例如，链接到其他库），可以在这里添加链接库
# target_link_libraries(KinematicsModelTest <库名>)
