cmake_minimum_required(VERSION 3.17)
project(EKFSlam)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

set(CMAKE_CXX_STANDARD 20)

add_executable(${PROJECT_NAME} main.cpp EkfSlamManager.cpp)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
