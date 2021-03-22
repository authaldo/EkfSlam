cmake_minimum_required(VERSION 3.17)
project(EKFSlam)

set(CMAKE_CXX_STANDARD 20)

find_package(PythonLibs 3.8)

add_executable(${PROJECT_NAME} main.cpp EkfSlamManager.cpp)

# no need to include eigen as a module since it is header-only
target_include_directories(${PROJECT_NAME} PRIVATE extern/eigen/)

# python and numpy needed for matplotlibcpp
target_include_directories(${PROJECT_NAME} PRIVATE ${PYTHON_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PRIVATE ~/.local/lib/python3.8/site-packages/numpy/core/include)

# matplotlib header
target_include_directories(${PROJECT_NAME} PRIVATE extern/matplotlib-cpp/ SYSTEM)

target_link_libraries(${PROJECT_NAME} ${PYTHON_LIBRARIES})
