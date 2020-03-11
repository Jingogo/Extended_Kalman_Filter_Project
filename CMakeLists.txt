cmake_minimum_required (VERSION 3.5)

set(this ExtendedKF)

project(ExtendedKF)

add_definitions(-std=c++11)
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(sources src/main.cpp src/tools.cpp src/EKF.cpp  src/tools.h src/EKF.h)

add_executable(${this} ${sources})
target_link_libraries(${this})