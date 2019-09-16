cmake_minimum_required (VERSION 3.5)

set(this SensorFusion)
set(sources main_test.cpp tools.cpp FusionEKF.cpp kalman_filter.cpp tools.h FusionEKF.h kalman_filter.h)

add_executable(${this} ${sources})

target_link_libraries(${this} z ssl uv uWS)

