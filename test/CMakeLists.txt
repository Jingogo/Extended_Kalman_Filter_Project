cmake_minimum_required (VERSION 3.5)

set(this ExtendedKFTests)


set(sources ExtendedKFTests.cpp)

add_executable(${this} ${sources})

target_link_libraries(${this} PUBLIC gtest_main ExtendedKF)

add_test(
    NAME ${this}
    COMMAND ${this}
)