cmake_minimum_required (VERSION 3.5)

set(this ExtendedKF)

project(${this})

set(CMAKE_CXX_STANDARD 11)
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

SET(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 
include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(src)
add_subdirectory(src)

enable_testing()
add_subdirectory(googletest)

set(sources src/main_test.cpp src/tools.cpp src/FusionEKF.cpp src/kalman_filter.cpp src/tools.h src/FusionEKF.h src/kalman_filter.h)

add_library(${this} STATIC ${sources})
target_link_libraries(${this} z ssl uv uWS)

add_subdirectory(test)

#add_executable(ExtendedKF ${sources})
#target_link_libraries(ExtendedKF z ssl uv uWS)


