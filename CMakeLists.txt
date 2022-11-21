cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_sdk)

include_directories(include)
include_directories(./)

link_directories(lib/cpp/amd64)

add_compile_options(-std=c++11)

message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
endif()
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
  set(ARCH arm64)
endif()

link_directories(lib/cpp/${ARCH})
set(EXTRA_LIBS -pthread libunitree_legged_sdk.so lcm)

set(CMAKE_CXX_FLAGS "-O3 -fPIC")
 
add_executable(example_walk_aliengo examples/example_walk_aliengo_sport.cpp)
target_link_libraries(example_walk_aliengo ${EXTRA_LIBS})

add_executable(example_start_aliengo examples/example_start_aliengo_sport.cpp)
target_link_libraries(example_start_aliengo ${EXTRA_LIBS})

add_executable(example_joystick_aliengo examples/example_joystick_aliengo_sport.cpp)
target_link_libraries(example_joystick_aliengo ${EXTRA_LIBS})

add_executable(example_recovery_aliengo examples/example_recovery_aliengo_sport.cpp)
target_link_libraries(example_recovery_aliengo ${EXTRA_LIBS})

add_executable(example_position_aliengo examples/example_position_aliengo.cpp)
target_link_libraries(example_position_aliengo ${EXTRA_LIBS})

add_executable(example_velocity_aliengo examples/example_velocity_aliengo.cpp)
target_link_libraries(example_velocity_aliengo ${EXTRA_LIBS})

add_executable(example_torque_aliengo examples/example_torque_aliengo.cpp)
target_link_libraries(example_torque_aliengo ${EXTRA_LIBS})



 
