cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_sdk)

include_directories(include)

link_directories(lib)

add_compile_options(-std=c++11)

set(EXTRA_LIBS -pthread libunitree_legged_sdk_arm64.so lcm)

set(CMAKE_CXX_FLAGS "-O3 -fPIC")

# one pc one process
add_executable(example_position examples/example_position.cpp)
target_link_libraries(example_position ${EXTRA_LIBS})

add_executable(example_velocity examples/example_velocity.cpp)
target_link_libraries(example_velocity ${EXTRA_LIBS})

add_executable(example_torque examples/example_torque.cpp)
target_link_libraries(example_torque ${EXTRA_LIBS})

add_executable(example_walk examples/example_walk.cpp)
target_link_libraries(example_walk ${EXTRA_LIBS})

add_executable(example_wirelessHandle examples/example_wirelessHandle.cpp)
target_link_libraries(example_wirelessHandle ${EXTRA_LIBS})

# multi pc
add_executable(udp_send_test examples/multi_pc_udp_send.cpp)
target_link_libraries(udp_send_test ${EXTRA_LIBS})

add_executable(udp_recv_test examples/multi_pc_udp_recv.cpp)
target_link_libraries(udp_recv_test ${EXTRA_LIBS})

# lcm server
add_executable(lcm_server examples/lcm_server.cpp)
target_link_libraries(lcm_server ${EXTRA_LIBS})
