cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_sdk_python_wrapper)

add_compile_options(-std=c++14)

# check arch and os
message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")

if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
    set(ARCH amd64)
endif()

if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
    set(ARCH arm64)
endif()

# set(CMAKE_CXX_FLAGS "-O3 -no-pie")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -O3 -fPIC")
file(GLOB_RECURSE sources "cpp/*.cpp")


# python
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../lib/python/${ARCH}) # .so .dylib
add_subdirectory(third-party/pybind11)

pybind11_add_module(robot_interface python_interface.cpp)

link_directories(lib/cpp/${ARCH})
set(EXTRA_LIBS -pthread rt m libunitree_legged_sdk.a)

# target_compile_definitions(robot_interface PRIVATE $<$<CXX_COMPILER_ID:Intel>:PYBIND11_CPP14>)
target_link_libraries(robot_interface ${EXTRA_LIBS} unitree_legged_sdk)

