cmake_minimum_required(VERSION 3.8.0)
project(unitree_legged_sdk
  VERSION 3.8.0
  LANGUAGES CXX
)

find_package(Threads REQUIRED)
find_package(Boost REQUIRED)

# check arch and os
message(STATUS "CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
endif()
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
  set(ARCH arm64)
endif()

list(APPEND unitree_legged_sdk_INCLUDE_DIRS ${Boost_INCLUDE_DIRS})
list(APPEND unitree_legged_sdk_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(unitree_legged_sdk STATIC IMPORTED)
set_target_properties(unitree_legged_sdk
  PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/cpp/${ARCH}/libunitree_legged_sdk.a
    INTERFACE_INCLUDE_DIRECTORIES "${unitree_legged_sdk_INCLUDE_DIRS}"
    VERSION ${PROJECT_VERSION}
)

# one pc one process
add_executable(example_position example/example_position.cpp)
target_link_libraries(example_position
  PUBLIC
    Threads::Threads
    unitree_legged_sdk
)
target_compile_features(example_position
  PUBLIC
    cxx_std_14
)
set_target_properties(example_position
  PROPERTIES
    VERSION ${PROJECT_VERSION}
)

add_executable(example_velocity example/example_velocity.cpp)
target_link_libraries(example_velocity
  PUBLIC
    Threads::Threads
    unitree_legged_sdk
)
target_compile_features(example_velocity
  PUBLIC
    cxx_std_14
)
set_target_properties(example_velocity
  PROPERTIES
    VERSION ${PROJECT_VERSION}
)

add_executable(example_torque example/example_torque.cpp)
target_link_libraries(example_torque
  PUBLIC
    Threads::Threads
    unitree_legged_sdk
)
target_compile_features(example_torque
  PUBLIC
    cxx_std_14
)
set_target_properties(example_torque
  PROPERTIES
    VERSION ${PROJECT_VERSION}
)

add_executable(example_walk example/example_walk.cpp)
target_link_libraries(example_walk
  PUBLIC
    Threads::Threads
    unitree_legged_sdk
)
target_compile_features(example_walk
  PUBLIC
    cxx_std_14
)
set_target_properties(example_walk
  PROPERTIES
    VERSION ${PROJECT_VERSION}
)

add_executable(example_joystick example/example_joystick.cpp)
target_link_libraries(example_joystick
  PUBLIC
    Threads::Threads
    unitree_legged_sdk
)
target_compile_features(example_joystick
  PUBLIC
    cxx_std_14
)
set_target_properties(example_joystick
  PROPERTIES
    VERSION ${PROJECT_VERSION}
)

include(GNUInstallDirs)

# install
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/lib/cpp/${ARCH}/libunitree_legged_sdk.a
  RENAME libunitree_legged_sdk.a.${PROJECT_VERSION}
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/unitree
)
install(
  TARGETS
    example_position
    example_velocity
    example_torque
    example_walk
    example_joystick
  EXPORT ${PROJECT_NAME}Config
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/unitree
)
install(
  EXPORT ${PROJECT_NAME}Config
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/cmake/${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::
)
