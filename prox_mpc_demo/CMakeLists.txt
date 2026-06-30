cmake_minimum_required(VERSION 3.28)
project(prox_mpc_demo)

# C++ version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build type: Release by default to optimize the SQP/QP solver hot path. Guarded
# so an explicit -DCMAKE_BUILD_TYPE (Debug/RelWithDebInfo) is honored.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Remove Eigen's internal assertions in the hot path (safe under Release)
add_compile_definitions(EIGEN_NO_DEBUG)

# Packages
find_package(ament_cmake REQUIRED)
find_package(prox_mpc_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(proxsuite REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(prox_mpc_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Simulation node
add_executable(prox_mpc_simulation src/simulation_node.cpp)
target_include_directories(prox_mpc_simulation PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(prox_mpc_simulation
  Eigen3::Eigen
  proxsuite::proxsuite
)
ament_target_dependencies(prox_mpc_simulation
  prox_mpc_core
  rclcpp
  geometry_msgs
  nav_msgs
  prox_mpc_msgs
  tf2_ros
)

# Install
install(
  TARGETS prox_mpc_simulation
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch config urdf rviz worlds models maps
  DESTINATION share/${PROJECT_NAME}
)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # uncrustify is the enforced C++ formatter; cpplint is intentionally disabled.
  # Copyright is disabled (short SPDX header per file; full text in LICENSE).
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # Drives the simulation node's control cycle directly (no wall-timer spin); owns
  # rclcpp init via its own main.
  ament_add_gtest(test_simulation_node
    test/test_simulation_node.cpp
    SKIP_LINKING_MAIN_LIBRARIES)
  target_include_directories(test_simulation_node PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_simulation_node
    Eigen3::Eigen
    proxsuite::proxsuite
  )
  ament_target_dependencies(test_simulation_node
    prox_mpc_core
    rclcpp
    geometry_msgs
    nav_msgs
    prox_mpc_msgs
    tf2_ros
  )
endif()

ament_package()
