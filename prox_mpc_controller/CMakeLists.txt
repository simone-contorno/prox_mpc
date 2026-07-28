cmake_minimum_required(VERSION 3.28)
project(prox_mpc_controller)

# C++ version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build
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
find_package(prox_mpc_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
# 0.6.5 is the floor package.xml declares. It matters: 0.6.5 inserted an
# enumerator into QPSolverOutput, so an older copy on CMAKE_PREFIX_PATH
# (for example an unmanaged /usr/local install) compiles against a different
# solver contract than the declared dependency.
find_package(proxsuite 0.6.5 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# Plugin library
add_library(${PROJECT_NAME} SHARED
  src/prox_mpc_controller.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  proxsuite::proxsuite
)
ament_target_dependencies(${PROJECT_NAME}
  prox_mpc_core
  prox_mpc_msgs
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_ros
  visualization_msgs
)

# Register the plugin with nav2_core
pluginlib_export_plugin_description_file(nav2_core prox_mpc_controller_plugin.xml)

# Install
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Export
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  prox_mpc_core
  prox_mpc_msgs
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_ros
  visualization_msgs
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

  # The wrapper unit tests bring up a LifecycleNode, a tf2 buffer, and a
  # Costmap2DROS, then drive the plugin's public interface and fail-safe
  # branches; the test owns rclcpp init/shutdown via its own main.
  ament_add_gtest(test_prox_mpc_controller
    test/test_prox_mpc_controller.cpp
    SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(test_prox_mpc_controller
    ${PROJECT_NAME}
    Eigen3::Eigen
    proxsuite::proxsuite
  )
  ament_target_dependencies(test_prox_mpc_controller
    geometry_msgs
    nav2_core
    nav2_costmap_2d
    nav_msgs
    pluginlib
    prox_mpc_core
    prox_mpc_msgs
    rclcpp
    rclcpp_lifecycle
    tf2
    tf2_ros
    visualization_msgs
  )
endif()

ament_package()
