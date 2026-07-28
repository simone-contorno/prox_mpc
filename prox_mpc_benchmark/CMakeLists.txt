cmake_minimum_required(VERSION 3.28)
project(prox_mpc_benchmark)

# C++ version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build type: Release by default; guarded so an explicit -DCMAKE_BUILD_TYPE wins.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(prox_mpc_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

# C++ live metrics node
add_executable(metrics_node src/metrics_node.cpp)
target_include_directories(metrics_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(metrics_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  prox_mpc_msgs
)
install(TARGETS metrics_node
  DESTINATION lib/${PROJECT_NAME}
)

# Kinematic plant for mode (b2): integrates the controller's body twist and
# publishes odom + TF so Nav2 can run a controller comparison without Gazebo.
add_executable(kinematic_plant src/kinematic_plant.cpp)
ament_target_dependencies(kinematic_plant
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
)
install(TARGETS kinematic_plant
  DESTINATION lib/${PROJECT_NAME}
)

# Obstacle scan simulator for mode (b2): ray-casts the scenario obstacles into a
# LaserScan so the costmap obstacle_layer feeds every controller the same obstacle.
add_executable(scan_simulator src/scan_simulator.cpp)
target_include_directories(scan_simulator PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(scan_simulator
  rclcpp
  nav_msgs
  sensor_msgs
)
install(TARGETS scan_simulator
  DESTINATION lib/${PROJECT_NAME}
)

# nav2_core::Controller decorator that times the wrapped controller, so the
# cross-controller compute comparison measures every controller identically.
add_library(timing_controller_wrapper SHARED src/timing_controller_wrapper.cpp)
target_include_directories(timing_controller_wrapper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(timing_controller_wrapper
  rclcpp
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  nav2_core
  nav2_costmap_2d
  pluginlib
)
pluginlib_export_plugin_description_file(nav2_core timing_controller_plugin.xml)
install(TARGETS timing_controller_wrapper
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Attribution for the Apache-2.0-derived scalable world and recording RViz
# config ships alongside the assets it covers.
install(
  FILES THIRD_PARTY_LICENSES.md
  DESTINATION share/${PROJECT_NAME}
)

# Importable helpers shared by the installed scripts and the launch files, so the
# scenario -> obstacle-array mapping has one definition rather than one per caller.
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

# Installed Python tooling (orchestration / metrics / aggregation / generation).
install(PROGRAMS
  scripts/run_matrix.py
  scripts/run_nav2.py
  scripts/resource_sampler.py
  scripts/generate_map.py
  scripts/goal_sender.py
  scripts/compute_metrics.py
  scripts/aggregate.py
  scripts/gen_blind_multi.py
  scripts/gt_obstacle_publisher.py
  scripts/space_time_feasible.py
  scripts/record_scenarios.py
  scripts/obstacle_markers.py
  scripts/combine_grid.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Launch + config + scalable world / Ackermann model (mode a).
# Keep byte-compiled launch files out of the install space. Both patterns are
# needed: plain CMake matches the "__pycache__" directory name, while
# ament's --symlink-install reimplementation globs files only and needs "*.pyc".
install(DIRECTORY launch config worlds models rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # uncrustify is the enforced C++ formatter; cpplint is intentionally disabled.
  # Copyright is disabled (short SPDX header per file; full text in LICENSE).
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_metrics test/test_metrics.cpp)
  target_include_directories(test_metrics PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_add_gtest(test_obstacle_field test/test_obstacle_field.cpp)
  target_include_directories(test_obstacle_field PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
endif()

ament_package()
