cmake_minimum_required(VERSION 3.28)
project(prox_mpc_obstacle_tracker)

# C++ version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

# Compiler warnings, matching upstream Nav2's package-level policy
# (nav2_common/cmake/nav2_package.cmake) so this package is held to the same
# bar as every other Nav2 plugin it ships alongside.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC -Wshadow -Wnull-dereference)
  # GCC 13 raises -Wnull-dereference inside libstdc++'s own <regex> headers
  # (stl_algobase.h, reached through regex_traits::transform_primary) once
  # --coverage changes its inlining decisions, which is how the coverage build
  # hits it and an ordinary build does not. The report is attributed to the
  # translation unit that instantiates the template, so -Wno-system-headers
  # does not reach it. Keep the diagnostic, but do not let a false positive in
  # the standard library fail the build.
  add_compile_options(-Wno-error=null-dereference)
  add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-Wnon-virtual-dtor>")
endif()

# Remove Eigen's internal assertions in the hot path (safe under Release)
add_compile_definitions(EIGEN_NO_DEBUG)

# Packages
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(prox_mpc_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcpputils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Pure (Eigen-only) detection + tracking library, unit-testable without ROS.
add_library(${PROJECT_NAME}_core SHARED
  src/clustering.cpp
  src/imm_filter.cpp
  src/tracker.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core Eigen3::Eigen)

# Composable component library: the lifecycle node, registered for
# rclcpp_components so it can be loaded into a shared-process container
# (MultiThreadedExecutor) as well as run standalone.
add_library(${PROJECT_NAME}_component SHARED
  src/obstacle_tracker_node.cpp
)
target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_component ${PROJECT_NAME}_core Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME}_component
  geometry_msgs
  lifecycle_msgs
  prox_mpc_msgs
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  rcpputils
  sensor_msgs
  tf2
  tf2_ros
)
rclcpp_components_register_nodes(${PROJECT_NAME}_component
  "prox_mpc_obstacle_tracker::ObstacleTrackerNode")

# Standalone executable: the signal-safe driver linking the component.
add_executable(obstacle_tracker src/main.cpp)
target_link_libraries(obstacle_tracker ${PROJECT_NAME}_component ${PROJECT_NAME}_core)
ament_target_dependencies(obstacle_tracker
  lifecycle_msgs
  rclcpp
  rclcpp_lifecycle
)

# Install
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS obstacle_tracker
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Export
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # uncrustify is the enforced C++ formatter; cpplint is intentionally disabled.
  # Copyright is disabled (short SPDX header per file; full text in LICENSE).
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_clustering test/test_clustering.cpp)
  target_link_libraries(test_clustering ${PROJECT_NAME}_core Eigen3::Eigen)

  ament_add_gtest(test_imm_filter test/test_imm_filter.cpp)
  target_link_libraries(test_imm_filter ${PROJECT_NAME}_core Eigen3::Eigen)

  ament_add_gtest(test_tracker test/test_tracker.cpp)
  target_link_libraries(test_tracker ${PROJECT_NAME}_core Eigen3::Eigen)

  # Lifecycle-node integration test: drives the transitions and the scan path
  # (synthetic scan in the tracking frame, no TF). Owns rclcpp init via its main.
  ament_add_gtest(test_obstacle_tracker_node
    test/test_obstacle_tracker_node.cpp
    SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(test_obstacle_tracker_node
    ${PROJECT_NAME}_component ${PROJECT_NAME}_core Eigen3::Eigen)
  ament_target_dependencies(test_obstacle_tracker_node
    geometry_msgs
    lifecycle_msgs
    prox_mpc_msgs
    rclcpp
    rclcpp_components
    rclcpp_lifecycle
    sensor_msgs
    tf2
    tf2_ros
  )
endif()

ament_package()
