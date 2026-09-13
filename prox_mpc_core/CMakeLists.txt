cmake_minimum_required(VERSION 3.28)
project(prox_mpc_core)

# C++ version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build type: Release by default to optimize the SQP/QP solver hot path.
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
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
# 0.6.5 is the floor package.xml declares. It matters: 0.6.5 inserted an
# enumerator into QPSolverOutput, so an older copy on CMAKE_PREFIX_PATH
# (for example an unmanaged /usr/local install) compiles against a different
# solver contract than the declared dependency.
find_package(proxsuite 0.6.5 REQUIRED)

# Library
# SHARED so the core can be linked into downstream shared plugin libraries (for
# example prox_mpc_controller) with position-independent code. The bundled Model
# plugins are registered in a separate ${PROJECT_NAME}_models library below.
add_library(${PROJECT_NAME} SHARED
  src/model.cpp
  src/mpc.cpp
  src/proxqp.cpp
  src/utils.cpp
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
  geometry_msgs
  nav_msgs
  pluginlib
  rclcpp
)

# Model plugins exported for pluginlib. These live in a dedicated library that
# downstream code does NOT link, so pluginlib's class loader is the only loader of
# their factories (mirrors nav2_costmap_2d's core vs layers/filters split).
# Linking this library directly would register the factories outside the class
# loader, leaving them unfindable at load time.
add_library(${PROJECT_NAME}_models SHARED
  src/plugins.cpp
)
target_include_directories(${PROJECT_NAME}_models PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_models
  ${PROJECT_NAME}
  Eigen3::Eigen
  proxsuite::proxsuite
)
ament_target_dependencies(${PROJECT_NAME}_models
  geometry_msgs
  nav_msgs
  pluginlib
  rclcpp
)

# Export prox_mpc::Model as a pluginlib base so models are selected outside the
# core (no core edit per new model). The bundled plugins are BicycleFrontAxle,
# BicycleRearAxle, the deprecated Bicycle alias, and Unicycle.
pluginlib_export_plugin_description_file(prox_mpc_core prox_mpc_core_plugins.xml)

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
# The model-plugin library is loaded only by pluginlib, so it is installed but not
# exported as a link target (nothing downstream should link it).
install(
  TARGETS ${PROJECT_NAME}_models
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export (so prox_mpc_controller and prox_mpc_demo can link the library)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp Eigen3 proxsuite geometry_msgs nav_msgs pluginlib)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # uncrustify is the enforced C++ formatter; cpplint is intentionally disabled
  # to avoid duplicating/contradicting its checks. Copyright is disabled because
  # files carry a short SPDX header (full text lives in LICENSE).
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # (a) model interface, (b) regression baseline, (c) custom model, (d) obstacle K,
  # (e) ROS-facing utils helpers, (f) transactional solve/commit contract,
  # (g) post-init configuration lifecycle and goal/obstacle shape validation,
  # (h) the deprecated alias's once-per-process warning, which needs a process
  #     where nothing else has constructed the alias first
  foreach(test_name
      test_model_interface
      test_mpc_regression
      test_custom_model
      test_obstacle_k
      test_utils
      test_transactional_solve
      test_config_lifecycle
      test_deprecated_alias)
    # None of these targets fit inside CTest's 60 s default once ASan/UBSan
    # instrumentation is linked in (test_obstacle_k alone needs ~57 s), so every
    # target in this loop gets the same generous budget rather than singling one
    # out; a slower CI runner is covered too.
    ament_add_gtest(${test_name} test/${test_name}.cpp TIMEOUT 300)
    target_link_libraries(${test_name}
      ${PROJECT_NAME}
      Eigen3::Eigen
      proxsuite::proxsuite
    )
    ament_target_dependencies(${test_name}
      geometry_msgs
      nav_msgs
      pluginlib
      rclcpp
    )
  endforeach()
endif()

ament_package()
