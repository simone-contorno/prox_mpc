cmake_minimum_required(VERSION 3.28)
project(prox_mpc_test_models)

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

# Match prox_mpc_core's Eigen assertion setting so this fixture builds against
# identical Eigen semantics.
add_compile_definitions(EIGEN_NO_DEBUG)

# Packages
find_package(ament_cmake REQUIRED)
find_package(prox_mpc_core REQUIRED)
find_package(Eigen3 REQUIRED)
# 0.6.5 is the floor package.xml declares. It matters: 0.6.5 inserted an
# enumerator into QPSolverOutput, so an older copy on CMAKE_PREFIX_PATH
# (for example an unmanaged /usr/local install) compiles against a different
# solver contract than the declared dependency.
find_package(proxsuite 0.6.5 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

# Model-plugin library, loaded only by pluginlib's class loader (mirrors the core
# models split). Nothing downstream links it directly.
add_library(${PROJECT_NAME} SHARED
  src/plugins.cpp
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
  geometry_msgs
  pluginlib
  rclcpp
)

# Register the test models against the prox_mpc::Model base so the controller's
# pluginlib::ClassLoader<prox_mpc::Model>("prox_mpc_core", ...) discovers them.
pluginlib_export_plugin_description_file(prox_mpc_core prox_mpc_test_models_plugins.xml)

# Install
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export
ament_export_include_directories(include)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # uncrustify is the enforced C++ formatter; cpplint is intentionally disabled.
  # Copyright is disabled (short SPDX header per file; full text in LICENSE).
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
