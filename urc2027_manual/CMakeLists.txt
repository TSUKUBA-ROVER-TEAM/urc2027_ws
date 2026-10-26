cmake_minimum_required(VERSION 3.8)
project(urc2027_manual)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(manual_controller SHARED
  src/manual_controller.cpp
)

rclcpp_components_register_node(manual_controller
  PLUGIN "trc2026_manual::ManualController"
  EXECUTABLE manual_controller_node
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_launcher_trigger test/test_launcher_trigger.cpp)
  target_include_directories(test_launcher_trigger PRIVATE include)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
