cmake_minimum_required(VERSION 3.8)
project(urc2027_control)

find_package(ament_cmake REQUIRED)
install(PROGRAMS src/recovery_manager.py
  DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_recovery_manager test/test_recovery_manager.py)
endif()
ament_package()
