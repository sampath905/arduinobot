cmake_minimum_required(VERSION 3.8)
project(arduinobot_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(moveit_ros_planning_interface REQUIRED)
find_package(rcl_interfaces REQUIRED)



add_executable(simple_moveit_interface scripts/simple_moveit_interface.cpp)
ament_target_dependencies(simple_moveit_interface rclcpp moveit_ros_planning_interface)

add_executable(rough scripts/rough.cpp)
ament_target_dependencies(rough rclcpp moveit_ros_planning_interface)

install(
  DIRECTORY launch config scripts
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS
  simple_moveit_interface
  rough
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
