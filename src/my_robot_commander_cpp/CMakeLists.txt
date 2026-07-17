cmake_minimum_required(VERSION 3.8)
project(my_robot_commander_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(arm_hardware_interface REQUIRED)

add_executable(joy_to_servo_node
  src/commander_template.cpp
)
target_include_directories(joy_to_servo_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(joy_to_servo_node PRIVATE
  ${arm_hardware_interface_INCLUDE_DIRS}
)

ament_target_dependencies(joy_to_servo_node
  rclcpp
  moveit_servo
  moveit_ros_planning_interface
  control_msgs
  geometry_msgs
  sensor_msgs
  arm_hardware_interface
)

install(TARGETS joy_to_servo_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
