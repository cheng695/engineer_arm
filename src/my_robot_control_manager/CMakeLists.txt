cmake_minimum_required(VERSION 3.8)
project(my_robot_control_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(control_mode_manager
  src/control_fsm.cpp
  src/control_mode_manager.cpp
)

target_include_directories(control_mode_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(control_mode_manager
  controller_manager_msgs
  control_msgs
  geometry_msgs
  rclcpp
  rclcpp_action
  trajectory_msgs
  std_msgs
)

target_link_libraries(control_mode_manager tinyxml2::tinyxml2)

install(TARGETS control_mode_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
