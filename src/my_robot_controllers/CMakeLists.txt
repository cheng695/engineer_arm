cmake_minimum_required(VERSION 3.8)
project(my_robot_controllers)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/gravity/gravity_compensation_controller.cpp
  src/gravity/gravity_solver.cpp
  src/dls/dls_cartesian_controller.cpp
  src/dls/dls_solver.cpp
  src/joint/joint_command_solver.cpp
  src/joint/joint_command_controller.cpp
  src/hold/hold_controller.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  realtime_tools
  controller_interface
  geometry_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

target_link_libraries(${PROJECT_NAME}
  pinocchio::pinocchio
  Eigen3::Eigen
)

pluginlib_export_plugin_description_file(
  controller_interface
  my_robot_controllers.xml
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES my_robot_controllers.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  controller_interface
  geometry_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

ament_package()
