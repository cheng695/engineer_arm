cmake_minimum_required(VERSION 3.8)
project(arm_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(moveit_core REQUIRED)

# arm_can is optional — only RealArmHardwareInterface needs it
find_package(arm_can QUIET)

# Shared sources (always built)
set(LIB_SOURCES
  src/arm_hardware_base.cpp
  src/mock_arm_hardware_interface.cpp
  src/dls_controller.cpp
  src/control_fsm.cpp
  src/remote.cpp
  src/joint_controller.cpp
  src/gravity_compensator.cpp
  src/pinocchio_kinematics_plugin.cpp
)

# Conditionally add real plugin
if(arm_can_FOUND)
  list(APPEND LIB_SOURCES src/real_arm_hardware_interface.cpp)
  message(STATUS "arm_can FOUND — building RealArmHardwareInterface")
else()
  message(STATUS "arm_can NOT FOUND — building only MockArmHardwareInterface")
endif()

# Define hardware interface library
add_library(${PROJECT_NAME} SHARED ${LIB_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  example_interfaces
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  moveit_core
)

if(arm_can_FOUND)
  ament_target_dependencies(${PROJECT_NAME} arm_can)
endif()

# Use target_link_libraries for non-ament packages
target_link_libraries(${PROJECT_NAME}
  pinocchio::pinocchio
  Eigen3::Eigen
)

# Export plugins
pluginlib_export_plugin_description_file(hardware_interface mock_arm_hardware_interface.xml)
if(arm_can_FOUND)
  pluginlib_export_plugin_description_file(hardware_interface real_arm_hardware_interface.xml)
endif()
pluginlib_export_plugin_description_file(moveit_core pinocchio_kinematics_plugin_description.xml)

# Install
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
