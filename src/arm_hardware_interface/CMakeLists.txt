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
find_package(sensor_msgs REQUIRED)

# MuJoCo is optional so existing real/mock builds remain usable without it.
set(MUJOCO_ROOT "$ENV{MUJOCO_ROOT}" CACHE PATH "MuJoCo installation prefix")
find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h
  HINTS "${MUJOCO_ROOT}/include")
find_library(MUJOCO_LIBRARY NAMES mujoco
  HINTS "${MUJOCO_ROOT}/lib" "${MUJOCO_ROOT}/bin")

# arm_can is optional — only RealArmHardwareInterface needs it
find_package(arm_can QUIET)

# Shared sources (always built)
set(LIB_SOURCES
  src/arm_hardware_base.cpp
  src/mock_arm_hardware_interface.cpp
  src/remote.cpp
)

# Conditionally add real plugin
if(arm_can_FOUND)
  list(APPEND LIB_SOURCES src/real_arm_hardware_interface.cpp)
  message(STATUS "arm_can FOUND — building RealArmHardwareInterface")
else()
  message(STATUS "arm_can NOT FOUND — building only MockArmHardwareInterface")
endif()

if(MUJOCO_INCLUDE_DIR AND MUJOCO_LIBRARY)
  list(APPEND LIB_SOURCES src/mujoco_arm_hardware_interface.cpp)
  message(STATUS "MuJoCo FOUND - building MujocoArmHardwareInterface")
else()
  message(STATUS "MuJoCo NOT FOUND - MujocoArmHardwareInterface is not built")
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
  sensor_msgs
)

if(arm_can_FOUND)
  ament_target_dependencies(${PROJECT_NAME} arm_can)
endif()

if(MUJOCO_INCLUDE_DIR AND MUJOCO_LIBRARY)
  target_include_directories(${PROJECT_NAME} PRIVATE ${MUJOCO_INCLUDE_DIR})
  target_link_libraries(${PROJECT_NAME} ${MUJOCO_LIBRARY})
endif()

# Use target_link_libraries for non-ament packages
# Export plugins
pluginlib_export_plugin_description_file(hardware_interface mock_arm_hardware_interface.xml)
if(arm_can_FOUND)
  pluginlib_export_plugin_description_file(hardware_interface real_arm_hardware_interface.xml)
endif()
if(MUJOCO_INCLUDE_DIR AND MUJOCO_LIBRARY)
  pluginlib_export_plugin_description_file(hardware_interface mujoco_arm_hardware_interface.xml)
endif()

# Install
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
if(BUILD_TESTING)
  enable_testing()
  add_executable(hardware_contract_test test/hardware_contract_test.cpp)
  target_link_libraries(hardware_contract_test ${PROJECT_NAME})
  ament_target_dependencies(hardware_contract_test hardware_interface rclcpp sensor_msgs std_msgs)
  add_test(NAME hardware_contract_test COMMAND hardware_contract_test)
endif()

ament_package()
