cmake_minimum_required(VERSION 3.8)
project(arm_can)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# arm_can is a header-only library.
# We declare an INTERFACE target so downstream packages (arm_hardware_interface)
# can depend on it via ament_target_dependencies or target_link_libraries.
add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Export as a CMake package so find_package(arm_can) works
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
