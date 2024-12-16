cmake_minimum_required(VERSION 3.8)
project(robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(serial REQUIRED)  # For serial communication

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Add include directory
include_directories(include)

# Add hardware interface source files
add_library(${PROJECT_NAME}_hardware_interface SHARED
  src/MyRobotHardwareInterface.cpp
)

# Link dependencies to hardware interface
ament_target_dependencies(${PROJECT_NAME}_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  serial
)

# Export plugin
pluginlib_export_plugin_description_file(hardware_interface MyRobotHardwareInterface.xml)

# Install hardware interface library
install(
  TARGETS ${PROJECT_NAME}_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install source files
install(DIRECTORY 
  launch 
  meshes
  urdf
  config
  DESTINATION share/${PROJECT_NAME}
)

# Environment hooks
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
