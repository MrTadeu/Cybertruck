cmake_minimum_required(VERSION 3.8)
project(robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)

#ros2 control hardware interface
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_manager REQUIRED)
include_directories(/opt/ros/jazzy/include/hardware_interface)

#find_library(LIBSERIAL_LIB serial REQUIRED)
#target_link_libraries(${PROJECT_NAME}_hardware_interface ${LIBSERIAL_LIB}) # idk if needed
#maybe find_package(backward_ros REQUIRED)

## COMPILE
add_library(
  robot_description
  SHARED
  hardware/cyber_hardware.cpp
)
target_compile_features(robot_description PUBLIC cxx_std_17)
target_include_directories(robot_description PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/robot_description>
)
ament_target_dependencies(
  robot_description PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface robot_description.xml)

install(
  DIRECTORY hardware/include/
  DESTINATION include/robot_description
)

install(DIRECTORY 
  launch 
  meshes
  urdf
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS robot_description
  #TARGETS ${PROJECT_NAME}_hardware_interface # idk if needed
  EXPORT export_robot_description
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#hooks share stl files
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

#Exports
ament_export_targets(export_robot_description HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
