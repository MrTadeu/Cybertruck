cmake_minimum_required(VERSION 3.8)
project(robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  xacro
  robot_state_publisher 
  hardware_interface #ros2 control hardware interface
  pluginlib #ros2 control hardware interface
  rclcpp #ros2 control hardware interface
  rclcpp_lifecycle #ros2 control hardware interface
  controller_manager #ros2 control hardware interface
)

# find dependencies
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Locate libserial manually
find_library(LIBSERIAL_LIB NAMES serial PATHS /usr/lib /usr/local/lib REQUIRED)
find_path(LIBSERIAL_INCLUDE_DIR NAMES SerialPort.h PATHS /usr/include/libserial /usr/local/include/libserial REQUIRED)

if(NOT LIBSERIAL_LIB OR NOT LIBSERIAL_INCLUDE_DIR)
  message(FATAL_ERROR "libserial not found. Ensure 'libserial-dev' is installed.")
endif()

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
  ${LIBSERIAL_INCLUDE_DIR}
)
ament_target_dependencies(
  robot_description PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(robot_description PUBLIC ${LIBSERIAL_LIB})

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
  set(ament_cmake_cppcheck_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#hooks share stl files
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

#Exports
ament_export_targets(export_robot_description HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
