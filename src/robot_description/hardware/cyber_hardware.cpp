#include "include/robot_description/cyber_hardware.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"

namespace robot_description
{

hardware_interface::CallbackReturn RobotHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  // Initialize hardware parameters
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  // Setup communication with hardware
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  // Activate hardware
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Deactivate hardware
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  // Cleanup resources
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotHardwareInterface::on_shutdown(const rclcpp_lifecycle::State &)
{
  // Shutdown hardware
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobotHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Read hardware state
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Send commands to hardware
  return hardware_interface::return_type::OK;
}

}  // namespace robot_description


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  robot_description::RobotHardwareInterface, hardware_interface::SystemInterface)
