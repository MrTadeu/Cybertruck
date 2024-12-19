#ifndef robot_description__RRBOT_HPP_
#define robot_description__RRBOT_HPP_

#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>


#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "robot_description/arduino_comms.hpp"

#define LEFT 0
#define RIGHT 1

namespace robot_description
{
class RobotHardwareInterface : public hardware_interface::SystemInterface
{

  typedef struct {
    std::string device = "";
    std::string device2 = ""; // for lights
    int baud_rate = 0;
    int timeout_ms = 0;
  } Config;
  typedef struct {
    std::string name_rotation = "";
    std::string name_steering = "";
    double pos = 0;
    double vel = 0;
    double cmd = 0;
  } Wheel;

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RobotHardwareInterface)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  Config config;
  ArduinoComms arduino_comms;
  ArduinoComms arduino_comms2; // for lights
  std::vector<Wheel> wheel_front = {Wheel(), Wheel()};
  std::vector<Wheel> wheel_rear = {Wheel(), Wheel()};
};

} // namespace robot_description

#endif  
