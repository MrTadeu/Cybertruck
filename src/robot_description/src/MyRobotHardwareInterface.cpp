#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <serial/serial.h> // Include serial communication library

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace my_robot_hardware
{
class MyRobotHardwareInterface : public hardware_interface::SystemInterface
{
public:
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }

        for (const auto &joint : info.joints)
        {
            joint_positions_.push_back(0.0);
            joint_velocities_.push_back(0.0);
            joint_commands_.push_back(0.0);
        }

        serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        baud_rate_ = declare_parameter<int>("baud_rate", 115200);

        try
        {
            serial_connection_.setPort(serial_port_);
            serial_connection_.setBaudrate(baud_rate_);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_connection_.setTimeout(timeout);
            serial_connection_.open();
        }
        catch (const serial::IOException &e)
        {
            RCLCPP_FATAL(rclcpp::get_logger("MyRobotHardwareInterface"), "Unable to open serial port");
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, "position", &joint_positions_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, "velocity", &joint_velocities_[i]));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, "velocity", &joint_commands_[i]));
        }
        return command_interfaces;
    }

    hardware_interface::return_type read() override
    {
        if (serial_connection_.available())
        {
            std::string feedback = serial_connection_.readline();
            RCLCPP_INFO(rclcpp::get_logger("MyRobotHardwareInterface"), "Feedback: %s", feedback.c_str());
            // Parse feedback and update joint_positions_ and joint_velocities_
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type write() override
    {
        std::string command;
        for (size_t i = 0; i < joint_commands_.size(); i++)
        {
            command += std::to_string(joint_commands_[i]) + ",";
        }
        if (!command.empty())
        {
            command.pop_back(); // Remove trailing comma
        }
        serial_connection_.write(command + "\n");
        return hardware_interface::return_type::OK;
    }

private:
    std::vector<double> joint_positions_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_commands_;
    std::string serial_port_;
    int baud_rate_;
    serial::Serial serial_connection_;
};

} // namespace my_robot_hardware

PLUGINLIB_EXPORT_CLASS(my_robot_hardware::MyRobotHardwareInterface, hardware_interface::SystemInterface)
