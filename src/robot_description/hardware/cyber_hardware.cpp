#include "include/robot_description/cyber_hardware.hpp"
static std::string prev_msg = "";

namespace robot_description
{

hardware_interface::CallbackReturn RobotHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) return hardware_interface::CallbackReturn::ERROR;

  // Get parameters from hardware info
  wheel_front[RIGHT].name_rotation = info.hardware_parameters.at("front_wheel_right_vel");
  wheel_front[RIGHT].name_steering = info.hardware_parameters.at("front_wheel_right_pos");
  wheel_front[LEFT].name_rotation = info.hardware_parameters.at("front_wheel_left_vel");
  wheel_front[LEFT].name_steering = info.hardware_parameters.at("front_wheel_left_pos");

  wheel_rear[RIGHT].name_rotation = info.hardware_parameters.at("rear_wheel_right_vel");
  wheel_rear[RIGHT].name_steering = info.hardware_parameters.at("rear_wheel_right_pos");
  wheel_rear[LEFT].name_rotation = info.hardware_parameters.at("rear_wheel_left_vel");
  wheel_rear[LEFT].name_steering = info.hardware_parameters.at("rear_wheel_left_pos");


  config.device = info.hardware_parameters.at("device");
  config.device2 = info.hardware_parameters.at("device2"); // for lights
  config.baud_rate = std::stoi(info.hardware_parameters.at("baud_rate"));
  config.timeout_ms = std::stoi(info.hardware_parameters.at("timeout_ms"));

  RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "\033[33mInitialized hardware interface\033[m");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  // Setup communication with hardware
  try {
    RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "\033[33mConfiguring serial connections...\033[m");

    RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "\t- Connecting to arduino at %s", config.device.c_str());
    arduino_comms.connect(config.device, config.baud_rate, config.timeout_ms);

    RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "\t- Connecting to arduino Lights at %s", config.device2.c_str());
    arduino_comms2.connect(config.device2, config.baud_rate, config.timeout_ms); // for lights

    RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "\033[32mSerial connections established successfully\033[m");
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  catch(const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("RobotHardwareInterface"), "Error connecting to hardware: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn RobotHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  // Activate hardware
  RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "Hardware activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Deactivate hardware
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Rodas de rotação: velocity
  state_interfaces.emplace_back(wheel_front[LEFT].name_rotation, hardware_interface::HW_IF_VELOCITY, &wheel_front[LEFT].vel);
  state_interfaces.emplace_back(wheel_front[RIGHT].name_rotation, hardware_interface::HW_IF_VELOCITY, &wheel_front[RIGHT].vel);
  state_interfaces.emplace_back(wheel_rear[LEFT].name_rotation, hardware_interface::HW_IF_VELOCITY, &wheel_rear[LEFT].vel);
  state_interfaces.emplace_back(wheel_rear[RIGHT].name_rotation, hardware_interface::HW_IF_VELOCITY, &wheel_rear[RIGHT].vel);

  // Rodas de direção: position
  state_interfaces.emplace_back(wheel_front[LEFT].name_steering, hardware_interface::HW_IF_POSITION, &wheel_front[LEFT].pos);
  state_interfaces.emplace_back(wheel_front[RIGHT].name_steering, hardware_interface::HW_IF_POSITION, &wheel_front[RIGHT].pos);
  state_interfaces.emplace_back(wheel_rear[LEFT].name_steering, hardware_interface::HW_IF_POSITION, &wheel_rear[LEFT].pos);
  state_interfaces.emplace_back(wheel_rear[RIGHT].name_steering, hardware_interface::HW_IF_POSITION, &wheel_rear[RIGHT].pos);

  RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "\033[32mExported state interfaces successfully\033[m");

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Comandos de velocidade (rodas de rotação)
  command_interfaces.emplace_back(wheel_front[LEFT].name_rotation, hardware_interface::HW_IF_VELOCITY, &wheel_front[LEFT].vel);
  command_interfaces.emplace_back(wheel_front[RIGHT].name_rotation, hardware_interface::HW_IF_VELOCITY, &wheel_front[RIGHT].vel);
  command_interfaces.emplace_back(wheel_rear[LEFT].name_rotation, hardware_interface::HW_IF_VELOCITY, &wheel_rear[LEFT].vel);
  command_interfaces.emplace_back(wheel_rear[RIGHT].name_rotation, hardware_interface::HW_IF_VELOCITY, &wheel_rear[RIGHT].vel);

  // Comandos de posição (rodas de direção)
  command_interfaces.emplace_back(wheel_front[LEFT].name_steering, hardware_interface::HW_IF_POSITION, &wheel_front[LEFT].pos);
  command_interfaces.emplace_back(wheel_front[RIGHT].name_steering, hardware_interface::HW_IF_POSITION, &wheel_front[RIGHT].pos);
  command_interfaces.emplace_back(wheel_rear[LEFT].name_steering, hardware_interface::HW_IF_POSITION, &wheel_rear[LEFT].pos);
  command_interfaces.emplace_back(wheel_rear[RIGHT].name_steering, hardware_interface::HW_IF_POSITION, &wheel_rear[RIGHT].pos);

  RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "\033[32mExported command interfaces successfully\033[m");

  return command_interfaces;
}

hardware_interface::return_type RobotHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  try
  {
    std::string response;
    if (arduino_comms.is_connected())
    {
      response = arduino_comms.receive_msg();

      if (!response.empty() && response.find("vel_front") != std::string::npos /* && response != prev_msg */) // Ignorar mensagens vazias
      {
        prev_msg = response;
        RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "Received: \033[34m %s\033[m", response.c_str());

        // Processar a mensagem aqui, se necessário
      }
    }
    return hardware_interface::return_type::OK;
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("RobotHardwareInterface"), "Error reading from Arduino: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
}

hardware_interface::return_type RobotHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  try
  {
    // Calcular a média dos valores de velocidade e posição para as rodas frontais
    double front_vel_avg = (wheel_front[LEFT].vel + wheel_front[RIGHT].vel) / 2.0;
    double front_pos_avg = (wheel_front[LEFT].pos + wheel_front[RIGHT].pos) / 2.0;

    // Calcular a média dos valores de velocidade e posição para as rodas traseiras
    double rear_vel_avg = (wheel_rear[LEFT].vel + wheel_rear[RIGHT].vel) / 2.0;
    double rear_pos_avg = (wheel_rear[LEFT].pos + wheel_rear[RIGHT].pos) / 2.0;

    front_vel_avg = std::clamp(front_vel_avg, -10.0, 10.0);
    rear_vel_avg = std::clamp(rear_vel_avg, -10.0, 10.0);
    
    front_pos_avg = std::clamp(front_pos_avg, -2.0, 2.0);
    rear_pos_avg = std::clamp(rear_pos_avg, -2.0, 2.0);

    if (std::isnan(front_vel_avg) || std::isnan(front_pos_avg) || std::isnan(rear_vel_avg) || std::isnan(rear_pos_avg)){
      RCLCPP_ERROR(rclcpp::get_logger("RobotHardwareInterface"), "Invalid data: NaN detected.");
      return hardware_interface::return_type::ERROR;
    }
    // Enviar os comandos para arduino
    if (arduino_comms.is_connected()){
      std::stringstream cmd;
      cmd << std::fixed << std::setprecision(3);
      cmd << "vel_front " << front_vel_avg
          << " pos_front " << front_pos_avg
          << " vel_rear " << rear_vel_avg
          << " pos_rear " << rear_pos_avg
          << "\n";
      arduino_comms.send_msg(cmd.str());

      RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "Preparing to send: %s", cmd.str().c_str());
    }
    else
    {
      arduino_comms.reconnect();
    }

    // Enviar os comandos para outro arduino // for lights
    if (arduino_comms2.is_connected())
    {
      std::stringstream cmd;
      cmd << std::fixed << std::setprecision(3);
      cmd << "vel_front " << front_vel_avg
          << " pos_front " << front_pos_avg
          << " vel_rear " << rear_vel_avg
          << " pos_rear " << rear_pos_avg
          << "\n";
      arduino_comms2.send_msg(cmd.str());

    }
    else
    {
      arduino_comms2.reconnect();
    }

    // Enviar os comandos para outro arduino
    /* if (arduino_comms2.connected())
    {
      std::stringstream cmd_rear;
      cmd_rear << "vel " << static_cast<int>(rear_vel_avg)
               << " pos " << static_cast<int>(rear_pos_avg) << "\r";
      arduino_comms2.send_msg(cmd_rear.str());

      RCLCPP_DEBUG(rclcpp::get_logger("RobotHardwareInterface"),
                   "Sent to rear: vel = %.2f, pos = %.2f", rear_vel_avg, rear_pos_avg);
    } */

    return hardware_interface::return_type::OK;
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("RobotHardwareInterface"), "Error writing to Arduinos: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
}

hardware_interface::CallbackReturn RobotHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  // Cleanup resources
  arduino_comms.disconnect();
  arduino_comms2.disconnect(); // for lights
  RCLCPP_INFO(rclcpp::get_logger("RobotHardwareInterface"), "\033[32mSerial connections closed.\033[m");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotHardwareInterface::on_shutdown(const rclcpp_lifecycle::State &)
{
  // Shutdown hardware
  return hardware_interface::CallbackReturn::SUCCESS;
}

}  // namespace robot_description


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  robot_description::RobotHardwareInterface, 
  hardware_interface::SystemInterface
)
