#include "robotarm_controller/robotarm_interface.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <cmath>
#include <sstream>

namespace robotarm_controller{
RobotArmInterface::RobotArmInterface(){
  // Constructor implementation
}
RobotArmInterface::~RobotArmInterface(){
  // Destructor implementation
  if (arduino.IsOpen()){
    try {
      arduino.Close();
    } catch (...) {
      RCLCPP_ERROR(rclcpp::get_logger("RobotArmInterface"), "Failed to close serial port: ");
    } 
  }
}

// Initialize the serial port
CallbackReturn RobotArmInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  try {
    port_ = info.hardware_parameters.at("port");
    arduino.Open(port_);
    arduino.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("RobotArmInterface"), "Failed to open serial port: " );
    return CallbackReturn::ERROR;
  }

  // Initialize joint states and commands
  position_commands_.resize(info_.joints.size(), 0.0);
  position_states_.resize(info_.joints.size(), 0.0);
  prev_position_commands_.resize(info_.joints.size(), 0.0);

  return CallbackReturn::SUCCESS;
};

CallbackReturn RobotArmInterface::on_activate(const rclcpp_lifecycle::State &)
{
  // Connect to real hardware here
  // std::stringstream ss;
  // ss << "en" << \n;
  RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Activating hardware...");

  std::string msg = "en\n";
  try {
    arduino.Write(msg);
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("RobotArmInterface"), "Failed to write to serial port:");
    return CallbackReturn::ERROR;
  }
  // Initialize robotarm h/w
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobotArmInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Disconnect from hardware
  RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Deactivating hardware...");
  std::string msg = "dis\n";
  try {
    arduino.Write(msg);
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("RobotArmInterface"), "Failed to write to serial port:");
    return CallbackReturn::ERROR;
  }

  if (arduino.IsOpen()){
    try {
      arduino.Close();
    } catch(...)  {
      RCLCPP_ERROR(rclcpp::get_logger("RobotArmInterface"), "Failed to close serial port:");
      return CallbackReturn::ERROR;
    } 
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotArmInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &position_states_[i]);
    
    // state_interfaces.emplace_back(
    //   info_.joints[i].name,
    //   hardware_interface::HW_IF_VELOCITY,
    //   &hw_velocities_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotArmInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &position_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::return_type RobotArmInterface::read([[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period)
{
  // Read from hardware (simulated here)
  for (size_t i = 0; i < position_states_.size(); ++i) {
    position_states_[i] = position_commands_[i];  // Simulate perfect tracking
    // hw_velocities_[i] = 0.1;  // Simulate velocity
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotArmInterface::write([[maybe_unused]] const rclcpp::Time & time, [[maybe_unused]] const rclcpp::Duration & period)
{
  if(position_commands_ == prev_position_commands_){
    return hardware_interface::return_type::OK;
  }
  // Write to hardware (simulated here)
  std::stringstream ss;
  ss << "j";
  for (size_t i = 0; i < position_commands_.size(); ++i) {
    double deg= position_commands_[i]* (180.0 / M_PI);
    ss << std::fixed << std::setprecision(2) << ',' << deg;
  }
  ss << '\n';
  std::string msg = ss.str();
  try {
    arduino.Write(msg);
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("RobotArmInterface"), "Failed to write to serial port:");
    return hardware_interface::return_type::ERROR;
  }
  prev_position_commands_ = position_commands_;
  return hardware_interface::return_type::OK;
}
}
// RobotarmInterface as a pluing in the pluing lib (base class of the robotarm interface that we have implemented is the hardware_interface::SystemInterface). in other words, Register the RobotArmInterface as a hardware interface. 
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robotarm_controller::RobotArmInterface, hardware_interface::SystemInterface)