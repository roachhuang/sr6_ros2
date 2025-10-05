// this code works w/ teensy.ino

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <string>
#include <vector>
#include <memory>
#include <chrono>

#include <rclcpp/serial_port.hpp> // You need serial port helper (custom or simple)

using namespace std::chrono_literals;

namespace robot_arm_hardware
{

  class RobotArmHardware : public hardware_interface::SystemInterface
  {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(RobotArmHardware)

    RobotArmHardware() {}

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override
    {
      if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
      {
        return hardware_interface::CallbackReturn::ERROR;
      }

      // Resize storage
      num_joints_ = info_.joints.size();
      position_commands_.resize(num_joints_, 0.0);
      position_states_.resize(num_joints_, 0.0);

      return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override
    {
      std::vector<hardware_interface::StateInterface> state_interfaces;
      for (size_t i = 0; i < num_joints_; ++i)
      {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
      }
      return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
    {
      std::vector<hardware_interface::CommandInterface> command_interfaces;
      for (size_t i = 0; i < num_joints_; ++i)
      {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
      }
      return command_interfaces;
    }

    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
    {
      // Open serial port
      serial_ = std::make_shared<rclcpp::SerialPort>(info_.hardware_parameters.at("port"), 115200);
      if (!serial_->open())
      {
        RCLCPP_ERROR(rclcpp::get_logger("RobotArmHardware"), "Failed to open serial port!");
        return hardware_interface::CallbackReturn::ERROR;
      }
      return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
    {
      return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
    {
      serial_->close();
      return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
    {
      if (!serial_->available())
        return hardware_interface::return_type::OK;

      std::string feedback = serial_->read(); // Read entire line from Teensy
      // Example feedback: "P1:44.8;P2:29.9;P3:15.2;P4:0.0;P5:0.0;P6:0.0;"

      std::stringstream ss(feedback);
      std::string token;
      size_t joint_idx = 0;

      while (std::getline(ss, token, ';') && joint_idx < num_joints_)
      {
        size_t pos = token.find(":");
        if (pos != std::string::npos)
        {
          double pos_value = std::stod(token.substr(pos + 1));
          position_states_[joint_idx++] = pos_value;
        }
      }

      return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
    {
      if (!serial_->is_open())
        return hardware_interface::return_type::ERROR;

      for (size_t i = 0; i < num_joints_; ++i)
      {
        std::string command = "P" + std::to_string(i + 1) + ":" + std::to_string(position_commands_[i]) + "\n";
        serial_->write(command);
      }

      return hardware_interface::return_type::OK;
    }

  private:
    size_t num_joints_;
    std::vector<double> position_commands_;
    std::vector<double> position_states_;
    std::shared_ptr<rclcpp::SerialPort> serial_;
  };

} // namespace robot_arm_hardware

PLUGINLIB_EXPORT_CLASS(robot_arm_hardware::RobotArmHardware, hardware_interface::SystemInterface)
