#include "robotarm_controller/robotarm_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <limits>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
// #include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <cmath>
#include <sstream>

namespace hw = hardware_interface;

namespace robot_arm_ns
{

  RobotArmInterface::RobotArmInterface()
  {
    // Constructor implementation
  }
  RobotArmInterface::~RobotArmInterface()
  {
    // Destructor implementation
    if (arduino_.IsOpen())
    {
      try
      {
        arduino_.Close();
      }
      catch (...)
      {
        RCLCPP_ERROR(rclcpp::get_logger("RobotArmInterface"), "Failed to close serial port: ");
      }
    }
  }
/*
  hw::CallbackReturn RobotArmInterface::on_configure(const rclcpp_lifecycle::State &)
  {
    try
    {
      arduino_.Open(port_);
      arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
      // arduino.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_RTS_CTS);
    }
    catch (const std::exception &e)
    {
      RCLCPP_FATAL(rclcpp::get_logger("ArduinoHardwareInterface"), "Serial open failed: %s", e.what());
      return hw::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Connected to Arduino on %s, @ baud rate enum %d", port_.c_str(), static_cast<int>(LibSerial::BaudRate::BAUD_115200));
    return hw::CallbackReturn::SUCCESS;
  }
*/
  // Initialize the serial port
  hw::CallbackReturn RobotArmInterface::on_init(const hw::HardwareInfo &info)
  {
    if (hw::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return hw::CallbackReturn::ERROR;
    }

    try
    {
      port_ = info.hardware_parameters.at("port");
      arduino_.Open(port_);
      arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
      // arduino.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_RTS_CTS);
    }
    catch (const std::exception &e)
    {
      RCLCPP_FATAL(rclcpp::get_logger("ArduinoHardwareInterface"), "Serial open failed: %s", e.what());
      return hw::CallbackReturn::ERROR;
    }
    
    // Initialize joint states and commands
    position_commands_.resize(info_.joints.size(), 0.0);
    position_states_.resize(info_.joints.size(), 0.0);
    prev_position_commands_.resize(info_.joints.size(), 0.0);
    return hw::CallbackReturn::SUCCESS;
  };

  hw::CallbackReturn RobotArmInterface::on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Activating hardware...");

    std::string msg = "en\n";
    try
    {
      arduino_.Write(msg);
    }
    catch (...)
    {
      RCLCPP_ERROR(rclcpp::get_logger("RobotArmInterface"), "Failed to write to serial port:");
      return hw::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "robotarm motors enabled");
    msg = "g0.0,0.0,0.0,0.0,0.0,0.0\n";
    try
    {
      arduino_.Write(msg);
    }
    catch (...)
    {
      RCLCPP_ERROR(rclcpp::get_logger("RobotArmInterface"), "Failed to write to serial port:");
      return hw::CallbackReturn::ERROR;
    }
    // Initialize robotarm h/w
    return hw::CallbackReturn::SUCCESS;
  }

  hw::CallbackReturn RobotArmInterface::on_deactivate(const rclcpp_lifecycle::State &)
  {
    // Disconnect from hardware
    RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Deactivating hardware...");
    std::string msg = "dis\n";
    try
    {
      arduino_.Write(msg);
    }
    catch (...)
    {
      RCLCPP_ERROR(rclcpp::get_logger("RobotArmInterface"), "Failed to write to serial port:");
      return hw::CallbackReturn::ERROR;
    }

    if (arduino_.IsOpen())
    {
      try
      {
        arduino_.Close();
      }
      catch (...)
      {
        RCLCPP_ERROR(rclcpp::get_logger("RobotArmInterface"), "Failed to close serial port:");
        return hw::CallbackReturn::ERROR;
      }
    }
    return hw::CallbackReturn::SUCCESS;
  }
  std::vector<hw::StateInterface> RobotArmInterface::export_state_interfaces()
  {
    std::vector<hw::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      state_interfaces.emplace_back(hw::StateInterface(info_.joints[i].name, "position", &position_states_[i]));
    }
    return state_interfaces;
  }

  // robotarm h/w interface
  std::vector<hw::CommandInterface> RobotArmInterface::export_command_interfaces()
  {
    std::vector<hw::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      command_interfaces.emplace_back(hw::CommandInterface(info_.joints[i].name, "position", &position_commands_[i]));
    }
    return command_interfaces;
  }

  hw::return_type RobotArmInterface::read([[maybe_unused]] const rclcpp::Time &time, [[maybe_unused]] const rclcpp::Duration &period)
  {
    // Read from hardware (simulated here)
    position_states_ = position_commands_;  // Simulate perfect tracking

    // hybride flow ctrl

    // === New part: read serial data from Arduino ===
    /*
    try
    {
      // Fixed-size buffer to avoid string dynamic allocations
      std::array<char, 256> buffer{}; // 256 bytes more than enough for a line

      while (arduino_.IsDataAvailable())
      {
        size_t index = 0;
        char c;

        // Read character by character
        while (index < buffer.size() - 1)
        {
          arduino_.ReadByte(c, 100); // Timeout 100ms (or whatever you prefer)
          if (c == '\n')
          {
            break;
          }
          buffer[index++] = c;
        }
        buffer[index] = '\0'; // Null-terminate C-string

        if (index > 0)
        {
          parseFeedback_(buffer.data());
          // RCLCPP_INFO(
          //     rclcpp::get_logger("RobotArmInterface"),
          //     "Arduino replied: '%s'", buffer.data());
        }
      }
    }
    catch (const LibSerial::ReadTimeout &)
    {
      // No big deal if timeout while reading byte
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(
          rclcpp::get_logger("ArduinoHardwareInterface"),
          "Serial read failed: %s", e.what());
    }

    if (isArduinoBusy_)
    {
      auto now = rclcpp::Clock().now();
      if (now - last_command_time_ > command_timeout_)
      {
        RCLCPP_ERROR(rclcpp::get_logger("RobotArmInterface"), "Command timeout! No ACK received from Arduino.");
        isArduinoBusy_ = false; // Reset to allow retry
      }
    }
    */

    // hw_velocities_[i] = 0.1;  // Simulate velocity
    return hw::return_type::OK;
  }

  // bool isArduinoBusy = false;

  hw::return_type RobotArmInterface::write([[maybe_unused]] const rclcpp::Time &time, [[maybe_unused]] const rclcpp::Duration &period)
  {
    if (isArduinoBusy_)
    {
      RCLCPP_WARN(rclcpp::get_logger("RobotArmInterface"), "Arduino is still busy, cannot send new command.");
      return hw::return_type::OK;
    }

    bool commands_equal = true;
    for (size_t i = 0; i < position_commands_.size(); ++i)
    {
      if (std::abs(position_commands_[i] - prev_position_commands_[i]) > 1e-6)
      {
        commands_equal = false;
        break;
      }
    }

    if (commands_equal)
    {
      // RCLCPP_WARN(rclcpp::get_logger("RobotArmInterface"), "cmd no difference.");
      return hw::return_type::OK;
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Position commands are different from previous commands.");
      // Write to hardware
      std::stringstream cmd;
      cmd << "g";
      for (size_t i = 0; i < position_commands_.size(); i++)
      {
        double deg = (position_commands_[i] * 180.0 / M_PI);
        if (i > 0)
        {
          cmd << ',';
        }
        cmd << std::fixed << std::setprecision(2) << deg;
      }
      cmd << '\n';
      std::string msg = cmd.str();
      RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Serial Write: %s", msg.c_str());
      try
      {
        arduino_.Write(msg);
        // isArduinoBusy_ = true;
        last_command_time_ = rclcpp::Clock().now();
      }
      catch (...)
      {
        RCLCPP_ERROR(rclcpp::get_logger("RobotArmInterface"), "Failed to write to serial port:");
        return hw::return_type::ERROR;
      }
    }
    prev_position_commands_ = position_commands_;
    return hw::return_type::OK;
  }

  void RobotArmInterface::parseFeedback_(const std::string &msg)
  {
    if (msg.empty())
    {
      return;
    }

    // 6 joints → (1 + 6×7) + '\n' = 44 characters total including the starting 'f'
    if (msg[0] == 'f' && msg.size() == 44)
    {
      // f+030.12+045.88+090.00+000.00+000.00+000.00\n
      // === Handle joint feedback ===
      const char *ptr = msg.c_str() + 1; // Skip 'f'
      size_t idx = 0;

      while (idx < position_states_.size())
      {
        char temp[8] = {0}; // Enough for "+000.00\0"
        std::memcpy(temp, ptr, 7);
        // double value = std::strtod(temp, nullptr);
        // position_states_[idx++] = value * (M_PI / 180.0); // Degrees to radians

        ptr += 7; // Move to next number
      }
      // RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "curPos: '%s'", msg.c_str());
    }
    else if (msg[0] == 'a')
    {
      isArduinoBusy_ = false;
      RCLCPP_DEBUG(rclcpp::get_logger("RobotArmInterface"), "Arduino ACK received");
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("RobotArmInterface"), "Unknown message: '%s'", msg.c_str());
    }
  }

} // namespace robot_arm

// RobotarmInterface as a pluing in the pluing lib (base class of the robotarm interface that we have implemented is the hw::SystemInterface). in other words, Register the RobotArmInterface as a hardware interface.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robot_arm_ns::RobotArmInterface, hw::SystemInterface)