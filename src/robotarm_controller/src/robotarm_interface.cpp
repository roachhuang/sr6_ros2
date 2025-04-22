#include "robotarm_controller/robotarm_interface.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <cmath>
#include <sstream>

namespace robotarm_controller
{
  RobotArmInterface::RobotArmInterface()
  {
    // Constructor implementation
  }
  RobotArmInterface::~RobotArmInterface()
  {
    // Destructor implementation
    if (arduino.IsOpen())
    {
      try
      {
        arduino.Close();
      }
      catch (...)
      {
        RCLCPP_ERROR(rclcpp::get_logger("RobotArmInterface"), "Failed to close serial port: ");
      }
    }
  }

  // Initialize the serial port
  CallbackReturn RobotArmInterface::on_init(const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }
    /*
    try
    {
      port_ = info.hardware_parameters.at("port");
      arduino.Open(port_);
      arduino.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    }
    catch (...)
    {
      RCLCPP_ERROR(rclcpp::get_logger("RobotArmInterface"), "Failed to open serial port: ");
      return CallbackReturn::ERROR;
    }
    */

    // Initialize joint states and commands
    isArduinoBusy_ = false;
    position_commands_.resize(info_.joints.size(), 0.0);
    position_states_.resize(info_.joints.size(), 0.0);
    prev_position_commands_.resize(info_.joints.size(), 0.0);

    return CallbackReturn::SUCCESS;
  };

  CallbackReturn RobotArmInterface::on_configure(const rclcpp_lifecycle::State &)
  {
    try
    {
      // Open Arduino serial port
      // port_ = info.hardware_parameters.at("port");
      std::string device = info_.hardware_parameters.at("port");
      // uint32_t baudrate = std::stoi(info_.hardware_parameters.at("baud_rate"));
      arduino.Open(device);
      arduino.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
      // if (!arduino.IsOpen())
      // {
      //   RCLCPP_ERROR(rclcpp::get_logger("RobotArmInterface"), "Failed to open Arduino serial port");
      //   return hardware_interface::CallbackReturn::ERROR;
      // }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("RobotArmInterface"), "Serial config error: %s", e.what());
      return hw::CallbackReturn::ERROR;
    }

    // RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Exporting %zu command interfaces", position_commands_.size());
    // RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Exporting %zu state interfaces", position_states_.size());

    RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Connected to Arduino on %s, @ baud rate enum %d", port_.c_str(), static_cast<int>(LibSerial::BaudRate::BAUD_115200));
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn RobotArmInterface::on_activate(const rclcpp_lifecycle::State &)
  {
    // Connect to real hardware here
    // std::stringstream ss;
    // ss << "en" << \n;
    RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Activating hardware...");
    isArduinoBusy_ = false;
    last_command_time_ = rclcpp::Clock().now();

    std::string msg = "en\n";
    try
    {
      arduino.Write(msg);
    }
    catch (...)
    {
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
    try
    {
      arduino.Write(msg);
    }
    catch (...)
    {
      RCLCPP_ERROR(rclcpp::get_logger("RobotArmInterface"), "Failed to write to serial port:");
      return CallbackReturn::ERROR;
    }

    if (arduino.IsOpen())
    {
      try
      {
        arduino.Close();
      }
      catch (...)
      {
        RCLCPP_ERROR(rclcpp::get_logger("RobotArmInterface"), "Failed to close serial port:");
        return CallbackReturn::ERROR;
      }
    }
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> RobotArmInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
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
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      command_interfaces.emplace_back(
          info_.joints[i].name,
          hardware_interface::HW_IF_POSITION,
          &position_commands_[i]);
    }
    return command_interfaces;
  }

  hardware_interface::return_type RobotArmInterface::read([[maybe_unused]] const rclcpp::Time &time, [[maybe_unused]] const rclcpp::Duration &period)
  {
    // Read from hardware (simulated here)
    position_states_ = position_commands_; // Simulate perfect tracking

    // Persistent buffer across read() calls
    static std::string persistent_buffer;
    
    try
    {
      // Read up to 256 bytes each cycle
      std::string temp_buffer;
      temp_buffer.resize(256);

      // Read from Arduino (non-blocking)
      arduino.Read(temp_buffer, temp_buffer.size(), 0); // 0ms timeout => non-blocking

      // Trim temp_buffer to actual received data
      auto first_null = temp_buffer.find('\0');
      if (first_null != std::string::npos)
      {
        temp_buffer.resize(first_null);
      }

      // Append to persistent_buffer
      persistent_buffer += temp_buffer;

      // Process all complete lines
      size_t newline_pos;
      while ((newline_pos = persistent_buffer.find('\n')) != std::string::npos)
      {
        // Extract one full line (excluding '\n')
        std::string line = persistent_buffer.substr(0, newline_pos);
        persistent_buffer.erase(0, newline_pos + 1); // +1 to remove '\n'

        // Parse the line
        if (!line.empty())
        {
          parseFeedback_(line);
        }
      }
    }
    catch (const LibSerial::ReadTimeout &)
    {
      // Expected sometimes — no big deal
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(
          rclcpp::get_logger("RobotArmInterface"),
          "Serial read failed: %s", e.what());
    }


    // Handle Arduino busy timeout
    if (isArduinoBusy_)
    {
      auto now = rclcpp::Clock().now();
      if (now - last_command_time_ > command_timeout_)
      {
        RCLCPP_ERROR(
            rclcpp::get_logger("RobotArmInterface"),
            "Command timeout! No ACK received from Arduino.");
        isArduinoBusy_ = false; // Allow retry
      }
    }
    
    // Simulate perfect tracking (optional if you want pure feedback instead)
    // position_states_ = position_commands_;
    return hw::return_type::OK;
  }

  void RobotArmInterface::parseFeedback_(const std::string &msg)
  {
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
        double value = std::strtod(temp, nullptr);
        position_states_[idx++] = value * (M_PI / 180.0); // Degrees to radians

        ptr += 7; // Move to next number
      }

      // Simulate perfect tracking (optional if you want pure feedback instead)
      position_states_ = position_commands_;

      // RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "curPos: %s", msg.c_str());
      // RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Parsed positions: [%f, %f, %f, %f, %f, %f]",
      //             position_states_[0], position_states_[1], position_states_[2],
      //             position_states_[3], position_states_[4], position_states_[5]);
    }
    else if (msg[0] == 'a')
    {
      isArduinoBusy_ = false;
      RCLCPP_DEBUG(rclcpp::get_logger("RobotArmInterface"), "Arduino ACK received");
    }
    else
    {

      RCLCPP_WARN(rclcpp::get_logger("RobotArmInterface"), "len: %d", static_cast<int>(msg.size()));
    }
  }

  hardware_interface::return_type RobotArmInterface::write([[maybe_unused]] const rclcpp::Time &time, [[maybe_unused]] const rclcpp::Duration &period)
  {
    if (isArduinoBusy_)
    {
      RCLCPP_WARN(rclcpp::get_logger("RobotArmInterface"), "Arduino is still busy, cannot send new command.");
      return hardware_interface::return_type::OK;
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
      RCLCPP_WARN(rclcpp::get_logger("RobotArmInterface"), "cmd no difference.");
      return hardware_interface::return_type::OK;
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
        arduino.Write(msg);
        isArduinoBusy_ = true;
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
}
// RobotarmInterface as a pluing in the pluing lib (base class of the robotarm interface that we have implemented is the hardware_interface::SystemInterface). in other words, Register the RobotArmInterface as a hardware interface.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robotarm_controller::RobotArmInterface, hardware_interface::SystemInterface)