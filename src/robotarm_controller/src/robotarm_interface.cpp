#include "robotarm_controller/robotarm_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <limits>
// #include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <cmath>
#include <sstream>
#include <iomanip>

namespace hw = hardware_interface;

namespace robot_arm_ns
{

  // RobotArmInterface::RobotArmInterface()
  // {
  //   // Constructor implementation
  // }
  // RobotArmInterface::~RobotArmInterface()
  // {
  //   // Destructor implementation
  //   if (arduino_.IsOpen())
  //   {
  //     try
  //     {
  //       arduino_.Close();
  //     }
  //     catch (...)
  //     {
  //       RCLCPP_ERROR(rclcpp::get_logger("RobotArmInterface"), "Failed to close serial port: ");
  //     }
  //   }
  // }

  // Initialize the serial port
  hw::CallbackReturn RobotArmInterface::on_init(const hw::HardwareInfo &info)
  {
    if (hw::SystemInterface::on_init(info) != hw::CallbackReturn::SUCCESS)
    {
      return hw::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "init hardware...");
    // Get joint names from URDF/parameters
    // joint_names_ = info_.joints;

    joint_names_.reserve(info.joints.size());
    for (const auto &joint : info.joints)
    {
      joint_names_.push_back(joint.name);
    }

    // Resize vectors
    position_states_.resize(info_.joints.size(), 0.0);
    position_commands_.resize(info_.joints.size(), 0.0);
    prev_position_commands_.resize(info_.joints.size(), 0.0);

    return hardware_interface::CallbackReturn::SUCCESS;
  };

  hw::CallbackReturn RobotArmInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    try
    {
      // Open Arduino serial port
      // port_ = info.hardware_parameters.at("port");
      std::string device = info_.hardware_parameters.at("port");
      // uint32_t baudrate = std::stoi(info_.hardware_parameters.at("baud_rate"));
      arduino_.Open(device);
      arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
      if (!arduino_.IsOpen())
      {
        RCLCPP_ERROR(rclcpp::get_logger("RobotArmInterface"), "Failed to open Arduino serial port");
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("RobotArmInterface"), "Serial config error: %s", e.what());
      return hw::CallbackReturn::ERROR;
    }

    // RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Exporting %zu command interfaces", position_commands_.size());
    // RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Exporting %zu state interfaces", position_states_.size());

    RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Connected to Arduino on %s, @ baud rate enum %d", port_.c_str(), static_cast<int>(LibSerial::BaudRate::BAUD_115200));
    return hw::CallbackReturn::SUCCESS;
  }

  hw::CallbackReturn RobotArmInterface::on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Activating hardware...");

    isArduinoBusy_ = false;
    last_command_time_ = rclcpp::Clock().now();

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
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      state_interfaces.emplace_back(joint_names_[i], hw::HW_IF_POSITION, &position_states_[i]);
    }
    return state_interfaces;
  }

  // robotarm h/w interface
  std::vector<hw::CommandInterface> RobotArmInterface::export_command_interfaces()
  {
    std::vector<hw::CommandInterface> command_interfaces;
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      command_interfaces.emplace_back(joint_names_[i], hw::HW_IF_POSITION, &position_commands_[i]);
    }
    return command_interfaces;
  }

  hw::return_type RobotArmInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
  {

    if (isArduinoBusy_)
    {
      return hardware_interface::return_type::OK;
    }

    // RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "write...");

    bool commands_equal = true;
    for (size_t i = 0; i < position_commands_.size(); ++i)
    {
      if (std::abs(position_commands_[i] - prev_position_commands_[i]) > 1e-6)
      {
        commands_equal = false;
        break;
      }
    }

    if (!commands_equal)
    {
      std::stringstream cmd;
      cmd << "g";
      for (size_t i = 0; i < position_commands_.size(); i++)
      {
        if (i > 0)
          cmd << ',';
        cmd << std::fixed << std::setprecision(2) << (position_commands_[i] * 180.0 / M_PI);
      }
      cmd << '\n';
      RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Serial Write: %s", cmd.str().c_str());
      try
      {
        arduino_.Write(cmd.str());
        // isArduinoBusy_ = true;
        last_command_time_ = rclcpp::Clock().now();
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(rclcpp::get_logger("RobotArmInterface"), "Serial write error: %s", e.what());
        return hardware_interface::return_type::ERROR;
      }
    }

    prev_position_commands_ = position_commands_;
    return hardware_interface::return_type::OK;
  }

  hw::return_type RobotArmInterface::read([[maybe_unused]] const rclcpp::Time &time, [[maybe_unused]] const rclcpp::Duration &period)
  {
    // Persistent buffer across read() calls
    static std::string persistent_buffer;
    /*
    try
    {
      // Read up to 256 bytes each cycle
      std::string temp_buffer;
      temp_buffer.resize(256);

      // Read from Arduino (non-blocking)
      arduino_.Read(temp_buffer, temp_buffer.size(), 0); // 0ms timeout => non-blocking

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
    */
    // Simulate perfect tracking (optional if you want pure feedback instead)
    position_states_ = position_commands_;
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

} // namespace robot_arm

// RobotarmInterface as a pluing in the pluing lib (base class of the robotarm interface that we have implemented is the hw::SystemInterface). in other words, Register the RobotArmInterface as a hardware interface.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robot_arm_ns::RobotArmInterface, hw::SystemInterface)