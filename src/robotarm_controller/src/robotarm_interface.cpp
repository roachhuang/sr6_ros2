
// #include <boost/asio.hpp>
// using namespace boost::asio;

#include "robotarm_controller/robotarm_interface.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <cmath>
#include <sstream>
#include <string>
#include <memory>
#include <mutex>
#include <chrono>
#include <thread>

// #include <boost/asio.hpp>
// for async_read
// #include <boost/asio/serial_port.hpp>
// #include <boost/bind/bind.hpp>
/*In Boost, everything is inside boost::asio namespace.
You can't just write asio::xxx unless you: using namespace boost::asio;
*/
// using namespace boost::asio;

namespace robotarm_controller
{
  // Initialize the serial port
  hw::CallbackReturn RobotArmInterface::on_init(const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != hw::CallbackReturn::SUCCESS)
    {
      return hw::CallbackReturn::ERROR;
    }

    // Initialize joint states and commands
    isArduinoBusy_ = false;
    num_joints_ = info_.joints.size();
    position_commands_.resize(info_.joints.size(), 0.0);
    // effort_commands_.resize(num_joints_, 0.0);
    // velocity_commands_.resize(num_joints_, 0.0);

    position_states_.resize(info_.joints.size(), 0.0);
    // effort_states_.resize(num_joints_, 0.0);
    // velocity_states_.resize(num_joints_, 0.0);

    prev_position_commands_.resize(info_.joints.size(), 0.0);
    RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Initialized with %zu joints, using position interface", info_.joints.size());

    return hw::CallbackReturn::SUCCESS;
  };

  hw::CallbackReturn RobotArmInterface::on_configure(const rclcpp_lifecycle::State &)
  {
    try
    {
      // Open Arduino serial port
      // port_ = info.hardware_parameters.at("port");
      std::string port = info_.hardware_parameters.at("port");
      // uint32_t baudrate = std::stoi(info_.hardware_parameters.at("baud_rate"));
      serial_.open(port);
      serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
      serial_.set_option(boost::asio::serial_port_base::character_size(8));
      serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
      serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
      serial_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
      
      // 🔥 ADD DELAY AFTER OPEN
      RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Waiting for Arduino reboot (2 seconds)...");
      std::this_thread::sleep_for(std::chrono::seconds(2)); 
     
      RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Dummy wakeup sent.");
      if (!serial_.is_open())
      {
        RCLCPP_ERROR(rclcpp::get_logger("RobotArmInterface"), "Failed to open Arduino serial port");
        return hw::CallbackReturn::ERROR;
      }
      // Force a dummy write immediately to wake Arduino
      std::string wakeup = "\n";
      boost::asio::write(serial_, boost::asio::buffer(wakeup));
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("RobotArmInterface"), "Serial config error: %s", e.what());
      return hw::CallbackReturn::ERROR;
    }

    // RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Exporting %zu command interfaces", position_commands_.size());
    // RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Exporting %zu state interfaces", position_states_.size());

    // RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Connected to Arduino on %s, @ baud rate enum %d", port_.c_str(), static_cast<int>(LibSerial::BaudRate::BAUD_115200));
    return hw::CallbackReturn::SUCCESS;
  }

  hw::CallbackReturn RobotArmInterface::on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Activating hardware...");
    isArduinoBusy_ = false;
    last_command_time_ = rclcpp::Clock().now();
    if (!serial_.is_open())
    {
      RCLCPP_ERROR(rclcpp::get_logger("RobotArmInterface"), "Serial port not open! Cannot activate.");
      return hw::CallbackReturn::ERROR;
    }
    
    try
    {
      std::string cmd = "en\n";
      boost::asio::write(serial_, boost::asio::buffer(cmd));
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(
          rclcpp::get_logger("RobotArmInterface"),
          "Failed to write to serial port: %s", e.what());
      return hw::CallbackReturn::ERROR;
    }
    // Initialize robotarm h/w
    return hw::CallbackReturn::SUCCESS;
  }

  hw::CallbackReturn RobotArmInterface::on_deactivate(const rclcpp_lifecycle::State &)
  {
    // Disconnect from hardware
    RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Deactivating hardware...");
    std::string cmd = "dis\n";
    try
    {
      boost::asio::write(serial_, boost::asio::buffer(cmd));
    }
    catch (...)
    {
      RCLCPP_ERROR(rclcpp::get_logger("RobotArmInterface"), "Failed to write to serial port:");
      return hw::CallbackReturn::ERROR;
    }

    if (serial_.is_open())
    {
      try
      {
        serial_.close();
      }
      catch (...)
      {
        RCLCPP_ERROR(rclcpp::get_logger("RobotArmInterface"), "Failed to close serial port:");
        return hw::CallbackReturn::ERROR;
      }
    }
    return hw::CallbackReturn::SUCCESS;
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

      // state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "velocity", &velocity_states_[i]));
      // state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "effort", &effort_states_[i]));
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
      // command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "velocity", &velocity_commands_[i]));
      // command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "effort", &effort_commands_[i]));
    }
    return command_interfaces;
  }

  hardware_interface::return_type RobotArmInterface::read([[maybe_unused]] const rclcpp::Time &time, [[maybe_unused]] const rclcpp::Duration &period)
  {
    // Read from hardware (simulated here)
    position_states_ = position_commands_; // Simulate perfect tracking
    boost::asio::streambuf buf;
    try
    {
      boost::asio::read_until(serial_, buf, '\n');
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(
          rclcpp::get_logger("RobotArmInterface"),
          "Serial read failed: %s", e.what());
    }
    std::istream is(&buf);
    std::string data;
    std::getline(is, data);
    parseFeedback_(data);

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
      // position_states_ = position_commands_;

      RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Parsed positions: [%f, %f, %f, %f, %f, %f]",
                  position_states_[0], position_states_[1], position_states_[2],
                  position_states_[3], position_states_[4], position_states_[5]);
    }
    else if (msg[0] == 'a')
    {
      isArduinoBusy_ = false;
      RCLCPP_DEBUG(rclcpp::get_logger("RobotArmInterface"), "Arduino ACK received");
    }
    // {
    // else
    //   RCLCPP_WARN(rclcpp::get_logger("RobotArmInterface"), "len: %d", static_cast<int>(msg.size()));
    // }
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
      // RCLCPP_WARN(rclcpp::get_logger("RobotArmInterface"), "cmd no difference.");
      return hardware_interface::return_type::OK;
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Position commands are different from previous commands.");
      /* Later (upgrade to optionally use velocity/effort):
        Gripper controlled by effort (grip with controlled force)
        Linear actuator for robot base with velocity control
        You don't touch the arm controller.yaml or URDF, just spawn a new controller for the gripper or base.
      for (size_t i = 0; i < info_.joints.size(); ++i)
      {
        double command_to_send = 0.0;

        if (!std::isnan(position_commands_[i]))
        {
          // Prefer position control
          command_to_send = position_commands_[i];
          send_position_to_motor(i, command_to_send);
        }
        else if (!std::isnan(velocity_commands_[i]))
        {
          // If no position, fallback to velocity control
          command_to_send = velocity_commands_[i];
          send_velocity_to_motor(i, command_to_send);
        }
        else if (!std::isnan(effort_commands_[i]))
        {
          // If neither, fallback to effort (torque/force) control
          command_to_send = effort_commands_[i];
          send_effort_to_motor(i, command_to_send);
        }
        else
        {
          RCLCPP_WARN(rclcpp::get_logger("RobotArmInterface"), "No valid command for joint %zu", i);
        }
      }

      return hardware_interface::return_type::OK;
      */

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
        boost::asio::write(serial_, boost::asio::buffer(cmd.str()));
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
  /* for future - industries grade.
  void RobotArmInterface::send_position_to_motor(size_t joint_index, double position)
  {
    // TODO: Implement real Teensy communication later
    // Example: send serial command "P1:123.45\n"  (P = Position, 1 = joint1, 123.45 degrees)
  }

  void RobotArmInterface::send_velocity_to_motor(size_t joint_index, double velocity)
  {
    // TODO: Placeholder
    // Example: send serial command "V1:10.0\n"  (V = Velocity)
  }

  void RobotArmInterface::send_effort_to_motor(size_t joint_index, double effort)
  {
    // TODO: Placeholder
    // Example: send serial command "E1:2.5\n"  (E = Effort in Nm)
  }
  */

} // namespace
// RobotarmInterface as a pluing in the pluing lib (base class of the robotarm interface that we have implemented is the hardware_interface::SystemInterface). in other words, Register the RobotArmInterface as a hardware interface.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robotarm_controller::RobotArmInterface, hardware_interface::SystemInterface)