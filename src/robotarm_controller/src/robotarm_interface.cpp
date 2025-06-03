// #include <boost/asio.hpp>
// using namespace boost::asio;
/*
### The rule:
- **Header (`.hpp`)**: Include headers required for types, constants, or functions used in the class declaration (member variables, base classes, function signatures, etc.).
- **Source (`.cpp`)**: Include headers only needed for the implementation (function bodies, local variables, etc.).

**Summary:**
Include these headers in your `.hpp` file because your class declaration depends on types and features from them. This ensures any file including your header can see the full type definitions and use your class without compilation errors.
*/
#include "robotarm_controller/robotarm_interface.hpp"

// #include <sstream>
#include <algorithm>

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
    position_commands_.resize(num_joints_, 0.0);
    // effort_commands_.resize(num_joints_, 0.0);
    // velocity_commands_.resize(num_joints_, 0.0);

    position_states_.resize(num_joints_, 0.0);
    // effort_states_.resize(num_joints_, 0.0);
    velocity_states_.resize(num_joints_, 0.0);

    prev_position_commands_.resize(num_joints_, 0.0);
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
      std::this_thread::sleep_for(std::chrono::seconds(1));
      cmd = "g0.0,0.0,0.0,0.0,0.0,0.0\n";
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
    if (serial_.is_open())
    {
      try
      {
        std::string cmd = "dis\n";
        boost::asio::write(serial_, boost::asio::buffer(cmd));
        serial_.close();
      }
      catch (const std::exception &e)
      {
        RCLCPP_WARN(rclcpp::get_logger("RobotArmInterface"), "Ser err: %s", e.what());
        return hw::CallbackReturn::ERROR;
      }
    }
    // release_interfaces();
    return hw::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> RobotArmInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < num_joints_; ++i)
    {
      RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"),
                  "Exporting state interface: joint='%s', type='position' ptr=%p", info_.joints[i].name.c_str(), (void *)&position_states_[i]);
      state_interfaces.emplace_back(
          info_.joints[i].name,
          hardware_interface::HW_IF_POSITION,
          &position_states_[i]);

      RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"),
                  "Exporting state interface: joint='%s', type='velocity' ptr=%p", info_.joints[i].name.c_str(), (void *)&velocity_states_[i]);
      state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "velocity", &velocity_states_[i]));
      // state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "effort", &effort_states_[i]));
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> RobotArmInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < num_joints_; ++i)
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
    /*
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
      return hw::return_type::ERROR;
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
    */

    // Simulate perfect tracking (open-loop: state = command)
    double dt = period.seconds(); // Duration in seconds
    double tau = 0.15;            // Motor time constant (tune this for realism)
    double alpha = dt / (tau + dt);

    for (size_t i = 0; i < position_states_.size(); ++i)
    {
      // Smoothly move state toward the command (like a real actuator)
      double previous_position = position_states_[i];

      // Low-pass filtering for position (realistic joint following)
      position_states_[i] += alpha * (position_commands_[i] - position_states_[i]);

      // Velocity = (new - old) / dt
      velocity_states_[i] = (position_states_[i] - previous_position) / dt;
    }
    return hw::return_type::OK;
  }

  void RobotArmInterface::parseFeedback_(const std::string &msg)
  {
    // RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "parseFeedback_ called with: %s", msg.c_str());
    if (msg.empty())
      return;

    if (msg == "ack")
    {
      isArduinoBusy_ = false;
      RCLCPP_DEBUG(rclcpp::get_logger("RobotArmInterface"), "Arduino ACK received");
    }
    else if (msg[0] == 'f')
    {
      return;

      // RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "'f' message received: %s", msg.c_str());
      size_t star_pos = msg.find('*');
      if (star_pos == std::string::npos)
      {
        RCLCPP_WARN(rclcpp::get_logger("RobotArmInterface"), "No checksum found in feedback: %s", msg.c_str());
        return;
      }

      std::string data = msg.substr(0, star_pos);
      std::string checksum_str = msg.substr(star_pos + 1);

      uint8_t calc_checksum = 0;
      for (char c : data)
      {
        calc_checksum += static_cast<uint8_t>(c);
      }
      calc_checksum = calc_checksum % 256;

      int recv_checksum = -1;
      try
      {
        recv_checksum = std::stoi(checksum_str);
      }
      catch (...)
      {
        RCLCPP_WARN(rclcpp::get_logger("RobotArmInterface"), "Invalid checksum in feedback: %s", msg.c_str());
        return;
      }

      // RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Data: %s, Calc checksum: %d, Recv checksum: %d", data.c_str(), calc_checksum, recv_checksum);

      if (calc_checksum != recv_checksum)
      {
        RCLCPP_WARN(rclcpp::get_logger("RobotArmInterface"), "Checksum mismatch! Calculated: %d, Received: %d, Data: %s", calc_checksum, recv_checksum, data.c_str());
        return;
      }

      std::vector<double> positions;
      std::string numbers = data.substr(1); // skip 'f'
      std::stringstream ss(numbers);
      std::string item;
      while (std::getline(ss, item, ','))
      {
        try
        {
          // from arduino, the data is in radians, no need to convert to radians.
          positions.push_back(std::stod(item));
        }
        catch (...)
        {
          RCLCPP_WARN(rclcpp::get_logger("RobotArmInterface"), "Failed to parse position: %s", item.c_str());
          return;
        }
      }
      // RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Parsed %zu positions, expected %zu", positions.size(), position_states_.size());
      if (positions.size() == position_states_.size())
      {
        position_states_ = positions;
        // Create a copy for degree conversion
        // std::vector<double> positions_deg = positions;
        // for (auto &pos : positions_deg)
        // {
        //   pos *= (180.0 / M_PI); // Convert to degrees
        // }

        // RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Parsed positions (deg): [%f, %f, %f, %f, %f, %f]",
        //             positions_deg[0], positions_deg[1], positions_deg[2],
        //             positions_deg[3], positions_deg[4], positions_deg[5]);
      }
      else
      {
        RCLCPP_WARN(rclcpp::get_logger("RobotArmInterface"), "Incorrect number of joints in feedback: %s", data.c_str());
      }
    }
  }

  hardware_interface::return_type RobotArmInterface::write([[maybe_unused]] const rclcpp::Time &time, [[maybe_unused]] const rclcpp::Duration &period)
  {
    if (isArduinoBusy_)
    {
      RCLCPP_WARN(rclcpp::get_logger("RobotArmInterface"), "Arduino is still busy, cannot send new command.");
      return hardware_interface::return_type::OK;
    }

    // Clamp position commands to joint limits (in radians)
    for (size_t i = 0; i < position_commands_.size(); ++i)
    {
      double lower = lower_limit[i] * M_PI / 180.0;
      double upper = upper_limit[i] * M_PI / 180.0;
      position_commands_[i] = std::clamp(position_commands_[i], lower, upper);
    }

    // Only send if commands changed
    bool commands_equal = std::equal(
        position_commands_.begin(), position_commands_.end(),
        prev_position_commands_.begin(),
        [](double a, double b)
        { return std::abs(a - b) < 1e-6; });
    if (commands_equal)
      return hardware_interface::return_type::OK;

    // Format and send command
    std::stringstream cmd;
    cmd << "g";
    for (size_t i = 0; i < position_commands_.size(); i++)
    {
      double deg = position_commands_[i] * 180.0 / M_PI;
      if (i > 0)
        cmd << ',';
      cmd << std::fixed << std::setprecision(2) << deg;
    }
    cmd << '\n';
    try
    {
      boost::asio::write(serial_, boost::asio::buffer(cmd.str()));
      // to be used in read function.
      last_command_time_ = rclcpp::Clock().now();
      prev_position_commands_ = position_commands_;
      // isArduinoBusy_ = true; // Set busy state until ACK received
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(rclcpp::get_logger("RobotArmInterface"), "write cmd failed: %s", e.what());
    }
    return hardware_interface::return_type::OK;
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