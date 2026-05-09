/*
### The rule:
- **Header (`.hpp`)**: Include headers required for types, constants, or functions used in the class declaration (member variables, base classes, function signatures, etc.).
- **Source (`.cpp`)**: Include headers only needed for the implementation (function bodies, local variables, etc.).

**Summary:**
Include these headers in your `.hpp` file because your class declaration depends on types and features from them. This ensures any file including your header can see the full type definitions and use your class without compilation errors.
*/
#include "robotarm_controller/robotarm_interface.hpp"
#include <algorithm>
#include <thread>

namespace robotarm_controller
{  
  RobotArmInterface::RobotArmInterface() : driver_(std::make_unique<RobotArmDriver>()) {}
  
  RobotArmInterface::~RobotArmInterface(){
    RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "destructor...");
    disconnect_hardware();
  }
  
  // Initialize the serial port
  hw::CallbackReturn RobotArmInterface::on_init(const hw::HardwareComponentInterfaceParams & params)
  {
    if (hardware_interface::SystemInterface::on_init(params) != hw::CallbackReturn::SUCCESS)
    {
      return hw::CallbackReturn::ERROR;
    }

    // Initialize joint states and commands
    num_joints_ = info_.joints.size();
    if (num_joints_ > lower_limit_.size() || num_joints_ > upper_limit_.size())
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("RobotArmInterface"),
        "Unsupported joint count %zu. Limits are defined for up to %zu joints.",
        num_joints_,
        lower_limit_.size());
      return hw::CallbackReturn::ERROR;
    }

    position_commands_.resize(num_joints_, 0.0);

    position_states_.resize(num_joints_, 0.0);
    velocity_states_.resize(num_joints_, 0.0);

    prev_position_commands_.resize(num_joints_, 0.0);
    RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Initialized with %zu joints, using position interface", info_.joints.size());

    return hw::CallbackReturn::SUCCESS;
  }

  hw::CallbackReturn RobotArmInterface::on_configure(const rclcpp_lifecycle::State & previous_state)
  {
    (void)previous_state;
    try
    {
      std::string port = info_.hardware_parameters.at("port");
      uint32_t baudrate = baud_rate_;
      if (!driver_->connect(port, baudrate)) {
        RCLCPP_ERROR(rclcpp::get_logger("RobotArmInterface"), "Failed to open Arduino serial port");
        return hw::CallbackReturn::ERROR;
      }
      RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "a 2s delay for Arduino to get ready...");
      std::this_thread::sleep_for(std::chrono::seconds(2));
      RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Connected successfully.");
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("RobotArmInterface"), "Serial config error: %s", e.what());
      return hw::CallbackReturn::ERROR;
    }

    return hw::CallbackReturn::SUCCESS;
  }

  hw::CallbackReturn RobotArmInterface::on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    (void)previous_state;

    RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Activating hardware...");
    last_command_time_ = rclcpp::Clock().now();
    if (!driver_ || !driver_->is_connected()) {
      RCLCPP_ERROR(rclcpp::get_logger("RobotArmInterface"), "Driver not connected! Cannot activate.");
      return hw::CallbackReturn::ERROR;
    }
    if (!driver_->activate()) {
      RCLCPP_ERROR(rclcpp::get_logger("RobotArmInterface"), "Failed to enable hardware");
      return hw::CallbackReturn::ERROR;
    }
    // Initialize robotarm h/w
    return hw::CallbackReturn::SUCCESS;
  }

  hw::CallbackReturn RobotArmInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state){
    (void)previous_state;
    RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "on deactivate...");
    return disconnect_hardware();
  }

  hw::CallbackReturn RobotArmInterface::on_shutdown(const rclcpp_lifecycle::State & previous_state)
  {
    (void)previous_state;
    // Disconnect from hardware
    RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "on shutdown...");
    return disconnect_hardware();
  }

  std::vector<hardware_interface::StateInterface> RobotArmInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < num_joints_; ++i)
    {
      state_interfaces.emplace_back(
          info_.joints[i].name,
          hardware_interface::HW_IF_POSITION,
          &position_states_[i]);

      state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "velocity", &velocity_states_[i]));
    }

    RCLCPP_INFO(rclcpp::get_logger("RobotArmInterface"), "Exported %zu state interfaces", state_interfaces.size());
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
      // command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_cmds_[i]));
    }
    return command_interfaces;
  }

  hardware_interface::return_type RobotArmInterface::read([[maybe_unused]] const rclcpp::Time &time, [[maybe_unused]] const rclcpp::Duration &period)
  {
    // Mirror commanded state with a small first-order lag so simulated feedback is stable.
    const double dt = std::max(period.seconds(), 1e-6);
    const double tau = 0.15;
    double alpha = dt / (tau + dt);

    for (std::size_t i = 0; i < info_.joints.size(); ++i)
    {
      double previous_position = position_states_[i];
      position_states_[i] += alpha * (position_commands_[i] - position_states_[i]);
      velocity_states_[i] = (position_states_[i] - previous_position) / dt;
    }
    return hw::return_type::OK;
  }

  hardware_interface::return_type RobotArmInterface::write([[maybe_unused]] const rclcpp::Time &time, [[maybe_unused]] const rclcpp::Duration &period)
  {
    // Clamp position commands to joint limits (in radians)
    for (size_t i = 0; i < position_commands_.size(); ++i)
    {
      double lower = lower_limit_[i] * M_PI / 180.0;
      double upper = upper_limit_[i] * M_PI / 180.0;
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

    // Avoid spamming the serial link when the command has not changed.
    try
    {
      if (driver_ && driver_->is_connected()) {
        driver_->send_command(position_commands_);
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_FATAL(rclcpp::get_logger("RobotArmInterface"), "write cmd failed: %s", e.what());
      return hardware_interface::return_type::ERROR;
    }
    last_command_time_ = rclcpp::Clock().now();
    prev_position_commands_ = position_commands_;
    return hardware_interface::return_type::OK;
  }

  hw::CallbackReturn RobotArmInterface::disconnect_hardware()
  {
    if (driver_ && driver_->is_connected()) {
      try {
        driver_->send_home();
        driver_->disconnect();
      } catch (const std::exception &e) {
        RCLCPP_WARN(rclcpp::get_logger("RobotArmInterface"), "Driver err: %s", e.what());
        return hw::CallbackReturn::ERROR;
      }
    }
    return hw::CallbackReturn::SUCCESS;
  }

} // namespace
// RobotarmInterface as a pluing in the pluing lib (base class of the robotarm interface that we have implemented is the hardware_interface::SystemInterface). in other words, Register the RobotArmInterface as a hardware interface.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robotarm_controller::RobotArmInterface, hw::SystemInterface)
