
#ifndef ROBOTARM_INTERFACE_H
#define ROBOTARM_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <hardware_interface/system_interface.hpp>
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include <vector>
#include <string>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
namespace hw = hardware_interface; // ✨ alias once

namespace robot_arm_ns
{
    class RobotArmInterface : public hw::SystemInterface
    {
    public:
        RobotArmInterface();
        virtual ~RobotArmInterface();
        // ROS 2 Lifecycle Methods, override automatically implies it's virtual.
        hw::CallbackReturn on_init(const hw::HardwareInfo &info) override;
        // hw::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
        hw::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
        hw::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

        // Hardware Interface Methods
        std::vector<hw::StateInterface> export_state_interfaces() override;
        std::vector<hw::CommandInterface> export_command_interfaces() override;
        hw::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        hw::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        void parseFeedback_(const std::string &msg);

        // Serial port for communication with the robot arm
        LibSerial::SerialPort arduino_;
        std::string port_;
        bool isArduinoBusy_ = false;
        rclcpp::Time last_command_time_;
        const rclcpp::Duration command_timeout_{std::chrono::seconds(2)};

        // Joint states (position, velocity, effort)
        // std::vector<double> hw_positions_;
        // std::vector<double> hw_velocities_;
        // std::vector<double> hw_efforts_;
        // std::vector<std::string> joint_names_;

        std::vector<double> position_commands_;
        std::vector<double> prev_position_commands_;
        std::vector<double> position_states_;
    };
}
#endif
