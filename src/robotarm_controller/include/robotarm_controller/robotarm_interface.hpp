
#ifndef ROBOTARM_INTERFACE_H
#define ROBOTARM_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <hardware_interface/system_interface.hpp>

#include <vector>
#include <string>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
namespace robotarm_controller
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class RobotArmInterface : public hardware_interface::SystemInterface
    {
    public:
        RobotArmInterface();
        virtual ~RobotArmInterface();
        // ROS 2 Lifecycle Methods
        virtual CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
        virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
        virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

        // Hardware Interface Methods
        virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;        
        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;        

    private:
        // Serial port for communication with the robot arm
        LibSerial::SerialPort arduino;
        std::string port_;

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
