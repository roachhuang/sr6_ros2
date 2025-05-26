#ifndef ROBOTARM_INTERFACE_H
#define ROBOTARM_INTERFACE_H

#include <cmath>
#include <string>
#include <memory>
#include <boost/asio.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <hardware_interface/system_interface.hpp>
// #include <pluginlib/class_list_macros.hpp>

#include <vector>

namespace hw = hardware_interface; // alias once
namespace robotarm_controller
{
    // using hw::CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::hw::CallbackReturn;

    class RobotArmInterface : public hardware_interface::SystemInterface
    {
    public:
        RobotArmInterface() : io_(), serial_(io_) {}
        // RobotArmInterface();
        // ~RobotArmInterface();
        // ROS 2 Lifecycle Methods
        hw::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
        hw::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
        hw::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
        hw::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

        // Hardware Interface Methods
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;        
        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;        

    private:
        // void start_async_read();
        void parseFeedback_(const std::string &msg);
        // void send_position_to_motor(size_t joint_index, double position);
        // void send_velocity_to_motor(size_t joint_index, double velocity);
        // void send_effort_to_motor(size_t joint_index, double effort);

        // Serial port for communication with the robot arm
        // LibSerial::SerialPort arduino;
        boost::asio::io_service io_;
        boost::asio::serial_port serial_;
        std::string port_;
        int baud_rate_ = 115200;

        bool isArduinoBusy_;
        rclcpp::Time last_command_time_;
        rclcpp::Duration command_timeout_{rclcpp::Duration::from_seconds(2.0)};
        std::size_t num_joints_;
        // Joint states (position, velocity, effort)
       
        // std::vector<std::string> joint_names_;
        std::vector<double> position_commands_;
        // std::vector<double> velocity_commands_;
        std::vector<double> effort_commands_;

        std::vector<double> position_states_;
        // std::vector<double> velocity_states_;        
        // std::vector<double> effort_states_;

        const double lower_limit[6] ={-114.0, -81.0, -180.0, -180.0, -139.0, -180.0};
        const double upper_limit[6]= {114.0,  77.0,   70.0,   180.0,  139.0,  180};
        std::vector<double> prev_position_commands_;
    };
}
#endif