
#ifndef ROBOTARM_INTERFACE_H
#define ROBOTARM_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <hardware_interface/system_interface.hpp>
// #include <pluginlib/class_list_macros.hpp>


#include <vector>
#include <string>

#include <boost/asio.hpp>
// using namespace boost::asio;

// #include <boost/bind/bind.hpp>

// #include <libserial/SerialPort.h>
// #include <libserial/SerialStream.h>

namespace hw = hardware_interface; // ✨ alias once
namespace robotarm_controller
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class RobotArmInterface : public hardware_interface::SystemInterface
    {
    public:
        RobotArmInterface() : io_(), serial_(io_) {}
        // RobotArmInterface();
        // ~RobotArmInterface();
        // ROS 2 Lifecycle Methods
        CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

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
        // std::vector<double> hw_positions_;
        // std::vector<double> hw_velocities_;
        // std::vector<double> hw_efforts_;
        // std::vector<std::string> joint_names_;
        std::vector<double> position_commands_;
        // std::vector<double> velocity_commands_;
        std::vector<double> effort_commands_;

        std::vector<double> position_states_;
        // std::vector<double> velocity_states_;
        // std::vector<double> effort_states_;
        
        std::vector<double> prev_position_commands_;
    };
}
#endif