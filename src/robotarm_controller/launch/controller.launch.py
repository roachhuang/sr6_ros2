import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration

# after luanch - ros2 control list_controllers
# after launch - ros2 control list_hardware_interfaces
# after launch - ros2 control list_state_interfaces
# after launch - ros2 control list_joint_state_interfaces
# after launch - ros2 control list_joint_interfaces
def generate_launch_description():
    # Generate the robot description in urdf format from the xacro file
    robot_description = ParameterValue(
        # a space after xacro is essential!!!
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("smallrobot_description"),
                    "urdf",
                    "smallrobot.urdf.xacro",
                ),
            ]
        ),
        value_type=str,
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        # output='both'
    )

    join_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        # output='both'
    )

    arm_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        # output='both'
    )

    return LaunchDescription([
        robot_state_publisher_node, 
        join_state_broadcaster_node,
        arm_controller_node,
    ])