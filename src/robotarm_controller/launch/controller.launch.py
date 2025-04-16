import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import UnlessCondition


# after luanch - ros2 control list_controllers
# after launch - ros2 control list_hardware_interfaces
# after launch - ros2 control list_state_interfaces
# after launch - ros2 control list_joint_state_interfaces
# after launch - ros2 control list_joint_interfaces
def generate_launch_description():
    # Declare the launch argument for the robotarm to take real h/w or simulate h/w interface, so we can pass in is_sim arg upon launching this file
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="true",
        description="Set to true if running in simulation",
    )
    # read in the argument from launching
    is_sim = LaunchConfiguration("is_sim")

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
                # now this urdf.xacro file also takes an arg is_sim to indicate which h/w interface we want to start.
                'is_sim:="false',
            ]
        ),
        value_type=str,
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        # coz in it is started in our gazebo launch file, we don't need to start it again here if is_sim = true. (gazeo is for simulation)
        condition=UnlessCondition(is_sim),
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": is_sim},
            os.path.join(
                get_package_share_directory("smallrobot_controller"),
                "config",
                "robotarm_controller.yaml",
            ),
        ],
        condition=UnlessCondition(is_sim),
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

    return LaunchDescription(
        [
            is_sim_arg,
            robot_state_publisher_node,
            # this is the robot state publisher node, which will publish the robot state to tf
            # this is the controller manager node, which will load the robot description and start the controller manager
            # this is the joint state broadcaster node, which will broadcast the joint state to tf 
            controller_manager_node,
            join_state_broadcaster_node,
            arm_controller_node,
        ]
    )
