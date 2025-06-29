import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import Command, LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import UnlessCondition


def generate_launch_description():
    # Declare simulation argument
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="true",
        description="Set to true if running in simulation",
    )
    # to acces the param that were declared with DeclareLaunchArgument!!!
    # This allows us to use the value of is_sim in the xacro command and yaml
    # file selection
    # This is useful for selecting different configurations based on whether
    # the robot is in simulation or not.
    # For example, you might have different controller configurations for
    # simulation and real hardware.
    # In this case, we will use 'sim_controllers.yaml' for simulation and
    # 'real_controllers.yaml' for real hardware.
    # This is done using a PythonExpression to conditionally select the file
    # based on the value of is_sim.
    # The file paths are constructed using PathJoinSubstitution to ensure
    # correct path formatting across different operating systems.
    is_sim = LaunchConfiguration("is_sim")
    yaml_filename = PythonExpression(
        [
            "'sim_controllers.yaml' if '",
            is_sim,
            "' == 'true' else 'real_controllers.yaml'",
        ]
    )
    
    # path for configuration file
    yaml_path = PathJoinSubstitution(
        [get_package_share_directory("robotarm_controller"), "config", yaml_filename]
    )
    
    log_yaml = LogInfo(msg=["yaml: ", yaml_filename])

    # Generate robot description using xacro
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("smallrobot_description"),
                    "urdf",
                    "smallrobot.urdf.xacro",
                ),
                " is_sim:=",
                is_sim,  # Pass is_sim to xacro
            ]
        ),
        value_type=str,
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, 'use_sim_time': is_sim}],
        # condition=UnlessCondition(is_sim),  # Only start when not simulating
        output="screen",
    )

    # Controller Manager Node (ros2_control_node). must have!!!
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": is_sim},
            yaml_path,
        ],
        # remappings=[("/robot_description", "/robot_description")],  # <- FIX: remove ~
        output="both",
    )

    # Delay spawning controllers. avoid urdf not fully load yet, h/w interface not init, race conditions
    delay_spawners = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "arm_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "gripper_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            ),
        ],
    )

    return LaunchDescription(
        [            
            is_sim_arg,
            robot_state_publisher_node,
            controller_manager_node,
            delay_spawners,
        ]
    )
