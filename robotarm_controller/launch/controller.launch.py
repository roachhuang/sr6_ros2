import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import Command, LaunchConfiguration
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

    is_sim = LaunchConfiguration("is_sim")
    log_is_sim = LogInfo(msg=["Launching with is_sim: ", is_sim])

    # Generate robot description using xacro
    robot_description = ParameterValue(
        Command([
            "xacro ",
            os.path.join(
                get_package_share_directory("smallrobot_description"),
                "urdf",
                "smallrobot.urdf.xacro",
            ),
            " is_sim:=", is_sim,  # Pass is_sim to xacro
        ]),
        value_type=str,
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        condition=UnlessCondition(is_sim),  # Only start when not simulating
        output="screen",
    )

    # Controller Manager Node (ros2_control_node)
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": is_sim},
            os.path.join(
                get_package_share_directory("robotarm_controller"),
                "config",
                "robotarm_controllers.yaml",
            ),
        ],
        remappings=[("/robot_description", "/robot_description")],  # <- FIX: remove ~
        output="screen",
    )

    # Delay spawning controllers
    delay_spawners = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["arm_controller", "--controller-manager", "/controller_manager"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
                output="screen",
            ),
        ],
    )

    return LaunchDescription([
        is_sim_arg,
        log_is_sim,
        robot_state_publisher_node,
        controller_manager_node,
        delay_spawners,
    ])
