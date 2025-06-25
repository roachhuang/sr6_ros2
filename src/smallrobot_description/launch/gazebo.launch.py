import os
from ament_index_python import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    SetEnvironmentVariable,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from pathlib import Path

def generate_launch_description():
    # Declare the model argument
    model_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(
            get_package_share_directory("smallrobot_description"),
            "urdf",
            "smallrobot.urdf.xacro",
        ),
        description="The robot model to load",
    )

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(
            get_package_share_directory("bringup"), "worlds", "empty_world.world"
        ),
        description="Gazebo Sim world file",
    )

    description_path = os.path.join(
        get_package_share_directory('smallrobot_description')
    )

    bringup_path = os.path.join(
        get_package_share_directory('bringup')
    )
    
    # Set gazebo sim resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(bringup_path, 'worlds'),
            ':' + str(Path(description_path).parent.resolve()),
        ],
    )
    
    
    # Generate the robot description from the xacro file
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")])
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        # output='both'
    )

    # Start Gazebo Sim directly (no ruby, no ros_gz_sim launch)
    start_gazebo = ExecuteProcess(
        cmd=["gz", "sim", LaunchConfiguration("world"), "-r", "-v", "4"],
        output="screen",
    )

    # Delay spawn to ensure Gazebo is ready
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-entity",
                    "smallrobot",
                    "-topic",
                    "robot_description",
                    "-x",
                    "0",
                    "-y",
                    "0",
                    "-z",
                    "0.1",
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            gazebo_resource_path,
            model_arg,
            world_arg,
            robot_state_publisher_node,
            start_gazebo,
            spawn_robot,
        ]
    )
