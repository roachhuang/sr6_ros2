from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    task_server = Node(
        package="smallrobot_remote",
        executable="task_server",
        name="task_server",
    )
    alexa_interface = Node(
        package="smallrobot_remote",
        executable="alex_interface",
    )

    return LaunchDescription([
        task_server,
        alexa_interface,
    ])
