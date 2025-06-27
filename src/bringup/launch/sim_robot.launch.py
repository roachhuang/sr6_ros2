from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os


def generate_launch_description():
     # Declare launch arguments
    declared_arguments = [        
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Start robot in Gazebo simulation.',
        ),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Use fake hardware mirroring command.',
        ),
        DeclareLaunchArgument(
            'fake_sensor_commands',
            default_value='false',
            description='Enable fake sensor commands.',
        ),
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/ttyUSB0',
            description='Port name for hardware connection.',
        ),      
    ]
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("robotarm_controller"),
            "launch",
            "controller.launch.py",
        ),
        launch_arguments={"is_sim": "true"}.items(),
    )
    
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("smallrobot_description"),
            "launch",
            "gazebo.launch.py",
        ),
        launch_arguments={"is_sim": "true"}.items(),
    )

    moveit = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("smallrobot_moveit"),
            "launch",
            "moveit.launch.py",
        ),
        launch_arguments={"is_sim": "true"}.items(),
    )

    remote_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("smallrobot_remote"),
            "launch",
            "alexa.launch.py",
        ),
        launch_arguments={"is_sim": "True"}.items(),
    )

    return LaunchDescription([
        controller,
        gazebo,
        moveit,
        remote_interface,       
    ])
