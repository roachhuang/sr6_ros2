import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "is_sim",
            default_value="true",
            description="Start robot in Gazebo simulation.",
        ),
        DeclareLaunchArgument(
            "start_remote_interface",
            default_value="false",
            description="Start the Alexa remote interface.",
        ),
    ]

    start_remote_interface = LaunchConfiguration("start_remote_interface")

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robotarm_controller"),
                "launch",
                "controller.launch.py",
            )
        ),
        launch_arguments={"is_sim": "true"}.items(),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("smallrobot_description"),
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={"is_sim": "true"}.items(),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("smallrobot_moveit"),
                "launch",
                "moveit.launch.py",
            )
        ),
        launch_arguments={"is_sim": "true"}.items(),
    )

    remote_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("smallrobot_remote"),
                "launch",
                "alexa.launch.py",
            )
        ),
        launch_arguments={"is_sim": "true"}.items(),
        # The remote interface is not needed for simulation unless explicitly requested.
        condition=IfCondition(start_remote_interface),
    )

    return LaunchDescription(
        declared_arguments
        + [
            controller,
            gazebo,
            moveit,
            remote_interface,
        ]
    )
