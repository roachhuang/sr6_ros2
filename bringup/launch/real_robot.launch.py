import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch the real robot stack with optional remote interface."""
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="false",
        description="Start robot in real hardware mode by default.",
    )
    start_remote_interface_arg = DeclareLaunchArgument(
        "start_remote_interface",
        default_value="false",
        description="Start the Alexa remote interface.",
    )
    is_sim = LaunchConfiguration("is_sim")
    start_remote_interface = LaunchConfiguration("start_remote_interface")

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robotarm_controller"),
                "launch",
                "controller.launch.py",
            )
        ),
        launch_arguments={"is_sim": is_sim}.items(),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("smallrobot_moveit"),
                "launch",
                "moveit.launch.py",
            )
        ),
        launch_arguments={"is_sim": is_sim}.items(),
    )

    remote_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("smallrobot_remote"),
                "launch",
                "alexa.launch.py",
            )
        ),
        launch_arguments={"is_sim": is_sim}.items(),
        # Alexa/Flask dependencies are optional; keep the default bringup path independent.
        condition=IfCondition(start_remote_interface),
    )

    return LaunchDescription(
        [
            is_sim_arg,
            start_remote_interface_arg,
            controller,
            moveit,
            remote_interface,
        ]
    )
