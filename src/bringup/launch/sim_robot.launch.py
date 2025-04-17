from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os


def generate_launch_description():

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("smallrobot_description"),
            "launch",
            "gazebo.launch.py",
        ),
        launch_arguments={"is_sim": "true"}.items(),
    )


    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("robotarm_controller"),
            "launch",
            "controller.launch.py",
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

    # remote_interface = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("smallrobot_remote"),
    #         "launch",
    #         "remote_interface.launch.py",
    #     ),
    #     launch_arguments={"is_sim": "True"}.items(),
    # )

    return LaunchDescription([
        gazebo,
        controller,
        moveit,
        # remote_interface,       
    ])
