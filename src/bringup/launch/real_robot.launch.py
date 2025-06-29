from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os


def generate_launch_description():
    '''
    ros2 launch your_package your_main_launch.py use_sim:=true will set is_sim to true in all the included launch files.
    '''
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="false",
        description="Start robot in readl h/w by default.",
    )
    is_sim = LaunchConfiguration('is_sim')
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("robotarm_controller"),
            "launch",
            "controller.launch.py",
        ),
        launch_arguments={"is_sim": is_sim}.items(),
    )

    moveit = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("smallrobot_moveit"),
            "launch",
            "moveit.launch.py",
        ),
        launch_arguments={"is_sim": is_sim}.items(),
    )

    # remote_interface = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("smallrobot_remote"),
    #         "launch",
    #         "remote_interface.launch.py",
    #     ),
    #     launch_arguments={"is_sim": "True"}.items(),
    # )

    return LaunchDescription(
        [
            is_sim_arg,
            controller,
            moveit,
            # remote_interface,
        ]
    )
