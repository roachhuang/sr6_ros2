from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="false",
        description="Set to true to run in simulation mode",
    )
    is_sim = LaunchConfiguration("is_sim")
    moveit_config = (
        MoveItConfigsBuilder("smallrobot", package_name="smallrobot_moveit")
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("smallrobot_description"),
                "urdf",
                "smallrobot.urdf.xacro",
            )
        )
        .robot_description_semantic(file_path="config/smallrobot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": is_sim},
            {"publish_robot_description_semantic": True},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )
    rviz_config = os.path.join(
        get_package_share_directory("smallrobot_moveit"),
        "config",
        "moveit.rviz",
    )

    # Load the robot description from the MoveIt configuration
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        # name="rviz2", comment up to avoid duplicate rvis2 instances!
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )
    return LaunchDescription(
        [
            is_sim_arg,
            move_group_node,
            rviz_node            
        ]
    )

