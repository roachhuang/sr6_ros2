from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os


def generate_launch_description():

    # Set is_sim true
    DeclareLaunchArgument("is_sim", default_value="true"),

    # Robot State Publisher
    sp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {
                "robot_description": Command(
                    [
                        "xacro ",
                        os.path.join(
                            get_package_share_directory("smallrobot_description"),
                            "urdf/smallrobot.urdf.xacro",
                        ),
                        " is_sim:=true",
                    ]
                )
            }
        ],
    )
    # Controller Manager Node (ros2_control_node)
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {
                "robot_description": Command(
                    [
                        "xacro ",
                        os.path.join(
                            get_package_share_directory("smallrobot_description"),
                            "urdf",
                            "smallrobot.urdf.xacro",
                        ),
                    ]
                )
            },
            os.path.join(
                get_package_share_directory("robotarm_controller"),
                "config",
                "robotarm_controllers.yaml",
            ),
        ],
        arguments=["--ros-args", "--log-level", "debug"],
        # remappings=[("/robot_description", "/robot_description")],  # <- FIX: remove ~
        output="screen",
    )

    # Gazebo Launch
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("smallrobot_description"),
            "launch",
            "gazebo.launch.py",
        ),
        launch_arguments={"is_sim": "true"}.items(),
    )

    # Controller manager spawner (position_controllers, etc)
    jb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )
    arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    return LaunchDescription([sp, controller_manager_node, gazebo, jb, arm])
