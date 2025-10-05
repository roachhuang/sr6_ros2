import os
from ament_index_python import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Declare the model argument
    model_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(
            get_package_share_directory("smallrobot_description"),
            "urdf",
            "smallrobot.urdf.xacro",  # Corrected file name
        ),
        description="The robot model to load",
    )

    # smallrobot_description_path = get_package_share_directory("smallrobot_description")
    # meshes_path = os.path.join(smallrobot_description_path, "meshes")
    # if not os.path.exists(meshes_path):
    #     raise FileNotFoundError(
    #         f"The directory '{meshes_path}' does not exist. Please ensure it is correctly set up."
    #     )
    # env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", meshes_path)
    env_var = SetEnvironmentVariable(
        "GAZEBO_MODEL_PATH",
        os.path.join(get_package_prefix("smallrobot_description"), "share"),
    )

    # print("GAZEBO_MODEL_PATH:", meshes_path)

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

    # Start Gazebo Sim (Garden)
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gzserver.launch.py",
            )
        )
        # launch_arguments={"verbose": "true", "extra_gazebo_args": "--verbose"}.items(),
    )
    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gzclient.launch.py",
            )
        )
        # launch_arguments={"verbose": "true", "extra_gazebo_args": "--verbose"}.items(),
    )
    # Joint State Publisher GUI Node

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "smallrobot", "-topic", "robot_description"],
        # output="screen",
    )
    return LaunchDescription(
        [
            env_var,
            model_arg,
            robot_state_publisher_node,
            start_gazebo_server,
            start_gazebo_client,
            spawn_robot,
        ]
    )
