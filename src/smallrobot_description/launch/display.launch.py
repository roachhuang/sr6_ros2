import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    # Declare the model argument
    model_argument = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(
            get_package_share_directory('smallrobot_description'),
            'urdf',
            'smallrobot.urdf.xacro'  # Corrected file name
        ),
        description='The robot model to load'
    )

    # Generate the robot description from the xacro file
    robot_description_content = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
        # output='both'
    )

    # Joint State Publisher GUI Node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        # output='both'
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(
            get_package_share_directory('smallrobot_description'),
            'rviz',
            'smallrobot.rviz'
        )]
    )

    return LaunchDescription([
        model_argument,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])