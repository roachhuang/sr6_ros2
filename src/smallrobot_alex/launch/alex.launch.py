
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    task_server_node = Node(
        package="smallrobot_remote",
        executable="task_server_node",        
    )
    alexa_interface_node = Node(
        package="smallrobot_remote",
        executable="alexa_interface.py",             
    )
    return LaunchDescription([
        task_server_node,
        alexa_interface_node,
    ])