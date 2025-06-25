cd ~/ros2_ws/src/smallrobot_description/urdf
source ~/ros2_ws/install/setup.bash
ros2 run xacro xacro smallrobot.urdf.xacro > generated_robot.urdf


pre-requiste:
    sudo apt install ros-jazzy-gz-ros2-control
    ensure that you have a dependency declared for gz_ros2_control in your package.xml