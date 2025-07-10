cd ~/ros2_ws/src/smallrobot_description/urdf
source ~/ros2_ws/install/setup.bash
ros2 run xacro xacro smallrobot.urdf.xacro > generated_robot.urdf
check_urdf generated_robot.urdf

rosrun xacro xacro.py `rospack find smallrobot_description`/urdf/smallrobot.urdf.xacro -o /tmp/smallrobot.urdf


pre-requiste:
    sudo apt install ros-jazzy-gz-ros2-control
    ensure that you have a dependency declared for gz_ros2_control in your package.xml