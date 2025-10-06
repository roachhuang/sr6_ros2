# SR6 ROS2 Robot Arm Package

A ROS2 package for controlling a 6-DOF robot arm with hardware interface, simulation, and MoveIt integration.

## Quick Setup

```bash
echo $ROS_DISTRO
cd ~/ros2_ws/src
git clone https://github.com/roachhuang/sr6_ros2.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash
```

## Prerequisites

### USB Port Permissions
Add your user to the `dialout` group for hardware communication:
```bash
sudo usermod -aG dialout $USER
```
**Note: Login and logout required after this command.**

### Required Packages
```bash
sudo apt-get update && sudo apt-get install -y \
    libboost-all-dev \
    ros-$ROS_DISTRO-hardware-interface \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-tf-transformations \
    ros-$ROS_DISTRO-gz* \
    ros-$ROS_DISTRO-pal-statistics \
    ros-$ROS_DISTRO-urdf-tutorial

sudo apt-get install -y ros-$ROS_DISTRO-moveit-* --no-install-recommends
```

### UDEV Rules
```bash
ros2 run open_manipulator_bringup om_create_udev_rules
```

## Usage

### Launch Real Robot
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch bringup real_robot.launch.py
```

### Launch Simulation
```bash
ros2 launch bringup sim_robot.launch.py
```

### Control Commands

#### Check Available Controllers
```bash
ros2 control list_controllers
```
Expected output:
```
joint_state_broadcaster[active]
arm_controller[active]
```

#### Send Joint Commands

**For Joint Trajectory Controller:**
```bash
ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
  points: [
    {
      positions: [0.0, 0.5, 1.0, 0.0, -0.5, 0.0],
      time_from_start: {sec: 2, nanosec: 0}
    }
  ]
}"
```

**For Position Controller:**
```bash
ros2 topic pub /arm_controller/commands std_msgs/msg/Float64MultiArray "{
  data: [0.0, 0.5, 1.0, 0.0, -0.5, 0.0]
}"
```

#### Monitor Joint States
```bash
ros2 topic echo /joint_states
```

## Development

### Build Single Package
```bash
cd ~/ros2_ws
colcon build --packages-select smallrobot_alex --symlink-install
source install/setup.bash
```

### URDF Validation
```bash
cd urdf/
ros2 run xacro xacro smallrobot.urdf.xacro > smallrobot.urdf
check_urdf smallrobot.urdf
```

### Visualization
```bash
ros2 launch urdf_tutorial display.launch.py model:=/home/roach/ros2_ws/src/smallrobot/urdf/smallrobot.urdf
```

## Troubleshooting

### Hardware Connection Issues
```bash
# Check USB devices
ls /dev/ttyUSB*

# Test serial communication
sudo cat /dev/ttyUSB0

# Add user to dialout group (if not done)
sudo usermod -a -G dialout $USER
```

### Build Issues
```bash
# Clean build cache
colcon build --packages-select robotarm_controller --cmake-clean-cache

# Full clean build
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --symlink-install
```

### Hardware Interface Debugging
```bash
# View hardware interface logs only
ros2 launch bringup real_robot.launch.py | grep 'RobotArmInterface'
```

## Package Structure

- `bringup/` - Launch files and world configurations
- `robotarm_controller/` - Hardware interface implementation
- `smallrobot_description/` - URDF, meshes, and visualization
- `smallrobot_moveit/` - MoveIt configuration
- `smallrobot_firmware/` - Arduino firmware
- `smallrobot_remote/` - Remote control interface
- `my_arm_rl/` - Reinforcement learning components

## Notes

- Arduino processing time must be faster than hardware interface send interval
- Hardware interface must not flood Arduino with commands
- Run `sudo cat /dev/ttyUSB0` before launching for robot movement
- Always source workspace after building: `source install/setup.bash`