#!/bin/bash
set -eo pipefail

cleanup() {
    echo "Cleaning up..."
    sleep 5.0
    pkill -9 -f "ros2|gazebo|gz|nav2|amcl|bt_navigator|nav_to_pose|rviz2|assisted_teleop|cmd_vel_relay|robot_state_publisher|joint_state_publisher|move_to_free|mqtt|autodock|cliff_detection|moveit|move_group|basic_navigator"
}

# Set up cleanup trap
trap 'cleanup' SIGINT SIGTERM

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"

source /opt/ros/jazzy/setup.bash
if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
    source "${WORKSPACE_DIR}/install/setup.bash"
fi
set -u

if ! ros2 pkg prefix bringup >/dev/null 2>&1; then
    echo "Package 'bringup' is not available yet. Building the SR6 workspace..."
    (
        cd "${WORKSPACE_DIR}"
        colcon build --base-paths "${WORKSPACE_DIR}/src/sr6_ros2" --packages-up-to bringup
    )
    source "${WORKSPACE_DIR}/install/setup.bash"
fi

echo "Launching Gazebo simulation..."
ros2 launch bringup sim_robot.launch.py \
    is_sim:=true
