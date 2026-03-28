#!/usr/bin/env bash
set -euo pipefail

# Real gripper test for the official StarAI ROS2 stack.
#
# Before running this script, start the official nodes in separate terminals:
#
# 1. ros2 launch cello_moveit_config driver.launch.py
# 2. ros2 launch cello_moveit_config actual_robot_demo.launch.py
# 3. ros2 launch cello_moveit_config moveit_write_read.launch.py
#
# Then run:
#   bash test/test_real_gripper.sh
# or:
#   bash test/test_real_gripper.sh open
#   bash test/test_real_gripper.sh close

COMMAND="${1:-toggle}"
DELAY_SECONDS="${DELAY_SECONDS:-2}"

set +u
source /opt/ros/humble/setup.bash
source /home/misca/starai_ws/install/setup.bash
set -u

publish_gripper() {
  local cmd="$1"
  echo "[gripper] send command: ${cmd}"
  ros2 topic pub --once /gripper_command_topic robo_interfaces/msg/GripperCommand "{command: ${cmd}}"
}

case "${COMMAND}" in
  open)
    publish_gripper open
    ;;
  close)
    publish_gripper close
    ;;
  toggle)
    publish_gripper open
    sleep "${DELAY_SECONDS}"
    publish_gripper close
    ;;
  *)
    echo "Usage: bash test/test_real_gripper.sh [open|close|toggle]"
    exit 1
    ;;
esac
