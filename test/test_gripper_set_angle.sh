#!/usr/bin/env bash
set -euo pipefail

# Direct gripper debug script.
# This bypasses arm_moveit_write and publishes SetAngle straight to robo_driver.
#
# Before running:
# 1. source /opt/ros/humble/setup.zsh
# 2. source ~/starai_ws/install/setup.zsh
# 3. ros2 launch cello_moveit_config driver.launch.py
#
# Usage:
#   bash test/test_gripper_set_angle.sh open
#   bash test/test_gripper_set_angle.sh close
#   bash test/test_gripper_set_angle.sh

COMMAND="${1:-toggle}"
DELAY_SECONDS="${DELAY_SECONDS:-2}"

set +u
source /opt/ros/humble/setup.bash
source /home/misca/starai_ws/install/setup.bash
set -u

publish_set_angle() {
  local angle="$1"
  echo "[set_angle] servo=6 angle=${angle}"
  ros2 topic pub --once /set_angle_topic robo_interfaces/msg/SetAngle \
    "{servo_id: [6], target_angle: [${angle}], time: [1500]}"
}

case "${COMMAND}" in
  open)
    publish_set_angle 100.0
    ;;
  close)
    publish_set_angle 0.0
    ;;
  toggle)
    publish_set_angle 100.0
    sleep "${DELAY_SECONDS}"
    publish_set_angle 0.0
    ;;
  *)
    echo "Usage: bash test/test_gripper_set_angle.sh [open|close|toggle]"
    exit 1
    ;;
esac
