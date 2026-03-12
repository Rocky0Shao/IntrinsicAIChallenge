#!/bin/bash

# Prompt the user for the trial number
read -p "Which trial do you want to teleop? (1, 2, or 3): " trial_num

# Base teleop command using your repo ID and standard flags
BASE_CMD="pixi run lerobot-teleoperate \
  --robot.type=aic_controller --robot.id=aic \
  --teleop.type=aic_cheatcode --teleop.id=aic \
  --robot.teleop_target_mode=cartesian --robot.teleop_frame_id=gripper/tcp \
  --display_data=true"

# Execute the command with the correct trial-specific TF frames
case $trial_num in
  1)
    echo "Starting LeRobot teleop for Trial 1..."
    eval "$BASE_CMD \
      --teleop.task_cable_name=cable_0 \
      --teleop.task_plug_name=sfp_tip \
      --teleop.task_module_name=nic_card_mount_0 \
      --teleop.task_port_name=sfp_port_0"
    ;;
  2)
    echo "Starting LeRobot teleop for Trial 2..."
    eval "$BASE_CMD \
      --teleop.task_cable_name=cable_0 \
      --teleop.task_plug_name=sfp_tip \
      --teleop.task_module_name=nic_card_mount_1 \
      --teleop.task_port_name=sfp_port_0"
    ;;
  3)
    echo "Starting LeRobot teleop for Trial 3..."
    eval "$BASE_CMD \
      --teleop.task_cable_name=cable_1 \
      --teleop.task_plug_name=sc_tip \
      --teleop.task_module_name=sc_port_1 \
      --teleop.task_port_name=sc_port_base"
    ;;
  *)
    echo "Invalid choice. Please run the script again and enter 1, 2, or 3."
    exit 1
    ;;
esac