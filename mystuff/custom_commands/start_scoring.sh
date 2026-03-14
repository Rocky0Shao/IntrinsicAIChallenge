#/entrypoint.sh ground_truth:=true start_aic_engine:=true
#!/bin/bash

# Prompt the user for the trial number
read -p "Which trial do you want to run? (1, 2, or 3): " trial_num

case $trial_num in
  1)
    echo "Starting Engine for Trial 1..."

    pixi run /entrypoint.sh spawn_task_board:=true \
      task_board_x:=0.15 task_board_y:=-0.2 task_board_z:=1.14 \
      task_board_roll:=0.0 task_board_pitch:=0.0 task_board_yaw:=3.1415 \
      nic_card_mount_0_present:=true nic_card_mount_0_translation:=0.036 \
      sc_port_0_present:=true sc_port_0_translation:=0.042 \
      lc_mount_rail_0_present:=true lc_mount_rail_0_translation:=0.02 \
      sfp_mount_rail_0_present:=true sfp_mount_rail_0_translation:=0.03 \
      sc_mount_rail_0_present:=true sc_mount_rail_0_translation:=-0.02 \
      lc_mount_rail_1_present:=true lc_mount_rail_1_translation:=-0.01 \
      spawn_cable:=true cable_type:=sfp_sc_cable attach_cable_to_gripper:=true \
      ground_truth:=true start_aic_engine:=false
    ;;
  2)
    echo "Starting Engine for Trial 2..."

    pixi run /entrypoint.sh spawn_task_board:=true \
      task_board_x:=0.15 task_board_y:=-0.2 task_board_z:=1.14 \
      task_board_roll:=0.0 task_board_pitch:=0.0 task_board_yaw:=3.1415 \
      nic_card_mount_1_present:=true nic_card_mount_1_translation:=0.036 \
      sc_port_0_present:=true sc_port_0_translation:=0.042 \
      lc_mount_rail_0_present:=true lc_mount_rail_0_translation:=0.02 \
      sfp_mount_rail_0_present:=true sfp_mount_rail_0_translation:=0.03 \
      sc_mount_rail_0_present:=true sc_mount_rail_0_translation:=-0.02 \
      lc_mount_rail_1_present:=true lc_mount_rail_1_translation:=-0.01 \
      spawn_cable:=true cable_type:=sfp_sc_cable attach_cable_to_gripper:=true \
      ground_truth:=true start_aic_engine:=false
    ;;
  3)
    echo "Starting Engine for Trial 3..."

    pixi run /entrypoint.sh spawn_task_board:=true \
      task_board_x:=0.17 task_board_y:=0.0 task_board_z:=1.14 \
      task_board_roll:=0.0 task_board_pitch:=0.0 task_board_yaw:=3.0 \
      sc_port_1_present:=true sc_port_1_translation:=-0.055 \
      sfp_mount_rail_0_present:=true sfp_mount_rail_0_translation:=0.05 \
      sc_mount_rail_0_present:=true sc_mount_rail_0_translation:=-0.03 \
      lc_mount_rail_1_present:=true lc_mount_rail_1_translation:=0.04 \
      spawn_cable:=true cable_type:=sfp_sc_cable_reversed attach_cable_to_gripper:=true \
      ground_truth:=true start_aic_engine:=false
    ;;
  *)
    echo "Invalid choice. Please run the script again and enter 1, 2, or 3."
    exit 1
    ;;
esac