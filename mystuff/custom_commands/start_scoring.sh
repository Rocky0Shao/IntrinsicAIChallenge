#/entrypoint.sh ground_truth:=true start_aic_engine:=true
/entrypoint.sh spawn_task_board:=true \
    task_board_x:=0.15 task_board_y:=-0.2 task_board_z:=1.14 \
    task_board_roll:=0.0 task_board_pitch:=0.0 task_board_yaw:=3.1415 \
    sfp_mount_rail_0_present:=true sfp_mount_rail_0_translation:=0.03 \
    sc_mount_rail_0_present:=true sc_mount_rail_0_translation:=-0.02 \
    nic_card_mount_0_present:=true nic_card_mount_0_translation:=0.036 \
    sc_port_0_present:=true sc_port_0_translation:=0.042 \
    spawn_cable:=true cable_type:=sfp_sc_cable attach_cable_to_gripper:=true \
    ground_truth:=true start_aic_engine:=false