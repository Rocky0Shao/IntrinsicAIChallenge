cd ~/ws_aic/src/aic
pixi run lerobot-record \
  --robot.type=aic_controller --robot.id=aic \
  --teleop.type=aic_cheatcode --teleop.id=aic \
  --robot.teleop_target_mode=cartesian --robot.teleop_frame_id=gripper/tcp \
  --dataset.repo_id=rockyshao22/Intrinsic_AI \
  --dataset.single_task="insert plug" \
  --dataset.push_to_hub=true \
  --dataset.private=true \
  --play_sounds=false \
  --display_data=true
