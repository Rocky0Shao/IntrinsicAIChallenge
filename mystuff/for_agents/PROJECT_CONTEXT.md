# Rocky's AI for Industry Challenge - Project Context

## Competition Overview
- **AI for Industry Challenge** by Intrinsic (with Open Robotics, Nvidia, Google DeepMind)
- **Prize Pool**: $180,000 ($100k 1st, $40k 2nd, $20k 3rd, $10k 4th/5th)
- **Goal**: Train an AI model to autonomously insert fiber optic cable connectors into ports on a task board using a UR5e robot arm
- **Qualification Phase**: 3 trials scored on model validity, trajectory quality, and insertion success
- **Deadline**: Qualification evaluation by May 15-27, 2026
- **Forum Thread**: https://discourse.openrobotics.org/t/rockys-open-source-build-thread-ai-for-industry-challenge/53155

## Rocky's Approach (Current Strategy)

### Pipeline Overview
1. **Cheatcode Teleop** (current step - pre-data gathering): Use ground-truth TF transforms to automatically teleoperate the robot to insert plugs -> generates demonstration data
2. **Record Training Data**: Use LeRobot's recording system to capture observations (3 cameras, joint states, force/torque) and actions during cheatcode teleop
3. **Train ACT Model**: Use LeRobot's ACT (Action Chunking with Transformers) model via Google Colab notebook
4. **Deploy**: Package trained model as aic_model ROS2 node in Docker container for submission

### Current Status
- **Phase**: Pre-data gathering - improving cheatcode teleop reliability
- **Main Problem (previously)**: Force rampdown was too aggressive (older thresholding caused stalling around ~10N during INSERT)
- **Current Fix**: Raised force rampdown thresholds and simplified insertion behavior to mirror official push-through strategy, while keeping force safety holds
- **State Machine**: Simplified to INIT → APPROACH → ALIGN → INSERT → DONE (SEARCH/RECOVERY removed)
- HuggingFace repo: rockyshao22/Intrinsic_AI

## Repository Structure

### Key Directories
```
/home/rocky/ws_aic/src/aic/
├── mystuff/                          # Rocky's custom code
│   ├── custom_commands/              # Shell scripts for common operations
│   │   ├── teleop.sh                 # Run cheatcode teleop (trial 1/2/3)
│   │   ├── record_multiple_data.sh   # Record training data (trial 1/2/3)
│   │   ├── record_training_data.sh   # Single recording command
│   │   ├── start_scoring.sh          # Start eval engine (trial 1/2/3)
│   │   ├── start_docker.sh           # Enter distrobox container
│   │   ├── display_wrench.sh         # Monitor force/torque readings
│   │   └── pull_from_competiton.sh   # Git pull upstream
│   ├── for_agents/                   # Context files for AI agents
│   ├── lerobot_record/               # Custom LeRobot recording package
│   ├── test_cheatcode/               # Standalone CheatCode policy test
│   └── test_wave_node/               # Standalone WaveArm test
├── aic_utils/lerobot_robot_aic/      # LeRobot integration (MAIN EDITING TARGET)
│   └── lerobot_robot_aic/
│       ├── aic_teleop.py             # ★ THE MAIN FILE - all teleop implementations
│       ├── aic_robot_aic_controller.py  # Robot controller/driver
│       ├── aic_robot.py              # Joint names, camera configs
│       └── types.py                  # MotionUpdateActionDict, JointMotionUpdateActionDict
├── aic_example_policies/             # Official example policies (reference)
│   └── aic_example_policies/ros/
│       ├── CheatCode.py              # Official cheatcode (position-based, NOT velocity)
│       ├── WaveArm.py                # Dummy wave arm example
│       └── RunACT.py                 # ACT policy runner
├── docs/                             # Competition documentation
│   ├── scoring.md                    # ★ SCORING GUIDE
│   ├── qualification_phase.md        # Trial descriptions
│   ├── aic_controller.md             # Controller docs
│   └── ...
└── aic_controller/                   # Low-level robot controller (C++)
```

## Key File: aic_teleop.py (779 lines)

Located at: `aic_utils/lerobot_robot_aic/lerobot_robot_aic/aic_teleop.py`

Contains 4 teleoperator classes:
1. **AICKeyboardJointTeleop** - Joint-level keyboard control
2. **AICKeyboardEETeleop** - End-effector keyboard control
3. **AICSpaceMouseTeleop** - SpaceMouse 6-DOF control
4. **AICCheatCodeTeleop** (★ MAIN FOCUS) - Automated teleop using ground-truth TFs

### AICCheatCodeTeleop Architecture

**Config parameters** (AICCheatCodeTeleopConfig):
- kp_linear=1.0, ki_linear=0.2, max_integrator_windup=0.05
- kp_angular=2.0
- max_linear_vel=0.04, max_angular_vel=0.5
- force_rampdown_start=15.0, force_rampdown_full=19.0
- insertion_base_speed=0.008
- hover_height=0.03, approach_height=0.20
- insertion_depth=-0.015, insertion_dwell=2.0
- align_xy_tolerance=0.0005, align_angular_tolerance=0.03, align_timeout=8.0, align_min_dwell=2.0
- task_cable_name, task_plug_name, task_module_name, task_port_name (set per trial)

**State Machine**: INIT → APPROACH → ALIGN → INSERT → DONE (force-proportional slowdown in INSERT, hard safety hold at 19.5N)

**How it works**:
1. Looks up ground-truth TF frames for port and plug positions
2. Computes target gripper position = port_position + gripper-to-plug offset + z_offset
3. Computes target orientation using quaternion math
4. Uses PI velocity controller to drive towards target
5. During INSERT, applies simplified force handling: force rampdown linearly from 15N to 19N (speed 1.0 → 0.0), hard safety hold at 19.5N (zero velocity until force drops), and no SEARCH/wiggle or RECOVERY/retreat behavior
6. Transforms world-frame velocities to TCP-frame velocities

**Key difference from official CheatCode**:
- Official CheatCode uses POSITION targets (set_pose_target with MotionUpdate MODE_POSITION)
- Teleop CheatCode uses VELOCITY targets (MotionUpdate MODE_VELOCITY via LeRobot framework)
- This means the teleop version must output incremental velocity commands, not absolute poses
- During INSERT, force-proportional speed ramp is used (15N→19N maps to 100%→0% speed)
- Hard safety hold at 19.5N for 0.8s sustained
- Base insertion speed is 0.008 m/s (vs official's 0.01 m/s)
- INSERT linear velocity cap is max_linear_vel * 0.5
- Universal DONE detection monitors actual plug TF and can finish successfully from any phase

### Known Issues with Current Cheatcode Teleop
1. ✅ Frame-rate dependent descent - FIXED (uses dt-based time scaling)
2. ✅ No fine-alignment dwell - FIXED (ALIGN phase with align_min_dwell=2.0s and tighter tolerances)
3. ✅ XY overshoot during approach - FIXED (integrator zeroed during APPROACH phase)
4. ✅ Force rampdown tuned: 15N start, 19N full stop, base speed 0.008 m/s — allows robot to push through normal insertion friction (~10-13N)
5. Note: previous versions used force_rampdown_start=10N, which caused stalling during insertion at normal friction levels
6. Current behavior intentionally has NO SEARCH phase and NO recovery/retreat phase; insertion either pushes through with rampdown or holds safely on sustained high force
7. DONE detection uses actual plug TF position (universal check across phases) with 2s dwell confirmation

## Scoring System (Max 100 per trial, 3 trials)

### Tier 1: Model Validity (0-1 pts)
- Policy loads and sends valid commands

### Tier 2: Performance (up to 24 pts, penalties down to -36)
- **Trajectory smoothness**: 0-6 pts (lower jerk = higher score)
- **Task duration**: 0-12 pts (≤5s = 12pts, ≥60s = 0pts, linear interpolation)
- **Trajectory efficiency**: 0-6 pts (shorter path = higher score)
- **Force penalty**: 0 to -12 pts (>20N for >1s triggers full penalty)
- **Off-limit contact penalty**: 0 to -24 pts (any contact with enclosure/task board)

### Tier 3: Task Success (up to 75 pts)
- **Correct port insertion**: 75 pts
- **Wrong port insertion**: -12 pts
- **Partial insertion**: 38-50 pts (proportional to depth)
- **Proximity**: 0-25 pts (closer to port = higher score)

**Key insight**: Force penalty (-12) and off-limit contacts (-24) are the biggest risks. The cheatcode must avoid >20N for >1s at all costs.

## Trial Configurations

### Trial 1: SFP → NIC Card Mount 0
- cable_name=cable_0, plug_name=sfp_tip
- module_name=nic_card_mount_0, port_name=sfp_port_0
- Task board at (0.15, -0.2, 1.14), yaw=π

### Trial 2: SFP → NIC Card Mount 1
- cable_name=cable_0, plug_name=sfp_tip
- module_name=nic_card_mount_1, port_name=sfp_port_0
- Task board at (0.15, -0.2, 1.14), yaw=π

### Trial 3: SC → SC Port 1
- cable_name=cable_0, plug_name=sc_tip
- module_name=sc_port_1, port_name=sc_port_base
- cable_type=sfp_sc_cable_reversed
- Task board at (0.17, 0.0, 1.14), yaw=3.0

## How to Run

### Prerequisites
1. Start Docker: `bash mystuff/custom_commands/start_docker.sh`
2. Inside container: `/entrypoint.sh spawn_task_board:=true ... ground_truth:=true start_aic_engine:=false`
   - Or use: `bash mystuff/custom_commands/start_scoring.sh` (interactive trial selector)
3. Tare F/T sensor: `pixi run ros2 service call /aic_controller/tare_force_torque_sensor std_srvs/srv/Trigger`

### Teleop (testing cheatcode)
```bash
bash mystuff/custom_commands/teleop.sh  # Interactive trial selector
```

### Record Training Data
```bash
bash mystuff/custom_commands/record_multiple_data.sh  # Interactive trial selector
```

### Key Commands
```bash
# Reinstall after code changes
pixi reinstall ros-kilted-lerobot-robot-aic

# Monitor force/torque
bash mystuff/custom_commands/display_wrench.sh
```

## Technical Details

### Controller Interface
- Commands at ~10-30 Hz, controller interpolates to ~500 Hz
- Cartesian velocity mode: frame_id="gripper/tcp", trajectory_generation_mode=MODE_VELOCITY
- Stiffness: diag(85.0), Damping: diag(75.0)
- Controller resets tracking error if significant error persists (prevents accumulated error)

### Force/Torque Sensor
- Tare before each episode (NOT available during evaluation)
- Raw readings minus tare offset = tared force
- Force penalty threshold: 20N sustained for >1s
- Monitor via /observations topic (wrist_wrench field)

### TF Frames (ground truth, only during training)
- Port: `task_board/{module_name}/{port_name}_link`
- Plug tip: `{cable_name}/{plug_name}_link`
- Gripper: `gripper/tcp`
- Base: `base_link`

### Cameras
- 3 wrist cameras: left, center, right
- Resolution: 1152x1024, scaled to 0.25x for recording
- Topics: /left_camera/image, /center_camera/image, /right_camera/image

## Git History (Recent Changes)
```
c5324f1 small polishes
4efc434 added force feedback -- though not working
8b2811a Merge remote-tracking branch 'upstream/main'
3bc5916 change to cable_0 fix ground-truth
da26549 lerobot record & hugging face push working. more custom commands
19a0ac3 insertion worked
b6ae92d partially fix max. Vertical above port
10a667b Custom record working. Math is wrong
9d20cbd added custom packages
```

## Important Notes for Agents
1. After editing aic_teleop.py, run: `pixi reinstall ros-kilted-lerobot-robot-aic`
2. The teleop outputs VELOCITY commands (not position) - this is critical
3. Ground truth TFs are ONLY available during training (not during competition eval)
4. The ACT model will learn from the velocity-based demonstrations - smooth, consistent trajectories are crucial
5. Force penalty is the #1 issue to solve - every triggered penalty costs 12 points
6. The cable is flexible and its physics are non-trivial - gentle, slow insertion is better than fast
7. The official CheatCode example in aic_example_policies/ scores 60pts insertion on all 3 trials with no force penalty - use it as reference

---

