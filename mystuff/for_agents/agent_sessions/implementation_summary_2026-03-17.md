# Implementation Summary - CheatCode Teleop Improvements
**Date**: 2026-03-17
**Status**: ✅ All Phase 1 & Phase 2 fixes implemented

## Changes Made

### File 1: `aic_robot_aic_controller.py`

**Line 412**: Added wrench feedback gains for passive compliance
```python
# BEFORE:
msg.wrench_feedback_gains_at_tip = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# AFTER:
msg.wrench_feedback_gains_at_tip = [0.5, 0.5, 0.5, 0.0, 0.0, 0.0]
```

**Impact**:
- Provides automatic lateral compliance during insertion
- When plug contacts port wall (10N lateral), controller auto-deflects with 5N corrective force
- Matches original CheatCode's passive centering mechanism
- **Expected to solve 50%+ of insertion failures**

---

### File 2: `aic_teleop.py` (AICCheatCodeTeleop)

#### Change 1: Updated Config Parameters (Lines 372-374)

```python
# BEFORE:
hover_height: float = 0.10
insertion_base_speed: float = 0.008

# AFTER:
hover_height: float = 0.03  # Reduced - less drift time before insertion
insertion_base_speed: float = 0.012  # Increased - faster, more visible descent
```

**Impact**:
- Plug starts INSERT closer to port (3cm vs 10cm) → less lateral drift
- Descent rate: 0.012 m/s = 12 mm/s (vs 8 mm/s before) → more visible motion
- At 30Hz: moves 0.4mm per iteration vs 0.26mm before

---

#### Change 2: Removed Force Rampdown (Lines 693-724)

**BEFORE**: Force-proportional descent with linear rampdown 15N→19N
```python
speed_mult = self._compute_force_speed_multiplier()  # Returns 0.0-1.0
descent = cfg.insertion_base_speed * speed_mult * dt
```

**AFTER**: Constant-speed descent (matches original CheatCode)
```python
descent = cfg.insertion_base_speed * dt  # No speed_mult
```

**Impact**:
- Eliminates stalling at 15-18N friction levels
- Relies on wrench feedback gains for contact force compliance
- Matches original's "push through" strategy
- Kept 19.5N safety hold as emergency stop

**Added Debug Logging**:
```python
print(
    f"[INSERT] z_offset={self.z_offset:.4f} | descent_rate={descent/dt:.4f}m/s | "
    f"force={self._latest_force_mag:.1f}N | dt={dt:.3f}s"
)
```

---

#### Change 3: XY-Only Integrator with Plug-Tip Tracking (Lines 730-765)

**BEFORE**: 3D integrator on gripper error
```python
lin_err = target_pos - gripper_pos
self._lin_err_integrator = np.clip(
    self._lin_err_integrator + lin_err,  # All 3 axes
    -cfg.max_integrator_windup,
    cfg.max_integrator_windup,
)
v_linear_world = (
    cfg.kp_linear * lin_err
    + cfg.ki_linear * self._lin_err_integrator
)
```

**AFTER**: XY-only integrator on plug-tip error
```python
lin_err = target_pos - gripper_pos

# XY-only integrator using plug-tip-to-port error (like original CheatCode)
plug_xy_error = port_pos[:2] - plug_pos[:2]
self._lin_err_integrator[:2] = np.clip(
    self._lin_err_integrator[:2] + plug_xy_error,
    -cfg.max_integrator_windup,
    cfg.max_integrator_windup,
)
# Never integrate Z - it's managed by explicit z_offset descent
self._lin_err_integrator[2] = 0.0

# Proportional term uses full gripper error, integral uses plug-tip XY error
v_linear_world = cfg.kp_linear * lin_err + np.array([
    cfg.ki_linear * self._lin_err_integrator[0],
    cfg.ki_linear * self._lin_err_integrator[1],
    0.0,  # No integral term for Z
])
```

**Impact**:
- Directly tracks plug-tip-to-port XY distance (not gripper-to-target)
- Compensates for cable droop and gripper compliance offset
- Z integrator can't wind up during ALIGN phase (was causing velocity spikes)
- Matches original CheatCode's integrator architecture

---

#### Change 4: Reduced Angular Gain During INSERT (Lines 750-763)

**BEFORE**: Constant `kp_angular = 2.0` throughout
```python
v_angular_world = np.clip(
    cfg.kp_angular * r_error.as_rotvec(),
    -cfg.max_angular_vel,
    cfg.max_angular_vel,
)
```

**AFTER**: Reduced gain during INSERT
```python
# Reduce angular gain during INSERT to prevent binding torques
if self.phase == "INSERT":
    kp_angular_effective = cfg.kp_angular * 0.25  # 2.0 → 0.5
else:
    kp_angular_effective = cfg.kp_angular

v_angular_world = np.clip(
    kp_angular_effective * r_error.as_rotvec(),
    -cfg.max_angular_vel,
    cfg.max_angular_vel,
)
```

**Impact**:
- During INSERT, port physically constrains plug orientation
- Lower gain prevents controller from fighting port geometry
- Reduces binding torques that cause high lateral forces
- Allows wrench feedback to passively guide orientation

---

#### Change 5: Enhanced ALIGN Phase Logging (Lines 671-690)

**Added periodic status logging**:
```python
plug_port_xy_dist = np.linalg.norm(port_pos[:2] - plug_pos[:2])
if int(align_elapsed * 10) % 5 == 0:  # Log every 0.5s
    print(
        f"[ALIGN] t={align_elapsed:.1f}s | gripper_xy_err={xy_error:.4f}m | "
        f"plug_xy_err={plug_port_xy_dist:.4f}m | ang_err={angular_error:.3f}rad | "
        f"integrator=[{self._lin_err_integrator[0]:.3f}, {self._lin_err_integrator[1]:.3f}]"
    )
```

**Impact**:
- User can see real-time alignment progress
- Shows both gripper error and actual plug-tip error
- Displays integrator values to diagnose XY correction behavior

---

## Expected Outcomes

### Trials 1 & 2 (SFP Connectors)
**Baseline**: 0% (plug hovers, doesn't descend)
**Expected after fixes**: 80-100% success rate

**Key improvements**:
1. Faster, constant-speed descent (12mm/s) → visible motion
2. Wrench feedback → automatic centering during insertion
3. XY-only integrator → no Z velocity spikes
4. Plug-tip tracking → accurate alignment despite cable droop

### Trial 3 (SC Connector)
**Baseline**: 0% (poor insertion precision)
**Expected after fixes**: 60-80% success rate

**Note**: Trial 3 may need additional tuning due to:
- Spring-loaded latch (variable force profile)
- Different cable routing (sfp_sc_cable_reversed)
- Round geometry (different alignment requirements)

If trial 3 still struggles, consider:
- Slower insertion speed (0.010 vs 0.012)
- Longer align dwell (3.0s vs 2.0s)
- Higher force threshold (22N vs 19.5N for latch engagement)

---

## Next Steps

### 1. Reinstall Package
```bash
pixi reinstall ros-kilted-lerobot-robot-aic
```

### 2. Test Protocol

**For each trial**:
1. Start sim with ground truth:
   ```bash
   bash mystuff/custom_commands/start_scoring.sh
   # Select trial 1/2/3
   ```

2. Tare F/T sensor:
   ```bash
   pixi run ros2 service call /aic_controller/tare_force_torque_sensor std_srvs/srv/Trigger
   ```

3. Run teleop:
   ```bash
   bash mystuff/custom_commands/teleop.sh
   # Select trial 1/2/3
   ```

4. Observe:
   - Does plug descend visibly during INSERT?
   - What's the force reading during insertion?
   - Does it complete successfully?
   - Any binding or lateral drift?

### 3. Expected Behavior

**APPROACH Phase**:
- Gripper moves smoothly toward port
- Stays ~20cm above port initially

**ALIGN Phase**:
- Gripper descends to 3cm above port (hover_height)
- Logs show xy_err and ang_err converging
- Integrator values should stabilize around ±0.01
- Should dwell ~2s at good alignment before transitioning

**INSERT Phase**:
- Plug descends at ~12mm/s (visible motion!)
- Logs show `z_offset` decreasing from 0.03 → -0.015
- Force should stay under 15N (normal friction 10-13N)
- With wrench feedback, plug should self-center if it touches port walls
- Descent should NOT stall

**DONE Phase**:
- Plug reaches insertion_depth (-0.015m below port)
- Dwells 2s to confirm
- All velocities zero

### 4. Debugging Failed Insertions

If insertion still fails:

**Symptom: Still not descending**
- Check logs for `z_offset` value - is it decreasing?
- Check `descent_rate` in logs - should be ~0.012 m/s
- Check force - is safety hold activating prematurely?
- Verify tare was applied (check tare offset values)

**Symptom: Descending but misses port**
- Check ALIGN logs - was alignment good before INSERT?
- Check `plug_xy_err` - should be <0.001m before INSERT
- Check integrator values - should be small
- Try increasing align_min_dwell to 3.0s

**Symptom: Binding/jamming during insertion**
- Check force readings - exceeding 15N consistently?
- This suggests wrench feedback isn't working or alignment was bad
- Verify wrench_feedback_gains change was applied (check controller logs)

**Symptom: High forces (>19.5N) triggering safety hold**
- Check if plug is misaligned before INSERT
- Try slower insertion speed (reduce to 0.008)
- For trial 3 only: might need higher threshold (22N) for latch

---

## Rollback Plan

If changes cause problems:

### Quick Rollback
```bash
git diff HEAD aic_utils/lerobot_robot_aic/lerobot_robot_aic/
git checkout HEAD -- aic_utils/lerobot_robot_aic/lerobot_robot_aic/
pixi reinstall ros-kilted-lerobot-robot-aic
```

### Selective Rollback

**If wrench feedback makes keyboard/spacemouse feel bad**:
- Revert `aic_robot_aic_controller.py:412` to `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]`
- Consider implementing configurable wrench feedback instead

**If descent is too fast/aggressive**:
- Reduce `insertion_base_speed` from 0.012 back to 0.008 or 0.010

**If alignment is less stable**:
- Increase `align_min_dwell` from 2.0 to 3.0 or 4.0

---

## Success Criteria

**Minimum Success (ready for data collection)**:
- ✅ Trial 1: ≥80% success rate (5/5 or 4/5 insertions)
- ✅ Trial 2: ≥80% success rate
- ✅ Trial 3: ≥60% success rate (3/5 insertions)
- ✅ No force penalties (no sustained >20N forces)

**Ideal Success (ready for high-quality training data)**:
- ✅ Trial 1: 100% (10/10 insertions)
- ✅ Trial 2: 100% (10/10 insertions)
- ✅ Trial 3: ≥80% (8/10 insertions)
- ✅ Smooth trajectories (low jerk)
- ✅ Consistent timing (15-30s per insertion)

Once at 100% on trials 1&2, collect 50+ episodes per trial for ACT training.

---

## Files Modified

1. `/home/rocky/ws_aic/src/aic/aic_utils/lerobot_robot_aic/lerobot_robot_aic/aic_robot_aic_controller.py`
   - Line 412: wrench_feedback_gains_at_tip

2. `/home/rocky/ws_aic/src/aic/aic_utils/lerobot_robot_aic/lerobot_robot_aic/aic_teleop.py`
   - Lines 372-374: Config parameters (hover_height, insertion_base_speed)
   - Lines 671-690: ALIGN phase logging
   - Lines 693-724: INSERT phase (removed rampdown, added logging)
   - Lines 730-765: PI controller (XY-only integrator, plug-tip tracking, angular gain reduction)
