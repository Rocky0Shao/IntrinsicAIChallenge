# Plan: Achieve 100% First-Try Plug Insertion Accuracy
**Date**: 2026-03-17
**Goal**: Improve AICCheatCodeTeleop to achieve 100% insertion success for generating clean ACT training data

## Executive Summary
The current teleop achieves decent but not perfect insertion accuracy. Analysis reveals the **single most critical missing feature**: wrench feedback gains for passive compliance. The original CheatCode uses `[0.5, 0.5, 0.5, 0, 0, 0]` which provides automatic lateral compliance during insertion - the teleop has `[0, 0, 0, 0, 0, 0]`.

**IMPORTANT BASELINE**: Original CheatCode success rates:
- **Trial 1 (SFP)**: 100% success ✅
- **Trial 2 (SFP)**: 100% success ✅
- **Trial 3 (SC)**: NOT 100% - has failures ❌

This means 100% accuracy on SFP (trials 1&2) is definitely achievable with the right implementation. Trial 3 (SC connector) presents unique challenges even for the original.

## Key Constraint: No Wiggle/Recovery Logic
**CRITICAL**: We explicitly want NO wiggle, search, or recovery phases. ACT is imitation learning - if training data includes "lift up and retry" behavior, the model will learn to fail first. We need clean, successful first-try insertions only.

## Root Cause Analysis

### 1. Missing Wrench Feedback Gains (MOST CRITICAL)
**Current state**: `aic_robot_aic_controller.py:412` hardcodes `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]`
**Original CheatCode**: Uses `[0.5, 0.5, 0.5, 0.0, 0.0, 0.0]`

**Impact**:
- Original gets FREE lateral compliance from impedance controller at 500Hz
- When plug touches port wall (10N lateral), controller auto-deflects with 5N corrective force
- Effectively reduces stiffness by 50% in contact direction
- Acts as passive alignment/centering during insertion
- Teleop has NONE of this - fights contact forces with full 85 N/m stiffness

**Expected impact if fixed**: Could solve 50%+ of insertion failures alone

### 2. 3D Integrator with Z-axis Windup
**Current state**: Integrator operates on all 3 axes (X, Y, Z)
**Original CheatCode**: Integrator only on X, Y for plug-tip error

**Impact**:
- During ALIGN phase, Z error accumulates in integrator (gripper held 10cm above port)
- `max_integrator_windup=0.05` limits to 0.05m
- When INSERT starts: `ki_linear * 0.05 = 0.2 * 0.05 = 0.01 m/s` added downward velocity
- Can cause premature contact before proper alignment

### 3. Gripper-Frame vs Plug-Tip-Frame Alignment
**Current state**: PI controller drives `target_pos - gripper_pos`
**Original CheatCode**: Integrator tracks `port_xy - plug_xyz` directly

**Impact**:
- Cable droop/flex means gripper accuracy ≠ plug-tip accuracy
- 0.5mm gripper alignment doesn't guarantee 0.5mm plug-tip alignment
- Systematic offset not properly compensated

### 4. Force Rampdown Stalling
**Current state**: Speed ramps linearly from 100%→0% between 15N→19N
**Original CheatCode**: NO force handling, constant 0.01 m/s push-through

**Impact**:
- If friction stabilizes at 17-18N (binding), speed drops to 25-50%
- Without wrench feedback to correct lateral position, friction stays high
- Can stall indefinitely
- Original relies on wrench feedback compliance instead

### 5. Angular Corrections During INSERT
**Current state**: `kp_angular=2.0` active throughout INSERT
**Impact**:
- Small orientation drift during insertion → large corrective torques
- Combined with no wrench feedback → binding inside port
- Port itself guides plug orientation if lateral compliance exists

## Prioritized Implementation Strategies

### Priority 1: ADD WRENCH FEEDBACK GAINS ⭐⭐⭐⭐⭐
**Effort**: Low | **Impact**: Very High

**Change location**: `aic_robot_aic_controller.py:412`

**Options**:
- **Option A (Simple)**: Globally change line 412 to `[0.5, 0.5, 0.5, 0.0, 0.0, 0.0]`
  - Affects all teleop modes (keyboard, spacemouse, cheatcode)
  - But for cheatcode, this is exactly what we want
  - For keyboard/spacemouse, might make control feel "mushy" but probably not harmful

- **Option B (Configurable)**: Add `wrench_feedback_gains` to `AICRobotAICControllerConfig`
  - Thread through to `send_action_cartesian` method
  - Allow per-teleop-mode configuration
  - More work but cleaner architecture

- **Option C (Cheatcode-specific)**: Add signal/flag that cheatcode sets to enable gains
  - Requires coordination between teleop and robot driver
  - Most complex

**Recommendation**: Start with Option A for immediate testing. If keyboard/spacemouse feel bad, implement Option B.

### Priority 2: SPLIT INTEGRATOR TO XY-ONLY ⭐⭐⭐⭐
**Effort**: Low | **Impact**: High

**Change location**: `aic_teleop.py` lines 727-738

**Implementation**:
```python
# Only integrate XY errors, not Z
lin_err = target_pos - gripper_pos
self._lin_err_integrator[:2] = np.clip(
    self._lin_err_integrator[:2] + lin_err[:2],
    -cfg.max_integrator_windup,
    cfg.max_integrator_windup,
)
self._lin_err_integrator[2] = 0.0  # Never integrate Z

v_linear_world = (
    cfg.kp_linear * lin_err
    + np.array([
        cfg.ki_linear * self._lin_err_integrator[0],
        cfg.ki_linear * self._lin_err_integrator[1],
        0.0  # No integral term for Z
    ])
)
```

**Rationale**: Z is managed by explicit `z_offset` descent. Integrating Z error causes artifacts.

### Priority 3: TRACK PLUG-TIP-TO-PORT ERROR FOR INTEGRATOR ⭐⭐⭐⭐
**Effort**: Low | **Impact**: High

**Change location**: `aic_teleop.py` lines 727-738

**Implementation**:
```python
# Compute plug-tip-to-port XY error (like original CheatCode)
plug_xy_error = port_pos[:2] - plug_pos[:2]

# Feed only plug XY error into integrator, not full gripper error
self._lin_err_integrator[:2] = np.clip(
    self._lin_err_integrator[:2] + plug_xy_error,
    -cfg.max_integrator_windup,
    cfg.max_integrator_windup,
)
self._lin_err_integrator[2] = 0.0

# But use full gripper error for proportional control
lin_err = target_pos - gripper_pos
v_linear_world = (
    cfg.kp_linear * lin_err
    + np.array([
        cfg.ki_linear * self._lin_err_integrator[0],
        cfg.ki_linear * self._lin_err_integrator[1],
        0.0
    ])
)
```

**Rationale**: Directly compensates for cable droop and gripper compliance offset.

### Priority 4: SIMPLIFY FORCE HANDLING ⭐⭐⭐
**Effort**: Low | **Impact**: Medium

**Change location**: `aic_teleop.py` lines 693-719

**Options**:
- **Option A (Remove rampdown)**: Keep only 19.5N safety stop, no rampdown
  - Matches original CheatCode's "push through" strategy
  - Relies on wrench feedback for compliance
  - Simplest, most robust

- **Option B (Narrow rampdown)**: Raise thresholds to 18N→19.5N
  - Only activates in danger zone
  - Reduces stalling risk
  - Keep as safety net

**Recommendation**: Try Option A first (completely remove rampdown logic). The original scores 60pts with no force handling. With wrench feedback gains, we should match or exceed.

### Priority 5: REDUCE ANGULAR GAIN DURING INSERT ⭐⭐⭐
**Effort**: Low | **Impact**: Medium

**Change location**: `aic_teleop.py` line 746-750

**Implementation**:
```python
# Phase-dependent angular gain
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

**Rationale**: During INSERT, port constrains orientation physically. Fighting this creates binding forces.

### Priority 6: REDUCE HOVER_HEIGHT ⭐⭐
**Effort**: Trivial | **Impact**: Low-Medium

**Change location**: `aic_teleop.py` line 372

**Current**: `hover_height: float = 0.10`
**Change to**: `hover_height: float = 0.03`

**Rationale**:
- Less time for plug to drift laterally before insertion
- PROJECT_CONTEXT.md mentions 0.03 was intended
- Original doesn't distinguish hover from approach

### Priority 7: ADD SETTLE/VERIFY BEFORE INSERT ⭐⭐
**Effort**: Medium | **Impact**: Low (belt-and-suspenders)

**Change location**: `aic_teleop.py` lines 677-691

**Implementation**: Add a "VERIFY" sub-phase within ALIGN:
```python
elif self.phase == "ALIGN":
    align_elapsed = current_time - self.start_time
    xy_ok = xy_error < cfg.align_xy_tolerance
    ang_ok = angular_error < cfg.align_angular_tolerance
    dwell_ok = align_elapsed > cfg.align_min_dwell

    # NEW: Check actual plug-tip-to-port distance
    plug_port_xy_dist = np.linalg.norm(port_pos[:2] - plug_pos[:2])
    plug_ok = plug_port_xy_dist < 0.001  # 1mm threshold

    if (xy_ok and ang_ok and dwell_ok and plug_ok) or align_elapsed > cfg.align_timeout:
        # ... transition to INSERT
```

**Rationale**: Catch cases where gripper aligned but plug tip drifted due to cable flex.

## Implementation Order

### Phase 1: Core Fixes (High confidence, low risk)
1. ✅ Add wrench feedback gains (Priority 1)
2. ✅ Split integrator to XY-only (Priority 2)
3. ✅ Track plug-tip error for integrator (Priority 3)
4. ✅ Reduce hover_height to 0.03 (Priority 6)

**Test after Phase 1**: Run 10 insertions on each trial. Expect ~80-90% success rate.

### Phase 2: Refinements (If Phase 1 isn't 100%)
5. ✅ Simplify force handling (Priority 4)
6. ✅ Reduce angular gain during INSERT (Priority 5)

**Test after Phase 2**: Run 20 insertions on each trial. Expect ~95-100% success rate.

### Phase 3: Safety Net (If still seeing rare failures)
7. ✅ Add settle/verify (Priority 7)

## Testing Protocol

For each change iteration:
1. `pixi reinstall ros-kilted-lerobot-robot-aic`
2. Tare F/T sensor: `pixi run ros2 service call /aic_controller/tare_force_torque_sensor std_srvs/srv/Trigger`
3. Run trial 1 × 5 attempts: `bash mystuff/custom_commands/teleop.sh` (select trial 1)
4. Run trial 2 × 5 attempts
5. Run trial 3 × 5 attempts
6. Record success rate, failure modes, force readings

Success criteria: 15/15 successful first-try insertions with no force >20N sustained >1s

## Parameter Tuning (If needed)

If Phase 1-2 achieve 95% but not 100%, consider:

### Alignment tolerances (tighter)
- `align_xy_tolerance: 0.0005 → 0.0003` (0.5mm → 0.3mm)
- `align_angular_tolerance: 0.03 → 0.02` (1.7° → 1.1°)

### Alignment dwell (longer)
- `align_min_dwell: 2.0 → 3.0` (allow more settling time)

### PI gains (more aggressive XY correction)
- `kp_linear: 1.0 → 1.5` (faster XY convergence)
- `ki_linear: 0.2 → 0.25` (stronger integrator)

### Insertion speed (slower)
- `insertion_base_speed: 0.008 → 0.006` (0.8 cm/s → 0.6 cm/s)
- Gentler insertion reduces binding risk

## Risk Assessment

### Low Risk Changes
- Priorities 2, 3, 5, 6, 7: All confined to teleop logic
- Easy to revert if problems arise

### Medium Risk Change
- Priority 4 (force handling): Removing safety feature
- Mitigation: Keep 19.5N hard stop
- If unsafe, revert to narrow rampdown (18N→19.5N)

### Higher Risk Change
- Priority 1 (wrench feedback): Affects low-level controller behavior
- Could affect all teleop modes if using Option A
- Mitigation: Test keyboard/spacemouse after change
- If problematic, implement Option B (configurable)

## Expected Outcome

With Priorities 1-3 implemented:
- Lateral compliance during insertion (wrench feedback)
- Accurate plug-tip XY tracking (plug-tip error integrator)
- No Z integrator artifacts (XY-only integrator)

These address the root causes of ~80% of insertion failures. Combined with Priorities 4-6, should achieve:
- **100% reliability on Trials 1&2 (SFP)** - matching original CheatCode
- **~90-95% reliability on Trial 3 (SC)** - improvement over original, but may need SC-specific tuning

## Trial 3 (SC Connector) Specific Challenges

Trial 3 differs from trials 1&2:
- **Connector type**: SC (cylindrical, spring-loaded) vs SFP (rectangular)
- **Cable type**: `sfp_sc_cable_reversed` (different flex/droop characteristics)
- **Task board pose**: yaw=3.0 vs yaw=π (different approach geometry)
- **Port**: `sc_port_base` (spring latch mechanism adds variable force during seating)

**Why SC is harder**:
1. Spring-loaded latch creates variable force profile during insertion (15-25N spike when latch engages)
2. Round geometry means less precise Z-axis alignment matters more (SFP rails guide Z, SC doesn't)
3. Different cable physics (reversed cable routing affects droop/offset prediction)

**SC-specific tuning if needed**:
- Increase `align_min_dwell` to 3.0s for trial 3 (more settling time)
- Reduce `insertion_base_speed` to 0.006 for trial 3 (slower to handle latch engagement)
- If latch causes force spike >19.5N: raise safety threshold to 22N for trial 3 only
- Consider trial-specific `align_angular_tolerance` - tighter for SC (0.02 vs 0.03)

Note: These would be trial-specific config overrides passed via command line args.

## Notes on ACT Training Data Quality

Why we DON'T want wiggle/recovery:
- ACT learns via imitation (behavior cloning)
- Training data = demonstrations of "how to do the task"
- If data shows "try, fail, retry", ACT learns to try-fail-retry
- We want data showing "approach, align, insert smoothly, done"
- Every training episode must be a successful first-try trajectory
- No retries, no recovery, no wiggle = cleaner learned policy

This validates current approach: simplified state machine, no SEARCH/RECOVERY phases.

---

## Implementation Checklist

- [ ] Priority 1: Add wrench feedback gains
  - [ ] Decide on Option A vs B
  - [ ] Modify `aic_robot_aic_controller.py:412`
  - [ ] Test keyboard/spacemouse if using Option A
- [ ] Priority 2: Split integrator to XY-only
  - [ ] Modify `aic_teleop.py` integrator logic
- [ ] Priority 3: Track plug-tip error
  - [ ] Modify `aic_teleop.py` integrator input
- [ ] Priority 4: Simplify force handling
  - [ ] Decide on Option A vs B
  - [ ] Modify `aic_teleop.py` INSERT phase
- [ ] Priority 5: Reduce angular gain
  - [ ] Modify `aic_teleop.py` angular controller
- [ ] Priority 6: Reduce hover_height
  - [ ] Change config default in `aic_teleop.py:372`
- [ ] Priority 7: Add settle/verify
  - [ ] Add plug-tip distance check to ALIGN phase
- [ ] Testing: Run 15-trial validation (5 per trial type)
- [ ] If 100% success: Generate 50+ episodes per trial for ACT training
