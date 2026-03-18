#
#  Copyright (C) 2026 Intrinsic Innovation LLC
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#

from dataclasses import dataclass, field
from threading import Thread
from typing import Any, cast

import pyspacemouse
import rclpy
from geometry_msgs.msg import Twist
from lerobot.teleoperators import Teleoperator, TeleoperatorConfig
from lerobot.teleoperators.keyboard import (
    KeyboardEndEffectorTeleop,
    KeyboardEndEffectorTeleopConfig,
)
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot_teleoperator_devices import KeyboardJointTeleop, KeyboardJointTeleopConfig
from rclpy.executors import SingleThreadedExecutor

from .aic_robot import arm_joint_names
from .types import JointMotionUpdateActionDict, MotionUpdateActionDict

from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
@TeleoperatorConfig.register_subclass("aic_keyboard_joint")
@dataclass
class AICKeyboardJointTeleopConfig(KeyboardJointTeleopConfig):
    arm_action_keys: list[str] = field(
        default_factory=lambda: [f"{x}" for x in arm_joint_names]
    )
    high_command_scaling: float = 0.05
    low_command_scaling: float = 0.02


class AICKeyboardJointTeleop(KeyboardJointTeleop):
    def __init__(self, config: AICKeyboardJointTeleopConfig):
        super().__init__(config)

        self.config = config
        self._low_scaling = config.low_command_scaling
        self._high_scaling = config.high_command_scaling
        self._current_scaling = self._high_scaling

        self.curr_joint_actions: JointMotionUpdateActionDict = {
            "shoulder_pan_joint": 0.0,
            "shoulder_lift_joint": 0.0,
            "elbow_joint": 0.0,
            "wrist_1_joint": 0.0,
            "wrist_2_joint": 0.0,
            "wrist_3_joint": 0.0,
        }

    @property
    def action_features(self) -> dict:
        return {"names": JointMotionUpdateActionDict.__annotations__}

    def _get_action_value(self, is_pressed: bool) -> float:
        return self._current_scaling if is_pressed else 0.0

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError()

        self._drain_pressed_keys()

        for key, is_pressed in self.current_pressed.items():

            if key == "u" and is_pressed:
                is_low_scaling = self._current_scaling == self._low_scaling
                self._current_scaling = (
                    self._high_scaling if is_low_scaling else self._low_scaling
                )
                print(f"Command scaling toggled to: {self._current_scaling}")
                continue

            val = self._get_action_value(is_pressed)

            if key == "q":
                self.curr_joint_actions["shoulder_pan_joint"] = val
            elif key == "a":
                self.curr_joint_actions["shoulder_pan_joint"] = -val
            elif key == "w":
                self.curr_joint_actions["shoulder_lift_joint"] = val
            elif key == "s":
                self.curr_joint_actions["shoulder_lift_joint"] = -val
            elif key == "e":
                self.curr_joint_actions["elbow_joint"] = val
            elif key == "d":
                self.curr_joint_actions["elbow_joint"] = -val
            elif key == "r":
                self.curr_joint_actions["wrist_1_joint"] = val
            elif key == "f":
                self.curr_joint_actions["wrist_1_joint"] = -val
            elif key == "t":
                self.curr_joint_actions["wrist_2_joint"] = val
            elif key == "g":
                self.curr_joint_actions["wrist_2_joint"] = -val
            elif key == "y":
                self.curr_joint_actions["wrist_3_joint"] = val
            elif key == "h":
                self.curr_joint_actions["wrist_3_joint"] = -val
            elif is_pressed:
                # If the key is pressed, add it to the misc_keys_queue
                # this will record key presses that are not part of the delta_x, delta_y, delta_z
                # this is useful for retrieving other events like interventions for RL, episode success, etc.
                self.misc_keys_queue.put(key)

        self.current_pressed.clear()

        return cast(dict, self.curr_joint_actions)


@TeleoperatorConfig.register_subclass("aic_keyboard_ee")
@dataclass(kw_only=True)
class AICKeyboardEETeleopConfig(KeyboardEndEffectorTeleopConfig):
    high_command_scaling: float = 0.1
    low_command_scaling: float = 0.02


class AICKeyboardEETeleop(KeyboardEndEffectorTeleop):
    def __init__(self, config: AICKeyboardEETeleopConfig):
        super().__init__(config)
        self.config = config

        self._high_scaling = config.high_command_scaling
        self._low_scaling = config.low_command_scaling
        self._current_scaling = self._high_scaling

        self._current_actions: MotionUpdateActionDict = {
            "linear.x": 0.0,
            "linear.y": 0.0,
            "linear.z": 0.0,
            "angular.x": 0.0,
            "angular.y": 0.0,
            "angular.z": 0.0,
        }

    @property
    def action_features(self) -> dict:
        return MotionUpdateActionDict.__annotations__

    def _get_action_value(self, is_pressed: bool) -> float:
        return self._current_scaling if is_pressed else 0.0

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError()

        self._drain_pressed_keys()

        for key, is_pressed in self.current_pressed.items():

            if key == "t" and is_pressed:
                is_low_speed = self._current_scaling == self._low_scaling
                self._current_scaling = (
                    self._high_scaling if is_low_speed else self._low_scaling
                )
                print(f"Command scaling toggled to: {self._current_scaling}")
                continue

            val = self._get_action_value(is_pressed)

            if key == "w":
                self._current_actions["linear.y"] = -val
            elif key == "s":
                self._current_actions["linear.y"] = val
            elif key == "a":
                self._current_actions["linear.x"] = -val
            elif key == "d":
                self._current_actions["linear.x"] = val
            elif key == "r":
                self._current_actions["linear.z"] = -val
            elif key == "f":
                self._current_actions["linear.z"] = val
            elif key == "W":
                self._current_actions["angular.x"] = val
            elif key == "S":
                self._current_actions["angular.x"] = -val
            elif key == "A":
                self._current_actions["angular.y"] = -val
            elif key == "D":
                self._current_actions["angular.y"] = val
            elif key == "q":
                self._current_actions["angular.z"] = -val
            elif key == "e":
                self._current_actions["angular.z"] = val
            elif is_pressed:
                # If the key is pressed, add it to the misc_keys_queue
                # this will record key presses that are not part of the delta_x, delta_y, delta_z
                # this is useful for retrieving other events like interventions for RL, episode success, etc.
                self.misc_keys_queue.put(key)

        self.current_pressed.clear()

        return cast(dict, self._current_actions)


@TeleoperatorConfig.register_subclass("aic_spacemouse")
@dataclass(kw_only=True)
class AICSpaceMouseTeleopConfig(TeleoperatorConfig):
    operator_position_front: bool = True
    device: str | None = None  # only needed for multiple space mice
    command_scaling: float = 0.1


class AICSpaceMouseTeleop(Teleoperator):
    def __init__(self, config: AICSpaceMouseTeleopConfig):
        super().__init__(config)
        self.config = config
        self._is_connected = False
        self._device: pyspacemouse.SpaceMouseDevice | None = None

        self._current_actions: MotionUpdateActionDict = {
            "linear.x": 0.0,
            "linear.y": 0.0,
            "linear.z": 0.0,
            "angular.x": 0.0,
            "angular.y": 0.0,
            "angular.z": 0.0,
        }

    @property
    def name(self) -> str:
        return "aic_spacemouse"

    @property
    def action_features(self) -> dict:
        return MotionUpdateActionDict.__annotations__

    @property
    def feedback_features(self) -> dict:
        # TODO
        return {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError()

        if not rclpy.ok():
            rclpy.init()

        self._node = rclpy.create_node("spacemouse_teleop")
        if calibrate:
            self._node.get_logger().warn(
                "Calibration not supported, ensure the robot is calibrated before running teleop."
            )

        self._device = pyspacemouse.open(
            dof_callback=None,
            # button_callback_arr=[
            #     pyspacemouse.ButtonCallback([0], self._button_callback),  # Button 1
            #     pyspacemouse.ButtonCallback([1], self._button_callback),  # Button 2
            # ],
            device=self.config.device,
        )

        if self._device is None:
            raise RuntimeError("Failed to open SpaceMouse device")

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._executor_thread = Thread(target=self._executor.spin)
        self._executor_thread.start()
        self._is_connected = True

    @property
    def is_calibrated(self) -> bool:
        # Calibration not supported
        return True

    def calibrate(self) -> None:
        # Calibration not supported
        pass

    def configure(self) -> None:
        pass

    def apply_deadband(self, value, threshold=0.02):
        return value if abs(value) > threshold else 0.0

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected or not self._device:
            raise DeviceNotConnectedError()

        state = self._device.read()

        clean_x = self.apply_deadband(float(state.x))
        clean_y = self.apply_deadband(float(state.y))
        clean_z = self.apply_deadband(float(state.z))
        clean_roll = self.apply_deadband(float(state.roll))
        clean_pitch = self.apply_deadband(float(state.pitch))
        clean_yaw = self.apply_deadband(float(state.yaw))

        twist_msg = Twist()
        twist_msg.linear.x = clean_x**1 * self.config.command_scaling
        twist_msg.linear.y = -(clean_y**1) * self.config.command_scaling
        twist_msg.linear.z = -(clean_z**1) * self.config.command_scaling
        twist_msg.angular.x = -(clean_pitch**1) * self.config.command_scaling
        twist_msg.angular.y = clean_roll**1 * self.config.command_scaling  #
        twist_msg.angular.z = clean_yaw**1 * self.config.command_scaling

        if not self.config.operator_position_front:
            twist_msg.linear.x *= -1
            twist_msg.linear.y *= -1
            twist_msg.angular.x *= -1
            twist_msg.angular.y *= -1

        self._current_actions = {
            "linear.x": twist_msg.linear.x,
            "linear.y": twist_msg.linear.y,
            "linear.z": twist_msg.linear.z,
            "angular.x": twist_msg.angular.x,
            "angular.y": twist_msg.angular.y,
            "angular.z": twist_msg.angular.z,
        }

        return cast(dict, self._current_actions)

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass

    def disconnect(self) -> None:
        if self._device:
            self._device.close()
        self._is_connected = False
        pass
# ==============================================================================
# START OF NEW ADDITION: AICCheatCodeTeleop (Simplified)
# ==============================================================================
import numpy as np
from aic_model_interfaces.msg import Observation
from scipy.spatial.transform import Rotation as R
from tf2_ros import Buffer, TransformListener
from transforms3d._gohlketransforms import quaternion_multiply


@TeleoperatorConfig.register_subclass("aic_cheatcode")
@dataclass(kw_only=True)
class AICCheatCodeTeleopConfig(TeleoperatorConfig):
    # PI gains for world-frame linear velocity control
    kp_linear: float = 1.0
    ki_linear: float = 0.2
    max_integrator_windup: float = 0.05
    kp_angular: float = 2.0

    # Velocity limits
    max_linear_vel: float = 0.04
    max_angular_vel: float = 0.5

    # Force-aware insertion slowdown (no retreat/recovery)
    force_rampdown_start: float = 15.0
    force_rampdown_full: float = 19.0

    # Insertion geometry/timing
    hover_height: float = 0.03  # Reduced from 0.10 - less drift time
    approach_height: float = 0.20
    insertion_base_speed: float = 0.012  # Increased from 0.008 - faster descent
    insertion_depth: float = -0.015
    insertion_dwell: float = 2.0

    # Alignment convergence criteria
    align_xy_tolerance: float = 0.0005
    align_angular_tolerance: float = 0.03
    align_timeout: float = 8.0
    align_min_dwell: float = 2.0

    # --- Task Variables (Override via command line) ---
    task_cable_name: str = "cable_0"
    task_plug_name: str = "sfp_tip"
    task_module_name: str = "nic_card_mount_0"
    task_port_name: str = "sfp_port_0"


class AICCheatCodeTeleop(Teleoperator):
    def __init__(self, config: AICCheatCodeTeleopConfig):
        super().__init__(config)
        self.config = config
        self._is_connected = False

        # State machine: INIT -> APPROACH -> ALIGN -> INSERT -> DONE
        self.phase = "INIT"
        self.z_offset = config.approach_height
        self.start_time = 0.0

        # Timing/state bookkeeping
        self._last_action_time = None
        self._insertion_depth_reached_time = None
        self._force_exceed_start_time = None
        self._actual_plug_port_z = float("nan")

        # Integrator for linear PI controller
        self._lin_err_integrator = np.zeros(3)

        # Force/tare state
        self._latest_force_mag = 0.0
        self._has_tare = False
        self._tare_offset = np.zeros(3)

        self._current_actions: MotionUpdateActionDict = {
            "linear.x": 0.0,
            "linear.y": 0.0,
            "linear.z": 0.0,
            "angular.x": 0.0,
            "angular.y": 0.0,
            "angular.z": 0.0,
        }

    @property
    def name(self) -> str:
        return "aic_cheatcode"

    @property
    def action_features(self) -> dict:
        return MotionUpdateActionDict.__annotations__

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError()

        if not rclpy.ok():
            rclpy.init()

        self._node = rclpy.create_node("cheatcode_teleop")
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self._node)

        self._latest_force_mag = 0.0
        self._force_exceed_start_time = None
        self._last_action_time = None
        self._insertion_depth_reached_time = None
        self._has_tare = False
        self._tare_offset = np.zeros(3)

        self._obs_sub = self._node.create_subscription(
            Observation,
            "/observations",
            self._obs_callback,
            qos_profile_sensor_data,
        )

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._executor_thread = Thread(target=self._executor.spin, daemon=True)
        self._executor_thread.start()

        self._is_connected = True
        print(
            f"\n\nCheatCode Teleop connected. "
            f"Target: {self.config.task_port_name} on {self.config.task_module_name}"
        )

    def _obs_callback(self, msg: Observation) -> None:
        # Capture tare offset from controller state
        tare = msg.controller_state.fts_tare_offset.wrench.force
        self._tare_offset = np.array([tare.x, tare.y, tare.z])
        self._has_tare = True

        # Get raw wrist wrench and apply tare
        raw_force = msg.wrist_wrench.wrench.force
        raw_force_vec = np.array([raw_force.x, raw_force.y, raw_force.z])
        tared_force = raw_force_vec - self._tare_offset

        # Rotation-invariant force magnitude
        self._latest_force_mag = float(np.linalg.norm(tared_force))

        print(
            f"[CheatCode] Force: {self._latest_force_mag:.1f}N | "
            f"Phase: {self.phase} | z_off: {self.z_offset:.4f} | "
            f"plug_z_actual: {self._actual_plug_port_z}"
        )

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def _get_transform(self, target_frame: str, source_frame: str):
        try:
            return self._tf_buffer.lookup_transform(target_frame, source_frame, Time())
        except Exception:
            return None

    def _compute_force_speed_multiplier(self) -> float:
        """Linearly ramp insertion speed from 1.0 -> 0.0 based on force."""
        f = self._latest_force_mag
        if f <= self.config.force_rampdown_start:
            return 1.0
        if f >= self.config.force_rampdown_full:
            return 0.0
        return 1.0 - (
            (f - self.config.force_rampdown_start)
            / (self.config.force_rampdown_full - self.config.force_rampdown_start)
        )

    def _set_zero_action(self) -> dict[str, Any]:
        for key in self._current_actions:
            self._current_actions[key] = 0.0
        return cast(dict, self._current_actions)

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError()

        cfg = self.config
        current_time = self._node.get_clock().now().nanoseconds / 1e9

        # Required TF frames from config
        port_frame = f"task_board/{cfg.task_module_name}/{cfg.task_port_name}_link"
        cable_tip_frame = f"{cfg.task_cable_name}/{cfg.task_plug_name}_link"

        # Current transforms
        port_tf = self._get_transform("base_link", port_frame)
        plug_tf = self._get_transform("base_link", cable_tip_frame)
        gripper_tf = self._get_transform("base_link", "gripper/tcp")

        # If TFs are missing, hold still
        if not port_tf or not plug_tf or not gripper_tf:
            if self.phase == "INIT":
                print("Waiting for ground truth TFs...", end="\r")
            return self._set_zero_action()

        # Transition out of INIT
        if self.phase == "INIT":
            print("\nTFs found! Starting APPROACH phase.")
            self.phase = "APPROACH"
            self.z_offset = cfg.approach_height
            self.start_time = current_time
            self._lin_err_integrator = np.zeros(3)

        elapsed = current_time - self.start_time

        # Extract positions
        gripper_pos = np.array(
            [
                gripper_tf.transform.translation.x,
                gripper_tf.transform.translation.y,
                gripper_tf.transform.translation.z,
            ]
        )
        plug_pos = np.array(
            [
                plug_tf.transform.translation.x,
                plug_tf.transform.translation.y,
                plug_tf.transform.translation.z,
            ]
        )
        port_pos = np.array(
            [
                port_tf.transform.translation.x,
                port_tf.transform.translation.y,
                port_tf.transform.translation.z,
            ]
        )

        # Quaternions (w, x, y, z)
        q_port = (
            port_tf.transform.rotation.w,
            port_tf.transform.rotation.x,
            port_tf.transform.rotation.y,
            port_tf.transform.rotation.z,
        )
        q_plug = (
            plug_tf.transform.rotation.w,
            plug_tf.transform.rotation.x,
            plug_tf.transform.rotation.y,
            plug_tf.transform.rotation.z,
        )
        q_gripper = (
            gripper_tf.transform.rotation.w,
            gripper_tf.transform.rotation.x,
            gripper_tf.transform.rotation.y,
            gripper_tf.transform.rotation.z,
        )

        # Target orientation (align plug to port)
        q_plug_inv = (-q_plug[0], q_plug[1], q_plug[2], q_plug[3])
        q_diff = quaternion_multiply(q_port, q_plug_inv)
        q_gripper_target = quaternion_multiply(q_diff, q_gripper)

        # Target position
        plug_offset = gripper_pos - plug_pos
        target_pos = np.array(
            [
                port_pos[0] + plug_offset[0],
                port_pos[1] + plug_offset[1],
                port_pos[2] + plug_offset[2] + self.z_offset,
            ]
        )

        # Alignment errors
        xy_error = np.linalg.norm(target_pos[:2] - gripper_pos[:2])
        dist_to_target = np.linalg.norm(target_pos - gripper_pos)

        # Angular error
        r_current = R.from_quat([q_gripper[1], q_gripper[2], q_gripper[3], q_gripper[0]])
        r_target = R.from_quat(
            [
                q_gripper_target[1],
                q_gripper_target[2],
                q_gripper_target[3],
                q_gripper_target[0],
            ]
        )
        r_error = r_target * r_current.inv()
        angular_error = float(np.linalg.norm(r_error.as_rotvec()))

        # Universal insertion completion check
        if self.phase not in ("INIT", "APPROACH", "DONE"):
            actual_plug_port_z = plug_pos[2] - port_pos[2]
            self._actual_plug_port_z = actual_plug_port_z
            if actual_plug_port_z <= cfg.insertion_depth + 0.005:
                if self._insertion_depth_reached_time is None:
                    self._insertion_depth_reached_time = current_time
                    print(
                        "[UNIVERSAL] Plug at insertion depth "
                        f"(actual_z={actual_plug_port_z:.4f}m). "
                        f"Dwelling for {cfg.insertion_dwell}s..."
                    )
                elif (
                    current_time - self._insertion_depth_reached_time
                ) >= cfg.insertion_dwell:
                    print(
                        "[UNIVERSAL] Insertion complete after "
                        f"{cfg.insertion_dwell}s dwell "
                        f"(actual_z={actual_plug_port_z:.4f}m). DONE."
                    )
                    self.phase = "DONE"
            else:
                self._insertion_depth_reached_time = None

        # State machine transitions
        if self.phase == "APPROACH":
            self._lin_err_integrator = np.zeros(3)
            if dist_to_target < 0.01 and elapsed > 1.5:
                print(f"Hover reached (err={dist_to_target:.4f}m). Entering ALIGN phase.")
                self.phase = "ALIGN"
                self.z_offset = cfg.hover_height
                self.start_time = current_time
                # Keep integrator — don't reset XY correction across phases

        elif self.phase == "ALIGN":
            align_elapsed = current_time - self.start_time
            xy_ok = xy_error < cfg.align_xy_tolerance
            ang_ok = angular_error < cfg.align_angular_tolerance
            dwell_ok = align_elapsed > cfg.align_min_dwell

            # Periodic logging during ALIGN
            plug_port_xy_dist = np.linalg.norm(port_pos[:2] - plug_pos[:2])
            if int(align_elapsed * 10) % 5 == 0:  # Log every 0.5s
                print(
                    f"[ALIGN] t={align_elapsed:.1f}s | gripper_xy_err={xy_error:.4f}m | "
                    f"plug_xy_err={plug_port_xy_dist:.4f}m | ang_err={angular_error:.3f}rad | "
                    f"integrator=[{self._lin_err_integrator[0]:.3f}, {self._lin_err_integrator[1]:.3f}]"
                )

            if (xy_ok and ang_ok and dwell_ok) or align_elapsed > cfg.align_timeout:
                if align_elapsed > cfg.align_timeout:
                    print(
                        f"ALIGN timeout ({cfg.align_timeout}s). Proceeding "
                        f"(xy={xy_error:.4f}, ang={angular_error:.3f})"
                    )
                else:
                    print(
                        "Aligned! "
                        f"(xy={xy_error:.4f}m, ang={angular_error:.3f}rad, "
                        f"dwell={align_elapsed:.1f}s). Starting INSERT."
                    )
                self.phase = "INSERT"
                self.start_time = current_time
                # Keep integrator — carries XY correction built during ALIGN

        elif self.phase == "INSERT":
            # Safety hold only - no rampdown (rely on wrench feedback for compliance)
            force_hold_threshold = 19.5
            force_hold_duration = 0.8
            if self._latest_force_mag > force_hold_threshold:
                if self._force_exceed_start_time is None:
                    self._force_exceed_start_time = current_time
                elif (current_time - self._force_exceed_start_time) > force_hold_duration:
                    print(
                        f"[INSERT] High force sustained ({self._latest_force_mag:.1f}N) - "
                        "safety hold until force drops."
                    )
                    self._last_action_time = current_time
                    return self._set_zero_action()
            else:
                self._force_exceed_start_time = None

            # Constant-speed descent (no force rampdown - matches original CheatCode)
            if self._last_action_time is None:
                dt = 0.033
            else:
                dt = min(current_time - self._last_action_time, 0.1)

            descent = cfg.insertion_base_speed * dt
            self.z_offset = max(cfg.insertion_depth, self.z_offset - descent)
            target_pos[2] = port_pos[2] + plug_offset[2] + self.z_offset

            # Debug logging
            print(
                f"[INSERT] z_offset={self.z_offset:.4f} | descent_rate={descent/dt:.4f}m/s | "
                f"force={self._latest_force_mag:.1f}N | dt={dt:.3f}s"
            )

        elif self.phase == "DONE":
            self._last_action_time = current_time
            return self._set_zero_action()

        # PI velocity controller (world frame)
        lin_err = target_pos - gripper_pos

        # XY-only integrator using plug-tip-to-port error (like original CheatCode)
        # This directly compensates for cable droop and gripper compliance
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

        # Slower velocity limit in INSERT for smooth motion
        if self.phase == "INSERT":
            max_lin = cfg.max_linear_vel * 0.5
        else:
            max_lin = cfg.max_linear_vel
        v_linear_world = np.clip(v_linear_world, -max_lin, max_lin)

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

        # Transform world-frame velocities into TCP-frame velocities
        v_linear_tcp = r_current.inv().apply(v_linear_world)
        v_angular_tcp = r_current.inv().apply(v_angular_world)

        self._current_actions = {
            "linear.x": float(v_linear_tcp[0]),
            "linear.y": float(v_linear_tcp[1]),
            "linear.z": float(v_linear_tcp[2]),
            "angular.x": float(v_angular_tcp[0]),
            "angular.y": float(v_angular_tcp[1]),
            "angular.z": float(v_angular_tcp[2]),
        }

        self._last_action_time = current_time
        return cast(dict, self._current_actions)

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass

    def disconnect(self) -> None:
        self._is_connected = False
        if hasattr(self, "_node"):
            self._node.destroy_node()


# ==============================================================================
# END OF NEW ADDITION: AICCheatCodeTeleop
# ==============================================================================