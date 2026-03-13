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
# START OF NEW ADDITION: AICCheatCodeTeleop
# ==============================================================================
import numpy as np
from dataclasses import dataclass
from typing import Any, cast
from threading import Thread

import rclpy
from rclpy.executors import SingleThreadedExecutor
from scipy.spatial.transform import Rotation as R
from tf2_ros import Buffer, TransformListener
from transforms3d._gohlketransforms import quaternion_multiply

# Assuming these are imported elsewhere in your file based on your snippets:
# from lerobot.common.robot_devices.teleoperation.utils import Teleoperator, TeleoperatorConfig
# from lerobot.common.robot_devices.utils import DeviceAlreadyConnectedError, DeviceNotConnectedError
# from lerobot.common.robot_devices.control.utils import MotionUpdateActionDict

@TeleoperatorConfig.register_subclass("aic_cheatcode")
@dataclass(kw_only=True)
class AICCheatCodeTeleopConfig(TeleoperatorConfig):
    # Proportional-Integral gains for the velocity controller
    kp_linear: float = 1.0
    ki_linear: float = 0.15  # Added to fix steady-state error (Zeno's paradox)
    max_integrator_windup: float = 0.05
    kp_angular: float = 1.5
    
    # Max velocity clamping to keep demonstrations smooth and safe
    max_linear_vel: float = 0.1
    max_angular_vel: float = 0.5
    
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
        
        # State machine variables
        self.phase = "INIT"  # INIT -> APPROACH -> INSERT -> DONE
        self.z_offset = 0.2
        self.start_time = 0.0  # Must be 0.0 here, _node doesn't exist yet!
        
        # Integrator for the PI controller
        self._lin_err_integrator = np.zeros(3)
        
        self._current_actions: MotionUpdateActionDict = {
            "linear.x": 0.0, "linear.y": 0.0, "linear.z": 0.0,
            "angular.x": 0.0, "angular.y": 0.0, "angular.z": 0.0,
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

        # Spin up a background ROS 2 node to listen to TF ground truth
        self._node = rclpy.create_node("cheatcode_teleop")
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self._node)

        # Force feedback state
        self._latest_force_mag = 0.0
        self._force_exceed_start_time = None
        self._has_tare = False
        self._tare_offset = np.zeros(3)

        # Import Observation message here to avoid circular dependencies if any
        from aic_model_interfaces.msg import Observation
        
        # Subscribe to observations to get wrist wrench and tare offset
        self._obs_sub = self._node.create_subscription(
            Observation,
            "/observations",
            self._obs_callback,
            rclpy.qos.qos_profile_sensor_data
        )
        if self._obs_sub is not None:
            print("Observation subscriber created successfully.")
        else:
            print("Failed to create observation subscriber.")


        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._executor_thread = Thread(target=self._executor.spin, daemon=True)
        self._executor_thread.start()
        
        self._is_connected = True
        print(f"CheatCode Teleop connected. Target: {self.config.task_port_name} on {self.config.task_module_name}")

    def _obs_callback(self, msg):
        """Processes the observation message to calculate world-frame insertion force."""
        # 1. Capture tare offset from controller state
        tare = msg.controller_state.fts_tare_offset.wrench.force
        self._tare_offset = np.array([tare.x, tare.y, tare.z])
        self._has_tare = True

        # 2. Get raw wrist wrench
        raw_force = msg.wrist_wrench.wrench.force
        raw_force_vec = np.array([raw_force.x, raw_force.y, raw_force.z])

        # 3. Apply tare
        tared_force = raw_force_vec - self._tare_offset

        # 4. Transform force to world frame
        # The wrench is in the sensor frame (typically 'wrist_3_link' or 'tool0')
        # We need its orientation in the base_link frame to get the true 3D force magnitude correctly? 
        # Actually, the magnitude of a 3D vector is invariant to pure rotation!
        # So we don't strictly *need* to transform it if we are just looking at overall magnitude.
        # But to be robust and allow future Z-axis checks, let's look up the transform.
        
        sensor_frame = msg.wrist_wrench.header.frame_id
        if not sensor_frame:
            sensor_frame = "gripper/tcp" # Fallback guess if empty

        tf_msg = self._get_transform("base_link", sensor_frame)
        if tf_msg:
            q = tf_msg.transform.rotation
            rot = R.from_quat([q.x, q.y, q.z, q.w])
            world_force = rot.apply(tared_force)
            # Magnitude is the same, but now it's in world frame
            self._latest_force_mag = np.linalg.norm(world_force)
        else:
            # Fallback to local magnitude
            self._latest_force_mag = np.linalg.norm(tared_force)


    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def _get_transform(self, target_frame: str, source_frame: str):
        """Helper to get transforms without throwing exceptions in the main loop."""
        try:
            return self._tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
        except Exception:
            return None

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError()

        # 1. Define required TF frames from config
        port_frame = f"task_board/{self.config.task_module_name}/{self.config.task_port_name}_link"
        cable_tip_frame = f"{self.config.task_cable_name}/{self.config.task_plug_name}_link"
        
        # 2. Look up current transforms
        port_tf = self._get_transform("base_link", port_frame)
        plug_tf = self._get_transform("base_link", cable_tip_frame)
        gripper_tf = self._get_transform("base_link", "gripper/tcp")

        # If we are missing TFs, output 0 velocity (Runaway robot fix)
        if not port_tf or not plug_tf or not gripper_tf:
            if self.phase == "INIT":
                print("Waiting for ground truth TFs...", end="\r")
            else:
                for key in self._current_actions:
                    self._current_actions[key] = 0.0
            return cast(dict, self._current_actions)

        # Transition out of INIT once TFs are found
        if self.phase == "INIT":
            print("\nTFs found! Starting APPROACH phase.")
            self.phase = "APPROACH"
            self.start_time = self._node.get_clock().now().nanoseconds / 1e9 # Fixed to use ROS time

        # 3. Calculate target orientation
        q_port = (
            port_tf.transform.rotation.w, port_tf.transform.rotation.x,
            port_tf.transform.rotation.y, port_tf.transform.rotation.z
        )
        q_plug = (
            plug_tf.transform.rotation.w, plug_tf.transform.rotation.x,
            plug_tf.transform.rotation.y, plug_tf.transform.rotation.z
        )
        q_plug_inv = (-q_plug[0], q_plug[1], q_plug[2], q_plug[3])
        q_diff = quaternion_multiply(q_port, q_plug_inv)
        
        q_gripper = (
            gripper_tf.transform.rotation.w, gripper_tf.transform.rotation.x,
            gripper_tf.transform.rotation.y, gripper_tf.transform.rotation.z
        )
        q_gripper_target = quaternion_multiply(q_diff, q_gripper)

        # 4. Calculate target position
        gripper_pos = np.array([
            gripper_tf.transform.translation.x,
            gripper_tf.transform.translation.y,
            gripper_tf.transform.translation.z
        ])
        plug_pos = np.array([
            plug_tf.transform.translation.x,
            plug_tf.transform.translation.y,
            plug_tf.transform.translation.z
        ])
        plug_offset = gripper_pos - plug_pos
        
        # Base target is the port position + the offset of how the gripper holds the plug
        target_pos = np.array([
            port_tf.transform.translation.x + plug_offset[0],
            port_tf.transform.translation.y + plug_offset[1],
            port_tf.transform.translation.z + plug_offset[2] + self.z_offset
        ])

        # 5. State Machine Logic
        current_time = self._node.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.start_time
        dist_to_target = np.linalg.norm(target_pos - gripper_pos)

        # Force penalty check logic
        if self.phase == "INSERT":
            if self._latest_force_mag > 20.0:
                if self._force_exceed_start_time is None:
                    self._force_exceed_start_time = current_time
                elif (current_time - self._force_exceed_start_time) > 0.8:
                    print(f"\nExcessive force detected ({self._latest_force_mag:.1f}N). Initiating recovery...")
                    self.phase = "RECOVERY"
                    self._force_exceed_start_time = None
                    self.z_offset = 0.2  # Lift back up to hover height
                    self._lin_err_integrator = np.zeros(3)  # Reset integrator to prevent jerking
            else:
                self._force_exceed_start_time = None

        if self.phase == "APPROACH":
            if dist_to_target < 0.01 and elapsed > 2.0:
                print("Hover position reached. Starting INSERT phase.")
                self.phase = "INSERT"
                self.start_time = current_time
                self._lin_err_integrator = np.zeros(3) 

        elif self.phase == "RECOVERY":
            # Wait until the robot lifts the plug back to the safe z_offset
            if dist_to_target < 0.01:
                print("Recovery complete. Restarting APPROACH phase.")
                self.phase = "APPROACH"
                self.start_time = current_time
                self._lin_err_integrator = np.zeros(3)

        elif self.phase == "INSERT":
            insert_elapsed = current_time - self.start_time
            # Slowly lower the target z_offset.
            self.z_offset = max(-0.015, 0.2 - (0.07 * insert_elapsed))
            
            # Check if the target has finished descending AND the physical
            # gripper has reached the target (or is physically blocked by the port)
            z_error = abs(target_pos[2] - gripper_pos[2])
            
            if self.z_offset <= -0.015 and z_error < 0.005:
                print("Insertion complete. DONE phase.")
                self.phase = "DONE"

        elif self.phase == "DONE":
            for key in self._current_actions:
                self._current_actions[key] = 0.0
            return cast(dict, self._current_actions)

        # 6. Proportional-Integral (PI) Velocity Controller (World Frame)
        lin_err = target_pos - gripper_pos
        self._lin_err_integrator = np.clip(
            self._lin_err_integrator + lin_err,
            -self.config.max_integrator_windup,
            self.config.max_integrator_windup
        )
        
        v_linear_world = (self.config.kp_linear * lin_err) + (self.config.ki_linear * self._lin_err_integrator)
        v_linear_world = np.clip(v_linear_world, -self.config.max_linear_vel, self.config.max_linear_vel)

        r_current = R.from_quat([q_gripper[1], q_gripper[2], q_gripper[3], q_gripper[0]])
        r_target = R.from_quat([q_gripper_target[1], q_gripper_target[2], q_gripper_target[3], q_gripper_target[0]])
        
        r_error = r_target * r_current.inv()
        v_angular_world = np.clip(self.config.kp_angular * r_error.as_rotvec(), -self.config.max_angular_vel, self.config.max_angular_vel)

        # FIX: Transform World-Frame velocities into TCP-Frame velocities
        v_linear_tcp = r_current.inv().apply(v_linear_world)
        v_angular_tcp = r_current.inv().apply(v_angular_world)

        # 7. Map to LeRobot action dict using the TCP-Frame velocities
        self._current_actions = {
            "linear.x": float(v_linear_tcp[0]),
            "linear.y": float(v_linear_tcp[1]),
            "linear.z": float(v_linear_tcp[2]),
            "angular.x": float(v_angular_tcp[0]),
            "angular.y": float(v_angular_tcp[1]),
            "angular.z": float(v_angular_tcp[2]),
        }

        return cast(dict, self._current_actions)

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass

    def disconnect(self) -> None:
        self._is_connected = False
        if hasattr(self, '_node'):
            self._node.destroy_node()

# ==============================================================================
# END OF NEW ADDITION: AICCheatCodeTeleop
# ==============================================================================