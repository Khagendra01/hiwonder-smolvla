#!/usr/bin/env python3
"""JetArmROS — thin ROS2 client that exposes the arm as read-joints / send-joints.

This is the ONLY code that talks to the robot's ROS2 control seams, so the rest
of the pipeline (LeRobot bridge, teleop, rollout) can stay ROS-agnostic.

Verified seams (see ../README.md):
  command : publish servo_controller_msgs/msg/ServosPosition on /servo_controller
            position_unit="pulse", duration seconds
  feedback: subscribe servo_controller_msgs/msg/ServoStateList on
            /controller_manager/servo_states   (id -> pulse 0..1000)
  camera  : subscribe sensor_msgs/msg/Image on /depth_cam/rgb/image_raw

Thread-safe-ish: rclpy.spin is driven by an internal thread; callers use the
public methods which are safe to call from another thread.
"""
from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass, field

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from servo_controller_msgs.msg import ServoPosition, ServosPosition, ServoStateList
from std_srvs.srv import Trigger


@dataclass
class JointMap:
    names: list[str] = field(default_factory=lambda: ["joint1", "joint2", "joint3", "joint4", "joint5", "gripper"])
    ids: list[int] = field(default_factory=lambda: [1, 2, 3, 4, 5, 10])
    pulse_min: int = 0
    pulse_max: int = 1000


class JetArmROS(Node):
    def __init__(
        self,
        joint_map: JointMap | None = None,
        servo_cmd_topic: str = "/servo_controller",
        servo_state_topic: str = "/controller_manager/servo_states",
        camera_topic: str = "/depth_cam/rgb/image_raw",
        init_finish_service: str = "/controller_manager/init_finish",
    ):
        super().__init__("jetarm_ros_bridge")
        self.jm = joint_map or JointMap()
        self._lock = threading.Lock()
        self._joints = {i: None for i in self.jm.ids}
        self._image = None
        self._image_encoding = None
        self._last_cmd = None
        self._ready = False
        self._spin_thread = None
        self._stop = threading.Event()

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._pub = self.create_publisher(ServosPosition, servo_cmd_topic, 10)
        self.create_subscription(ServoStateList, servo_state_topic, self._on_state, qos)
        self.create_subscription(Image, camera_topic, self._on_image, qos)
        self.create_subscription(ServosPosition, servo_cmd_topic, self._on_cmd, 10)
        self._init_cli = self.create_client(Trigger, init_finish_service)

    # ---- callbacks ----
    def _on_state(self, msg: ServoStateList):
        with self._lock:
            for s in msg.servo_state:
                if s.id in self._joints:
                    self._joints[s.id] = int(s.position)
            if self._last_cmd is None and all(v is not None for v in self._joints.values()):
                self._last_cmd = {jid: float(self._joints[jid]) for jid in self.jm.ids}

    def _on_image(self, msg: Image):
        arr = self._decode_image(msg)
        with self._lock:
            self._image = arr
            self._image_encoding = (msg.encoding or "").lower()

    def _on_cmd(self, msg: ServosPosition):
        with self._lock:
            if self._last_cmd is None:
                self._last_cmd = {}
            for sp in msg.position:
                self._last_cmd[int(sp.id)] = float(sp.position)

    @staticmethod
    def _decode_image(msg: Image) -> np.ndarray | None:
        enc = (msg.encoding or "").lower()
        h, w = msg.height, msg.width
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if enc in ("rgb8", "bgr8"):
            return buf.reshape(h, w, 3).copy()
        if enc in ("mono8",):
            return np.repeat(buf.reshape(h, w, 1), 3, axis=2)
        if enc in ("16uc1", "32fc1"):
            return None  # depth not needed for the policy observation
        return None

    # ---- lifecycle ----
    def start(self, wait_ready_s: float = 10.0):
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()
        self.wait_ready(wait_ready_s)

    def _spin(self):
        while not self._stop.is_set() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_ready(self, timeout_s: float):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            with self._lock:
                have_joints = all(v is not None for v in self._joints.values())
            if have_joints:
                self._ready = True
                return True
            time.sleep(0.05)
        return False

    def stop(self):
        self._stop.set()
        if self._spin_thread:
            self._spin_thread.join(timeout=1.0)
        self.destroy_node()

    # ---- API ----
    @property
    def ready(self) -> bool:
        return self._ready

    def read_joints(self) -> dict[str, int] | None:
        with self._lock:
            if any(v is None for v in self._joints.values()):
                return None
            return {name: self._joints[jid] for name, jid in zip(self.jm.names, self.jm.ids)}

    def read_joint_vector(self) -> np.ndarray | None:
        j = self.read_joints()
        if j is None:
            return None
        return np.array([j[n] for n in self.jm.names], dtype=np.float32)

    def read_image(self) -> np.ndarray | None:
        with self._lock:
            return None if self._image is None else self._image.copy()

    def read_image_encoding(self) -> str | None:
        with self._lock:
            return self._image_encoding

    def read_last_command(self) -> np.ndarray | None:
        with self._lock:
            if not self._last_cmd or any(jid not in self._last_cmd for jid in self.jm.ids):
                return None
            return np.array([self._last_cmd[jid] for jid in self.jm.ids], dtype=np.float32)

    def send_joint_vector(self, pulses, duration_s: float = 0.1):
        pulses = np.asarray(pulses).astype(float)
        if pulses.shape[0] != len(self.jm.ids):
            raise ValueError(f"expected {len(self.jm.ids)} joints, got {pulses.shape[0]}")
        msg = ServosPosition()
        msg.duration = float(duration_s)
        msg.position_unit = "pulse"
        for jid, p in zip(self.jm.ids, pulses):
            clamped = int(max(self.jm.pulse_min, min(self.jm.pulse_max, round(float(p)))))
            sp = ServoPosition()
            sp.id = int(jid)
            sp.position = float(clamped)
            msg.position.append(sp)
        self._pub.publish(msg)


def make_from_env() -> JetArmROS:
    """Convenience constructor reading topic names from env (config/robot.env)."""
    return JetArmROS(
        servo_cmd_topic=os.environ.get("TOPIC_SERVO_CMD", "/servo_controller"),
        servo_state_topic=os.environ.get("TOPIC_SERVO_STATES", "/controller_manager/servo_states"),
        camera_topic=os.environ.get("TOPIC_CAMERA_RGB", "/depth_cam/rgb/image_raw"),
        init_finish_service=os.environ.get("SRV_INIT_FINISH", "/controller_manager/init_finish"),
    )
