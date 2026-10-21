#!/usr/bin/env python3
"""LeRobot Robot adapter for the Hiwonder JetArm (via the ROS2 seams in jetarm_ros.py).

WORK IN PROGRESS - skeleton with the correct wiring. It must run inside the
LeRobot environment (jetson-containers `lerobot` image) WITH the host ROS2
environment available (see ../README.md for the two deployment options).

Action space : absolute pulses for [joint1..joint5, gripper]  (0..1000)
Observation  : { "<joint>.pos": float, "front": uint8 HxWx3 RGB }
"""
from __future__ import annotations

from dataclasses import dataclass, field
from functools import cached_property

import numpy as np

from lerobot.cameras import CameraConfig  # noqa: F401  (kept for parity with other robots)
from lerobot.robots.config import RobotConfig
from lerobot.robots.robot import Robot
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from .jetarm_ros import JointMap, make_from_env

JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "gripper")


@RobotConfig.register_subclass("jetarm")
@dataclass
class JetArmConfig(RobotConfig):
    # topic names (defaults match config/robot.env)
    servo_cmd_topic: str = "/servo_controller"
    servo_state_topic: str = "/controller_manager/servo_states"
    camera_topic: str = "/depth_cam/rgb/image_raw"
    init_finish_service: str = "/controller_manager/init_finish"
    # control
    action_duration_s: float = 0.1
    image_key: str = "front"


class JetArm(Robot):
    config_class = JetArmConfig
    name = "jetarm"

    def __init__(self, config: JetArmConfig):
        super().__init__(config)
        self.config = config
        self._ros = None

    # ---- feature declarations (drive dataset schema + policy dims) ----
    @cached_property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{n}.pos": float for n in JOINT_NAMES}

    @cached_property
    def observation_features(self) -> dict:
        return {**self._motors_ft, self.config.image_key: (480, 640, 3)}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return self._ros is not None and self._ros.ready

    @property
    def is_calibrated(self) -> bool:
        # The JetArm's calibration (offsets/limits) lives in the STM32 board +
        # servo_controller.yaml; nothing to calibrate on the LeRobot side.
        return True

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        self._ros = make_from_env()
        self._ros.servo_cmd_topic = self.config.servo_cmd_topic
        self._ros.servo_state_topic = self.config.servo_state_topic
        self._ros.camera_topic = self.config.camera_topic
        self._ros.start(wait_ready_s=10.0)

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    @check_if_not_connected
    def get_observation(self) -> dict:
        joints = self._ros.read_joint_vector()
        img = self._ros.read_image()
        if joints is None:
            raise RuntimeError("no joint state from JetArm yet")
        obs: dict = {f"{n}.pos": float(v) for n, v in zip(JOINT_NAMES, joints)}
        obs[self.config.image_key] = img if img is not None else np.zeros((480, 640, 3), np.uint8)
        return obs

    @check_if_not_connected
    def send_action(self, action: dict) -> dict:
        vec = np.array([float(action[f"{n}.pos"]) for n in JOINT_NAMES], dtype=np.float32)
        self._ros.send_joint_vector(vec, duration_s=self.config.action_duration_s)
        return {f"{n}.pos": float(v) for n, v in zip(JOINT_NAMES, vec)}

    @check_if_not_connected
    def disconnect(self) -> None:
        if self._ros is not None:
            self._ros.stop()
            self._ros = None
