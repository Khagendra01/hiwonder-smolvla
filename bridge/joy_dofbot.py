#!/usr/bin/env python3
"""DOFBOT-style analog gamepad teleop for the JetArm.

Hiwonder's ``joystick_control`` gives both sticks to the tank chassis and only
steps the arm in 10-pulse increments from buttons/D-pad. This node mirrors a
DOFBOT-style pad instead: both sticks drive the arm continuously, buttons handle
wrist/gripper, and the D-pad keeps the tank usable.

It publishes the same seams as Hiwonder, so the recorder/rollout work unchanged:

    /servo_controller                 servo_controller_msgs/ServosPosition
    /chassis_controller/command       chassis_msgs/Mecanum
    /controller_manager/servo_states  feedback (pulses 0..1000)

Run it INSTEAD of ``joystick_control`` (both command /servo_controller):

    bash scripts/gamepad.sh

The mapping lives in ``config/gamepad.yaml``. Verify the pad's raw axis/button
numbers first with::

    python3 bridge/gamepad.py --monitor

and use ``--dry-run`` to watch the intended commands without moving the arm.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml

_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

from gamepad import Gamepad  # noqa: E402

import rclpy  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from chassis_msgs.msg import Mecanum  # noqa: E402
from servo_controller_msgs.msg import (  # noqa: E402
    ServoPosition,
    ServosPosition,
    ServoStateList,
)

DEFAULT_CONFIG = _HERE.parent / "config" / "gamepad.yaml"


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class DofbotPad(Node):
    def __init__(self, cfg: dict, dry_run: bool = False):
        super().__init__("joy_dofbot")
        self.cfg = cfg
        self.dry_run = dry_run

        gp = cfg.get("gamepad") or {}
        self.index = int(gp.get("index", 0))
        self.deadzone = float(gp.get("deadzone", 0.15))
        self.rate_hz = float(gp.get("rate_hz", 20.0))

        joints = cfg.get("joints") or {}
        self.names = list(joints.get("names") or ["joint1", "joint2", "joint3", "joint4", "joint5", "gripper"])
        self.ids = [int(i) for i in (joints.get("ids") or [1, 2, 3, 4, 5, 10])]
        if len(self.names) != len(self.ids):
            raise ValueError("joints.names and joints.ids must have the same length")
        self.jid = dict(zip(self.names, self.ids))

        servo = cfg.get("servo") or {}
        self.cmd_topic = str(servo.get("topic", "/servo_controller"))
        self.state_topic = str(servo.get("state_topic", "/controller_manager/servo_states"))
        self.pmin = int(servo.get("min_pulse", 0))
        self.pmax = int(servo.get("max_pulse", 1000))
        self.step = float(servo.get("step", 8.0))
        self.duration = float(servo.get("duration", 0.05))
        home = [int(v) for v in (servo.get("home") or [500, 500, 500, 500, 500, 700])]
        if len(home) != len(self.ids):
            raise ValueError("servo.home must have one pulse per joint")
        self.home_by_id = dict(zip(self.ids, home))

        self.axes = {name: m for name, m in (cfg.get("axes") or {}).items() if m}
        buttons = cfg.get("buttons") or {}
        self.btn_step = int(buttons.get("step", 20))
        self.gripper_step = int(buttons.get("gripper_step", 30))
        self.btn = {name: int(i) for name, i in (buttons.get("map") or {}).items()}
        reserved = {int(i) for i in (cfg.get("reserved") or [])}
        conflict = reserved.intersection(self.btn.values())
        if conflict:
            raise ValueError(f"buttons {sorted(conflict)} are reserved for the recorder")

        chassis = cfg.get("chassis") or {}
        self.chassis_enabled = bool(chassis.get("enabled", True))
        self.chassis_topic = str(chassis.get("topic", "/chassis_controller/command"))
        self.hat_index = int(chassis.get("hat", 0))
        self.chassis_speed = float(chassis.get("speed", 160))
        self.chassis_turn = float(chassis.get("turn", 50))
        self.chassis_forward_sign = float(chassis.get("forward_sign", 1))
        self.chassis_turn_sign = float(chassis.get("turn_sign", 1))

        self.pos: dict[int, int] = {}
        self.ready = False
        self.last_cmd: dict[int, int] | None = None
        self.last_chassis = (0.0, 0.0)

        self.cmd_pub = self.create_publisher(ServosPosition, self.cmd_topic, 10)
        self.chassis_pub = (
            self.create_publisher(Mecanum, self.chassis_topic, 10) if self.chassis_enabled else None
        )
        self.create_subscription(ServoStateList, self.state_topic, self._on_state, 10)

        try:
            self.pad = Gamepad(self.index, self.deadzone)
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            raise

        self.create_timer(1.0 / max(self.rate_hz, 1.0), self._tick)
        self.get_logger().info(
            "joy_dofbot: %s axes=%d buttons=%d at %.0f Hz"
            % (self.pad.name, self.pad.num_axes, self.pad.num_buttons, self.rate_hz)
        )
        if self.dry_run:
            self.get_logger().info("dry-run: no commands will be published")

    # ---- state ----
    def _on_state(self, msg: ServoStateList):
        for state in msg.servo_state:
            sid = int(state.id)
            if sid in self.ids:
                self.pos[sid] = int(state.position)
        if not self.ready and all(sid in self.pos for sid in self.ids):
            self.ready = True
            self.get_logger().info("joint feedback ready: %s" % self.pos)

    # ---- main loop ----
    def _tick(self):
        self.pad.poll()

        if not self.ready:
            if not self.dry_run:
                return
            self.pos = dict(self.home_by_id)
            self.ready = True

        values = {sid: float(self.pos.get(sid, self.home_by_id[sid])) for sid in self.ids}
        changed = False
        home_pressed = False

        for name, mapping in self.axes.items():
            sid = self.jid.get(name)
            if sid is None:
                continue
            try:
                raw = self.pad.axis(int(mapping["axis"]))
            except (IndexError, KeyError, TypeError):
                continue
            amount = self.pad.dz(raw)
            if amount:
                values[sid] += float(mapping.get("sign", 1)) * amount * self.step
                changed = True

        for button in self.pad.pressed_edges():
            if button == self.btn.get("joint5_minus") and "joint5" in self.jid:
                values[self.jid["joint5"]] -= self.btn_step
                changed = True
            elif button == self.btn.get("joint5_plus") and "joint5" in self.jid:
                values[self.jid["joint5"]] += self.btn_step
                changed = True
            elif button == self.btn.get("gripper_close") and "gripper" in self.jid:
                values[self.jid["gripper"]] -= self.gripper_step
                changed = True
            elif button == self.btn.get("gripper_open") and "gripper" in self.jid:
                values[self.jid["gripper"]] += self.gripper_step
                changed = True
            elif button == self.btn.get("home"):
                values.update({sid: float(p) for sid, p in self.home_by_id.items()})
                changed = True
                home_pressed = True

        if changed:
            msg = ServosPosition()
            msg.duration = 1.0 if home_pressed else self.duration
            msg.position_unit = "pulse"
            for sid in self.ids:
                pulse = int(round(_clamp(values[sid], self.pmin, self.pmax)))
                values[sid] = float(pulse)
                servo = ServoPosition()
                servo.id = sid
                servo.position = float(pulse)
                msg.position.append(servo)
            if not self.dry_run:
                self.cmd_pub.publish(msg)
            self.last_cmd = {sid: int(values[sid]) for sid in self.ids}
            if self.dry_run:
                self.pos.update(self.last_cmd)

        self._chassis_tick()

    def _chassis_tick(self):
        if not self.chassis_enabled or self.chassis_pub is None:
            return
        hat_x, hat_y = self.pad.hat(self.hat_index)

        translation = 0.0
        if hat_y > 0:
            translation = self.chassis_speed * self.chassis_forward_sign
        elif hat_y < 0:
            translation = -self.chassis_speed * self.chassis_forward_sign

        rotation = 0.0
        if hat_x > 0:
            rotation = self.chassis_turn * self.chassis_turn_sign
        elif hat_x < 0:
            rotation = -self.chassis_turn * self.chassis_turn_sign

        state = (translation, rotation)
        if state == self.last_chassis:
            return
        self.last_chassis = state

        msg = Mecanum()
        msg.velocity = float(abs(translation))
        msg.direction = 90.0 if translation >= 0 else 270.0
        msg.angular_rate = float(rotation)
        if not self.dry_run:
            self.chassis_pub.publish(msg)

    def shutdown(self):
        try:
            self.pad.close()
        except Exception:
            pass


def load_config(path: Path) -> dict:
    with open(path, "r", encoding="utf-8") as handle:
        cfg = yaml.safe_load(handle) or {}
    if not isinstance(cfg, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return cfg


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="mapping file (default: config/gamepad.yaml)")
    parser.add_argument("--dry-run", action="store_true", help="read the pad and log commands without publishing")
    args = parser.parse_args(argv)

    try:
        cfg = load_config(args.config)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f"config error: {exc}", file=sys.stderr)
        return 2

    rclpy.init()
    node = None
    try:
        node = DofbotPad(cfg, dry_run=args.dry_run)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
