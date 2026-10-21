#!/usr/bin/env python3
"""Read-only arm state query over ROS2.

Subscribes to /controller_manager/servo_states (servo_controller_msgs/ServoStateList)
and prints the 6 arm servo positions (pulse 0..1000). Sends NO motion.

Requires the arm stack to be running (scripts/bringup.sh).
"""
import os
import sys
import time

import rclpy
from rclpy.node import Node
from servo_controller_msgs.msg import ServoStateList

JOINT_IDS = [1, 2, 3, 4, 5, 10]
JOINT_NAMES = {1: "joint1", 2: "joint2", 3: "joint3", 4: "joint4", 5: "joint5", 10: "gripper"}
TOPIC = os.environ.get("TOPIC_SERVO_STATES", "/controller_manager/servo_states")


class Reader(Node):
    def __init__(self):
        super().__init__("jetarm_state_reader")
        self.latest = None
        self.create_subscription(ServoStateList, TOPIC, self._cb, 10)

    def _cb(self, msg):
        self.latest = msg


def main():
    rclpy.init()
    node = Reader()
    deadline = time.monotonic() + 5.0
    while node.latest is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.latest is None:
        print(f"  no data on {TOPIC} within timeout (is the arm stack running?)")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    by_id = {s.id: s.position for s in node.latest.servo_state}
    print(f"  source topic: {TOPIC}")
    for jid in JOINT_IDS:
        p = by_id.get(jid)
        print(f"    {JOINT_NAMES[jid]:8s} (id {jid:2d}) = {p}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
