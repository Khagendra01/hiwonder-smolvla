#!/usr/bin/env python3
"""Gamepad input for JetArm teleop/recording (pygame/SDL, no ROS dependency).

Detected pad: SHANWAN "Android Gamepad" (6 axes, 15 buttons, 1 hat) on /dev/input/js0.

Use the monitor to discover your controller's axis/button numbers:

    python3 bridge/gamepad.py --monitor
"""
from __future__ import annotations

import os
import sys
import time

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")


def _pygame():
    import pygame

    return pygame


class Gamepad:
    def __init__(self, index: int = 0, deadzone: float = 0.15):
        self.pygame = _pygame()
        self.pygame.init()
        self.pygame.joystick.init()
        count = self.pygame.joystick.get_count()
        if count == 0:
            raise RuntimeError("no gamepad detected (is it plugged in?)")
        if index >= count:
            raise RuntimeError(f"gamepad index {index} out of range (found {count})")
        self.index = index
        self.deadzone = float(deadzone)
        self.js = self.pygame.joystick.Joystick(index)
        self.js.init()
        self._prev_buttons = [0] * self.js.get_numbuttons()

    @property
    def name(self) -> str:
        return self.js.get_name()

    @property
    def num_axes(self) -> int:
        return self.js.get_numaxes()

    @property
    def num_buttons(self) -> int:
        return self.js.get_numbuttons()

    def poll(self):
        self.pygame.event.pump()

    def axis(self, i: int) -> float:
        return float(self.js.get_axis(i))

    def button(self, i: int) -> int:
        return int(self.js.get_button(i))

    def hat(self, i: int = 0):
        return self.js.get_hat(i)

    def dz(self, v: float) -> float:
        return 0.0 if abs(v) < self.deadzone else v

    def pressed_edges(self) -> list[int]:
        """Button indices that went 0 -> 1 since the previous call."""
        edges = []
        for i in range(len(self._prev_buttons)):
            b = int(self.js.get_button(i))
            if b and not self._prev_buttons[i]:
                edges.append(i)
            self._prev_buttons[i] = b
        return edges

    def close(self):
        try:
            self.pygame.joystick.quit()
        except Exception:
            pass
        try:
            self.pygame.quit()
        except Exception:
            pass


def monitor(index: int = 0):
    gp = Gamepad(index)
    print(f"gamepad: {gp.name}  axes={gp.num_axes} buttons={gp.num_buttons}")
    print("move sticks / press buttons; Ctrl+C to stop\n")
    try:
        while True:
            gp.poll()
            axes = " ".join(f"{i}:{gp.axis(i):+.2f}" for i in range(gp.num_axes))
            buttons = " ".join(str(i) for i in range(gp.num_buttons) if gp.button(i))
            hx, hy = gp.hat()
            sys.stdout.write(f"\raxes [{axes}]  hat ({hx:+d},{hy:+d})  buttons [{buttons}]    ")
            sys.stdout.flush()
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()
    finally:
        gp.close()


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--index", type=int, default=0)
    ap.add_argument("--monitor", action="store_true", help="print live axis/button values")
    args = ap.parse_args()
    monitor(args.index)
