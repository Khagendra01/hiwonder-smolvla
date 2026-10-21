#!/usr/bin/env python3
"""Dummy policy server (Phase 1) — validates the server/client topology.

Stands in for the SmolVLA policy server. Speaks bridge/policy_protocol.py over
TCP and implements `infer(obs) -> action`, where:
    obs    = {"state": np.ndarray(6), "home": np.ndarray(6)}
    action = np.ndarray(6)   # absolute target pulses

Policy: hold the home pose, but drive ONE joint with a slow sine about home.
No robot/ROS dependencies here on purpose — the server is meant to live in the
ML container.
"""
from __future__ import annotations

import argparse
import math
import socket
import threading

import numpy as np

from policy_protocol import send_msg, recv_msg


class DummyPolicy:
    def __init__(self, idx: int = 0, amp: float = 15.0, period_s: float = 2.0, dt: float = 0.05):
        self.idx = idx
        self.amp = amp
        self.period_s = period_s
        self.dt = dt
        self.step = 0
        self.lock = threading.Lock()

    def infer(self, obs) -> np.ndarray:
        with self.lock:
            t = self.step * self.dt
            self.step += 1
        base = np.asarray(obs.get("home", obs["state"]), dtype=np.float32)
        action = base.copy()
        action[self.idx] += self.amp * math.sin(2.0 * math.pi * t / self.period_s)
        return action


def _handle(conn, addr, policy: DummyPolicy):
    with conn:
        while True:
            obs = recv_msg(conn)
            if obs is None:
                break
            send_msg(conn, {"action": policy.infer(obs)})
    print(f"client disconnected {addr}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5555)
    ap.add_argument("--joint", type=int, default=0)
    ap.add_argument("--amp", type=float, default=15.0)
    ap.add_argument("--period", type=float, default=2.0)
    ap.add_argument("--dt", type=float, default=0.05)
    a = ap.parse_args()

    policy = DummyPolicy(a.joint, a.amp, a.period, a.dt)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((a.host, a.port))
    s.listen(1)
    print(f"dummy policy server listening on {a.host}:{a.port} "
          f"joint={a.joint} amp={a.amp} period={a.period}s", flush=True)
    try:
        while True:
            conn, addr = s.accept()
            print(f"client connected {addr}", flush=True)
            threading.Thread(target=_handle, args=(conn, addr, policy), daemon=True).start()
    except KeyboardInterrupt:
        pass
    finally:
        s.close()


if __name__ == "__main__":
    main()
