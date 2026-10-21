#!/usr/bin/env python3
"""Client for the policy server (Phase 1).

Same interface the rollout will use against SmolVLA: connect once, call
infer(obs) -> action per control step.
"""
from __future__ import annotations

import socket

import numpy as np

from policy_protocol import send_msg, recv_msg


class PolicyClient:
    def __init__(self, host: str = "127.0.0.1", port: int = 5555, timeout: float = 5.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)

    def infer(self, obs) -> np.ndarray:
        send_msg(self.sock, obs)
        msg = recv_msg(self.sock)
        if msg is None:
            raise ConnectionError("policy server closed the connection")
        return np.asarray(msg["action"], dtype=np.float32)

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass


class ModalPolicyClient:
    """HTTP client for modal/serve_smolvla.py (same infer() interface).

    obs: {"image_jpeg": bytes} or {"image": BGR uint8 HxWx3}, plus
         {"state": np.ndarray(6), "task": str}.
    """

    def __init__(self, url: str, run: str = "smoke", task: str = "", timeout: float = 180.0):
        self.url = url.rstrip("/") + "/act"
        self.run = run
        self.task = task
        self.timeout = timeout

    def infer(self, obs) -> np.ndarray:
        import base64
        import json
        import urllib.request

        img = obs.get("image_jpeg")
        if img is None:
            import cv2

            bgr = obs["image"]
            ok, buf = cv2.imencode(".jpg", bgr[..., ::-1])
            if not ok:
                raise ValueError("failed to encode image")
            img = buf.tobytes()

        payload = {
            "run": self.run,
            "task": obs.get("task", self.task),
            "state": np.asarray(obs["state"], dtype=np.float32).tolist(),
            "image": base64.b64encode(img).decode(),
        }
        req = urllib.request.Request(
            self.url,
            data=json.dumps(payload).encode(),
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(req, timeout=self.timeout) as resp:
            msg = json.load(resp)
        if "error" in msg:
            raise RuntimeError(msg["error"])
        return np.asarray(msg["action"], dtype=np.float32)

    def close(self) -> None:
        pass


def parse_hostport(s: str) -> tuple[str, int]:
    host, port = s.rsplit(":", 1)
    return host, int(port)
