#!/usr/bin/env python3
"""Length-prefixed pickle framing for the policy server <-> client link.

Deliberately tiny and dependency-free. The wire contract (dict in, dict out) is
the same one a real SmolVLA/LeRobot server will expose, so only the server
implementation changes when we swap the dummy policy for SmolVLA.

Frame = 4-byte big-endian length + pickle payload.
"""
from __future__ import annotations

import pickle
import struct

_HDR = struct.Struct("!I")


def send_msg(sock, obj) -> None:
    data = pickle.dumps(obj, protocol=pickle.HIGHEST_PROTOCOL)
    sock.sendall(_HDR.pack(len(data)) + data)


def _recv_exact(sock, n: int) -> bytes | None:
    chunks = []
    need = n
    while need > 0:
        b = sock.recv(need)
        if not b:
            return None
        chunks.append(b)
        need -= len(b)
    return b"".join(chunks)


def recv_msg(sock):
    hdr = _recv_exact(sock, _HDR.size)
    if hdr is None:
        return None
    (n,) = _HDR.unpack(hdr)
    data = _recv_exact(sock, n)
    if data is None:
        return None
    return pickle.loads(data)
