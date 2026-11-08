# Copyright (C) 2026 Tim Michals
# SPDX-License-Identifier: GPL-3.0-or-later
"""Plugin side of the SITL TLP link: whole 64-byte TLPs over TCP to inav_abstractx_sitl.

@impl [SPEC-SITL-03] docs/SITL_SIMULATOR.md

The TLP header layout comes from AbstractX (tools/abstractx_wire.py); INAV keeps no copy.
Runs without the Studio GUI so scenario scripts and tests can use it ([SPEC-SITL-08]).
"""

import os
import socket
import sys
from pathlib import Path


def _import_abstractx_wire():
    root = os.environ.get("ABSTRACTX_PATH")
    if not root:
        # The tcmichals/flight manifest checks AbstractX out next to inav.
        root = str(Path(__file__).resolve().parents[3] / "AbstractX")
    tools = Path(root) / "tools"
    if not (tools / "abstractx_wire.py").is_file():
        raise ImportError(f"AbstractX tools not found at {tools}: build from the flight workspace "
                          "or set ABSTRACTX_PATH")
    sys.path.insert(0, str(tools))
    import abstractx_wire  # noqa: E402
    return abstractx_wire


wire = _import_abstractx_wire()

TLP_BYTES = wire.TLP_BYTES
PAYLOAD_BYTES = 40
DEFAULT_PORT = 5770


def encode_tlp(type_, channel, addr, payload=b"", *, tag=0, flags=0, seq=0, timestamp_ns=0,
               length_dw=None):
    """A 64-byte TLP. length_dw defaults to the payload in 32-bit words."""
    if len(payload) > PAYLOAD_BYTES:
        raise ValueError(f"payload is {len(payload)} bytes, at most {PAYLOAD_BYTES}")
    if length_dw is None:
        length_dw = (len(payload) + 3) // 4
    header = wire.TLP_HEADER.pack(type_, flags, tag, channel, addr, length_dw, seq, timestamp_ns)
    frame = header + bytes(payload).ljust(PAYLOAD_BYTES, b"\x00")
    return frame.ljust(TLP_BYTES, b"\x00")


decode_tlp = wire.decode_tlp


class TlpLink:
    """TCP client to the SITL binary. Nagle is off ([SPEC-SITL-03])."""

    def __init__(self, host="127.0.0.1", port=DEFAULT_PORT, timeout=5.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.tlps_in = 0
        self.tlps_out = 0

    def send(self, frame):
        if len(frame) != TLP_BYTES:
            raise ValueError(f"a TLP is {TLP_BYTES} bytes, got {len(frame)}")
        self.sock.sendall(frame)
        self.tlps_out += 1

    def recv(self):
        """The next whole TLP, decoded. Raises ConnectionError if INAV closed the link."""
        buf = bytearray()
        while len(buf) < TLP_BYTES:
            chunk = self.sock.recv(TLP_BYTES - len(buf))
            if not chunk:
                raise ConnectionError("SITL closed the TLP link")
            buf += chunk
        self.tlps_in += 1
        return decode_tlp(bytes(buf))

    def nodelay(self):
        return self.sock.getsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY) != 0

    def close(self):
        self.sock.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
