# Copyright (C) 2026 Tim Michals
# SPDX-License-Identifier: GPL-3.0-or-later
"""@verifies [SPEC-SITL-03] docs/SITL_SIMULATOR.md: the plugin's link client (framing, TCP_NODELAY,
AbstractX's header layout)."""

import socket
import struct
import sys
import threading
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools" / "studio_plugin"))
import sitl_link  # noqa: E402

failures = 0


def check(ok, what):
    global failures
    print(f"{what:<52} {'ok' if ok else 'FAIL'}")
    failures += 0 if ok else 1


# Field offsets of asp_tlp64_t (AbstractX/include/asp_tlp64.h), checked from the bytes.
f = sitl_link.encode_tlp(0x10, 0x02, 0x100C, bytes(range(1, 15)), tag=7, seq=0x1234,
                         timestamp_ns=0x1122334455667788)
check(len(f) == 64, "encoded TLP is 64 bytes")
check(f[0] == 0x10 and f[2] == 7 and f[3] == 0x02, "type@0 tag@2 channel@3")
check(struct.unpack_from("<I", f, 4)[0] == 0x100C, "target_address@4 little-endian")
check(struct.unpack_from("<H", f, 8)[0] == 4, "length_dw@8 = 4 words for 14 bytes")
check(struct.unpack_from("<H", f, 10)[0] == 0x1234, "sequence@10")
check(struct.unpack_from("<Q", f, 12)[0] == 0x1122334455667788, "timestamp_ns@12")
check(f[20:34] == bytes(range(1, 15)), "payload@20")
try:
    sitl_link.encode_tlp(0x10, 0x02, 0, bytes(41))
    check(False, "41-byte payload rejected")
except ValueError:
    check(True, "41-byte payload rejected")

# A server that sends two TLPs in uneven pieces and echoes what it receives.
srv = socket.socket()
srv.bind(("127.0.0.1", 0))
srv.listen(1)
port = srv.getsockname()[1]
a = sitl_link.encode_tlp(0x03, 0x01, 0x1000, b"\x47", tag=1)
b = sitl_link.encode_tlp(0x03, 0x01, 0x1000, b"\x48", tag=2)
echoed = []


def serve():
    c, _ = srv.accept()
    data = a + b
    for cut in (5, 70, 53):
        c.sendall(data[:cut])
        data = data[cut:]
    got = b""
    while len(got) < 64:
        got += c.recv(64 - len(got))
    echoed.append(got)
    c.close()


t = threading.Thread(target=serve)
t.start()
with sitl_link.TlpLink(port=port) as link:
    check(link.nodelay(), "TCP_NODELAY set on the plugin side")
    r1, r2 = link.recv(), link.recv()
    check(r1["tag"] == 1 and r1["payload"][0] == 0x47, "first TLP reassembled")
    check(r2["tag"] == 2 and r2["payload"][0] == 0x48, "second TLP reassembled")
    link.send(f)
    t.join(timeout=5)
    check(echoed and echoed[0] == f, "sent TLP arrives byte for byte")
    try:
        link.recv()
        check(False, "closed link raises ConnectionError")
    except ConnectionError:
        check(True, "closed link raises ConnectionError")
srv.close()

print(f"{failures} failure(s)")
sys.exit(1 if failures else 0)
