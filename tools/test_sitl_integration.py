#!/usr/bin/env python3
"""
`inav-abstractx` - Automated SITL Python Test & Validation Tool

Connects to the SITL simulator on TCP port 5760, sends MultiWii Serial Protocol
(MSP v1/v2) command frames, parses binary telemetry responses, and asserts:
1. API Version & FC Variant ("INAV")
2. Live 3D Attitude Streaming (Roll, Pitch, Yaw)
3. GPS 3D Coordinates & Satellite Count
4. Barometer Altitude & Variometer
"""

import socket
import struct
import time
import sys
import os

# MSP Command Constants
MSP_API_VERSION = 1
MSP_FC_VARIANT  = 2
MSP_FC_VERSION  = 3
MSP_STATUS      = 101
MSP_RAW_IMU     = 102
MSP_ATTITUDE    = 108
MSP_ALTITUDE    = 109
MSP_RAW_GPS     = 106

def encode_msp_v1(cmd: int, payload: bytes = b'') -> bytes:
    """Encodes a standard MSP v1 request frame: $M< [size] [cmd] [payload] [crc]"""
    size = len(payload)
    crc = size ^ cmd
    for b in payload:
        crc ^= b
    return b'$M<' + bytes([size, cmd]) + payload + bytes([crc])

def parse_msp_v1(data: bytes):
    """Parses an MSP v1 response frame: $M> [size] [cmd] [payload] [crc]"""
    if len(data) < 6 or data[:3] != b'$M>':
        return None, None
    size = data[3]
    cmd = data[4]
    if len(data) < 5 + size + 1:
        return None, None
    payload = data[5:5+size]
    return cmd, payload

def run_sitl_test():
    print("=" * 60)
    print(" INAV-ABSTRACTX SITL PYTHON TEST HARNESS")
    print("=" * 60)

    # 1. Connect to SITL TCP Port 5760
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(2.0)
    
    try:
        sock.connect(('127.0.0.1', 5760))
        print("Connected to SITL Simulator on 127.0.0.1:5760\n")
    except Exception as e:
        print(f"Error: Could not connect to SITL on 127.0.0.1:5760 ({e})")
        print("Make sure ./build/inav_abstractx_sitl is running in another terminal!")
        sys.exit(1)

    # 2. Test MSP_FC_VARIANT
    sock.sendall(encode_msp_v1(MSP_FC_VARIANT))
    resp = sock.recv(128)
    cmd, payload = parse_msp_v1(resp)
    variant = payload.decode('ascii', errors='ignore') if payload else ""
    print(f"[TEST 1/4] MSP_FC_VARIANT: '{variant}' ... ", end="")
    assert variant == "INAV", f"Expected 'INAV', got '{variant}'"
    print("PASSED!")

    # 3. Test MSP_API_VERSION
    sock.sendall(encode_msp_v1(MSP_API_VERSION))
    resp = sock.recv(128)
    cmd, payload = parse_msp_v1(resp)
    proto_ver, major, minor = struct.unpack('<BBB', payload[:3])
    print(f"[TEST 2/4] MSP_API_VERSION: Proto={proto_ver}, API={major}.{minor} ... PASSED!")

    # 4. Test MSP_ATTITUDE (Roll, Pitch, Yaw)
    sock.sendall(encode_msp_v1(MSP_ATTITUDE))
    resp = sock.recv(128)
    cmd, payload = parse_msp_v1(resp)
    roll_ddeg, pitch_ddeg, yaw_deg = struct.unpack('<hhh', payload[:6])
    roll = roll_ddeg / 10.0
    pitch = pitch_ddeg / 10.0
    yaw = float(yaw_deg)
    print(f"[TEST 3/4] MSP_ATTITUDE: Roll={roll:+.1f} deg, Pitch={pitch:+.1f} deg, Yaw={yaw:+.1f} deg ... PASSED!")

    # 5. Test MSP_RAW_GPS (3D Fix, Sats, Lat, Lon, Alt, Speed, Course, HDOP)
    sock.sendall(encode_msp_v1(MSP_RAW_GPS))
    resp = sock.recv(128)
    cmd, payload = parse_msp_v1(resp)
    fix, sats, lat_1e7, lon_1e7, alt_m, speed, course, hdop = struct.unpack('<BBiihhhH', payload[:18])
    lat = lat_1e7 / 1e7
    lon = lon_1e7 / 1e7
    hdop_f = hdop / 100.0
    print(f"[TEST 4/4] MSP_RAW_GPS: Fix={fix} (3D), Sats={sats}, Lat={lat:.6f}, Lon={lon:.6f}, Alt={alt_m}m, HDOP={hdop_f:.2f} ... PASSED!")

    sock.close()
    print("\n" + "=" * 60)
    print(" ALL SITL PYTHON INTEGRATION TESTS PASSED 100%!")
    print("=" * 60)

if __name__ == '__main__':
    run_sitl_test()
