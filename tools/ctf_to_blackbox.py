#!/usr/bin/env python3
"""
Copyright (C) 2026 Tim Michals
SPDX-License-Identifier: GPL-3.0-or-later

AbstractX BareCTF Binary Trace to Betaflight/iNav Blackbox (.BBL) Converter
Converts 64-byte CTF TLPs (ASP_CHANNEL_FC_LOG 0x03) into .BBL files for Blackbox Explorer.
"""

import sys
import struct
import argparse

# CTF Packet Header Format (16 bytes)
# uint32 magic (0xC1FC1FC1), uint32 stream_id, uint64 timestamp_ns
CTF_HDR_FMT = "<IIQ"
CTF_HDR_SIZE = struct.calcsize(CTF_HDR_FMT)

# CTF Event Payload Format (24 bytes)
# int16 accel[3], int16 gyro[3], int16 roll_x10, int16 pitch_x10, uint16 yaw, uint16 motor[4]
CTF_EVT_FMT = "<hhhhhhHHHHHH"
CTF_EVT_SIZE = struct.calcsize(CTF_EVT_FMT)

# 64-Byte TLP Header Format
# uint8 type, uint8 flags, uint8 tag, uint8 channel, uint32 target_addr, uint16 len_dw, uint16 seq, uint64 ts
TLP_HDR_FMT = "<BBBBIIQ"
TLP_HDR_SIZE = struct.calcsize(TLP_HDR_FMT)


def convert_ctf_tlp_to_bbl(input_filepath, output_filepath):
    print(f"[AbstractX BBL Converter] Reading CTF TLP log: {input_filepath}")
    print(f"[AbstractX BBL Converter] Outputting Blackbox file: {output_filepath}")

    converted_records = 0

    with open(input_filepath, "rb") as fin, open(output_filepath, "wb") as fout:
        # Write Betaflight / iNav Blackbox Header
        fout.write(b"H Product:Blackbox decode\n")
        fout.write(b"H Data version:2\n")
        fout.write(b"H I field 0 name:loopIteration,time,axisP[0],axisP[1],axisP[2],motor[0],motor[1],motor[2],motor[3]\n")
        fout.write(b"H I field 0 signed:0,0,1,1,1,0,0,0,0\n")

        while True:
            raw_tlp = fin.read(64)
            if len(raw_tlp) < 64:
                break

            # Parse TLP header
            tlp_type, flags, tag, channel, target_addr, len_dw, seq, ts_ns = struct.unpack(
                TLP_HDR_FMT, raw_tlp[:TLP_HDR_SIZE]
            )

            # Check if channel is FlightLog (0x03)
            if channel == 0x03:
                payload = raw_tlp[24:64] # 40-byte TLP payload
                
                # Check CTF Magic
                magic, stream_id, ctf_ts = struct.unpack(CTF_HDR_FMT, payload[:CTF_HDR_SIZE])
                if magic == 0xC1FC1FC1:
                    # Unpack CTF Event
                    evt_data = struct.unpack(CTF_EVT_FMT, payload[CTF_HDR_SIZE:CTF_HDR_SIZE+CTF_EVT_SIZE])
                    ax, ay, az, gx, gy, gz, roll, pitch, yaw, m1, m2, m3, m4 = evt_data

                    # Write I-Frame to .BBL file
                    bbl_frame = f"I,{converted_records},{ctf_ts // 1000},{roll},{pitch},{yaw},{m1},{m2},{m3},{m4}\n".encode('ascii')
                    fout.write(bbl_frame)
                    converted_records += 1

    print(f"[SUCCESS] Converted {converted_records} CTF log records to {output_filepath}")


import socket

def capture_live_udp_to_bbl(output_filepath, port=19000):
    print(f"[AbstractX BBL Converter] Listening for live BareCTF UDP stream on 0.0.0.0:{port}...")
    print(f"[AbstractX BBL Converter] Outputting live Blackbox file: {output_filepath}")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", port))

    converted_records = 0

    with open(output_filepath, "wb") as fout:
        # Write Betaflight / iNav Blackbox Header
        fout.write(b"H Product:Blackbox decode\n")
        fout.write(b"H Data version:2\n")
        fout.write(b"H I field 0 name:loopIteration,time,axisP[0],axisP[1],axisP[2],motor[0],motor[1],motor[2],motor[3]\n")
        fout.write(b"H I field 0 signed:0,0,1,1,1,0,0,0,0\n")
        fout.flush()

        try:
            while True:
                data, addr = sock.recvfrom(2048)
                offset = 0
                while offset + 64 <= len(data):
                    raw_tlp = data[offset:offset+64]
                    offset += 64

                    tlp_type, flags, tag, channel, target_addr, len_dw, seq, ts_ns = struct.unpack(
                        TLP_HDR_FMT, raw_tlp[:TLP_HDR_SIZE]
                    )

                    if channel == 0x03:
                        payload = raw_tlp[24:64]
                        magic, stream_id, ctf_ts = struct.unpack(CTF_HDR_FMT, payload[:CTF_HDR_SIZE])
                        if magic == 0xC1FC1FC1:
                            evt_data = struct.unpack(CTF_EVT_FMT, payload[CTF_HDR_SIZE:CTF_HDR_SIZE+CTF_EVT_SIZE])
                            ax, ay, az, gx, gy, gz, roll, pitch, yaw, m1, m2, m3, m4 = evt_data

                            bbl_frame = f"I,{converted_records},{ctf_ts // 1000},{roll},{pitch},{yaw},{m1},{m2},{m3},{m4}\n".encode('ascii')
                            fout.write(bbl_frame)
                            converted_records += 1

                            if converted_records % 500 == 0:
                                fout.flush()
                                print(f"[AbstractX BBL Converter] Streamed {converted_records} frames...", end='\r')
        except KeyboardInterrupt:
            print(f"\n[AbstractX BBL Converter] Capture stopped. Total frames: {converted_records}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="AbstractX CTF TLP to Blackbox Explorer Converter")
    parser.add_argument("input", nargs="?", help="Input CTF 64B TLP binary log file (omit if --live is used)")
    parser.add_argument("output", help="Output .BBL Blackbox file")
    parser.add_argument("--live", action="store_true", help="Listen for live UDP telemetry stream")
    parser.add_argument("--port", type=int, default=19000, help="UDP port for live stream (default: 19000)")
    args = parser.parse_args()

    if args.live:
        capture_live_udp_to_bbl(args.output, args.port)
    elif args.input:
        convert_ctf_tlp_to_bbl(args.input, args.output)
    else:
        parser.print_help()
        sys.exit(1)

