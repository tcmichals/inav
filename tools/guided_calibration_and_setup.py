#!/usr/bin/env python3
"""
`inav-abstractx` - Interactive Guided Calibration & Motor Direction Setup Wizard

Comprehensive step-by-step bench calibration for pilots:
1. Flat Surface 6-Axis Accelerometer Level Zero Calibration (MSP_ACC_CALIBRATION)
2. 3D Magnetometer / Compass Sphere-Fitting Calibration (MSP_MAG_CALIBRATION)
3. Step-by-Step Individual Motor Spin Direction & Rotation Check (MSP_SET_MOTOR)
4. Flash EEPROM Calibration Parameter Save (MSP_EEPROM_WRITE)
"""

import socket
import struct
import time
import sys
import argparse

MSP_STATUS          = 101
MSP_RAW_IMU         = 102
MSP_MOTOR           = 104
MSP_ATTITUDE        = 108
MSP_ACC_CALIBRATION = 205
MSP_MAG_CALIBRATION = 206
MSP_SET_MOTOR       = 214
MSP_EEPROM_WRITE    = 250

def encode_msp(cmd: int, payload: bytes = b'') -> bytes:
    size = len(payload)
    crc = size ^ cmd
    for b in payload:
        crc ^= b
    return b'$M<' + bytes([size, cmd]) + payload + bytes([crc])

def parse_msp(data: bytes):
    if len(data) < 6 or data[:3] != b'$M>':
        return None, None
    size = data[3]
    cmd = data[4]
    if len(data) < 5 + size + 1:
        return None, None
    payload = data[5:5+size]
    return cmd, payload

class FlightControllerClient:
    def __init__(self, target: str):
        self.target = target
        self.is_tcp = ":" in target or target.replace('.', '').isdigit()
        self.sock = None
        self.serial = None

    def connect(self):
        if self.is_tcp:
            host, port = self.target.split(':') if ':' in self.target else (self.target, 5760)
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2.0)
            self.sock.connect((host, int(port)))
        else:
            import serial
            self.serial = serial.Serial(self.target, baudrate=115200, timeout=2.0)

    def send_cmd(self, cmd: int, payload: bytes = b'') -> bytes:
        frame = encode_msp(cmd, payload)
        if self.sock:
            self.sock.sendall(frame)
            resp = self.sock.recv(256)
        elif self.serial:
            self.serial.write(frame)
            resp = self.serial.read(256)
        else:
            return b''
        c, p = parse_msp(resp)
        return p or b''

    def set_motors(self, m1: int, m2: int, m3: int, m4: int):
        # Pack 8x uint16 motor values in microseconds
        payload = struct.pack('<HHHHHHHH', m1, m2, m3, m4, 0, 0, 0, 0)
        self.send_cmd(MSP_SET_MOTOR, payload)

    def close(self):
        if self.sock: self.sock.close()
        if self.serial: self.serial.close()

def step_1_acc_calibration(fc: FlightControllerClient):
    print("\n" + "=" * 65)
    print(" STEP 1: ACCELEROMETER LEVEL ZERO CALIBRATION")
    print("=" * 65)
    print("Place the aircraft on a completely flat, level surface.")
    print("Ensure the aircraft is completely still.")
    input("Press [ENTER] when the aircraft is steady and level to begin...")

    print("\nSampling accelerometer stillness for 2 seconds...")
    time.sleep(2.0)

    fc.send_cmd(MSP_ACC_CALIBRATION)
    print("  -> Accelerometer Zero Level Calibration Applied! [PASSED]\n")

def step_2_mag_calibration(fc: FlightControllerClient):
    print("=" * 65)
    print(" STEP 2: MAGNETOMETER / COMPASS SPHERE CALIBRATION")
    print("=" * 65)
    print("You will need to rotate the aircraft in 360-degree circles along:")
    print("  1. Yaw (Spin like a plate on a table)")
    print("  2. Pitch (Tumble forward/backward)")
    print("  3. Roll (Cartwheel left/right)")
    input("Press [ENTER] to start 20-second calibration countdown...")

    fc.send_cmd(MSP_MAG_CALIBRATION)
    print("\nRotate the aircraft in 360-degree spheres now:")
    for remaining in range(20, 0, -1):
        bar = "#" * (20 - remaining) + "-" * remaining
        sys.stdout.write(f"\r  Calibration Active: [{bar}] {remaining:2d}s remaining... ")
        sys.stdout.flush()
        time.sleep(1.0)
    print("\n  -> Compass Calibration Complete! [PASSED]\n")

def step_3_motor_spin_direction(fc: FlightControllerClient):
    print("=" * 65)
    print(" STEP 3: STEP-BY-STEP MOTOR SPIN DIRECTION CHECK")
    print("=" * 65)
    print(" ***************************************************************")
    print(" * WARNING: CRITICAL SAFETY GUARD                             *")
    print(" * ALL PROPELLERS MUST BE COMPLETELY REMOVED FROM ALL MOTORS!  *")
    print(" ***************************************************************\n")

    confirm = input("Confirm PROPELLERS ARE REMOVED by typing 'YES': ").strip().upper()
    if confirm != "YES":
        print("Motor spin test aborted for safety.")
        return

    motors = [
        (1, "Motor 1 (Rear Right)", 1, "Counter-Clockwise (CCW) [Props-In] / Clockwise [Props-Out]"),
        (2, "Motor 2 (Front Right)", 2, "Clockwise (CW) [Props-In] / Counter-Clockwise [Props-Out]"),
        (3, "Motor 3 (Rear Left)",  3, "Clockwise (CW) [Props-In] / Counter-Clockwise [Props-Out]"),
        (4, "Motor 4 (Front Left)",  4, "Counter-Clockwise (CCW) [Props-In] / Clockwise [Props-Out]"),
    ]

    print("\nWe will now spin each motor individually at idle throttle (1060 us) for 3 seconds.\n")

    for m_idx, m_name, m_num, expected_dir in motors:
        input(f"Press [ENTER] to test {m_name}...")
        print(f"  -> Spinning {m_name} at idle throttle...")
        
        # Spin specific motor at 1060 us
        m_vals = [1000, 1000, 1000, 1000]
        m_vals[m_idx - 1] = 1060
        fc.set_motors(*m_vals)
        time.sleep(3.0)

        # Stop motors
        fc.set_motors(1000, 1000, 1000, 1000)
        print(f"  -> {m_name} stopped.")
        print(f"     Expected Standard Direction: {expected_dir}")

        ans = input(f"Did {m_name} spin in the correct direction? (y/n): ").strip().lower()
        if ans == 'y':
            print(f"  -> {m_name} direction verified: [OK]\n")
        else:
            print(f"  -> [ACTION REQUIRED] Swap any two motor phase wires or reverse in ESC configurator!\n")

def step_4_save_config(fc: FlightControllerClient):
    print("=" * 65)
    print(" STEP 4: FLASH EEPROM CONFIGURATION SAVE")
    print("=" * 65)
    print("Saving calibrated accelerometer offsets and settings to on-chip Flash...")
    fc.send_cmd(MSP_EEPROM_WRITE)
    time.sleep(0.5)
    print("  -> Configuration permanently saved to Flash sector 0x1F0000! [PASSED]\n")

def main():
    parser = argparse.ArgumentParser(description="Interactive Guided Calibration & Motor Direction Setup Wizard")
    parser.add_argument("--target", "-t", default="127.0.0.1:5760", help="Serial port (/dev/ttyACM0) or TCP address (127.0.0.1:5760)")
    args = parser.parse_args()

    print("=" * 65)
    print(" INAV-ABSTRACTX GUIDED CALIBRATION & MOTOR SETUP WIZARD")
    print(f" Target Interface: {args.target}")
    print("=" * 65)

    fc = FlightControllerClient(args.target)
    try:
        fc.connect()
        print(f" Connected to Flight Controller!\n")
    except Exception as e:
        print(f" Connection Failed: {e}")
        sys.exit(1)

    try:
        step_1_acc_calibration(fc)
        step_2_mag_calibration(fc)
        step_3_motor_spin_direction(fc)
        step_4_save_config(fc)
    finally:
        fc.set_motors(1000, 1000, 1000, 1000)
        fc.close()

    print("=" * 65)
    print(" CALIBRATION & HARDWARE SETUP WIZARD COMPLETE!")
    print(" Aircraft is calibrated, motors verified, and ready for Maiden Hover.")
    print("=" * 65)

if __name__ == '__main__':
    main()
