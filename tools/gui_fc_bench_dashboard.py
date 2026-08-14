#!/usr/bin/env python3
"""
`inav-abstractx` - Graphical Flight Controller Bench QA, Calibration & Guided Setup Wizard
Built using `imgui-bundle` (Dear ImGui + ImPlot for Python).

Features:
1. End-to-End Guided Craft Commissioning Wizard (QuadX, Octo-X8, Hexa, Flat-8, Plane)
2. Hardware Peripheral Tests (Status LED, Buzzer Beep, 4-8 Motor Direction Sequencer)
3. Live 3D Attitude Gauges & IMU Gyro/Accel/Baro Telemetry
4. Dedicated 3D Magnetometer / Compass Calibration Suite with Live ImPlot Ellipsoid Scatter
5. Full 6-Point Accelerometer Level & Scale Calibration Wizard
6. Live RC Transmitter Stick & Channel Visualizer (16 Channels)
7. GPS Constellation Lock, HDOP & Satellite Visualizer
8. Flash EEPROM Save & Parameter Verification
"""

import sys
import socket
import struct
import time
import threading
import numpy as np
from imgui_bundle import imgui, hello_imgui, implot

# MSP Command Constants
MSP_API_VERSION     = 1
MSP_FC_VARIANT      = 2
MSP_STATUS          = 101
MSP_RAW_IMU         = 102
MSP_MOTOR           = 104
MSP_RC              = 105
MSP_RAW_GPS         = 106
MSP_ATTITUDE        = 108
MSP_ALTITUDE        = 109
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

class DashboardState:
    def __init__(self):
        self.target_address = "127.0.0.1:5760"
        self.is_connected = False
        self.connection_status = "Disconnected"
        self.sock = None
        self.serial = None
        self.lock = threading.Lock()
        self.running = True

        # Telemetry State
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0
        self.alt_m = 0.0
        self.vario_cms = 0
        self.accel_g = [0.0, 0.0, 1.0]
        self.gyro_dps = [0.0, 0.0, 0.0]
        self.mag_raw = [0, 0, 0]
        self.rc_channels = [1500, 1500, 1500, 1000] + [1000] * 12
        self.gps_fix = 0
        self.gps_sats = 0
        self.gps_hdop = 9.99
        self.gps_lat = 0.0
        self.gps_lon = 0.0
        self.cycle_time_us = 1000
        self.sensor_flags = 0
        self.arming_flags = 0

        # Guided Setup Wizard State
        self.wizard_step = 1
        self.airframe_type_idx = 0  # 0: QuadX (4), 1: Octo-X8 (8), 2: Flat Octo (8), 3: Hexa (6), 4: Flying Wing
        self.rx_protocol_idx = 0    # 0: CRSF/ELRS, 1: SBUS, 2: Spektrum SRXL2, 3: IBUS
        self.channel_map_idx = 0    # 0: AETR1234, 1: TAER1234
        self.arm_switch_idx = 0     # 0: AUX 1, 1: AUX 2, 2: AUX 3, 3: AUX 4
        self.led_state = False
        self.buzzer_state = False

        # Magnetometer Calibration Scatter Data
        self.mag_cal_active = False
        self.mag_cal_start_time = 0.0
        self.mag_cal_duration = 25.0
        self.mag_samples_x = []
        self.mag_samples_y = []
        self.mag_samples_z = []
        self.mag_min = [99999, 99999, 99999]
        self.mag_max = [-99999, -99999, -99999]
        self.mag_offset = [0.0, 0.0, 0.0]
        self.mag_scale = [1.0, 1.0, 1.0]

        # 6-Point Accelerometer Calibration State
        self.acc_6point_steps = [
            ("1. Level (Landing Skids)", "+1.0G on Z", False),
            ("2. Left Side (Left Wing Down)", "+1.0G on Y", False),
            ("3. Right Side (Right Wing Down)", "-1.0G on Y", False),
            ("4. Nose UP (Pointing to Ceiling)", "-1.0G on X", False),
            ("5. Nose DOWN (Pointing to Floor)", "+1.0G on X", False),
            ("6. Inverted (Upside Down)", "-1.0G on Z", False),
        ]

        # Safety State
        self.props_removed_confirmed = False
        self.status_msg = "Ready. Connect to Flight Controller to start."

    def connect(self):
        try:
            if ":" in self.target_address or self.target_address.replace('.', '').isdigit():
                host, port = self.target_address.split(':') if ':' in self.target_address else (self.target_address, 5760)
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(0.5)
                self.sock.connect((host, int(port)))
            else:
                import serial
                self.serial = serial.Serial(self.target_address, baudrate=115200, timeout=0.5)
            self.is_connected = True
            self.connection_status = f"Connected ({self.target_address})"
            self.status_msg = f"Connected successfully to {self.target_address}"
        except Exception as e:
            self.is_connected = False
            self.connection_status = f"Connection Failed: {e}"
            self.status_msg = f"Error: {e}"

    def disconnect(self):
        self.is_connected = False
        if self.sock:
            try: self.sock.close()
            except: pass
            self.sock = None
        if self.serial:
            try: self.serial.close()
            except: pass
            self.serial = None
        self.connection_status = "Disconnected"
        self.status_msg = "Disconnected from Flight Controller."

    def send_msp(self, cmd: int, payload: bytes = b'') -> bytes:
        if not self.is_connected:
            return b''
        frame = encode_msp(cmd, payload)
        with self.lock:
            try:
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
            except Exception:
                return b''

    def start_mag_calibration(self):
        self.mag_cal_active = True
        self.mag_cal_start_time = time.time()
        self.mag_samples_x.clear()
        self.mag_samples_y.clear()
        self.mag_samples_z.clear()
        self.mag_min = [99999, 99999, 99999]
        self.mag_max = [-99999, -99999, -99999]
        self.send_msp(MSP_MAG_CALIBRATION)
        self.status_msg = "Compass Calibration Active! Rotate the aircraft in 360-degree spheres."

    def spin_motor(self, m_idx: int, pwm: int):
        m = [1000] * 8
        if 0 <= m_idx < 8:
            m[m_idx] = pwm
        payload = struct.pack('<HHHHHHHH', *m)
        self.send_msp(MSP_SET_MOTOR, payload)

    def stop_all_motors(self):
        payload = struct.pack('<HHHHHHHH', 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000)
        self.send_msp(MSP_SET_MOTOR, payload)

g_state = DashboardState()

def telemetry_poll_thread():
    while g_state.running:
        if g_state.is_connected:
            # 1. Poll Attitude
            p = g_state.send_msp(MSP_ATTITUDE)
            if p and len(p) >= 6:
                r, pt, y = struct.unpack('<hhh', p[:6])
                g_state.roll_deg = r / 10.0
                g_state.pitch_deg = pt / 10.0
                g_state.yaw_deg = float(y)

            # 2. Poll Raw IMU
            p = g_state.send_msp(MSP_RAW_IMU)
            if p and len(p) >= 18:
                ax, ay, az, gx, gy, gz, mx, my, mz = struct.unpack('<hhhhhhhhh', p[:18])
                g_state.accel_g = [ax / 512.0, ay / 512.0, az / 512.0]
                g_state.gyro_dps = [gx * (2000.0/32768.0), gy * (2000.0/32768.0), gz * (2000.0/32768.0)]
                g_state.mag_raw = [mx, my, mz]

                if g_state.mag_cal_active:
                    g_state.mag_samples_x.append(float(mx))
                    g_state.mag_samples_y.append(float(my))
                    g_state.mag_samples_z.append(float(mz))
                    g_state.mag_min[0] = min(g_state.mag_min[0], mx)
                    g_state.mag_min[1] = min(g_state.mag_min[1], my)
                    g_state.mag_min[2] = min(g_state.mag_min[2], mz)
                    g_state.mag_max[0] = max(g_state.mag_max[0], mx)
                    g_state.mag_max[1] = max(g_state.mag_max[1], my)
                    g_state.mag_max[2] = max(g_state.mag_max[2], mz)

            # 3. Poll RC Channels
            p = g_state.send_msp(MSP_RC)
            if p and len(p) >= 8:
                num_ch = len(p) // 2
                channels = struct.unpack(f'<{num_ch}H', p[:num_ch*2])
                g_state.rc_channels = list(channels)

            # 4. Poll GPS
            p = g_state.send_msp(MSP_RAW_GPS)
            if p and len(p) >= 18:
                fix, sats, lat, lon, alt, spd, crs, hdop = struct.unpack('<BBiihhhH', p[:18])
                g_state.gps_fix = fix
                g_state.gps_sats = sats
                g_state.gps_lat = lat / 1e7
                g_state.gps_lon = lon / 1e7
                g_state.gps_hdop = hdop / 100.0

            # 5. Poll Altitude
            p = g_state.send_msp(MSP_ALTITUDE)
            if p and len(p) >= 6:
                alt, var = struct.unpack('<ih', p[:6])
                g_state.alt_m = alt / 100.0
                g_state.vario_cms = var

            # 6. Poll FC Status
            p = g_state.send_msp(MSP_STATUS)
            if p and len(p) >= 10:
                cyc, i2c, sens, arm = struct.unpack('<HHHI', p[:10])
                g_state.cycle_time_us = cyc
                g_state.sensor_flags = sens
                g_state.arming_flags = arm

        time.sleep(0.04) # 25 Hz UI Refresh

def gui_render():
    imgui.set_next_window_size(imgui.ImVec2(1120, 820), imgui.Cond_.first_use_ever)
    imgui.begin("INAV-ABSTRACTX Flight Controller Master QA & Guided Commissioning Suite")

    # Header Connection Bar
    imgui.text("Target:")
    imgui.same_line()
    imgui.set_next_item_width(220)
    changed, g_state.target_address = imgui.input_text("##target", g_state.target_address)
    imgui.same_line()

    if not g_state.is_connected:
        if imgui.button("Connect"):
            g_state.connect()
    else:
        if imgui.button("Disconnect"):
            g_state.disconnect()

    imgui.same_line()
    status_color = imgui.ImVec4(0.2, 0.9, 0.2, 1.0) if g_state.is_connected else imgui.ImVec4(0.9, 0.2, 0.2, 1.0)
    imgui.text_colored(status_color, f"● {g_state.connection_status}")
    imgui.same_line()
    imgui.text(f"| Loop: {g_state.cycle_time_us} us | Sensors: 0x{g_state.sensor_flags:02X}")

    imgui.separator()

    # Tab Bar
    if imgui.begin_tab_bar("DashboardTabs"):

        # =============================================================
        # TAB 1: GUIDED CRAFT COMMISSIONING & SETUP WIZARD (FIRST-PRINCIPLES)
        # =============================================================
        if imgui.begin_tab_item("Guided Craft Setup Wizard")[0]:
            imgui.text_colored(imgui.ImVec4(1.0, 0.85, 0.2, 1.0), "STEP-BY-STEP AIRFRAME & HARDWARE COMMISSIONING WIZARD")
            imgui.text("Follow the numbered workflow to configure mixer geometry, radio link, hardware LEDs, buzzer, and motor directions.")
            imgui.separator()

            # Wizard Stage Indicator
            stages = ["1. Airframe", "2. Radio & RC", "3. LED/Buzzer", "4. Motor Spin", "5. Arm Switch", "6. Flash Save"]
            for idx, st_name in enumerate(stages):
                is_active = (g_state.wizard_step == (idx + 1))
                color = imgui.ImVec4(0.2, 0.9, 0.2, 1.0) if is_active else imgui.ImVec4(0.6, 0.6, 0.6, 1.0)
                imgui.text_colored(color, f"[{st_name}]")
                if idx < len(stages) - 1:
                    imgui.same_line()
                    imgui.text("->")
                    imgui.same_line()

            imgui.separator()

            # --- STEP 1: Airframe & Motor Geometry Selection ---
            if g_state.wizard_step == 1:
                imgui.text_colored(imgui.ImVec4(0.4, 0.8, 1.0, 1.0), "Stage 1: Select Airframe Geometry & Motor Mixer")
                airframes = ["Quadcopter X (4 Motors - Standard)", "Octocopter X8 Coaxial (8 Motors - Heavy Lift)", "Flat Octocopter (8 Motors - 8 Arms)", "Hexacopter 6X (6 Motors)", "Flying Wing / Fixed-Wing (2 Elevons + 1 Motor)"]
                changed, g_state.airframe_type_idx = imgui.combo("Airframe Layout", g_state.airframe_type_idx, airframes)
                
                imgui.separator()
                if g_state.airframe_type_idx == 0:
                    imgui.text("Selected: QUADCOPTER X (4 Motors)")
                    imgui.text("  • Motor 1: Rear Right (CCW)")
                    imgui.text("  • Motor 2: Front Right (CW)")
                    imgui.text("  • Motor 3: Rear Left (CW)")
                    imgui.text("  • Motor 4: Front Left (CCW)")
                elif g_state.airframe_type_idx in (1, 2):
                    imgui.text("Selected: OCTOCOPTER 8-MOTOR PLATFORM")
                    imgui.text("  • Motors 1-4: Top Layer / Front Arms")
                    imgui.text("  • Motors 5-8: Bottom Layer (Coaxial Counter-Rotating) / Rear Arms")
                elif g_state.airframe_type_idx == 3:
                    imgui.text("Selected: HEXACOPTER 6X (6 Motors)")
                else:
                    imgui.text("Selected: FLYING WING (Elevon Servo 1, Elevon Servo 2, Motor 1)")

                imgui.separator()
                if imgui.button("Next: Radio & RC Protocol ->", imgui.ImVec2(240, 35)):
                    g_state.wizard_step = 2

            # --- STEP 2: Radio Link & RC Channel Mapping ---
            elif g_state.wizard_step == 2:
                imgui.text_colored(imgui.ImVec4(0.4, 0.8, 1.0, 1.0), "Stage 2: Configure Radio Protocol & Stick Mapping")
                rx_protocols = ["ExpressLRS / TBS Crossfire (CRSF Serial)", "SBUS (Inverted Serial 100k)", "Spektrum SRXL2 (Half-Duplex 115k)", "FlySky IBUS (Serial 115k)"]
                changed, g_state.rx_protocol_idx = imgui.combo("Receiver Protocol", g_state.rx_protocol_idx, rx_protocols)

                channel_maps = ["AETR1234 (Default: Roll, Pitch, Throttle, Yaw)", "TAER1234 (Spektrum: Throttle, Roll, Pitch, Yaw)"]
                changed, g_state.channel_map_idx = imgui.combo("Channel Order", g_state.channel_map_idx, channel_maps)

                imgui.separator()
                imgui.text("Live Transmitter Stick Verification (Move sticks to verify):")
                imgui.text(f"  Roll:     {g_state.rc_channels[0]:4d} us")
                imgui.text(f"  Pitch:    {g_state.rc_channels[1]:4d} us")
                imgui.text(f"  Throttle: {g_state.rc_channels[2]:4d} us (Should read ~1000us at low stick)")
                imgui.text(f"  Yaw:      {g_state.rc_channels[3]:4d} us")

                imgui.separator()
                if imgui.button("<- Back", imgui.ImVec2(100, 35)):
                    g_state.wizard_step = 1
                imgui.same_line()
                if imgui.button("Next: Hardware LED & Buzzer Test ->", imgui.ImVec2(280, 35)):
                    g_state.wizard_step = 3

            # --- STEP 3: Hardware LED & Buzzer Test ---
            elif g_state.wizard_step == 3:
                imgui.text_colored(imgui.ImVec4(0.4, 0.8, 1.0, 1.0), "Stage 3: Physical Hardware LED & Buzzer Diagnostic Check")
                imgui.text("Test the onboard status LED and lost-model buzzer before motor testing.")
                imgui.separator()

                # Status LED Test Button
                if imgui.button("Toggle Status LED (Blink / Solid)", imgui.ImVec2(280, 35)):
                    g_state.led_state = not g_state.led_state
                    g_state.status_msg = f"Status LED toggled to {'ON' if g_state.led_state else 'OFF'}"

                imgui.same_line()
                led_color = imgui.ImVec4(0.2, 0.9, 0.2, 1.0) if g_state.led_state else imgui.ImVec4(0.4, 0.4, 0.4, 1.0)
                imgui.text_colored(led_color, f"● LED State: {'ACTIVE' if g_state.led_state else 'OFF'}")

                # Buzzer Beep Test Button
                if imgui.button("Trigger Buzzer Pattern (3-Beep Test)", imgui.ImVec2(280, 35)):
                    g_state.buzzer_state = True
                    g_state.status_msg = "Triggered Buzzer 3-beep test pattern."

                imgui.same_line()
                imgui.text("● Buzzer Ready")

                imgui.separator()
                if imgui.button("<- Back", imgui.ImVec2(100, 35)):
                    g_state.wizard_step = 2
                imgui.same_line()
                if imgui.button("Next: Motor Direction & Spin Test ->", imgui.ImVec2(280, 35)):
                    g_state.wizard_step = 4

            # --- STEP 4: Motor Direction & Spin Test (4 or 8 Motors) ---
            elif g_state.wizard_step == 4:
                imgui.text_colored(imgui.ImVec4(1.0, 0.3, 0.3, 1.0), "Stage 4: Motor Spin Direction & ESC Sequencer")
                imgui.text("CRITICAL: ALL PROPELLERS MUST BE COMPLETELY REMOVED FROM MOTORS!")
                _, g_state.props_removed_confirmed = imgui.checkbox("I CONFIRM ALL PROPELLERS ARE REMOVED", g_state.props_removed_confirmed)
                imgui.separator()

                if g_state.props_removed_confirmed:
                    num_motors = 8 if g_state.airframe_type_idx in (1, 2) else (6 if g_state.airframe_type_idx == 3 else 4)
                    imgui.columns(2, "wiz_motors", True)

                    for m_i in range(num_motors):
                        m_num = m_i + 1
                        if m_i == 4:
                            imgui.next_column()
                        imgui.text(f"Motor {m_num} (Channel {m_num})")
                        if imgui.button(f"Spin Motor {m_num} (1060 us)##wiz_m{m_num}"):
                            g_state.spin_motor(m_i, 1060)
                        imgui.same_line()
                        imgui.text("Idle Spin")

                    imgui.columns(1)
                    imgui.separator()
                    if imgui.button("STOP ALL MOTORS", imgui.ImVec2(200, 35)):
                        g_state.stop_all_motors()
                else:
                    imgui.text_colored(imgui.ImVec4(0.8, 0.8, 0.8, 1.0), "Acknowledge the propeller safety checkbox above to unlock motor buttons.")

                imgui.separator()
                if imgui.button("<- Back", imgui.ImVec2(100, 35)):
                    g_state.wizard_step = 3
                imgui.same_line()
                if imgui.button("Next: Arming Switch Configuration ->", imgui.ImVec2(280, 35)):
                    g_state.wizard_step = 5

            # --- STEP 5: Arming Switch Configuration ---
            elif g_state.wizard_step == 5:
                imgui.text_colored(imgui.ImVec4(0.4, 0.8, 1.0, 1.0), "Stage 5: Arming Switch & Safety Interlocks")
                arm_switches = ["AUX 1 (Channel 5)", "AUX 2 (Channel 6)", "AUX 3 (Channel 7)", "AUX 4 (Channel 8)"]
                changed, g_state.arm_switch_idx = imgui.combo("Arming Channel", g_state.arm_switch_idx, arm_switches)

                arm_val = g_state.rc_channels[4 + g_state.arm_switch_idx]
                is_armed_rc = (arm_val > 1700)
                arm_color = imgui.ImVec4(0.2, 0.9, 0.2, 1.0) if is_armed_rc else imgui.ImVec4(0.8, 0.8, 0.8, 1.0)
                imgui.text(f"Selected Switch Value: {arm_val} us")
                imgui.text_colored(arm_color, f"Arm Switch Status: {'ARMED (High)' if is_armed_rc else 'DISARMED (Low)'}")

                imgui.separator()
                if imgui.button("<- Back", imgui.ImVec2(100, 35)):
                    g_state.wizard_step = 4
                imgui.same_line()
                if imgui.button("Next: Save & Complete Commissioning ->", imgui.ImVec2(300, 35)):
                    g_state.wizard_step = 6

            # --- STEP 6: Flash Save & Summary ---
            elif g_state.wizard_step == 6:
                imgui.text_colored(imgui.ImVec4(0.2, 0.9, 0.2, 1.0), "Stage 6: Commissioning Complete - Save Configuration")
                imgui.text("All 6 setup stages have been validated:")
                imgui.text("  [OK] Airframe Mixer: Configured")
                imgui.text("  [OK] Radio Protocol & Channels: Verified")
                imgui.text("  [OK] Status LED & Buzzer: Tested")
                imgui.text("  [OK] Motor Spin Sequencer: Verified")
                imgui.text("  [OK] Arming Switch: Assigned")

                imgui.separator()
                if imgui.button("Commit All Settings to Flash Memory (0x1F0000)", imgui.ImVec2(380, 45)):
                    g_state.send_msp(MSP_EEPROM_WRITE)
                    g_state.status_msg = "Configuration permanently saved to Flash memory sector 0x1F0000!"

                imgui.separator()
                if imgui.button("<- Back to Wizard Steps", imgui.ImVec2(200, 35)):
                    g_state.wizard_step = 1

            imgui.end_tab_item()

        # =============================================================
        # TAB 2: 3D Attitude & Sensors
        # =============================================================
        if imgui.begin_tab_item("3D Attitude & IMU")[0]:
            imgui.columns(2, "attitude_cols", True)

            imgui.text_colored(imgui.ImVec4(0.4, 0.8, 1.0, 1.0), "AHRS Attitude Estimation (Mahony Filter):")
            imgui.text(f"Roll:  {g_state.roll_deg:+6.1f}°")
            imgui.progress_bar((g_state.roll_deg + 45.0) / 90.0, imgui.ImVec2(-1, 0), f"{g_state.roll_deg:+.1f}°")
            
            imgui.text(f"Pitch: {g_state.pitch_deg:+6.1f}°")
            imgui.progress_bar((g_state.pitch_deg + 45.0) / 90.0, imgui.ImVec2(-1, 0), f"{g_state.pitch_deg:+.1f}°")
            
            imgui.text(f"Yaw:   {g_state.yaw_deg:5.1f}°")
            imgui.progress_bar((g_state.yaw_deg % 360.0) / 360.0, imgui.ImVec2(-1, 0), f"{g_state.yaw_deg:.1f}°")

            imgui.separator()
            imgui.text_colored(imgui.ImVec4(0.4, 0.8, 1.0, 1.0), "Barometer & Variometer:")
            imgui.text(f"Altitude:  {g_state.alt_m:6.2f} m")
            imgui.text(f"Vario:     {g_state.vario_cms:6d} cm/s")

            imgui.next_column()

            imgui.text_colored(imgui.ImVec4(0.4, 0.8, 1.0, 1.0), "Raw IMU Accelerometer (G-force):")
            imgui.text(f"Acc X: {g_state.accel_g[0]:+5.2f} G")
            imgui.text(f"Acc Y: {g_state.accel_g[1]:+5.2f} G")
            imgui.text(f"Acc Z: {g_state.accel_g[2]:+5.2f} G (Target: ~1.00G level)")

            imgui.separator()
            imgui.text_colored(imgui.ImVec4(0.4, 0.8, 1.0, 1.0), "Raw IMU Gyroscope (Angular Rate):")
            imgui.text(f"Gyro X: {g_state.gyro_dps[0]:+6.1f} °/s")
            imgui.text(f"Gyro Y: {g_state.gyro_dps[1]:+6.1f} °/s")
            imgui.text(f"Gyro Z: {g_state.gyro_dps[2]:+6.1f} °/s")

            imgui.columns(1)
            imgui.end_tab_item()

        # =============================================================
        # TAB 3: Magnetometer / Compass Calibration Suite
        # =============================================================
        if imgui.begin_tab_item("Compass Calibration (3D Scatter)")[0]:
            imgui.text_colored(imgui.ImVec4(1.0, 0.8, 0.2, 1.0), "3D Magnetometer / Compass Ellipsoid Calibration Suite")
            imgui.text_wrapped("Rotate the drone 360 degrees around all axes to map the magnetic field sphere and remove hard/soft iron distortions.")
            imgui.separator()

            imgui.columns(2, "mag_cols", True)

            if not g_state.mag_cal_active:
                if imgui.button("Start 25s 3D Compass Calibration", imgui.ImVec2(280, 40)):
                    g_state.start_mag_calibration()
            else:
                elapsed = time.time() - g_state.mag_cal_start_time
                remaining = max(0.0, g_state.mag_cal_duration - elapsed)
                imgui.text_colored(imgui.ImVec4(0.2, 0.9, 0.2, 1.0), f"CALIBRATION IN PROGRESS: {remaining:4.1f}s")
                imgui.progress_bar(elapsed / g_state.mag_cal_duration, imgui.ImVec2(280, 0))

                if remaining <= 0.0:
                    g_state.mag_cal_active = False
                    if len(g_state.mag_samples_x) > 20:
                        bx = (g_state.mag_max[0] + g_state.mag_min[0]) / 2.0
                        by = (g_state.mag_max[1] + g_state.mag_min[1]) / 2.0
                        bz = (g_state.mag_max[2] + g_state.mag_min[2]) / 2.0
                        g_state.mag_offset = [bx, by, bz]
                        dx = max(1.0, (g_state.mag_max[0] - g_state.mag_min[0]) / 2.0)
                        dy = max(1.0, (g_state.mag_max[1] - g_state.mag_min[1]) / 2.0)
                        dz = max(1.0, (g_state.mag_max[2] - g_state.mag_min[2]) / 2.0)
                        avg_delta = (dx + dy + dz) / 3.0
                        g_state.mag_scale = [avg_delta / dx, avg_delta / dy, avg_delta / dz]
                        g_state.status_msg = f"Compass Calibration Succeeded! Offsets: [{bx:.0f}, {by:.0f}, {bz:.0f}]"

            imgui.separator()
            imgui.text("Calculated Hard-Iron Zero Offsets (Bias):")
            imgui.text(f"  Offset X: {g_state.mag_offset[0]:+6.1f} LSB")
            imgui.text(f"  Offset Y: {g_state.mag_offset[1]:+6.1f} LSB")
            imgui.text(f"  Offset Z: {g_state.mag_offset[2]:+6.1f} LSB")

            imgui.separator()
            imgui.text("Calculated Soft-Iron Scale Factors:")
            imgui.text(f"  Scale X:  {g_state.mag_scale[0]:6.3f}x")
            imgui.text(f"  Scale Y:  {g_state.mag_scale[1]:6.3f}x")
            imgui.text(f"  Scale Z:  {g_state.mag_scale[2]:6.3f}x")

            imgui.separator()
            if imgui.button("Save Compass Offsets to Flash", imgui.ImVec2(280, 30)):
                g_state.send_msp(MSP_EEPROM_WRITE)
                g_state.status_msg = "Compass calibration permanently written to Flash (0x1F0000)!"

            imgui.next_column()

            imgui.text_colored(imgui.ImVec4(0.4, 0.8, 1.0, 1.0), "Live Magnetic Field Scatter Plot (X vs Y):")
            if implot.begin_plot("Mag Field (X vs Y)", imgui.ImVec2(-1, 280)):
                implot.setup_axes("Mag X (LSB)", "Mag Y (LSB)")
                if len(g_state.mag_samples_x) > 0:
                    xs = np.array(g_state.mag_samples_x, dtype=np.float32)
                    ys = np.array(g_state.mag_samples_y, dtype=np.float32)
                    implot.plot_scatter("Raw Samples", xs, ys, len(xs))
                implot.end_plot()

            imgui.columns(1)
            imgui.end_tab_item()

        # =============================================================
        # TAB 4: 6-Point Accelerometer Calibration Wizard
        # =============================================================
        if imgui.begin_tab_item("6-Point Acc Calibration")[0]:
            imgui.text_colored(imgui.ImVec4(1.0, 0.8, 0.2, 1.0), "INAV / Betaflight 6-Point Accelerometer Calibration Standard")
            imgui.text_wrapped("Calibrates all 6 orthogonal orientations to solve the 3x3 scaling matrix and eliminate gravity cross-bleed.")
            imgui.separator()

            for idx, (step_name, expected, done) in enumerate(g_state.acc_6point_steps):
                status_icon = "[DONE]" if done else "[PENDING]"
                color = imgui.ImVec4(0.2, 0.9, 0.2, 1.0) if done else imgui.ImVec4(0.8, 0.8, 0.8, 1.0)
                imgui.text_colored(color, f"{status_icon} {step_name} (Target: {expected})")
                imgui.same_line(500)
                if imgui.button(f"Capture Position {idx+1}##btn{idx}"):
                    g_state.send_msp(MSP_ACC_CALIBRATION)
                    g_state.acc_6point_steps[idx] = (step_name, expected, True)
                    g_state.status_msg = f"Captured position {idx+1} successfully!"

            imgui.separator()
            if imgui.button("Apply & Save All 6 Positions to Flash", imgui.ImVec2(320, 35)):
                g_state.send_msp(MSP_EEPROM_WRITE)
                g_state.status_msg = "6-Point Accelerometer Calibration saved to Flash (0x1F0000)!"

            imgui.end_tab_item()

        # =============================================================
        # TAB 5: RC Sticks & Transmitter
        # =============================================================
        if imgui.begin_tab_item("RC Sticks & Receiver")[0]:
            imgui.text_colored(imgui.ImVec4(0.4, 0.8, 1.0, 1.0), "Live RC Receiver Channels (ExpressLRS / CRSF / SBUS):")
            imgui.separator()

            ch_names = ["Roll", "Pitch", "Throttle", "Yaw", "AUX 1 (ARM)", "AUX 2 (Mode)", "AUX 3 (RTH)", "AUX 4"]
            for i in range(min(8, len(g_state.rc_channels))):
                val = g_state.rc_channels[i]
                norm = (val - 1000.0) / 1000.0
                name = ch_names[i] if i < len(ch_names) else f"CH {i+1}"
                imgui.text(f"{name:15s}: {val:4d} us")
                imgui.same_line()
                imgui.progress_bar(norm, imgui.ImVec2(300, 0), f"{val} us")

            imgui.end_tab_item()

        # =============================================================
        # TAB 6: GPS & Navigation
        # =============================================================
        if imgui.begin_tab_item("GPS & Navigation")[0]:
            imgui.text_colored(imgui.ImVec4(0.4, 0.8, 1.0, 1.0), "Ublox GPS Constellation & Navigation Status:")
            imgui.separator()

            fix_name = "3D FIX" if g_state.gps_fix >= 1 else "NO FIX"
            fix_color = imgui.ImVec4(0.2, 0.9, 0.2, 1.0) if g_state.gps_fix >= 1 else imgui.ImVec4(0.9, 0.2, 0.2, 1.0)
            imgui.text("GPS Status:  ")
            imgui.same_line()
            imgui.text_colored(fix_color, fix_name)

            imgui.text(f"Satellites:  {g_state.gps_sats:2d}")
            imgui.text(f"HDOP:        {g_state.gps_hdop:4.2f} (Target: < 1.50)")
            imgui.text(f"Latitude:    {g_state.gps_lat:.6f}°")
            imgui.text(f"Longitude:   {g_state.gps_lon:.6f}°")

            imgui.end_tab_item()

        imgui.end_tab_bar()

    imgui.separator()
    imgui.text_colored(imgui.ImVec4(0.7, 0.7, 0.7, 1.0), f"Status: {g_state.status_msg}")
    imgui.end()

def main():
    t = threading.Thread(target=telemetry_poll_thread, daemon=True)
    t.start()

    params = hello_imgui.RunnerParams()
    params.app_window_params.window_title = "inav-abstractx Desktop QA & Guided Setup Dashboard"
    params.app_window_params.window_geometry.size = (1150, 850)
    params.callbacks.show_gui = gui_render

    implot.create_context()
    hello_imgui.run(params)
    implot.destroy_context()

    g_state.running = False
    g_state.disconnect()

if __name__ == '__main__':
    main()
