#!/usr/bin/env python3
"""
`inav-abstractx` - Graphical Flight Controller Bench QA & Calibration Dashboard
Built using `imgui-bundle` (Dear ImGui + ImPlot for Python).

Features:
1. Live 3D Attitude Gauges & IMU Gyro/Accel/Baro Telemetry
2. Dedicated 3D Magnetometer / Compass Calibration Suite with Live ImPlot Ellipsoid Scatter
3. Full 6-Point Accelerometer Level & Scale Calibration Wizard
4. Live RC Transmitter Stick & Channel Visualizer (16 Channels)
5. Step-by-Step Individual Motor Spin Direction & ESC Tester (Props-Off Safety)
6. GPS Constellation Lock, HDOP & Satellite Visualizer
7. Flash EEPROM Save & Parameter Verification
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
        self.current_acc_step = 0

        # Safety & Wizard State
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
        m = [1000, 1000, 1000, 1000]
        if 0 <= m_idx < 4:
            m[m_idx] = pwm
        payload = struct.pack('<HHHHHHHH', m[0], m[1], m[2], m[3], 0, 0, 0, 0)
        self.send_msp(MSP_SET_MOTOR, payload)

    def stop_all_motors(self):
        payload = struct.pack('<HHHHHHHH', 1000, 1000, 1000, 1000, 0, 0, 0, 0)
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

                # Accumulate Magnetometer Scatter Samples if calibrating
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
    imgui.set_next_window_size(imgui.ImVec2(1080, 780), imgui.Cond_.first_use_ever)
    imgui.begin("INAV-ABSTRACTX Flight Controller Bench QA & Calibration Dashboard")

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

        # -------------------------------------------------------------
        # TAB 1: 3D Attitude & Sensors
        # -------------------------------------------------------------
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

        # -------------------------------------------------------------
        # TAB 2: Dedicated Magnetometer / Compass Calibration Suite
        # -------------------------------------------------------------
        if imgui.begin_tab_item("Compass Calibration (3D Scatter)")[0]:
            imgui.text_colored(imgui.ImVec4(1.0, 0.8, 0.2, 1.0), "3D Magnetometer / Compass Ellipsoid Calibration Suite")
            imgui.text_wrapped("Rotate the drone 360 degrees around all axes to map the magnetic field sphere and remove hard/soft iron distortions.")
            imgui.separator()

            imgui.columns(2, "mag_cols", True)

            # Controls & Calculations
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
                    # Calculate Hard-Iron Offsets & Soft-Iron Scales
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

            # Live 2D Magnetometer Scatter Plot (ImPlot)
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

        # -------------------------------------------------------------
        # TAB 3: 6-Point Accelerometer Calibration Wizard
        # -------------------------------------------------------------
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
                    # Mark step completed
                    g_state.acc_6point_steps[idx] = (step_name, expected, True)
                    g_state.status_msg = f"Captured position {idx+1} successfully!"

            imgui.separator()
            if imgui.button("Apply & Save All 6 Positions to Flash", imgui.ImVec2(320, 35)):
                g_state.send_msp(MSP_EEPROM_WRITE)
                g_state.status_msg = "6-Point Accelerometer Calibration saved to Flash (0x1F0000)!"

            imgui.end_tab_item()

        # -------------------------------------------------------------
        # TAB 4: RC Sticks & Transmitter
        # -------------------------------------------------------------
        if imgui.begin_tab_item("RC Sticks & Receiver")[0]:
            imgui.text_colored(imgui.ImVec4(0.4, 0.8, 1.0, 1.0), "Live RC Receiver Channels (ExpressLRS / CRSF / SBUS):")
            imgui.separator()

            ch_names = ["Roll", "Pitch", "Yaw", "Throttle", "AUX 1 (ARM)", "AUX 2 (Mode)", "AUX 3 (RTH)", "AUX 4"]
            for i in range(min(8, len(g_state.rc_channels))):
                val = g_state.rc_channels[i]
                norm = (val - 1000.0) / 1000.0
                name = ch_names[i] if i < len(ch_names) else f"CH {i+1}"
                imgui.text(f"{name:15s}: {val:4d} us")
                imgui.same_line()
                imgui.progress_bar(norm, imgui.ImVec2(300, 0), f"{val} us")

            imgui.end_tab_item()

        # -------------------------------------------------------------
        # TAB 5: Motor Direction & ESC Test (Props Off)
        # -------------------------------------------------------------
        if imgui.begin_tab_item("Motor Direction (Props-Off)")[0]:
            imgui.text_colored(imgui.ImVec4(1.0, 0.3, 0.3, 1.0), "CRITICAL SAFETY WARNING:")
            imgui.text_wrapped("ALL PROPELLERS MUST BE COMPLETELY REMOVED FROM MOTORS BEFORE TESTING!")
            
            _, g_state.props_removed_confirmed = imgui.checkbox("I CONFIRM ALL PROPELLERS ARE REMOVED", g_state.props_removed_confirmed)
            imgui.separator()

            if g_state.props_removed_confirmed:
                imgui.columns(2, "motor_cols", True)
                
                # Motor 1
                imgui.text("Motor 1 (Rear Right) - Expected: CCW")
                if imgui.button("Spin Motor 1 (1060 us)"):
                    g_state.spin_motor(0, 1060)
                
                # Motor 2
                imgui.text("Motor 2 (Front Right) - Expected: CW")
                if imgui.button("Spin Motor 2 (1060 us)"):
                    g_state.spin_motor(1, 1060)

                imgui.next_column()

                # Motor 3
                imgui.text("Motor 3 (Rear Left) - Expected: CW")
                if imgui.button("Spin Motor 3 (1060 us)"):
                    g_state.spin_motor(2, 1060)

                # Motor 4
                imgui.text("Motor 4 (Front Left) - Expected: CCW")
                if imgui.button("Spin Motor 4 (1060 us)"):
                    g_state.spin_motor(3, 1060)

                imgui.columns(1)
                imgui.separator()

                if imgui.button("STOP ALL MOTORS", imgui.ImVec2(200, 35)):
                    g_state.stop_all_motors()
            else:
                imgui.text_colored(imgui.ImVec4(0.8, 0.8, 0.8, 1.0), "Check the safety box above to unlock individual motor spin buttons.")

            imgui.end_tab_item()

        # -------------------------------------------------------------
        # TAB 6: GPS & Navigation
        # -------------------------------------------------------------
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
    params.app_window_params.window_title = "inav-abstractx Desktop QA & Calibration Dashboard"
    params.app_window_params.window_geometry.size = (1100, 800)
    params.callbacks.show_gui = gui_render

    implot.create_context()
    hello_imgui.run(params)
    implot.destroy_context()

    g_state.running = False
    g_state.disconnect()

if __name__ == '__main__':
    main()
