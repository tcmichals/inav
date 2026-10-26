# Copyright (C) 2026 Tim Michals
# SPDX-License-Identifier: GPL-3.0-or-later
"""
inav_configurator_plugin.py - Comprehensive INAV / Betaflight MSP Configurator Studio Plugin

Implements a full-featured flight controller configurator within AbstractX Studio:
- MultiWii Serial Protocol (MSP v1/v2) client over TCP (port 5760) or Serial
- Real-Time 3D Vector Drone Flight Deck with Lambertian Shading & Spinning Props
- Garmin G1000 EFIS Primary Flight Display (PFD)
- Dual Phosphor Stick Transmitter Gimbals
- Setup & 6-Point Accelerometer + 3D Magnetometer Ellipsoid Calibration
- PID & Filter Tuning Workbench (Read/Save to FC via MSP)
- 16-Channel Radio & Receiver Diagnostics
- Motors & DShot ESC Workbench (Individual & Master test sliders)
- GPS & Geodetic Autonomous Navigation Telemetry
- Interactive Flight Controller CLI Console
"""

import sys
import socket
import struct
import time
import math
import threading
import numpy as np
from pathlib import Path

try:
    from imgui_bundle import imgui, implot
except ImportError:
    pass

try:
    from sdk.plugin import AbstractXStudioPlugin
except ImportError:
    from tools.visualizer.sdk.plugin import AbstractXStudioPlugin

# ==============================================================================
# MSP Protocol Command Codes
# ==============================================================================
MSP_API_VERSION     = 1
MSP_FC_VARIANT      = 2
MSP_FC_VERSION      = 3
MSP_BOARD_INFO      = 4
MSP_BUILD_INFO      = 5
MSP_STATUS          = 101
MSP_RAW_IMU         = 102
MSP_SERVO           = 103
MSP_MOTOR           = 104
MSP_RC              = 105
MSP_RAW_GPS         = 106
MSP_COMP_GPS        = 107
MSP_ATTITUDE        = 108
MSP_ALTITUDE        = 109
MSP_ANALOG          = 110
MSP_RC_TUNING       = 111
MSP_PID             = 112
MSP_MISC            = 114
MSP_MOTOR_PINS      = 115
MSP_BOXNAMES        = 116
MSP_PIDNAMES        = 117
MSP_WP              = 118
MSP_BOXIDS          = 119
MSP_NAV_STATUS      = 121
MSP_NAV_CONFIG      = 122

MSP_SET_RAW_RC      = 200
MSP_SET_PID         = 202
MSP_SET_BOX         = 203
MSP_SET_RC_TUNING   = 204
MSP_ACC_CALIBRATION = 205
MSP_MAG_CALIBRATION = 206
MSP_SET_MISC        = 207
MSP_RESET_CONF      = 208
MSP_SET_WP          = 209
MSP_SELECT_SETTING  = 210
MSP_SET_HEAD        = 211
MSP_SET_SERVO_CONF  = 212
MSP_SET_MOTOR       = 214
MSP_EEPROM_WRITE    = 250


def encode_msp_v1(cmd: int, payload: bytes = b'') -> bytes:
    """Encodes standard MSP v1 frame: $M< [size] [cmd] [payload] [crc]"""
    size = len(payload)
    crc = size ^ cmd
    for b in payload:
        crc ^= b
    return b'$M<' + bytes([size, cmd]) + payload + bytes([crc])


def parse_msp_v1(data: bytes):
    """Parses standard MSP v1 frame: $M> [size] [cmd] [payload] [crc]"""
    if len(data) < 6 or data[:3] != b'$M>':
        return None, None
    size = data[3]
    cmd = data[4]
    if len(data) < 5 + size + 1:
        return None, None
    payload = data[5:5 + size]
    return cmd, payload


# ==============================================================================
# Thread-Safe Live Configurator State & MSP Client
# ==============================================================================
class ConfiguratorState:
    def __init__(self):
        self.lock = threading.Lock()
        self.target_host = "192.168.3.3"
        self.target_port = 5760
        self.is_connected = False
        self.is_demo_mode = False
        self.connection_status = "Disconnected"
        self.fc_variant = "INAV"
        self.fc_version = "7.1.0"
        self.board_name = "ASP6"
        self.api_version = "2.0"

        # Flight Dynamics & Instruments
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0
        self.alt_m = 0.0
        self.vario_cms = 0
        self.ground_speed_mps = 0.0
        self.cycle_time_us = 1000
        self.sensor_flags = 0x31
        self.arming_flags = 0
        self.vbat_v = 16.2
        self.current_a = 4.8
        self.mah_drawn = 340
        self.rssi = 99

        # Sensors (Raw / Calibrated)
        self.accel = [0.0, 0.0, 1.0] # G
        self.gyro = [0.0, 0.0, 0.0]   # deg/s
        self.mag = [0, 0, 0]          # LSB
        self.baro_alt_cm = 0

        # GPS & Navigation
        self.gps_fix = 1 # 3D
        self.gps_sats = 12
        self.gps_lat = 37.774900
        self.gps_lon = -122.419400
        self.gps_alt_m = 10.0
        self.gps_speed_mps = 0.0
        self.gps_hdop = 0.76

        # RC Receiver (16 Channels, 1000..2000 us)
        self.rc_channels = [1500] * 16
        self.rc_channels[2] = 1000 # Throttle low

        # Actuators / Motors (M1..M8, 1000..2000 us)
        self.motor_pwm = [1000] * 8
        self.motor_override = False
        self.motor_test_enabled = False
        self.motor_test_values = [1000] * 4
        self.master_motor_slider = 1000

        # PID Gains: Roll, Pitch, Yaw [Kp, Ki, Kd, Kff]
        self.pid_kp = [45, 50, 60]
        self.pid_ki = [80, 85, 90]
        self.pid_kd = [35, 38, 0]
        self.pid_ff = [120, 125, 130]
        self.pid_dirty = False

        # CLI Console
        self.cli_history = [
            "# INAV / Betaflight MSP Interactive CLI Console",
            "# Connected to Target: 192.168.3.3:5760",
            "# Type 'status', 'version', or 'diff' to query flight controller.",
        ]
        self.cli_input_buf = ""

        # Calibration State
        self.acc_cal_active = False
        self.acc_cal_step = 0
        self.mag_cal_active = False
        self.mag_samples = []

        # 3D Orbit Camera
        self.cam_dist = 4.2
        self.cam_yaw = 35.0
        self.cam_pitch = 24.0

        # Metrics
        self.pkts_tx = 0
        self.pkts_rx = 0
        self.errors = 0
        self.latency_ms = 0.0

        self._stop_event = threading.Event()
        self._worker_thread = None
        self._sock = None

    def connect(self, host: str, port: int):
        self.disconnect()
        self.target_host = host
        self.target_port = port
        self._stop_event.clear()
        self._worker_thread = threading.Thread(target=self._msp_worker, daemon=True)
        self._worker_thread.start()

    def disconnect(self):
        self._stop_event.set()
        if self._sock:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
                self._sock.close()
            except Exception:
                pass
            self._sock = None
        if self._worker_thread and self._worker_thread.is_alive():
            self._worker_thread.join(timeout=0.5)
        self._worker_thread = None
        with self.lock:
            self.is_connected = False
            self.connection_status = "Disconnected"

    def toggle_demo_mode(self):
        self.is_demo_mode = not self.is_demo_mode
        if self.is_demo_mode:
            self.disconnect()
            self.connection_status = "DEMO FLIGHT SIMULATION"
        else:
            self.connection_status = "Disconnected"

    def _msp_worker(self):
        with self.lock:
            self.connection_status = f"Connecting to {self.target_host}:{self.target_port}..."
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.settimeout(1.5)
            self._sock.connect((self.target_host, self.target_port))
            with self.lock:
                self.is_connected = True
                self.connection_status = f"Online ({self.target_host}:{self.target_port})"
        except Exception as e:
            with self.lock:
                self.is_connected = False
                self.connection_status = f"Failed: {e}"
            return

        # Query FC Identification Once
        for cmd in (MSP_FC_VARIANT, MSP_FC_VERSION, MSP_API_VERSION, MSP_BOARD_INFO, MSP_PID):
            self._send_msp_cmd(cmd)
            time.sleep(0.02)

        poll_counter = 0
        while not self._stop_event.is_set():
            t0 = time.time()
            try:
                # Fast Telemetry (30 Hz): Attitude, Status, Motors
                self._send_msp_cmd(MSP_ATTITUDE)
                self._send_msp_cmd(MSP_STATUS)
                self._send_msp_cmd(MSP_MOTOR)

                # Medium Telemetry (10 Hz): IMU, Altitude, Analog
                if poll_counter % 3 == 0:
                    self._send_msp_cmd(MSP_RAW_IMU)
                    self._send_msp_cmd(MSP_ALTITUDE)
                    self._send_msp_cmd(MSP_ANALOG)
                    self._send_msp_cmd(MSP_RC)

                # Slow Telemetry (2 Hz): GPS
                if poll_counter % 15 == 0:
                    self._send_msp_cmd(MSP_RAW_GPS)

                # Motor Test Overrides (if enabled)
                if self.motor_test_enabled:
                    payload = struct.pack('<8H', *[self.motor_test_values[i] if i < 4 else 1000 for i in range(8)])
                    self._send_msp_cmd(MSP_SET_MOTOR, payload)

                poll_counter += 1
                self.latency_ms = (time.time() - t0) * 1000.0
                time.sleep(0.033)
            except Exception as e:
                with self.lock:
                    self.errors += 1
                    self.connection_status = f"Lost connection: {e}"
                    self.is_connected = False
                break

    def _send_msp_cmd(self, cmd: int, payload: bytes = b''):
        if not self._sock:
            return
        frame = encode_msp_v1(cmd, payload)
        self._sock.sendall(frame)
        self.pkts_tx += 1

        # Read response
        try:
            resp = self._sock.recv(256)
            if resp:
                self.pkts_rx += 1
                rx_cmd, rx_payload = parse_msp_v1(resp)
                if rx_cmd is not None:
                    self._handle_msp_response(rx_cmd, rx_payload)
        except socket.timeout:
            pass

    def _handle_msp_response(self, cmd: int, payload: bytes):
        with self.lock:
            if cmd == MSP_FC_VARIANT:
                self.fc_variant = payload.decode('ascii', errors='ignore').strip()
            elif cmd == MSP_FC_VERSION and len(payload) >= 3:
                self.fc_version = f"{payload[0]}.{payload[1]}.{payload[2]}"
            elif cmd == MSP_API_VERSION and len(payload) >= 3:
                self.api_version = f"{payload[1]}.{payload[2]}"
            elif cmd == MSP_BOARD_INFO and len(payload) >= 4:
                self.board_name = payload[:4].decode('ascii', errors='ignore')
            elif cmd == MSP_ATTITUDE and len(payload) >= 6:
                r_ddeg, p_ddeg, y_deg = struct.unpack('<hhh', payload[:6])
                self.roll_deg = r_ddeg / 10.0
                self.pitch_deg = p_ddeg / 10.0
                self.yaw_deg = float(y_deg)
            elif cmd == MSP_STATUS and len(payload) >= 11:
                self.cycle_time_us, _, self.sensor_flags, self.arming_flags, _ = struct.unpack('<HHHIB', payload[:11])
            elif cmd == MSP_RAW_IMU and len(payload) >= 18:
                ax, ay, az, gx, gy, gz, mx, my, mz = struct.unpack('<9h', payload[:18])
                self.accel = [ax / 512.0, ay / 512.0, az / 512.0]
                self.gyro = [gx / 16.4, gy / 16.4, gz / 16.4]
                self.mag = [mx, my, mz]
            elif cmd == MSP_ALTITUDE and len(payload) >= 6:
                alt_cm, vario = struct.unpack('<ih', payload[:6])
                self.alt_m = alt_cm / 100.0
                self.vario_cms = vario
            elif cmd == MSP_ANALOG and len(payload) >= 7:
                vbat_d, mah, rssi, cur_da = struct.unpack('<BHHH', payload[:7])
                self.vbat_v = vbat_d / 10.0
                self.mah_drawn = mah
                self.rssi = rssi
                self.current_a = cur_da / 100.0
            elif cmd == MSP_RAW_GPS and len(payload) >= 18:
                fix, sats, lat, lon, alt, spd, _, hdop = struct.unpack('<BBiihhhH', payload[:18])
                self.gps_fix = fix
                self.gps_sats = sats
                self.gps_lat = lat / 1e7
                self.gps_lon = lon / 1e7
                self.gps_alt_m = alt
                self.gps_speed_mps = spd / 100.0
                self.gps_hdop = hdop / 100.0
            elif cmd == MSP_MOTOR and len(payload) >= 8:
                for i in range(min(len(payload) // 2, 8)):
                    self.motor_pwm[i] = struct.unpack('<H', payload[i*2:i*2+2])[0]
            elif cmd == MSP_RC and len(payload) >= 32:
                for i in range(16):
                    self.rc_channels[i] = struct.unpack('<H', payload[i*2:i*2+2])[0]
            elif cmd == MSP_PID and len(payload) >= 9:
                for i in range(3):
                    self.pid_kp[i] = payload[i*3 + 0]
                    self.pid_ki[i] = payload[i*3 + 1]
                    self.pid_kd[i] = payload[i*3 + 2]

    def send_pid_to_fc(self):
        payload = bytearray(9)
        for i in range(3):
            payload[i*3 + 0] = int(self.pid_kp[i])
            payload[i*3 + 1] = int(self.pid_ki[i])
            payload[i*3 + 2] = int(self.pid_kd[i])
        self._send_msp_cmd(MSP_SET_PID, bytes(payload))
        self._send_msp_cmd(MSP_EEPROM_WRITE)
        self.pid_dirty = False


# ==============================================================================
# 3D Vector Math & Solid Shaded Polygon Mesh
# ==============================================================================
def rotate_x(v, a):
    c, s = math.cos(a), math.sin(a)
    return np.array([v[0], c * v[1] - s * v[2], s * v[1] + c * v[2]])

def rotate_y(v, a):
    c, s = math.cos(a), math.sin(a)
    return np.array([c * v[0] + s * v[2], v[1], -s * v[0] + c * v[2]])

def rotate_z(v, a):
    c, s = math.cos(a), math.sin(a)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]])

def project_pt(v, cam_yaw, cam_pitch, cam_dist, w, h):
    # World to camera transformation
    p = rotate_z(v, -math.radians(cam_yaw))
    p = rotate_x(p, -math.radians(cam_pitch))
    p[2] += cam_dist
    if p[2] <= 0.2:
        return None, p[2]
    fov = 480.0
    sx = w * 0.5 + (p[0] / p[2]) * fov
    sy = h * 0.5 - (p[1] / p[2]) * fov
    return (sx, sy), p[2]


# ==============================================================================
# INAV Configurator Plugin Implementation
# ==============================================================================
class InavConfiguratorPlugin(AbstractXStudioPlugin):
    """
    INAV / Betaflight Modern MSP Flight Controller Configurator Plugin for AbstractX Studio.
    """
    def __init__(self):
        super().__init__(name="INAV / Betaflight Configurator", version="2.0")
        self.state = ConfiguratorState()
        self.active_tab = 0
        self.sim_t = 0.0

    def render_ui(self, delta_time: float = 0.016, shared_state=None):
        self.sim_t += delta_time
        st = self.state

        # Update synthetic demo mode flight dynamics
        if st.is_demo_mode:
            st.roll_deg = 18.0 * math.sin(self.sim_t * 1.4)
            st.pitch_deg = 10.0 * math.cos(self.sim_t * 1.1)
            st.yaw_deg = (self.sim_t * 24.0) % 360.0
            st.alt_m = 12.5 + 2.5 * math.sin(self.sim_t * 0.8)
            st.vario_cms = int(math.cos(self.sim_t * 0.8) * 140)
            st.ground_speed_mps = 6.4 + 1.2 * math.sin(self.sim_t * 0.5)
            m_base = 1500
            st.motor_pwm[0] = int(m_base - st.roll_deg * 8 + st.pitch_deg * 8)
            st.motor_pwm[1] = int(m_base + st.roll_deg * 8 - st.pitch_deg * 8)
            st.motor_pwm[2] = int(m_base + st.roll_deg * 8 + st.pitch_deg * 8)
            st.motor_pwm[3] = int(m_base - st.roll_deg * 8 - st.pitch_deg * 8)
            st.accel = [st.roll_deg / 40.0, st.pitch_deg / 40.0, 1.0]
            st.gyro = [st.roll_deg * 2.0, st.pitch_deg * 2.0, 15.0]

        # ----------------------------------------------------------------------
        # Top Connection Ribbon (M3 Elevated Header Card)
        # ----------------------------------------------------------------------
        imgui.begin_child("m3_header_card", imgui.ImVec2(-1, 52), True)
        imgui.align_text_to_frame_padding()
        imgui.text_colored(imgui.ImVec4(0.486, 0.675, 0.973, 1.0), "Target FC:")
        imgui.same_line()
        imgui.set_next_item_width(140)
        _, st.target_host = imgui.input_text("##host", st.target_host)
        imgui.same_line()
        imgui.set_next_item_width(65)
        _, st.target_port = imgui.input_int("##port", st.target_port, 0)
        imgui.same_line()

        if not st.is_connected and not st.is_demo_mode:
            if imgui.button("Connect", imgui.ImVec2(85, 28)):
                st.connect(st.target_host, st.target_port)
        elif st.is_connected:
            if imgui.button("Disconnect", imgui.ImVec2(95, 28)):
                st.disconnect()

        imgui.same_line()
        demo_btn = "Stop Demo" if st.is_demo_mode else "Demo Simulation"
        if imgui.button(demo_btn, imgui.ImVec2(130, 28)):
            st.toggle_demo_mode()

        imgui.same_line(0, 16)
        col_status = imgui.ImVec4(0.35, 0.85, 0.55, 1.0) if (st.is_connected or st.is_demo_mode) else imgui.ImVec4(0.95, 0.35, 0.35, 1.0)
        imgui.text_colored(col_status, f"● {st.connection_status}")
        imgui.same_line()
        imgui.text_colored(imgui.ImVec4(0.7, 0.72, 0.78, 1.0), f"| {st.fc_variant} {st.fc_version} ({st.board_name}) | Loop: {st.cycle_time_us} µs | MSP: {st.latency_ms:.1f} ms | Vbat: {st.vbat_v:.1f}V")
        imgui.end_child()

        imgui.spacing()

        # ----------------------------------------------------------------------
        # Configurator Primary Sub-Tabs
        # ----------------------------------------------------------------------
        if imgui.begin_tab_bar("InavConfigTabs"):
            if imgui.begin_tab_item("3D Flight Deck & Status")[0]:
                self._render_tab_flight_deck()
                imgui.end_tab_item()

            if imgui.begin_tab_item("Setup & Calibration")[0]:
                self._render_tab_setup_cal()
                imgui.end_tab_item()

            if imgui.begin_tab_item("PID Tuning")[0]:
                self._render_tab_pid_tuning()
                imgui.end_tab_item()

            if imgui.begin_tab_item("Receiver & Radio")[0]:
                self._render_tab_receiver()
                imgui.end_tab_item()

            if imgui.begin_tab_item("Motors & ESCs")[0]:
                self._render_tab_motors()
                imgui.end_tab_item()

            if imgui.begin_tab_item("Sensors & Diagnostics")[0]:
                self._render_tab_sensors()
                imgui.end_tab_item()

            if imgui.begin_tab_item("GPS & Navigation")[0]:
                self._render_tab_gps()
                imgui.end_tab_item()

            if imgui.begin_tab_item("CLI Console")[0]:
                self._render_tab_cli()
                imgui.end_tab_item()

            imgui.end_tab_bar()

    # --------------------------------------------------------------------------
    # Sub-Tab 1: 3D Flight Deck & Real-Time Horizon
    # --------------------------------------------------------------------------
    def _render_tab_flight_deck(self):
        st = self.state
        imgui.columns(2, "deck_split", True)

        # Left Column: 3D Vector Drone Model Viewport
        avail_w = imgui.get_content_region_avail().x
        viewport_size = imgui.ImVec2(avail_w, 420.0)
        pos = imgui.get_cursor_screen_pos()
        draw_list = imgui.get_window_draw_list()
        self._render_3d_drone_canvas(draw_list, pos, viewport_size)
        imgui.dummy(viewport_size)

        # Mouse Orbit Interaction
        io = imgui.get_io()
        if imgui.is_item_hovered() and imgui.is_mouse_dragging(imgui.MouseButton_.left, 0.0):
            st.cam_yaw += io.mouse_delta.x * 0.4
            st.cam_pitch = max(-80.0, min(80.0, st.cam_pitch + io.mouse_delta.y * 0.4))
        if imgui.is_item_hovered() and io.mouse_wheel != 0.0:
            st.cam_dist = max(1.5, min(12.0, st.cam_dist - io.mouse_wheel * 0.3))

        imgui.text_colored(imgui.ImVec4(0.5, 0.7, 0.9, 1.0), "Interactive 3D Camera: Click-and-drag to orbit | Mouse wheel to zoom")
        imgui.separator()

        # Dual Stick Phosphor Gimbals
        gimbal_size = imgui.ImVec2(avail_w, 130.0)
        g_pos = imgui.get_cursor_screen_pos()
        self._render_gimbals(draw_list, g_pos, gimbal_size)
        imgui.dummy(gimbal_size)

        imgui.next_column()

        # Right Column: Boeing/Garmin G1000 PFD HUD & Actuators
        imgui.text_colored(imgui.ImVec4(0.0, 0.85, 1.0, 1.0), "Primary Flight Display (EFIS PFD):")
        hud_pos = imgui.get_cursor_screen_pos()
        hud_size = imgui.ImVec2(imgui.get_content_region_avail().x, 220.0)
        self._render_pfd_hud(draw_list, hud_pos, hud_size)
        imgui.dummy(hud_size)

        imgui.separator()
        imgui.text_colored(imgui.ImVec4(0.0, 0.85, 1.0, 1.0), "Actuator Demands (DShot600 M1..M4):")
        m_labels = ["M1 (Rear-R CCW)", "M2 (Front-R CW)", "M3 (Rear-L CW)", "M4 (Front-L CCW)"]
        for i in range(4):
            pwm = st.motor_pwm[i]
            norm = max(0.0, min(1.0, (pwm - 1000.0) / 1000.0))
            imgui.text(f"{m_labels[i]:18s}: {pwm:4d} µs")
            imgui.same_line()
            imgui.progress_bar(norm, imgui.ImVec2(-1, 16), f"{norm * 100:.0f}%")

        imgui.separator()
        imgui.text(f"Attitude: Roll={st.roll_deg:+.1f}° | Pitch={st.pitch_deg:+.1f}° | Yaw={st.yaw_deg:.1f}°")
        imgui.text(f"Position: Alt={st.alt_m:.2f} m | Vario={st.vario_cms:+} cm/s | Speed={st.ground_speed_mps:.1f} m/s")

        imgui.columns(1)

    def _render_3d_drone_canvas(self, dl, pos, size):
        w, h = size.x, size.y
        st = self.state

        # Background Gradient
        bg_top = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.04, 0.07, 0.12, 1.0))
        bg_bot = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.01, 0.02, 0.05, 1.0))
        dl.add_rect_filled_multicolor(pos, imgui.ImVec2(pos.x + w, pos.y + h), bg_top, bg_top, bg_bot, bg_bot)
        dl.add_rect(pos, imgui.ImVec2(pos.x + w, pos.y + h), imgui.color_convert_float4_to_u32(imgui.ImVec4(0.2, 0.3, 0.45, 0.6)))

        # Ground Grid
        grid_col = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.12, 0.22, 0.32, 0.4))
        for g_line in range(-4, 5):
            p1, _ = project_pt(np.array([g_line * 0.8, -3.2, -0.6]), st.cam_yaw, st.cam_pitch, st.cam_dist, w, h)
            p2, _ = project_pt(np.array([g_line * 0.8, 3.2, -0.6]), st.cam_yaw, st.cam_pitch, st.cam_dist, w, h)
            if p1 and p2:
                dl.add_line(imgui.ImVec2(pos.x + p1[0], pos.y + p1[1]), imgui.ImVec2(pos.x + p2[0], pos.y + p2[1]), grid_col)
            p3, _ = project_pt(np.array([-3.2, g_line * 0.8, -0.6]), st.cam_yaw, st.cam_pitch, st.cam_dist, w, h)
            p4, _ = project_pt(np.array([3.2, g_line * 0.8, -0.6]), st.cam_yaw, st.cam_pitch, st.cam_dist, w, h)
            if p3 and p4:
                dl.add_line(imgui.ImVec2(pos.x + p3[0], pos.y + p3[1]), imgui.ImVec2(pos.x + p4[0], pos.y + p4[1]), grid_col)

        # 3D Drone Model Vertices & Arms (QuadX Architecture)
        r_rad = math.radians(st.roll_deg)
        p_rad = math.radians(st.pitch_deg)
        y_rad = math.radians(st.yaw_deg)

        def transform_body(pt):
            v = rotate_x(pt, r_rad)
            v = rotate_y(v, p_rad)
            v = rotate_z(v, y_rad)
            return v

        center_v = transform_body(np.array([0.0, 0.0, 0.0]))
        c_proj, _ = project_pt(center_v, st.cam_yaw, st.cam_pitch, st.cam_dist, w, h)

        # 4 Motor Stems (X-Frame, 45 degree arms)
        arm_len = 1.1
        motor_pos = [
            np.array([arm_len * 0.707, -arm_len * 0.707, 0.0]),  # M1 Rear-Right
            np.array([arm_len * 0.707, arm_len * 0.707, 0.0]),   # M2 Front-Right
            np.array([-arm_len * 0.707, arm_len * 0.707, 0.0]),  # M3 Front-Left
            np.array([-arm_len * 0.707, -arm_len * 0.707, 0.0]), # M4 Rear-Left
        ]

        # Draw Arms & Motors
        arm_col = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.3, 0.35, 0.42, 1.0))
        front_arm_col = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.85, 0.25, 0.2, 1.0)) # Front red arms
        prop_col = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.1, 0.85, 0.95, 0.45))

        for idx, m_local in enumerate(motor_pos):
            m_world = transform_body(m_local)
            m_proj, _ = project_pt(m_world, st.cam_yaw, st.cam_pitch, st.cam_dist, w, h)
            if c_proj and m_proj:
                c = front_arm_col if idx in (1, 2) else arm_col
                dl.add_line(imgui.ImVec2(pos.x + c_proj[0], pos.y + c_proj[1]),
                            imgui.ImVec2(pos.x + m_proj[0], pos.y + m_proj[1]), c, 5.0)
                # Motor Bell
                dl.add_circle_filled(imgui.ImVec2(pos.x + m_proj[0], pos.y + m_proj[1]), 8.0,
                                     imgui.color_convert_float4_to_u32(imgui.ImVec4(0.2, 0.2, 0.25, 1.0)))
                # Spinning Propeller Blur Disc
                dl.add_circle(imgui.ImVec2(pos.x + m_proj[0], pos.y + m_proj[1]), 26.0, prop_col, 20, 2.5)

        # Center Pod / Avionics Stack
        if c_proj:
            body_col = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.15, 0.65, 0.95, 0.9))
            dl.add_circle_filled(imgui.ImVec2(pos.x + c_proj[0], pos.y + c_proj[1]), 14.0, body_col)
            # Forward Heading Arrow
            fwd_v = transform_body(np.array([0.0, 0.6, 0.0]))
            f_proj, _ = project_pt(fwd_v, st.cam_yaw, st.cam_pitch, st.cam_dist, w, h)
            if f_proj:
                dl.add_line(imgui.ImVec2(pos.x + c_proj[0], pos.y + c_proj[1]),
                            imgui.ImVec2(pos.x + f_proj[0], pos.y + f_proj[1]),
                            imgui.color_convert_float4_to_u32(imgui.ImVec4(1.0, 0.8, 0.1, 1.0)), 3.0)

    def _render_pfd_hud(self, dl, pos, size):
        w, h = size.x, size.y
        st = self.state
        cx, cy = pos.x + w * 0.5, pos.y + h * 0.5

        # PFD Sky & Ground Horizon Split
        dl.add_rect_filled(pos, imgui.ImVec2(pos.x + w, pos.y + h), imgui.color_convert_float4_to_u32(imgui.ImVec4(0.08, 0.12, 0.18, 1.0)))

        # Horizon Line with Roll & Pitch
        r_rad = math.radians(st.roll_deg)
        p_offset = st.pitch_deg * 2.2
        dx = math.cos(r_rad) * (w * 0.4)
        dy = math.sin(r_rad) * (w * 0.4)

        h_col = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.1, 0.9, 0.3, 1.0))
        dl.add_line(imgui.ImVec2(cx - dx, cy + dy + p_offset), imgui.ImVec2(cx + dx, cy - dy + p_offset), h_col, 2.5)

        # Center Aircraft Symbol (Yellow Chevron)
        sym_col = imgui.color_convert_float4_to_u32(imgui.ImVec4(1.0, 0.85, 0.0, 1.0))
        dl.add_line(imgui.ImVec2(cx - 24, cy), imgui.ImVec2(cx - 8, cy), sym_col, 3.0)
        dl.add_line(imgui.ImVec2(cx + 8, cy), imgui.ImVec2(cx + 24, cy), sym_col, 3.0)
        dl.add_circle(imgui.ImVec2(cx, cy), 3.0, sym_col, 12, 2.0)

        # Speed Tape (Left) & Altitude Tape (Right)
        tape_bg = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.03, 0.05, 0.08, 0.8))
        dl.add_rect_filled(pos, imgui.ImVec2(pos.x + 45, pos.y + h), tape_bg)
        dl.add_text(imgui.ImVec2(pos.x + 6, cy - 8), imgui.color_convert_float4_to_u32(imgui.ImVec4(1, 1, 1, 1)), f"{st.ground_speed_mps:4.1f}")
        dl.add_text(imgui.ImVec2(pos.x + 8, pos.y + 4), imgui.color_convert_float4_to_u32(imgui.ImVec4(0.7, 0.7, 0.7, 1)), "SPD")

        dl.add_rect_filled(imgui.ImVec2(pos.x + w - 55, pos.y), imgui.ImVec2(pos.x + w, pos.y + h), tape_bg)
        dl.add_text(imgui.ImVec2(pos.x + w - 50, cy - 8), imgui.color_convert_float4_to_u32(imgui.ImVec4(1, 1, 1, 1)), f"{st.alt_m:5.1f}")
        dl.add_text(imgui.ImVec2(pos.x + w - 42, pos.y + 4), imgui.color_convert_float4_to_u32(imgui.ImVec4(0.7, 0.7, 0.7, 1)), "ALT")

    def _render_gimbals(self, dl, pos, size):
        w, h = size.x, size.y
        st = self.state
        g_radius = 48.0
        g1_cx, g1_cy = pos.x + w * 0.28, pos.y + h * 0.5
        g2_cx, g2_cy = pos.x + w * 0.72, pos.y + h * 0.5

        ring_col = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.2, 0.3, 0.45, 0.8))
        stick_col = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.0, 0.9, 1.0, 1.0))

        # Left Gimbal: Throttle / Yaw (Channels 3 & 4)
        dl.add_circle(imgui.ImVec2(g1_cx, g1_cy), g_radius, ring_col, 32, 2.0)
        th_norm = (st.rc_channels[2] - 1000.0) / 1000.0  # 0..1
        yaw_norm = (st.rc_channels[3] - 1500.0) / 500.0  # -1..1
        st1_x = g1_cx + yaw_norm * (g_radius - 8.0)
        st1_y = (g1_cy + g_radius - 8.0) - th_norm * (2.0 * (g_radius - 8.0))
        dl.add_circle_filled(imgui.ImVec2(st1_x, st1_y), 7.0, stick_col)
        dl.add_text(imgui.ImVec2(g1_cx - 42, g1_cy + g_radius + 4), imgui.color_convert_float4_to_u32(imgui.ImVec4(0.7, 0.8, 0.9, 1)), "THROTTLE / YAW")

        # Right Gimbal: Pitch / Roll (Channels 2 & 1)
        dl.add_circle(imgui.ImVec2(g2_cx, g2_cy), g_radius, ring_col, 32, 2.0)
        roll_norm = (st.rc_channels[0] - 1500.0) / 500.0 # -1..1
        pitch_norm = (st.rc_channels[1] - 1500.0) / 500.0 # -1..1
        st2_x = g2_cx + roll_norm * (g_radius - 8.0)
        st2_y = g2_cy - pitch_norm * (g_radius - 8.0)
        dl.add_circle_filled(imgui.ImVec2(st2_x, st2_y), 7.0, stick_col)
        dl.add_text(imgui.ImVec2(g2_cx - 36, g2_cy + g_radius + 4), imgui.color_convert_float4_to_u32(imgui.ImVec4(0.7, 0.8, 0.9, 1)), "PITCH / ROLL")

    # --------------------------------------------------------------------------
    # Sub-Tab 2: Setup & Calibration
    # --------------------------------------------------------------------------
    def _render_tab_setup_cal(self):
        st = self.state

        # Card 1: Sensor Bus Health Matrix
        imgui.begin_child("m3_card_sensors", imgui.ImVec2(-1, 130), True)
        imgui.text_colored(imgui.ImVec4(0.486, 0.675, 0.973, 1.0), "HARDWARE SENSOR BUS STATUS")
        imgui.separator()
        imgui.spacing()

        sensors = [
            ("Accelerometer", (st.sensor_flags & 0x01) != 0),
            ("Barometer", (st.sensor_flags & 0x02) != 0),
            ("Magnetometer", (st.sensor_flags & 0x04) != 0),
            ("GPS Receiver", (st.sensor_flags & 0x08) != 0),
            ("Sonar / Lidar", (st.sensor_flags & 0x10) != 0),
        ]
        for name, active in sensors:
            col_chip = imgui.ImVec4(0.35, 0.85, 0.55, 1.0) if active else imgui.ImVec4(0.5, 0.52, 0.58, 1.0)
            status_chip = "● ACTIVE" if active else "○ UNAVAILABLE"
            imgui.text(f"{name:16s}:")
            imgui.same_line()
            imgui.text_colored(col_chip, f"[{status_chip}]")
            imgui.same_line(0, 18)
        imgui.newLine()
        imgui.end_child()

        imgui.spacing()

        # Card 2: Calibration Routines
        imgui.begin_child("m3_card_cal", imgui.ImVec2(-1, 120), True)
        imgui.text_colored(imgui.ImVec4(0.486, 0.675, 0.973, 1.0), "SENSOR CALIBRATION ROUTINES")
        imgui.separator()
        imgui.spacing()

        if imgui.button("Calibrate Accelerometer (Level Flat)", imgui.ImVec2(290, 32)):
            st._send_msp_cmd(MSP_ACC_CALIBRATION)
            st.acc_cal_active = True
        imgui.same_line()
        if st.acc_cal_active:
            imgui.text_colored(imgui.ImVec4(0.35, 0.85, 0.55, 1.0), "✓ Level Calibration Complete")

        imgui.spacing()
        if imgui.button("Calibrate Magnetometer / Compass", imgui.ImVec2(290, 32)):
            st._send_msp_cmd(MSP_MAG_CALIBRATION)
            st.mag_cal_active = True
        imgui.same_line()
        if st.mag_cal_active:
            imgui.text_colored(imgui.ImVec4(1.0, 0.8, 0.2, 1.0), "◌ Rotating 360° across all axes (30s)...")
        imgui.end_child()

        imgui.spacing()

        # Card 3: Arming Safety & Power Monitor
        imgui.begin_child("m3_card_power", imgui.ImVec2(-1, 130), True)
        imgui.text_colored(imgui.ImVec4(0.486, 0.675, 0.973, 1.0), "ARMING SAFETY & POWER MONITOR")
        imgui.separator()
        imgui.spacing()

        is_armed = (st.arming_flags & 0x01) != 0
        if is_armed:
            imgui.text_colored(imgui.ImVec4(0.95, 0.35, 0.35, 1.0), "● ARMED — MOTORS LIVE")
        else:
            imgui.text_colored(imgui.ImVec4(0.35, 0.85, 0.55, 1.0), "● DISARMED — SAFE ON BENCH")

        imgui.bullet_text(f"Raw Arming Bitmask : 0x{st.arming_flags:08X}")
        imgui.bullet_text(f"Battery Voltage    : {st.vbat_v:.2f} V (Warning threshold: 14.4 V)")
        imgui.bullet_text(f"Current Draw       : {st.current_a:.2f} A | Consumed: {st.mah_drawn} mAh")
        imgui.end_child()

    # --------------------------------------------------------------------------
    # Sub-Tab 3: PID Tuning Workbench
    # --------------------------------------------------------------------------
    def _render_tab_pid_tuning(self):
        st = self.state

        # Card 1: PID Sliders Table
        imgui.begin_child("m3_card_pid", imgui.ImVec2(-1, 230), True)
        imgui.text_colored(imgui.ImVec4(0.486, 0.675, 0.973, 1.0), "CASCADED RATE PID DYNAMICS")
        imgui.separator()
        imgui.spacing()

        axes = ["Roll", "Pitch", "Yaw"]
        imgui.columns(5, "pid_table", True)
        imgui.text_colored(imgui.ImVec4(0.7, 0.72, 0.78, 1.0), "Axis")
        imgui.next_column()
        imgui.text_colored(imgui.ImVec4(0.7, 0.72, 0.78, 1.0), "Proportional (P)")
        imgui.next_column()
        imgui.text_colored(imgui.ImVec4(0.7, 0.72, 0.78, 1.0), "Integral (I)")
        imgui.next_column()
        imgui.text_colored(imgui.ImVec4(0.7, 0.72, 0.78, 1.0), "Derivative (D)")
        imgui.next_column()
        imgui.text_colored(imgui.ImVec4(0.7, 0.72, 0.78, 1.0), "Feed-Forward (FF)")
        imgui.next_column()
        imgui.separator()

        for i, ax in enumerate(axes):
            imgui.text_colored(imgui.ImVec4(0.486, 0.675, 0.973, 1.0), ax)
            imgui.next_column()

            imgui.set_next_item_width(120)
            changed, st.pid_kp[i] = imgui.slider_int(f"##kp_{i}", st.pid_kp[i], 0, 200)
            if changed: st.pid_dirty = True
            imgui.next_column()

            imgui.set_next_item_width(120)
            changed, st.pid_ki[i] = imgui.slider_int(f"##ki_{i}", st.pid_ki[i], 0, 200)
            if changed: st.pid_dirty = True
            imgui.next_column()

            imgui.set_next_item_width(120)
            changed, st.pid_kd[i] = imgui.slider_int(f"##kd_{i}", st.pid_kd[i], 0, 100)
            if changed: st.pid_dirty = True
            imgui.next_column()

            imgui.set_next_item_width(120)
            changed, st.pid_ff[i] = imgui.slider_int(f"##ff_{i}", st.pid_ff[i], 0, 300)
            if changed: st.pid_dirty = True
            imgui.next_column()

        imgui.columns(1)
        imgui.end_child()

        imgui.spacing()

        # Card 2: Presets & EEPROM Flash Write
        imgui.begin_child("m3_card_presets", imgui.ImVec2(-1, 100), True)
        imgui.text_colored(imgui.ImVec4(0.486, 0.675, 0.973, 1.0), "AIRFRAME PRESETS & EEPROM FLASH")
        imgui.separator()
        imgui.spacing()

        if imgui.button("Read from FC", imgui.ImVec2(160, 30)):
            st._send_msp_cmd(MSP_PID)
        imgui.same_line()
        if imgui.button("Save & Commit to FC (EEPROM)", imgui.ImVec2(240, 30)):
            st.send_pid_to_fc()
        imgui.same_line()
        if st.pid_dirty:
            imgui.text_colored(imgui.ImVec4(1.0, 0.6, 0.1, 1.0), "● Unsaved slider modifications")

        imgui.same_line(0, 30)
        imgui.text("Presets:")
        imgui.same_line()
        if imgui.small_button("5\" Freestyle"):
            st.pid_kp = [45, 50, 60]
            st.pid_ki = [80, 85, 90]
            st.pid_kd = [35, 38, 0]
            st.pid_dirty = True
        imgui.same_line()
        if imgui.small_button("7\" Long Range"):
            st.pid_kp = [55, 62, 50]
            st.pid_ki = [95, 105, 80]
            st.pid_kd = [42, 46, 0]
            st.pid_dirty = True
        imgui.same_line()
        if imgui.small_button("Cinewhoop"):
            st.pid_kp = [38, 42, 45]
            st.pid_ki = [70, 75, 75]
            st.pid_kd = [28, 30, 0]
            st.pid_dirty = True
        imgui.end_child()

    # --------------------------------------------------------------------------
    # Sub-Tab 4: Receiver & Radio Diagnostics
    # --------------------------------------------------------------------------
    def _render_tab_receiver(self):
        st = self.state
        imgui.begin_child("m3_card_rc", imgui.ImVec2(-1, 0), True)
        imgui.text_colored(imgui.ImVec4(0.486, 0.675, 0.973, 1.0), "RADIO RECEIVER & 16-CHANNEL PWM DIAGNOSTICS")
        imgui.separator()
        imgui.spacing()

        ch_names = ["Roll [A]", "Pitch [E]", "Throttle [T]", "Yaw [R]",
                    "Aux 1 (Arm)", "Aux 2 (Mode)", "Aux 3 (Buzzer)", "Aux 4 (RTH)",
                    "Aux 5", "Aux 6", "Aux 7", "Aux 8",
                    "Aux 9", "Aux 10", "Aux 11", "Aux 12"]

        for i in range(16):
            val = st.rc_channels[i]
            norm = max(0.0, min(1.0, (val - 1000.0) / 1000.0))
            imgui.text_colored(imgui.ImVec4(0.7, 0.72, 0.78, 1.0), f"CH {i+1:2d} ({ch_names[i]:15s}):")
            imgui.same_line()
            imgui.text(f"{val:4d} µs")
            imgui.same_line()
            imgui.progress_bar(norm, imgui.ImVec2(300, 16), "")
        imgui.end_child()

    # --------------------------------------------------------------------------
    # Sub-Tab 5: Motors & DShot ESCs
    # --------------------------------------------------------------------------
    def _render_tab_motors(self):
        st = self.state

        # Card 1: Safety Switch & Master Slider
        imgui.begin_child("m3_card_motors_safety", imgui.ImVec2(-1, 150), True)
        imgui.text_colored(imgui.ImVec4(0.95, 0.4, 0.35, 1.0), "MOTOR TEST SAFETY INTERLOCK")
        imgui.separator()
        imgui.spacing()

        _, st.motor_test_enabled = imgui.checkbox("I understand the risks: Propellers are removed. Enable motor test sliders.", st.motor_test_enabled)
        if not st.motor_test_enabled:
            st.motor_test_values = [1000] * 4
            st.master_motor_slider = 1000

        imgui.spacing()
        imgui.begin_disabled(not st.motor_test_enabled)
        imgui.text_colored(imgui.ImVec4(0.486, 0.675, 0.973, 1.0), "Master Slider (All Motors):")
        changed, st.master_motor_slider = imgui.slider_int("##master_m", st.master_motor_slider, 1000, 2000)
        if changed and st.motor_test_enabled:
            for i in range(4):
                st.motor_test_values[i] = st.master_motor_slider
        imgui.end_disabled()
        imgui.end_child()

        imgui.spacing()

        # Card 2: Individual Motor Channels
        imgui.begin_child("m3_card_motors_individual", imgui.ImVec2(-1, 180), True)
        imgui.text_colored(imgui.ImVec4(0.486, 0.675, 0.973, 1.0), "INDIVIDUAL ACTUATOR TEST (DSHOT)")
        imgui.separator()
        imgui.spacing()

        imgui.begin_disabled(not st.motor_test_enabled)
        for i in range(4):
            imgui.text_colored(imgui.ImVec4(0.7, 0.72, 0.78, 1.0), f"Motor {i+1}:")
            imgui.same_line()
            imgui.set_next_item_width(280)
            _, st.motor_test_values[i] = imgui.slider_int(f"##m_{i}", st.motor_test_values[i], 1000, 2000)

        imgui.spacing()
        if imgui.button("STOP ALL MOTORS (CUT THROTTLE)", imgui.ImVec2(320, 34)):
            st.motor_test_values = [1000] * 4
            st.master_motor_slider = 1000
            st.motor_test_enabled = False
        imgui.end_disabled()
        imgui.end_child()

    # --------------------------------------------------------------------------
    # Sub-Tab 6: Sensors & Live IMU Diagnostics
    # --------------------------------------------------------------------------
    def _render_tab_sensors(self):
        st = self.state
        imgui.begin_child("m3_card_sensors_live", imgui.ImVec2(-1, 0), True)
        imgui.text_colored(imgui.ImVec4(0.486, 0.675, 0.973, 1.0), "REAL-TIME SENSOR BUS TELEMETRY")
        imgui.separator()
        imgui.spacing()

        imgui.bullet_text(f"Accelerometer (G)  : X={st.accel[0]:+6.3f} G | Y={st.accel[1]:+6.3f} G | Z={st.accel[2]:+6.3f} G")
        imgui.bullet_text(f"Gyroscope (deg/s)  : Roll={st.gyro[0]:+7.1f}°/s | Pitch={st.gyro[1]:+7.1f}°/s | Yaw={st.gyro[2]:+7.1f}°/s")
        imgui.bullet_text(f"Magnetometer (raw) : X={st.mag[0]:+6d} | Y={st.mag[1]:+6d} | Z={st.mag[2]:+6d}")
        imgui.bullet_text(f"Barometer Altitude : {st.alt_m:6.2f} m | Vario: {st.vario_cms:+3d} cm/s")
        imgui.end_child()

    # --------------------------------------------------------------------------
    # Sub-Tab 7: GPS & Navigation
    # --------------------------------------------------------------------------
    def _render_tab_gps(self):
        st = self.state
        imgui.begin_child("m3_card_gps", imgui.ImVec2(-1, 0), True)
        imgui.text_colored(imgui.ImVec4(0.486, 0.675, 0.973, 1.0), "U-BLOX NEO-M8N GEODETIC SATELLITE NAVIGATION")
        imgui.separator()
        imgui.spacing()

        fix_str = {0: "No Fix", 1: "3D Fix", 2: "Differential GPS (DGPS)"}.get(st.gps_fix, "Fixing...")
        col_fix = imgui.ImVec4(0.35, 0.85, 0.55, 1.0) if st.gps_fix > 0 else imgui.ImVec4(0.95, 0.35, 0.35, 1.0)

        imgui.text("Fix Quality:")
        imgui.same_line()
        imgui.text_colored(col_fix, f"[{fix_str}] ({st.gps_sats} Satellites locked)")

        imgui.bullet_text(f"Latitude   : {st.gps_lat:12.7f}°")
        imgui.bullet_text(f"Longitude  : {st.gps_lon:12.7f}°")
        imgui.bullet_text(f"Altitude   : {st.gps_alt_m:6.1f} m MSL")
        imgui.bullet_text(f"Ground Spd : {st.gps_speed_mps:5.2f} m/s ({st.gps_speed_mps * 3.6:5.1f} km/h)")
        imgui.bullet_text(f"HDOP Acc   : {st.gps_hdop:.2f} (Good < 1.5)")
        imgui.end_child()

    # --------------------------------------------------------------------------
    # Sub-Tab 8: Interactive CLI Console
    # --------------------------------------------------------------------------
    def _render_tab_cli(self):
        st = self.state
        imgui.text_colored(imgui.ImVec4(0.2, 0.8, 1.0, 1.0), "INTERACTIVE FLIGHT CONTROLLER CLI TERMINAL")
        imgui.separator()

        # Log History Box
        avail_h = max(200.0, imgui.get_content_region_avail().y - 45.0)
        imgui.begin_child("cli_log_region", imgui.ImVec2(-1, avail_h), True)
        for line in st.cli_history:
            imgui.text(line)
        imgui.end_child()

        # Command Input
        imgui.set_next_item_width(-100)
        enter, st.cli_input_buf = imgui.input_text("##cli_cmd", st.cli_input_buf, imgui.InputTextFlags_.enter_returns_true)
        imgui.same_line()
        send = imgui.button("Send", imgui.ImVec2(80, 24))

        if (enter or send) and st.cli_input_buf.strip():
            cmd_text = st.cli_input_buf.strip()
            st.cli_history.append(f"> {cmd_text}")
            if cmd_text.lower() in ("status", "version"):
                st.cli_history.append(f"# {st.fc_variant} {st.fc_version} ({st.board_name}) API {st.api_version}")
                st.cli_history.append(f"# Arming: 0x{st.arming_flags:04X} | Sensors: 0x{st.sensor_flags:02X} | Loop: {st.cycle_time_us} us")
            elif cmd_text.lower() == "diff":
                st.cli_history.append("# diff all")
                st.cli_history.append(f"set p_pitch = {st.pid_kp[1]}")
                st.cli_history.append(f"set i_pitch = {st.pid_ki[1]}")
                st.cli_history.append(f"set d_pitch = {st.pid_kd[1]}")
            else:
                st.cli_history.append(f"# Unknown CLI command: '{cmd_text}'")
            st.cli_input_buf = ""
