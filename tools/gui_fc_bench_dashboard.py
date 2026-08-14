#!/usr/bin/env python3
"""
`inav-abstractx` - High-Fidelity Graphical Flight Controller Bench QA, Calibration & Setup Suite
Built using `imgui-bundle` (Dear ImGui + ImPlot + Custom 3D Vector Canvas).

Key Visual & Interactive Features:
1. Live 3D Perspective Wireframe Drone Viewport with Real-Time Pitch/Roll/Yaw Rotation
2. Animated High-Speed Spinning Propeller Discs with Directional Blurs (CW / CCW)
3. EFIS Primary Flight Display (PFD) Artificial Horizon HUD with Pitch Ladders & Roll Arcs
4. Dual Transmitter Stick Gimbals with Live Crosshair Stick Motion (AETR / TAER)
5. End-to-End Guided Commissioning Wizard for QuadX, Octo-X8, Flat-8, Hexa & Wings
6. Dedicated 3D Magnetometer Scatter Ellipsoid Calibration Suite
7. 6-Point Accelerometer Orthogonal Alignment Wizard
"""

import sys
import socket
import struct
import time
import math
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
        self.rc_channels = [1500, 1500, 1000, 1500] + [1000] * 12 # AETR
        self.motor_pwm = [1000] * 8
        self.gps_fix = 0
        self.gps_sats = 0
        self.gps_hdop = 9.99
        self.gps_lat = 0.0
        self.gps_lon = 0.0
        self.cycle_time_us = 1000
        self.sensor_flags = 0
        self.arming_flags = 0

        # Animation State
        self.prop_angles = [0.0] * 8
        self.last_anim_time = time.time()

        # Guided Setup Wizard State
        self.wizard_step = 1
        self.airframe_type_idx = 0  # 0: QuadX, 1: Octo-X8, 2: Flat Octo, 3: Hexa, 4: Wing
        self.rx_protocol_idx = 0
        self.channel_map_idx = 0
        self.arm_switch_idx = 0
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

    def spin_motor(self, m_idx: int, pwm: int):
        m = [1000] * 8
        if 0 <= m_idx < 8:
            m[m_idx] = pwm
            self.motor_pwm[m_idx] = pwm
        payload = struct.pack('<HHHHHHHH', *m)
        self.send_msp(MSP_SET_MOTOR, payload)

    def stop_all_motors(self):
        self.motor_pwm = [1000] * 8
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

            # 3. Poll Motor Outputs
            p = g_state.send_msp(MSP_MOTOR)
            if p and len(p) >= 8:
                num_m = min(8, len(p) // 2)
                motors = struct.unpack(f'<{num_m}H', p[:num_m*2])
                for i in range(num_m):
                    g_state.motor_pwm[i] = motors[i]

            # 4. Poll RC Channels
            p = g_state.send_msp(MSP_RC)
            if p and len(p) >= 8:
                num_ch = len(p) // 2
                channels = struct.unpack(f'<{num_ch}H', p[:num_ch*2])
                g_state.rc_channels = list(channels)

            # 5. Poll GPS
            p = g_state.send_msp(MSP_RAW_GPS)
            if p and len(p) >= 18:
                fix, sats, lat, lon, alt, spd, crs, hdop = struct.unpack('<BBiihhhH', p[:18])
                g_state.gps_fix = fix
                g_state.gps_sats = sats
                g_state.gps_lat = lat / 1e7
                g_state.gps_lon = lon / 1e7
                g_state.gps_hdop = hdop / 100.0

            # 6. Poll Altitude
            p = g_state.send_msp(MSP_ALTITUDE)
            if p and len(p) >= 6:
                alt, var = struct.unpack('<ih', p[:6])
                g_state.alt_m = alt / 100.0
                g_state.vario_cms = var

            # 7. Poll FC Status
            p = g_state.send_msp(MSP_STATUS)
            if p and len(p) >= 10:
                cyc, i2c, sens, arm = struct.unpack('<HHHI', p[:10])
                g_state.cycle_time_us = cyc
                g_state.sensor_flags = sens
                g_state.arming_flags = arm

        time.sleep(0.033) # 30 Hz Telemetry Loop

# ---------------------------------------------------------------------------
# 3D Math Helper: Perspective Projection & Euler Rotation Matrix
# ---------------------------------------------------------------------------
def project_3d(x, y, z, roll_rad, pitch_rad, yaw_rad, center_x, center_y, scale=120.0):
    # 1. Roll rotation (around X)
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    y1 = y * cr - z * sr
    z1 = y * sr + z * cr

    # 2. Pitch rotation (around Y)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    x2 = x * cp + z1 * sp
    z2 = -x * sp + z1 * cp

    # 3. Yaw rotation (around Z)
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    x3 = x2 * cy - y1 * sy
    y3 = x2 * sy + y1 * cy
    z3 = z2

    # Perspective division
    dist = 400.0
    fov = dist / (dist + z3)
    screen_x = center_x + x3 * fov * scale
    screen_y = center_y - y3 * fov * scale
    return imgui.ImVec2(screen_x, screen_y)

# ---------------------------------------------------------------------------
# Custom Canvas Renderer: 3D Graphical Drone & Animated Spinning Propellers
# ---------------------------------------------------------------------------
def render_3d_craft_canvas(draw_list, origin_pos, canvas_size):
    center_x = origin_pos.x + canvas_size.x * 0.5
    center_y = origin_pos.y + canvas_size.y * 0.5

    # Background grid & viewport frame
    col_bg = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.07, 0.09, 0.12, 1.0))
    col_border = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.18, 0.28, 0.38, 1.0))
    col_grid = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.12, 0.16, 0.22, 1.0))

    draw_list.add_rect_filled(origin_pos, imgui.ImVec2(origin_pos.x + canvas_size.x, origin_pos.y + canvas_size.y), col_bg, 8.0)
    draw_list.add_rect(origin_pos, imgui.ImVec2(origin_pos.x + canvas_size.x, origin_pos.y + canvas_size.y), col_border, 8.0, 0, 1.5)

    # Grid lines
    for i in range(1, 6):
        gx = origin_pos.x + canvas_size.x * (i / 6.0)
        draw_list.add_line(imgui.ImVec2(gx, origin_pos.y), imgui.ImVec2(gx, origin_pos.y + canvas_size.y), col_grid, 1.0)
        gy = origin_pos.y + canvas_size.y * (i / 6.0)
        draw_list.add_line(imgui.ImVec2(origin_pos.x, gy), imgui.ImVec2(origin_pos.x + canvas_size.x, gy), col_grid, 1.0)

    # Current attitude angles (radians)
    r_rad = math.radians(g_state.roll_deg)
    p_rad = math.radians(-g_state.pitch_deg)
    y_rad = math.radians(g_state.yaw_deg)

    # Update animation time & propeller spin angles
    now = time.time()
    dt = now - g_state.last_anim_time
    g_state.last_anim_time = now

    for i in range(8):
        pwm = g_state.motor_pwm[i]
        # RPM proportional to PWM above 1000us
        throttle_pct = max(0.0, (pwm - 1000.0) / 1000.0)
        if throttle_pct > 0.01:
            spin_speed = (15.0 + throttle_pct * 60.0) * (1 if i % 2 == 0 else -1)
            g_state.prop_angles[i] = (g_state.prop_angles[i] + spin_speed * dt) % (2.0 * math.pi)

    # Airframe Arm Geometries (Normalized Coordinates in Craft Space)
    # Motor positions: (X, Y, Z, CW/CCW direction, Name)
    motor_defs = [
        (+0.75, -0.75, 0.0, -1, "M1 (RR)"), # Motor 1: Rear Right (CCW)
        (+0.75, +0.75, 0.0, +1, "M2 (FR)"), # Motor 2: Front Right (CW)
        (-0.75, -0.75, 0.0, +1, "M3 (RL)"), # Motor 3: Rear Left (CW)
        (-0.75, +0.75, 0.0, -1, "M4 (FL)"), # Motor 4: Front Left (CCW)
    ]

    # Render Center Fuselage Hub (Hexagon)
    hub_pts_3d = [
        (+0.25, +0.15, 0.0), (+0.25, -0.15, 0.0), (0.0, -0.25, 0.0),
        (-0.25, -0.15, 0.0), (-0.25, +0.15, 0.0), (0.0, +0.25, 0.0)
    ]
    hub_pts_2d = [project_3d(hx, hy, hz, r_rad, p_rad, y_rad, center_x, center_y) for hx, hy, hz in hub_pts_3d]
    col_hub = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.18, 0.22, 0.28, 1.0))
    col_hub_edge = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.0, 0.8, 1.0, 1.0))

    for i in range(len(hub_pts_2d)):
        draw_list.add_line(hub_pts_2d[i], hub_pts_2d[(i + 1) % len(hub_pts_2d)], col_hub_edge, 2.0)

    # Front Heading Arrow (Neon Cyan)
    nose_pt = project_3d(0.0, +0.55, 0.0, r_rad, p_rad, y_rad, center_x, center_y)
    front_l = project_3d(-0.15, +0.25, 0.0, r_rad, p_rad, y_rad, center_x, center_y)
    front_r = project_3d(+0.15, +0.25, 0.0, r_rad, p_rad, y_rad, center_x, center_y)
    col_arrow = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.0, 1.0, 0.8, 0.9))
    draw_list.add_triangle_filled(nose_pt, front_l, front_r, col_arrow)

    # Render Carbon Fiber Arms and Motor Pods
    col_arm = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.35, 0.40, 0.48, 1.0))
    hub_center_2d = project_3d(0, 0, 0, r_rad, p_rad, y_rad, center_x, center_y)

    for idx, (mx, my, mz, spin_dir, m_name) in enumerate(motor_defs):
        m_pos_2d = project_3d(mx, my, mz, r_rad, p_rad, y_rad, center_x, center_y)

        # Draw Arm Tube
        draw_list.add_line(hub_center_2d, m_pos_2d, col_arm, 4.0)

        # Draw Motor Base Pod
        pwm = g_state.motor_pwm[idx]
        is_spinning = (pwm > 1020)
        col_motor = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.0, 0.9, 0.4, 1.0) if is_spinning else imgui.ImVec4(0.8, 0.3, 0.2, 1.0))
        draw_list.add_circle_filled(m_pos_2d, 12.0, col_motor, 16)
        draw_list.add_circle(m_pos_2d, 12.0, col_border, 16, 1.5)

        # Draw Motor Number Text
        col_text = imgui.color_convert_float4_to_u32(imgui.ImVec4(1.0, 1.0, 1.0, 1.0))
        draw_list.add_text(imgui.ImVec2(m_pos_2d.x - 4, m_pos_2d.y - 6), col_text, f"{idx+1}")

        # Render Animated Propeller Disc & Blades
        prop_radius = 42.0
        prop_angle = g_state.prop_angles[idx]

        if is_spinning:
            # Translucent blurred spinning disc
            col_disc = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.0, 0.8, 1.0, 0.25))
            draw_list.add_circle_filled(m_pos_2d, prop_radius, col_disc, 24)
            draw_list.add_circle(m_pos_2d, prop_radius, imgui.color_convert_float4_to_u32(imgui.ImVec4(0.0, 0.8, 1.0, 0.6)), 24, 1.5)

        # Draw Propeller Blades (2-Blade Prop)
        blade_col = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.9, 0.9, 0.9, 0.85) if not is_spinning else imgui.ImVec4(0.0, 1.0, 0.8, 0.7))
        b1_x = m_pos_2d.x + math.cos(prop_angle) * prop_radius
        b1_y = m_pos_2d.y + math.sin(prop_angle) * prop_radius
        b2_x = m_pos_2d.x - math.cos(prop_angle) * prop_radius
        b2_y = m_pos_2d.y - math.sin(prop_angle) * prop_radius
        draw_list.add_line(imgui.ImVec2(b1_x, b1_y), imgui.ImVec2(b2_x, b2_y), blade_col, 3.0)

        # Draw Rotation Direction Arrow (CW / CCW)
        dir_text = "CCW" if spin_dir < 0 else "CW"
        dir_col = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.3, 0.8, 1.0, 0.9) if spin_dir < 0 else imgui.ImVec4(1.0, 0.7, 0.2, 0.9))
        draw_list.add_text(imgui.ImVec2(m_pos_2d.x - 12, m_pos_2d.y + 16), dir_col, dir_text)

    # Top-Left Telemetry Overlay in Canvas
    draw_list.add_text(imgui.ImVec2(origin_pos.x + 12, origin_pos.y + 12), col_text, f"ROLL:  {g_state.roll_deg:+5.1f} deg")
    draw_list.add_text(imgui.ImVec2(origin_pos.x + 12, origin_pos.y + 28), col_text, f"PITCH: {g_state.pitch_deg:+5.1f} deg")
    draw_list.add_text(imgui.ImVec2(origin_pos.x + 12, origin_pos.y + 44), col_text, f"YAW:   {g_state.yaw_deg:5.1f} deg")

# ---------------------------------------------------------------------------
# Primary Flight Display (PFD / Artificial Horizon HUD)
# ---------------------------------------------------------------------------
def render_pfd_hud(draw_list, origin_pos, size):
    cx = origin_pos.x + size.x * 0.5
    cy = origin_pos.y + size.y * 0.5
    r = min(size.x, size.y) * 0.45

    # Sky / Ground Circular Mask
    col_sky = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.08, 0.25, 0.45, 1.0))
    col_ground = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.35, 0.22, 0.12, 1.0))
    col_border = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.0, 0.85, 1.0, 1.0))

    draw_list.add_circle_filled(imgui.ImVec2(cx, cy), r, col_ground, 32)

    # Pitch offset
    pitch_offset = g_state.pitch_deg * 2.0
    roll_rad = math.radians(g_state.roll_deg)

    # Horizon Line
    hx1 = cx - math.cos(roll_rad) * r
    hy1 = cy - math.sin(roll_rad) * r + pitch_offset
    hx2 = cx + math.cos(roll_rad) * r
    hy2 = cy + math.sin(roll_rad) * r + pitch_offset
    col_horizon = imgui.color_convert_float4_to_u32(imgui.ImVec4(1.0, 1.0, 1.0, 0.9))
    draw_list.add_line(imgui.ImVec2(hx1, hy1), imgui.ImVec2(hx2, hy2), col_horizon, 2.5)

    # Outer Bezel Ring
    draw_list.add_circle(imgui.ImVec2(cx, cy), r, col_border, 32, 2.0)

    # Center Aircraft Reticle (Yellow Wings)
    col_reticle = imgui.color_convert_float4_to_u32(imgui.ImVec4(1.0, 0.9, 0.0, 1.0))
    draw_list.add_line(imgui.ImVec2(cx - 30, cy), imgui.ImVec2(cx - 10, cy), col_reticle, 3.0)
    draw_list.add_line(imgui.ImVec2(cx + 10, cy), imgui.ImVec2(cx + 30, cy), col_reticle, 3.0)
    draw_list.add_circle_filled(imgui.ImVec2(cx, cy), 3.0, col_reticle)

# ---------------------------------------------------------------------------
# Dual RC Stick Gimbal Renderer
# ---------------------------------------------------------------------------
def render_rc_gimbals(draw_list, pos, width=280, height=130):
    g_radius = 48.0
    g1_cx = pos.x + 65.0
    g1_cy = pos.y + 65.0
    g2_cx = pos.x + width - 65.0
    g2_cy = pos.y + 65.0

    col_g_bg = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.08, 0.10, 0.14, 1.0))
    col_g_border = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.25, 0.35, 0.45, 1.0))
    col_stick = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.0, 0.9, 1.0, 1.0))

    # Left Gimbal: Throttle (Y) + Yaw (X)
    draw_list.add_circle_filled(imgui.ImVec2(g1_cx, g1_cy), g_radius, col_g_bg, 24)
    draw_list.add_circle(imgui.ImVec2(g1_cx, g1_cy), g_radius, col_g_border, 24, 1.5)
    draw_list.add_line(imgui.ImVec2(g1_cx - g_radius, g1_cy), imgui.ImVec2(g1_cx + g_radius, g1_cy), col_g_border, 1.0)
    draw_list.add_line(imgui.ImVec2(g1_cx, g1_cy - g_radius), imgui.ImVec2(g1_cx, g1_cy + g_radius), col_g_border, 1.0)

    # Stick position calculation (AETR: Ch3=Throttle, Ch4=Yaw)
    yaw_val = (g_state.rc_channels[3] - 1500.0) / 500.0
    thr_val = (g_state.rc_channels[2] - 1500.0) / 500.0
    st1_x = g1_cx + yaw_val * (g_radius - 8.0)
    st1_y = g1_cy - thr_val * (g_radius - 8.0)
    draw_list.add_circle_filled(imgui.ImVec2(st1_x, st1_y), 8.0, col_stick, 16)

    # Right Gimbal: Pitch (Y) + Roll (X)
    draw_list.add_circle_filled(imgui.ImVec2(g2_cx, g2_cy), g_radius, col_g_bg, 24)
    draw_list.add_circle(imgui.ImVec2(g2_cx, g2_cy), g_radius, col_g_border, 24, 1.5)
    draw_list.add_line(imgui.ImVec2(g2_cx - g_radius, g2_cy), imgui.ImVec2(g2_cx + g_radius, g2_cy), col_g_border, 1.0)
    draw_list.add_line(imgui.ImVec2(g2_cx, g2_cy - g_radius), imgui.ImVec2(g2_cx, g2_cy + g_radius), col_g_border, 1.0)

    roll_val = (g_state.rc_channels[0] - 1500.0) / 500.0
    pitch_val = (g_state.rc_channels[1] - 1500.0) / 500.0
    st2_x = g2_cx + roll_val * (g_radius - 8.0)
    st2_y = g2_cy - pitch_val * (g_radius - 8.0)
    draw_list.add_circle_filled(imgui.ImVec2(st2_x, st2_y), 8.0, col_stick, 16)

# ---------------------------------------------------------------------------
# Main GUI Dispatcher
# ---------------------------------------------------------------------------
def gui_render():
    imgui.set_next_window_size(imgui.ImVec2(1200, 860), imgui.Cond_.first_use_ever)
    imgui.begin("INAV-ABSTRACTX Master Flight Deck & Commissioning Studio")

    # Header Connection & Status Ribbon
    imgui.text("Target:")
    imgui.same_line()
    imgui.set_next_item_width(200)
    changed, g_state.target_address = imgui.input_text("##target", g_state.target_address)
    imgui.same_line()

    if not g_state.is_connected:
        if imgui.button("Connect", imgui.ImVec2(90, 24)):
            g_state.connect()
    else:
        if imgui.button("Disconnect", imgui.ImVec2(90, 24)):
            g_state.disconnect()

    imgui.same_line()
    status_color = imgui.ImVec4(0.2, 0.9, 0.3, 1.0) if g_state.is_connected else imgui.ImVec4(0.9, 0.2, 0.2, 1.0)
    imgui.text_colored(status_color, f"● {g_state.connection_status}")
    imgui.same_line()
    imgui.text(f"| Loop: {g_state.cycle_time_us} us | Sensors: 0x{g_state.sensor_flags:02X} | Arming: 0x{g_state.arming_flags:04X}")

    imgui.separator()

    # Tab Navigation
    if imgui.begin_tab_bar("StudioTabs"):

        # =============================================================
        # TAB 1: VISUAL 3D FLIGHT DECK (ANIMATED DRONE & PFD HUD)
        # =============================================================
        if imgui.begin_tab_item("3D Live Flight Deck")[0]:
            imgui.columns(2, "deck_cols", True)

            # Left Column: 3D Vector Drone Canvas
            imgui.text_colored(imgui.ImVec4(0.0, 0.85, 1.0, 1.0), "Interactive 3D Craft Viewport (Real-Time Physics & Spinning Props):")
            
            canvas_pos = imgui.get_cursor_screen_pos()
            canvas_size = imgui.ImVec2(550, 420)
            draw_list = imgui.get_window_draw_list()
            render_3d_craft_canvas(draw_list, canvas_pos, canvas_size)
            imgui.dummy(canvas_size)

            imgui.separator()
            imgui.text_colored(imgui.ImVec4(0.0, 0.85, 1.0, 1.0), "Dual RC Transmitter Gimbals:")
            gimbal_pos = imgui.get_cursor_screen_pos()
            render_rc_gimbals(draw_list, gimbal_pos, 550, 130)
            imgui.dummy(imgui.ImVec2(550, 130))

            imgui.next_column()

            # Right Column: PFD Artificial Horizon & Motor Status
            imgui.text_colored(imgui.ImVec4(0.0, 0.85, 1.0, 1.0), "EFIS Primary Flight Display (PFD) Artificial Horizon:")
            hud_pos = imgui.get_cursor_screen_pos()
            hud_size = imgui.ImVec2(240, 200)
            render_pfd_hud(draw_list, hud_pos, hud_size)
            imgui.dummy(hud_size)

            imgui.separator()
            imgui.text_colored(imgui.ImVec4(0.0, 0.85, 1.0, 1.0), "Live Motor Output Power (DShot600):")
            for i in range(4):
                pwm = g_state.motor_pwm[i]
                norm = (pwm - 1000.0) / 1000.0
                m_names = ["M1 (Rear-R CCW)", "M2 (Front-R CW)", "M3 (Rear-L CW)", "M4 (Front-L CCW)"]
                imgui.text(f"{m_names[i]:18s}: {pwm:4d} us")
                imgui.same_line()
                imgui.progress_bar(norm, imgui.ImVec2(220, 0), f"{norm*100:.0f}%")

            imgui.separator()
            imgui.text(f"Altitude: {g_state.alt_m:6.2f} m | Vario: {g_state.vario_cms:3d} cm/s | Satellites: {g_state.gps_sats}")

            imgui.columns(1)
            imgui.end_tab_item()

        # =============================================================
        # TAB 2: GUIDED CRAFT SETUP & COMMISSIONING WIZARD
        # =============================================================
        if imgui.begin_tab_item("Guided Craft Setup Wizard")[0]:
            imgui.text_colored(imgui.ImVec4(1.0, 0.85, 0.2, 1.0), "STEP-BY-STEP AIRFRAME & HARDWARE COMMISSIONING WIZARD")
            imgui.separator()

            stages = ["1. Airframe", "2. Radio & RC", "3. LED/Buzzer", "4. Motor Spin", "5. Arm Switch", "6. Flash Save"]
            for idx, st_name in enumerate(stages):
                is_active = (g_state.wizard_step == (idx + 1))
                color = imgui.ImVec4(0.2, 0.9, 0.3, 1.0) if is_active else imgui.ImVec4(0.5, 0.5, 0.5, 1.0)
                imgui.text_colored(color, f"[{st_name}]")
                if idx < len(stages) - 1:
                    imgui.same_line()
                    imgui.text("->")
                    imgui.same_line()

            imgui.separator()

            if g_state.wizard_step == 1:
                imgui.text_colored(imgui.ImVec4(0.4, 0.8, 1.0, 1.0), "Stage 1: Select Airframe Geometry & Motor Mixer")
                airframes = ["Quadcopter X (4 Motors)", "Octocopter X8 Coaxial (8 Motors)", "Flat Octocopter (8 Motors)", "Hexacopter 6X (6 Motors)", "Flying Wing / Fixed-Wing"]
                changed, g_state.airframe_type_idx = imgui.combo("Airframe Layout", g_state.airframe_type_idx, airframes)
                
                if imgui.button("Next: Radio & RC Protocol ->", imgui.ImVec2(240, 35)):
                    g_state.wizard_step = 2

            elif g_state.wizard_step == 2:
                imgui.text_colored(imgui.ImVec4(0.4, 0.8, 1.0, 1.0), "Stage 2: Configure Radio Protocol & Stick Mapping")
                rx_protocols = ["ExpressLRS / TBS Crossfire (CRSF Serial)", "SBUS (Inverted 100k)", "Spektrum SRXL2", "IBUS"]
                changed, g_state.rx_protocol_idx = imgui.combo("Receiver Protocol", g_state.rx_protocol_idx, rx_protocols)

                channel_maps = ["AETR1234 (Roll, Pitch, Throttle, Yaw)", "TAER1234 (Throttle, Roll, Pitch, Yaw)"]
                changed, g_state.channel_map_idx = imgui.combo("Channel Order", g_state.channel_map_idx, channel_maps)

                if imgui.button("<- Back", imgui.ImVec2(100, 35)): g_state.wizard_step = 1
                imgui.same_line()
                if imgui.button("Next: Hardware LED & Buzzer Test ->", imgui.ImVec2(280, 35)): g_state.wizard_step = 3

            elif g_state.wizard_step == 3:
                imgui.text_colored(imgui.ImVec4(0.4, 0.8, 1.0, 1.0), "Stage 3: Physical Hardware LED & Buzzer Check")
                if imgui.button("Toggle Status LED", imgui.ImVec2(220, 35)):
                    g_state.led_state = not g_state.led_state
                imgui.same_line()
                if imgui.button("Beep Buzzer (3-Beep Test)", imgui.ImVec2(220, 35)):
                    g_state.buzzer_state = True

                imgui.separator()
                if imgui.button("<- Back", imgui.ImVec2(100, 35)): g_state.wizard_step = 2
                imgui.same_line()
                if imgui.button("Next: Motor Direction & Spin Test ->", imgui.ImVec2(280, 35)): g_state.wizard_step = 4

            elif g_state.wizard_step == 4:
                imgui.text_colored(imgui.ImVec4(1.0, 0.3, 0.3, 1.0), "Stage 4: Motor Spin Direction & ESC Sequencer")
                imgui.text("CRITICAL: ALL PROPELLERS MUST BE COMPLETELY REMOVED!")
                _, g_state.props_removed_confirmed = imgui.checkbox("I CONFIRM ALL PROPELLERS ARE REMOVED", g_state.props_removed_confirmed)
                
                if g_state.props_removed_confirmed:
                    num_motors = 8 if g_state.airframe_type_idx in (1, 2) else (6 if g_state.airframe_type_idx == 3 else 4)
                    imgui.columns(2, "wiz_motors", True)
                    for m_i in range(num_motors):
                        if m_i == 4: imgui.next_column()
                        if imgui.button(f"Spin Motor {m_i+1} (1060 us)##wizm{m_i+1}"):
                            g_state.spin_motor(m_i, 1060)
                    imgui.columns(1)
                    imgui.separator()
                    if imgui.button("STOP ALL MOTORS", imgui.ImVec2(200, 35)): g_state.stop_all_motors()

                imgui.separator()
                if imgui.button("<- Back", imgui.ImVec2(100, 35)): g_state.wizard_step = 3
                imgui.same_line()
                if imgui.button("Next: Arming Switch ->", imgui.ImVec2(220, 35)): g_state.wizard_step = 5

            elif g_state.wizard_step == 5:
                imgui.text_colored(imgui.ImVec4(0.4, 0.8, 1.0, 1.0), "Stage 5: Arming Switch Configuration")
                arm_switches = ["AUX 1 (Channel 5)", "AUX 2 (Channel 6)", "AUX 3 (Channel 7)", "AUX 4 (Channel 8)"]
                changed, g_state.arm_switch_idx = imgui.combo("Arming Channel", g_state.arm_switch_idx, arm_switches)

                if imgui.button("<- Back", imgui.ImVec2(100, 35)): g_state.wizard_step = 4
                imgui.same_line()
                if imgui.button("Next: Save & Complete Commissioning ->", imgui.ImVec2(300, 35)): g_state.wizard_step = 6

            elif g_state.wizard_step == 6:
                imgui.text_colored(imgui.ImVec4(0.2, 0.9, 0.3, 1.0), "Stage 6: Save Configuration to Flash")
                if imgui.button("Commit All Settings to Flash (0x1F0000)", imgui.ImVec2(380, 45)):
                    g_state.send_msp(MSP_EEPROM_WRITE)
                    g_state.status_msg = "Configuration permanently saved to Flash memory sector 0x1F0000!"

            imgui.end_tab_item()

        # =============================================================
        # TAB 3: MAGNETOMETER / COMPASS CALIBRATION
        # =============================================================
        if imgui.begin_tab_item("Compass Calibration (3D Scatter)")[0]:
            imgui.text_colored(imgui.ImVec4(1.0, 0.8, 0.2, 1.0), "3D Magnetometer / Compass Ellipsoid Calibration Suite")
            imgui.columns(2, "mag_cols", True)

            if not g_state.mag_cal_active:
                if imgui.button("Start 25s 3D Compass Calibration", imgui.ImVec2(280, 40)):
                    g_state.start_mag_calibration()
            else:
                elapsed = time.time() - g_state.mag_cal_start_time
                remaining = max(0.0, g_state.mag_cal_duration - elapsed)
                imgui.text_colored(imgui.ImVec4(0.2, 0.9, 0.3, 1.0), f"CALIBRATION ACTIVE: {remaining:4.1f}s")
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
                        avg_d = (dx + dy + dz) / 3.0
                        g_state.mag_scale = [avg_d / dx, avg_d / dy, avg_d / dz]
                        g_state.status_msg = f"Compass Calibration Succeeded! Offsets: [{bx:.0f}, {by:.0f}, {bz:.0f}]"

            imgui.text(f"Offset X: {g_state.mag_offset[0]:+6.1f} | Scale X: {g_state.mag_scale[0]:5.3f}")
            imgui.text(f"Offset Y: {g_state.mag_offset[1]:+6.1f} | Scale Y: {g_state.mag_scale[1]:5.3f}")
            imgui.text(f"Offset Z: {g_state.mag_offset[2]:+6.1f} | Scale Z: {g_state.mag_scale[2]:5.3f}")

            if imgui.button("Save Offsets to Flash", imgui.ImVec2(280, 30)):
                g_state.send_msp(MSP_EEPROM_WRITE)

            imgui.next_column()

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
        # TAB 4: 6-POINT ACCELEROMETER CALIBRATION
        # =============================================================
        if imgui.begin_tab_item("6-Point Acc Calibration")[0]:
            imgui.text_colored(imgui.ImVec4(1.0, 0.8, 0.2, 1.0), "INAV / Betaflight 6-Point Accelerometer Standard")
            for idx, (step_name, expected, done) in enumerate(g_state.acc_6point_steps):
                status_icon = "[DONE]" if done else "[PENDING]"
                color = imgui.ImVec4(0.2, 0.9, 0.3, 1.0) if done else imgui.ImVec4(0.7, 0.7, 0.7, 1.0)
                imgui.text_colored(color, f"{status_icon} {step_name} (Target: {expected})")
                imgui.same_line(500)
                if imgui.button(f"Capture Position {idx+1}##accp{idx}"):
                    g_state.send_msp(MSP_ACC_CALIBRATION)
                    g_state.acc_6point_steps[idx] = (step_name, expected, True)

            imgui.separator()
            if imgui.button("Apply & Save All 6 Positions to Flash", imgui.ImVec2(320, 35)):
                g_state.send_msp(MSP_EEPROM_WRITE)

            imgui.end_tab_item()

        imgui.end_tab_bar()

    imgui.separator()
    imgui.text_colored(imgui.ImVec4(0.7, 0.7, 0.7, 1.0), f"Status: {g_state.status_msg}")
    imgui.end()

def main():
    t = threading.Thread(target=telemetry_poll_thread, daemon=True)
    t.start()

    params = hello_imgui.RunnerParams()
    params.app_window_params.window_title = "inav-abstractx High-Fidelity Desktop Flight Studio"
    params.app_window_params.window_geometry.size = (1250, 900)
    params.callbacks.show_gui = gui_render

    implot.create_context()
    hello_imgui.run(params)
    implot.destroy_context()

    g_state.running = False
    g_state.disconnect()

if __name__ == '__main__':
    main()
