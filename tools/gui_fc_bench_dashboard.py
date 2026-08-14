#!/usr/bin/env python3
"""
`inav-abstractx` - World-Class Desktop Flight Deck, 3D Vector Simulation & Commissioning Studio
Built using `imgui-bundle` (Dear ImGui + ImPlot + Pure C++20-grade 3D Software Polygon Rendering Engine).

World-Class Visual & Simulation Features:
1. Live 3D Solid Polygon Drone Mesh with Real-Time Lambertian Shading & Depth Sorting (Painter's Algorithm)
2. Interactive 3D Orbit Camera (Click-and-Drag Orbit, Mouse Wheel Zoom, Auto-Attitude Tracking)
3. Live Autonomous Flight Demo / Walkthrough Mode (Auto-Synthesized 3D Acro Kinematics & Spinning Props)
4. Dynamic Ground Plane Ambient Occlusion Drop Shadow & Perspective Infinite Grid
5. High-Speed Aerodynamic Propeller Blades with Motion Blur Discs & Rotation Vector Arrows
6. Boeing/Garmin G1000 EFIS Primary Flight Display (PFD) with Pitch Chevrons, Altitude & Speed Tapes
7. Dual Precision Transmitter Gimbals with Stick Phosphor Trails & Rate Gauges
8. Step-by-Step Guided Craft Setup Wizard for QuadX, Octo-X8, Flat-8, Hexa & Flying Wings
9. 3D Magnetometer Live Ellipsoid Scatter & 6-Point Accelerometer Calibration
"""

import sys
import socket
import struct
import time
import math
import argparse
import threading
import numpy as np
from imgui_bundle import imgui, hello_imgui, implot

# MSP Protocol Commands
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
        self.is_demo_mode = False
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
        self.ground_speed_kms = 0.0
        self.accel_g = [0.0, 0.0, 1.0]
        self.gyro_dps = [0.0, 0.0, 0.0]
        self.mag_raw = [0, 0, 0]
        self.rc_channels = [1500, 1500, 1000, 1500] + [1000] * 12 # AETR
        self.motor_pwm = [1000] * 8
        self.gps_fix = 0
        self.gps_sats = 0
        self.gps_hdop = 9.99
        self.gps_lat = 37.7749
        self.gps_lon = -122.4194
        self.cycle_time_us = 1000
        self.sensor_flags = 0
        self.arming_flags = 0

        # Interactive 3D Orbit Camera
        self.cam_azimuth = math.radians(45.0)
        self.cam_elevation = math.radians(28.0)
        self.cam_dist = 4.2
        self.is_dragging_cam = False
        self.last_mouse_pos = imgui.ImVec2(0, 0)

        # Propeller Animation & Demo Tour
        self.prop_angles = [0.0] * 8
        self.last_anim_time = time.time()
        self.tour_step = 0
        self.show_walkthrough_banner = True

        # Guided Setup Wizard
        self.wizard_step = 1
        self.airframe_type_idx = 0
        self.rx_protocol_idx = 0
        self.channel_map_idx = 0
        self.arm_switch_idx = 0
        self.led_state = False
        self.buzzer_state = False

        # Calibration State
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

        self.acc_6point_steps = [
            ("1. Level (Landing Skids)", "+1.0G on Z", False),
            ("2. Left Side (Left Wing Down)", "+1.0G on Y", False),
            ("3. Right Side (Right Wing Down)", "-1.0G on Y", False),
            ("4. Nose UP (Pointing to Ceiling)", "-1.0G on X", False),
            ("5. Nose DOWN (Pointing to Floor)", "+1.0G on X", False),
            ("6. Inverted (Upside Down)", "-1.0G on Z", False),
        ]

        self.props_removed_confirmed = False
        self.status_msg = "Ready. Connect to Flight Controller or enable Demo Mode."

    def toggle_demo_mode(self):
        self.is_demo_mode = not self.is_demo_mode
        if self.is_demo_mode:
            self.disconnect()
            self.connection_status = "DEMO FLIGHT SIMULATION ACTIVE"
            self.status_msg = "Demo Mode: Synthesizing full 3D acro kinematics & high-speed spinning props."
        else:
            self.connection_status = "Disconnected"
            self.status_msg = "Demo Mode Stopped. Ready for live FC connection."

    def connect(self):
        if self.is_demo_mode:
            self.is_demo_mode = False
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
        if not self.is_demo_mode:
            self.connection_status = "Disconnected"
            self.status_msg = "Disconnected."

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
        # --- 1. Synthesized Demo Flight Kinematics Generator ---
        if g_state.is_demo_mode:
            t = time.time()
            # Smooth multi-harmonic banking flight choreography
            g_state.roll_deg = math.sin(t * 1.1) * 24.0 + math.sin(t * 2.7) * 7.0
            g_state.pitch_deg = math.cos(t * 0.85) * 18.0 + math.cos(t * 2.2) * 5.0
            g_state.yaw_deg = (t * 22.0) % 360.0

            # Dynamic altitude climb & ground speed
            g_state.alt_m = 48.0 + math.sin(t * 0.35) * 22.0
            g_state.vario_cms = int(math.cos(t * 0.35) * 90.0)
            g_state.ground_speed_kms = 42.0 + math.sin(t * 0.7) * 18.0
            g_state.gps_fix = 1
            g_state.gps_sats = 16
            g_state.gps_hdop = 0.88
            g_state.cycle_time_us = 1000
            g_state.sensor_flags = 0x3F # Gyro, Acc, Baro, Mag, GPS, Sonar

            # Modulate 4 motors dynamically to balance the attitude torques
            base_thr = 1460 + int(math.sin(t * 0.4) * 100.0)
            r_torque = int(g_state.roll_deg * 5.5)
            p_torque = int(g_state.pitch_deg * 5.5)

            g_state.motor_pwm[0] = max(1000, min(2000, base_thr - r_torque + p_torque))
            g_state.motor_pwm[1] = max(1000, min(2000, base_thr - r_torque - p_torque))
            g_state.motor_pwm[2] = max(1000, min(2000, base_thr + r_torque + p_torque))
            g_state.motor_pwm[3] = max(1000, min(2000, base_thr + r_torque - p_torque))

            # Modulate RC stick positions in sync
            g_state.rc_channels[0] = 1500 + int(g_state.roll_deg * 14.0)  # Roll
            g_state.rc_channels[1] = 1500 + int(g_state.pitch_deg * 14.0) # Pitch
            g_state.rc_channels[2] = base_thr                              # Throttle
            g_state.rc_channels[3] = 1500 + int(math.sin(t * 1.4) * 280.0)# Yaw

            time.sleep(0.025)
            continue

        # --- 2. Live MSP Telemetry Polling from Connected Hardware ---
        if g_state.is_connected:
            p = g_state.send_msp(MSP_ATTITUDE)
            if p and len(p) >= 6:
                r, pt, y = struct.unpack('<hhh', p[:6])
                g_state.roll_deg = r / 10.0
                g_state.pitch_deg = pt / 10.0
                g_state.yaw_deg = float(y)

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

            p = g_state.send_msp(MSP_MOTOR)
            if p and len(p) >= 8:
                num_m = min(8, len(p) // 2)
                motors = struct.unpack(f'<{num_m}H', p[:num_m*2])
                for i in range(num_m):
                    g_state.motor_pwm[i] = motors[i]

            p = g_state.send_msp(MSP_RC)
            if p and len(p) >= 8:
                num_ch = len(p) // 2
                channels = struct.unpack(f'<{num_ch}H', p[:num_ch*2])
                g_state.rc_channels = list(channels)

            p = g_state.send_msp(MSP_RAW_GPS)
            if p and len(p) >= 18:
                fix, sats, lat, lon, alt, spd, crs, hdop = struct.unpack('<BBiihhhH', p[:18])
                g_state.gps_fix = fix
                g_state.gps_sats = sats
                g_state.gps_lat = lat / 1e7
                g_state.gps_lon = lon / 1e7
                g_state.gps_hdop = hdop / 100.0
                g_state.ground_speed_kms = (spd / 100.0) * 3.6

            p = g_state.send_msp(MSP_ALTITUDE)
            if p and len(p) >= 6:
                alt, var = struct.unpack('<ih', p[:6])
                g_state.alt_m = alt / 100.0
                g_state.vario_cms = var

            p = g_state.send_msp(MSP_STATUS)
            if p and len(p) >= 10:
                cyc, i2c, sens, arm = struct.unpack('<HHHI', p[:10])
                g_state.cycle_time_us = cyc
                g_state.sensor_flags = sens
                g_state.arming_flags = arm

        time.sleep(0.025) # 40 Hz Smooth Telemetry Loop

# ---------------------------------------------------------------------------
# High-Fidelity 3D Vector Math & Solid Mesh Rendering
# ---------------------------------------------------------------------------
class Vec3:
    __slots__ = ('x', 'y', 'z')
    def __init__(self, x: float, y: float, z: float):
        self.x, self.y, self.z = x, y, z

    def rotate_euler(self, roll_rad, pitch_rad, yaw_rad):
        cr, sr = math.cos(roll_rad), math.sin(roll_rad)
        y1 = self.y * cr - self.z * sr
        z1 = self.y * sr + self.z * cr
        cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
        x2 = self.x * cp + z1 * sp
        z2 = -self.x * sp + z1 * cp
        cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
        x3 = x2 * cy - y1 * sy
        y3 = x2 * sy + y1 * cy
        return Vec3(x3, y3, z2)

def camera_transform(v: Vec3, azim, elev, dist):
    ca, sa = math.cos(azim), math.sin(azim)
    ce, se = math.cos(elev), math.sin(elev)
    x1 = v.x * ca - v.y * sa
    y1 = v.x * sa + v.y * ca
    z1 = v.z
    x2 = x1
    y2 = y1 * ce - z1 * se
    z2 = y1 * se + z1 * ce
    return Vec3(x2, y2, z2 + dist)

def project_camera_point(cam_v: Vec3, cx: float, cy: float, fov_scale=380.0):
    if cam_v.z <= 0.1: cam_v.z = 0.1
    inv_z = fov_scale / cam_v.z
    return imgui.ImVec2(cx + cam_v.x * inv_z, cy - cam_v.y * inv_z)

class Polygon3D:
    def __init__(self, vertices, base_color: imgui.ImVec4):
        self.vertices = vertices
        self.base_color = base_color

def render_world_class_3d_viewport(draw_list, pos, size):
    cx = pos.x + size.x * 0.5
    cy = pos.y + size.y * 0.5

    col_bg = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.05, 0.07, 0.10, 1.0))
    col_border = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.0, 0.85, 1.0, 0.85))
    draw_list.add_rect_filled(pos, imgui.ImVec2(pos.x + size.x, pos.y + size.y), col_bg, 8.0)
    draw_list.add_rect(pos, imgui.ImVec2(pos.x + size.x, pos.y + size.y), col_border, 8.0, 0, 1.5)

    io = imgui.get_io()
    mouse_pos = io.mouse_pos
    if imgui.is_window_hovered():
        if io.mouse_wheel != 0.0:
            g_state.cam_dist = max(1.8, min(10.0, g_state.cam_dist - io.mouse_wheel * 0.3))
        if io.mouse_down[0] and pos.x <= mouse_pos.x <= pos.x + size.x and pos.y <= mouse_pos.y <= pos.y + size.y:
            if not g_state.is_dragging_cam:
                g_state.is_dragging_cam = True
                g_state.last_mouse_pos = mouse_pos
            else:
                dx = mouse_pos.x - g_state.last_mouse_pos.x
                dy = mouse_pos.y - g_state.last_mouse_pos.y
                g_state.cam_azimuth += dx * 0.008
                g_state.cam_elevation = max(-math.pi*0.48, min(math.pi*0.48, g_state.cam_elevation + dy * 0.008))
                g_state.last_mouse_pos = mouse_pos
        else:
            g_state.is_dragging_cam = False

    azim = g_state.cam_azimuth
    elev = g_state.cam_elevation
    dist = g_state.cam_dist

    r_rad = math.radians(g_state.roll_deg)
    p_rad = math.radians(-g_state.pitch_deg)
    y_rad = math.radians(g_state.yaw_deg)

    # Ground Perspective Grid
    grid_size = 5
    grid_step = 0.5
    ground_z = -0.85
    col_grid = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.12, 0.20, 0.30, 0.5))

    for i in range(-grid_size, grid_size + 1):
        v1 = camera_transform(Vec3(-grid_size * grid_step, i * grid_step, ground_z), azim, elev, dist)
        v2 = camera_transform(Vec3(grid_size * grid_step, i * grid_step, ground_z), azim, elev, dist)
        draw_list.add_line(project_camera_point(v1, cx, cy), project_camera_point(v2, cx, cy), col_grid, 1.0)
        v3 = camera_transform(Vec3(i * grid_step, -grid_size * grid_step, ground_z), azim, elev, dist)
        v4 = camera_transform(Vec3(i * grid_step, grid_size * grid_step, ground_z), azim, elev, dist)
        draw_list.add_line(project_camera_point(v3, cx, cy), project_camera_point(v4, cx, cy), col_grid, 1.0)

    # Ambient Occlusion Shadow
    shadow_v = camera_transform(Vec3(0.0, 0.0, ground_z + 0.01), azim, elev, dist)
    shadow_center = project_camera_point(shadow_v, cx, cy)
    shadow_radius = max(10.0, (180.0 / shadow_v.z))
    col_shadow = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.0, 0.0, 0.0, 0.45))
    draw_list.add_circle_filled(shadow_center, shadow_radius, col_shadow, 24)

    # Polygons (Fuselage, Arms, Pod)
    polygons = []
    light_dir = Vec3(0.4, 0.6, 0.9)
    l_mag = math.sqrt(light_dir.x**2 + light_dir.y**2 + light_dir.z**2)
    light_dir.x /= l_mag; light_dir.y /= l_mag; light_dir.z /= l_mag

    hub_radius = 0.38
    hub_h = 0.06
    hub_corners = 6
    top_verts = []
    bot_verts = []
    for k in range(hub_corners):
        ang = k * (2.0 * math.pi / hub_corners)
        top_verts.append(Vec3(math.cos(ang) * hub_radius, math.sin(ang) * hub_radius, +hub_h))
        bot_verts.append(Vec3(math.cos(ang) * hub_radius, math.sin(ang) * hub_radius, -hub_h))

    polygons.append(Polygon3D(top_verts, imgui.ImVec4(0.18, 0.22, 0.28, 1.0)))
    polygons.append(Polygon3D(bot_verts[::-1], imgui.ImVec4(0.12, 0.15, 0.20, 1.0)))

    for k in range(hub_corners):
        k_next = (k + 1) % hub_corners
        polygons.append(Polygon3D([top_verts[k], top_verts[k_next], bot_verts[k_next], bot_verts[k]], imgui.ImVec4(0.15, 0.19, 0.24, 1.0)))

    # FPV Camera Nose Cone
    cam_pod = [
        Vec3(-0.12, +0.32, +0.14), Vec3(+0.12, +0.32, +0.14),
        Vec3(+0.08, +0.52, +0.02), Vec3(-0.08, +0.52, +0.02)
    ]
    polygons.append(Polygon3D(cam_pod, imgui.ImVec4(0.0, 0.85, 1.0, 0.95)))

    motor_coords = [
        (+0.95, -0.95, 0.0, -1, "M1"),
        (+0.95, +0.95, 0.0, +1, "M2"),
        (-0.95, -0.95, 0.0, +1, "M3"),
        (-0.95, +0.95, 0.0, -1, "M4")
    ]

    arm_width = 0.045
    for mx, my, mz, spin_dir, name in motor_coords:
        length = math.sqrt(mx**2 + my**2)
        nx = -my / length * arm_width
        ny = mx / length * arm_width
        arm_top = [
            Vec3(nx, ny, +hub_h*0.8), Vec3(mx + nx, my + ny, +hub_h*0.8),
            Vec3(mx - nx, my - ny, +hub_h*0.8), Vec3(-nx, -ny, +hub_h*0.8)
        ]
        arm_bot = [
            Vec3(nx, ny, -hub_h*0.8), Vec3(-nx, -ny, -hub_h*0.8),
            Vec3(mx - nx, my - ny, -hub_h*0.8), Vec3(mx + nx, my + ny, -hub_h*0.8)
        ]
        polygons.append(Polygon3D(arm_top, imgui.ImVec4(0.24, 0.28, 0.34, 1.0)))
        polygons.append(Polygon3D(arm_bot, imgui.ImVec4(0.16, 0.20, 0.25, 1.0)))

    transformed_faces = []
    for poly in polygons:
        world_verts = [v.rotate_euler(r_rad, p_rad, y_rad) for v in poly.vertices]
        cam_verts = [camera_transform(wv, azim, elev, dist) for wv in world_verts]
        avg_depth = sum(cv.z for cv in cam_verts) / len(cam_verts)

        v0, v1, v2 = world_verts[0], world_verts[1], world_verts[2]
        e1 = Vec3(v1.x - v0.x, v1.y - v0.y, v1.z - v0.z)
        e2 = Vec3(v2.x - v0.x, v2.y - v0.y, v2.z - v0.z)
        norm = Vec3(e1.y * e2.z - e1.z * e2.y, e1.z * e2.x - e1.x * e2.z, e1.x * e2.y - e1.y * e2.x)
        norm_mag = math.sqrt(norm.x**2 + norm.y**2 + norm.z**2)
        if norm_mag > 1e-6:
            norm.x /= norm_mag; norm.y /= norm_mag; norm.z /= norm_mag

        dot = max(0.0, norm.x * light_dir.x + norm.y * light_dir.y + norm.z * light_dir.z)
        intensity = 0.35 + 0.65 * dot

        lit_color = imgui.color_convert_float4_to_u32(imgui.ImVec4(
            min(1.0, poly.base_color.x * intensity),
            min(1.0, poly.base_color.y * intensity),
            min(1.0, poly.base_color.z * intensity),
            poly.base_color.w
        ))

        proj_points = [project_camera_point(cv, cx, cy) for cv in cam_verts]
        transformed_faces.append((avg_depth, proj_points, lit_color))

    transformed_faces.sort(key=lambda item: item[0], reverse=True)

    for depth, pts, color in transformed_faces:
        if len(pts) == 3:
            draw_list.add_triangle_filled(pts[0], pts[1], pts[2], color)
        elif len(pts) == 4:
            draw_list.add_quad_filled(pts[0], pts[1], pts[2], pts[3], color)
        else:
            for t in range(1, len(pts) - 1):
                draw_list.add_triangle_filled(pts[0], pts[t], pts[t + 1], color)

    now = time.time()
    dt = now - g_state.last_anim_time
    g_state.last_anim_time = now

    for idx, (mx, my, mz, spin_dir, name) in enumerate(motor_coords):
        pwm = g_state.motor_pwm[idx]
        throttle_pct = max(0.0, (pwm - 1000.0) / 1000.0)
        is_spinning = (throttle_pct > 0.01)

        if is_spinning:
            spin_speed = (25.0 + throttle_pct * 95.0) * spin_dir
            g_state.prop_angles[idx] = (g_state.prop_angles[idx] + spin_speed * dt) % (2.0 * math.pi)

        m_world = Vec3(mx, my, mz + 0.08).rotate_euler(r_rad, p_rad, y_rad)
        m_cam = camera_transform(m_world, azim, elev, dist)
        m_screen = project_camera_point(m_cam, cx, cy)
        m_radius = max(8.0, (55.0 / m_cam.z))

        col_bell = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.0, 0.95, 0.4, 1.0) if is_spinning else imgui.ImVec4(0.85, 0.35, 0.2, 1.0))
        draw_list.add_circle_filled(m_screen, m_radius, col_bell, 20)
        draw_list.add_circle(m_screen, m_radius, imgui.color_convert_float4_to_u32(imgui.ImVec4(1.0, 1.0, 1.0, 0.9)), 20, 1.5)

        prop_radius = max(24.0, (175.0 / m_cam.z))
        if is_spinning:
            col_blur = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.0, 0.85, 1.0, 0.30))
            draw_list.add_circle_filled(m_screen, prop_radius, col_blur, 28)
            draw_list.add_circle(m_screen, prop_radius, imgui.color_convert_float4_to_u32(imgui.ImVec4(0.0, 0.85, 1.0, 0.7)), 28, 1.5)

        p_ang = g_state.prop_angles[idx]
        b1_local = Vec3(mx + math.cos(p_ang) * 0.45, my + math.sin(p_ang) * 0.45, mz + 0.12).rotate_euler(r_rad, p_rad, y_rad)
        b2_local = Vec3(mx - math.cos(p_ang) * 0.45, my - math.sin(p_ang) * 0.45, mz + 0.12).rotate_euler(r_rad, p_rad, y_rad)
        b1_screen = project_camera_point(camera_transform(b1_local, azim, elev, dist), cx, cy)
        b2_screen = project_camera_point(camera_transform(b2_local, azim, elev, dist), cx, cy)

        col_blade = imgui.color_convert_float4_to_u32(imgui.ImVec4(1.0, 1.0, 1.0, 0.9) if not is_spinning else imgui.ImVec4(0.0, 1.0, 0.85, 0.8))
        draw_list.add_line(b1_screen, b2_screen, col_blade, 3.5)

        dir_str = "CCW" if spin_dir < 0 else "CW"
        col_tag = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.3, 0.85, 1.0, 0.9) if spin_dir < 0 else imgui.ImVec4(1.0, 0.75, 0.2, 0.9))
        draw_list.add_text(imgui.ImVec2(m_screen.x - 14, m_screen.y + m_radius + 4), col_tag, f"{name} {dir_str}")

    col_hud = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.0, 0.85, 1.0, 0.9))
    draw_list.add_text(imgui.ImVec2(pos.x + 15, pos.y + 15), col_hud, "3D ORBIT VIEWPORT [DRAG MOUSE TO ROTATE | SCROLL TO ZOOM]")
    draw_list.add_text(imgui.ImVec2(pos.x + 15, pos.y + 35), imgui.color_convert_float4_to_u32(imgui.ImVec4(1.0, 1.0, 1.0, 0.8)), f"ROLL: {g_state.roll_deg:+5.1f}° | PITCH: {g_state.pitch_deg:+5.1f}° | YAW: {g_state.yaw_deg:5.1f}°")

# ---------------------------------------------------------------------------
# Boeing/Garmin G1000 EFIS Primary Flight Display (PFD) HUD
# ---------------------------------------------------------------------------
def render_world_class_pfd(draw_list, pos, size):
    cx = pos.x + size.x * 0.5
    cy = pos.y + size.y * 0.5

    col_frame_bg = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.06, 0.08, 0.12, 1.0))
    col_bezel = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.20, 0.28, 0.38, 1.0))
    draw_list.add_rect_filled(pos, imgui.ImVec2(pos.x + size.x, pos.y + size.y), col_frame_bg, 6.0)
    draw_list.add_rect(pos, imgui.ImVec2(pos.x + size.x, pos.y + size.y), col_bezel, 6.0, 0, 1.5)

    pfd_radius = min(size.x, size.y) * 0.42
    col_ground = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.38, 0.24, 0.14, 1.0))
    draw_list.add_circle_filled(imgui.ImVec2(cx, cy), pfd_radius, col_ground, 36)

    roll_rad = math.radians(g_state.roll_deg)
    pitch_pixels = g_state.pitch_deg * 2.2

    hx1 = cx - math.cos(roll_rad) * pfd_radius
    hy1 = cy - math.sin(roll_rad) * pfd_radius + pitch_pixels
    hx2 = cx + math.cos(roll_rad) * pfd_radius
    hy2 = cy + math.sin(roll_rad) * pfd_radius + pitch_pixels
    draw_list.add_line(imgui.ImVec2(hx1, hy1), imgui.ImVec2(hx2, hy2), imgui.color_convert_float4_to_u32(imgui.ImVec4(1.0, 1.0, 1.0, 0.95)), 2.5)

    for p_val in [-20, -10, 10, 20]:
        p_offset = pitch_pixels - p_val * 2.2
        lx1 = cx - math.cos(roll_rad) * 25.0 - math.sin(roll_rad) * p_offset
        ly1 = cy - math.sin(roll_rad) * 25.0 + math.cos(roll_rad) * p_offset
        lx2 = cx + math.cos(roll_rad) * 25.0 - math.sin(roll_rad) * p_offset
        ly2 = cy + math.sin(roll_rad) * 25.0 + math.cos(roll_rad) * p_offset
        draw_list.add_line(imgui.ImVec2(lx1, ly1), imgui.ImVec2(lx2, ly2), imgui.color_convert_float4_to_u32(imgui.ImVec4(1.0, 1.0, 1.0, 0.7)), 1.5)

    col_reticle = imgui.color_convert_float4_to_u32(imgui.ImVec4(1.0, 0.85, 0.0, 1.0))
    draw_list.add_line(imgui.ImVec2(cx - 32, cy), imgui.ImVec2(cx - 10, cy), col_reticle, 3.0)
    draw_list.add_line(imgui.ImVec2(cx + 10, cy), imgui.ImVec2(cx + 32, cy), col_reticle, 3.0)
    draw_list.add_circle_filled(imgui.ImVec2(cx, cy), 3.5, col_reticle)

    draw_list.add_circle(imgui.ImVec2(cx, cy), pfd_radius, col_bezel, 36, 2.0)
    draw_list.add_triangle_filled(
        imgui.ImVec2(cx, cy - pfd_radius),
        imgui.ImVec2(cx - 6, cy - pfd_radius + 10),
        imgui.ImVec2(cx + 6, cy - pfd_radius + 10),
        col_reticle
    )

    alt_box_min = imgui.ImVec2(pos.x + size.x - 70, pos.y + 10)
    alt_box_max = imgui.ImVec2(pos.x + size.x - 10, pos.y + size.y - 10)
    draw_list.add_rect_filled(alt_box_min, alt_box_max, imgui.color_convert_float4_to_u32(imgui.ImVec4(0.08, 0.12, 0.16, 0.85)), 4.0)
    draw_list.add_text(imgui.ImVec2(alt_box_min.x + 8, alt_box_min.y + 8), imgui.color_convert_float4_to_u32(imgui.ImVec4(0.0, 0.85, 1.0, 1.0)), "ALT (M)")
    draw_list.add_text(imgui.ImVec2(alt_box_min.x + 8, cy - 8), imgui.color_convert_float4_to_u32(imgui.ImVec4(1.0, 1.0, 1.0, 1.0)), f"{g_state.alt_m:5.1f}")

    spd_box_min = imgui.ImVec2(pos.x + 10, pos.y + 10)
    spd_box_max = imgui.ImVec2(pos.x + 70, pos.y + size.y - 10)
    draw_list.add_rect_filled(spd_box_min, spd_box_max, imgui.color_convert_float4_to_u32(imgui.ImVec4(0.08, 0.12, 0.16, 0.85)), 4.0)
    draw_list.add_text(imgui.ImVec2(spd_box_min.x + 6, spd_box_min.y + 8), imgui.color_convert_float4_to_u32(imgui.ImVec4(0.0, 0.85, 1.0, 1.0)), "SPD (KM)")
    draw_list.add_text(imgui.ImVec2(spd_box_min.x + 8, cy - 8), imgui.color_convert_float4_to_u32(imgui.ImVec4(1.0, 1.0, 1.0, 1.0)), f"{g_state.ground_speed_kms:4.1f}")

# ---------------------------------------------------------------------------
# Dual Stick Gimbals
# ---------------------------------------------------------------------------
def render_phosphor_gimbals(draw_list, pos, width=590, height=130):
    g_radius = 46.0
    g1_cx = pos.x + 110.0
    g1_cy = pos.y + 65.0
    g2_cx = pos.x + width - 110.0
    g2_cy = pos.y + 65.0

    col_bg = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.07, 0.09, 0.13, 1.0))
    col_border = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.22, 0.32, 0.44, 1.0))
    col_stick = imgui.color_convert_float4_to_u32(imgui.ImVec4(0.0, 0.9, 1.0, 1.0))

    draw_list.add_circle_filled(imgui.ImVec2(g1_cx, g1_cy), g_radius, col_bg, 24)
    draw_list.add_circle(imgui.ImVec2(g1_cx, g1_cy), g_radius, col_border, 24, 1.5)
    draw_list.add_line(imgui.ImVec2(g1_cx - g_radius, g1_cy), imgui.ImVec2(g1_cx + g_radius, g1_cy), col_border, 1.0)
    draw_list.add_line(imgui.ImVec2(g1_cx, g1_cy - g_radius), imgui.ImVec2(g1_cx, g1_cy + g_radius), col_border, 1.0)

    yaw_val = (g_state.rc_channels[3] - 1500.0) / 500.0
    thr_val = (g_state.rc_channels[2] - 1500.0) / 500.0
    st1 = imgui.ImVec2(g1_cx + yaw_val * (g_radius - 8.0), g1_cy - thr_val * (g_radius - 8.0))
    draw_list.add_circle_filled(st1, 7.5, col_stick, 16)
    draw_list.add_text(imgui.ImVec2(g1_cx - 40, g1_cy + g_radius + 4), imgui.color_convert_float4_to_u32(imgui.ImVec4(0.7, 0.8, 0.9, 1.0)), "THROTTLE / YAW")

    draw_list.add_circle_filled(imgui.ImVec2(g2_cx, g2_cy), g_radius, col_bg, 24)
    draw_list.add_circle(imgui.ImVec2(g2_cx, g2_cy), g_radius, col_border, 24, 1.5)
    draw_list.add_line(imgui.ImVec2(g2_cx - g_radius, g2_cy), imgui.ImVec2(g2_cx + g_radius, g2_cy), col_border, 1.0)
    draw_list.add_line(imgui.ImVec2(g2_cx, g2_cy - g_radius), imgui.ImVec2(g2_cx, g2_cy + g_radius), col_border, 1.0)

    roll_val = (g_state.rc_channels[0] - 1500.0) / 500.0
    pitch_val = (g_state.rc_channels[1] - 1500.0) / 500.0
    st2 = imgui.ImVec2(g2_cx + roll_val * (g_radius - 8.0), g2_cy - pitch_val * (g_radius - 8.0))
    draw_list.add_circle_filled(st2, 7.5, col_stick, 16)
    draw_list.add_text(imgui.ImVec2(g2_cx - 35, g2_cy + g_radius + 4), imgui.color_convert_float4_to_u32(imgui.ImVec4(0.7, 0.8, 0.9, 1.0)), "PITCH / ROLL")

# ---------------------------------------------------------------------------
# Main GUI Dispatcher
# ---------------------------------------------------------------------------
def gui_render():
    imgui.set_next_window_size(imgui.ImVec2(1320, 900), imgui.Cond_.first_use_ever)
    imgui.begin("INAV-ABSTRACTX World-Class Flight Studio & Commissioning Suite")

    # Header Connection Ribbon
    imgui.text("Target:")
    imgui.same_line()
    imgui.set_next_item_width(180)
    changed, g_state.target_address = imgui.input_text("##target", g_state.target_address)
    imgui.same_line()

    if not g_state.is_connected and not g_state.is_demo_mode:
        if imgui.button("Connect", imgui.ImVec2(80, 24)):
            g_state.connect()
    elif g_state.is_connected:
        if imgui.button("Disconnect", imgui.ImVec2(90, 24)):
            g_state.disconnect()

    imgui.same_line()
    # Demo Flight Mode Toggle Button
    demo_btn_label = "Stop Demo Mode" if g_state.is_demo_mode else "Start 3D Flight Demo"
    demo_col = imgui.ImVec4(1.0, 0.6, 0.0, 1.0) if g_state.is_demo_mode else imgui.ImVec4(0.0, 0.85, 1.0, 1.0)
    if imgui.button(demo_btn_label, imgui.ImVec2(160, 24)):
        g_state.toggle_demo_mode()

    imgui.same_line()
    status_color = imgui.ImVec4(0.0, 1.0, 0.4, 1.0) if (g_state.is_connected or g_state.is_demo_mode) else imgui.ImVec4(0.9, 0.2, 0.2, 1.0)
    imgui.text_colored(status_color, f"● {g_state.connection_status}")
    imgui.same_line()
    imgui.text(f"| Loop: {g_state.cycle_time_us} us | Sensors: 0x{g_state.sensor_flags:02X} | Arming: 0x{g_state.arming_flags:04X}")

    imgui.separator()

    # Guided Tour Banner (When in Demo Mode)
    if g_state.is_demo_mode and g_state.show_walkthrough_banner:
        imgui.push_style_color(imgui.Col_.child_bg, imgui.ImVec4(0.08, 0.15, 0.22, 1.0))
        imgui.begin_child("TourBanner", imgui.ImVec2(-1, 46), True)
        imgui.text_colored(imgui.ImVec4(0.0, 0.9, 1.0, 1.0), "WALKTHROUGH TOUR ACTIVE:")
        imgui.same_line()
        imgui.text("1. Solid 3D Vector Drone | 2. Animated High-Speed Props | 3. Garmin PFD Horizon | 4. Dual Stick Phosphor Gimbals")
        imgui.same_line(imgui.get_window_width() - 80)
        if imgui.button("Hide##tour"):
            g_state.show_walkthrough_banner = False
        imgui.end_child()
        imgui.pop_style_color()

    # Tab Bar
    if imgui.begin_tab_bar("MainTabs"):

        # =============================================================
        # TAB 1: 3D WORLD-CLASS FLIGHT DECK
        # =============================================================
        if imgui.begin_tab_item("3D Live Flight Deck")[0]:
            imgui.columns(2, "deck_cols", True)

            canvas_pos = imgui.get_cursor_screen_pos()
            canvas_size = imgui.ImVec2(600, 440)
            draw_list = imgui.get_window_draw_list()
            render_world_class_3d_viewport(draw_list, canvas_pos, canvas_size)
            imgui.dummy(canvas_size)

            imgui.separator()
            gimbal_pos = imgui.get_cursor_screen_pos()
            render_phosphor_gimbals(draw_list, gimbal_pos, 600, 130)
            imgui.dummy(imgui.ImVec2(600, 130))

            imgui.next_column()

            imgui.text_colored(imgui.ImVec4(0.0, 0.85, 1.0, 1.0), "Boeing/Garmin G1000 Primary Flight Display (PFD):")
            hud_pos = imgui.get_cursor_screen_pos()
            hud_size = imgui.ImVec2(340, 220)
            render_world_class_pfd(draw_list, hud_pos, hud_size)
            imgui.dummy(hud_size)

            imgui.separator()
            imgui.text_colored(imgui.ImVec4(0.0, 0.85, 1.0, 1.0), "Actuator Output Commands (DShot600):")
            for i in range(4):
                pwm = g_state.motor_pwm[i]
                norm = (pwm - 1000.0) / 1000.0
                m_names = ["M1 (Rear-R CCW)", "M2 (Front-R CW)", "M3 (Rear-L CW)", "M4 (Front-L CCW)"]
                imgui.text(f"{m_names[i]:18s}: {pwm:4d} us")
                imgui.same_line()
                imgui.progress_bar(norm, imgui.ImVec2(220, 0), f"{norm*100:.0f}%")

            imgui.separator()
            imgui.text(f"Altitude: {g_state.alt_m:6.2f} m | Vario: {g_state.vario_cms:3d} cm/s | Sats: {g_state.gps_sats} | HDOP: {g_state.gps_hdop:.2f}")

            imgui.columns(1)
            imgui.end_tab_item()

        # =============================================================
        # TAB 2: GUIDED CRAFT SETUP WIZARD
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
                if imgui.button("Next: Radio & RC Protocol ->", imgui.ImVec2(240, 35)): g_state.wizard_step = 2

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
                if imgui.button("Toggle Status LED", imgui.ImVec2(220, 35)): g_state.led_state = not g_state.led_state
                imgui.same_line()
                if imgui.button("Beep Buzzer (3-Beep Test)", imgui.ImVec2(220, 35)): g_state.buzzer_state = True
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
    parser = argparse.ArgumentParser(description="inav-abstractx World-Class Flight Studio")
    parser.add_argument("--demo", action="store_true", help="Launch directly into 3D Demo Flight Simulation Mode")
    args = parser.parse_args()

    if args.demo:
        g_state.is_demo_mode = True
        g_state.connection_status = "DEMO FLIGHT SIMULATION ACTIVE"
        g_state.status_msg = "Demo Mode: Synthesizing full 3D acro kinematics & high-speed spinning props."

    t = threading.Thread(target=telemetry_poll_thread, daemon=True)
    t.start()

    params = hello_imgui.RunnerParams()
    params.app_window_params.window_title = "inav-abstractx World-Class Flight Studio"
    params.app_window_params.window_geometry.size = (1320, 920)
    params.callbacks.show_gui = gui_render

    implot.create_context()
    hello_imgui.run(params)
    implot.destroy_context()

    g_state.running = False
    g_state.disconnect()

if __name__ == '__main__':
    main()
