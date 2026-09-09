import serial
import math
from vpython import *
import gps_map_server

# Start GPS server for map visualisation
gps_map_server.start_server(port=8000)

# ==========================================
# 1. 3D VISUALIZATION CANVAS & SCENE SETUP
# ==========================================
# CRITICAL: align="left" prevents graphs from squishing the 3D scene canvas
scene = canvas(
    title="MRCC Rocket Telemetry Ground Station",
    width=600,
    height=550,
    align="left",
    center=vector(0, 0, 0),
    background=color.gray(0.08)
)

# Set aerospace reference: Z-axis is "UP"
scene.up = vector(0, 0, 1)

# Rocket 3D Model (Explicitly bound to scene canvas)
ROCKET_LENGTH = 2.5
rocket = cylinder(
    canvas=scene,
    pos=vector(0, 0, -ROCKET_LENGTH / 2),
    axis=vector(0, 0, ROCKET_LENGTH),
    radius=0.4,
    color=color.orange
)

# Launchpad Plane
grid_plane = box(
    canvas=scene,
    pos=vector(0, 0, -ROCKET_LENGTH / 2 - 0.05),
    size=vector(8, 8, 0.02),
    color=color.gray(0.2)
)

# Telemetry Overlay
telemetry_label = label(
    canvas=scene,
    pos=vector(-3, 0, 3.5),
    text="Awaiting Rocket Telemetry Link...",
    xoffset=10, yoffset=10,
    space=20, height=12,
    border=4, font='sans'
)

# ==========================================
# 2. REAL-TIME GRAPH WINDOWS SETUP
# ==========================================
MAX_GRAPH_POINTS = 300  # Rolling buffer (~15-30s window)

# Graph 1: Altitude
graph_alt = graph(
    title="<b>Barometric Altitude (m)</b>",
    xtitle="Flight Time (s)", ytitle="Altitude (m)",
    width=520, height=180, align="right", background=color.gray(0.12)
)
curve_alt = gcurve(color=color.cyan, width=2, graph=graph_alt)

# Graph 2: Acceleration Load Components (G)
graph_accel = graph(
    title="<b>Linear Accelerations (G)</b>",
    xtitle="Flight Time (s)", ytitle="G-Force",
    width=520, height=180, align="right", background=color.gray(0.12)
)
curve_ax = gcurve(color=color.red, label="AX", width=1.5, graph=graph_accel)
curve_ay = gcurve(color=color.green, label="AY", width=1.5, graph=graph_accel)
curve_az = gcurve(color=color.blue, label="AZ", width=1.5, graph=graph_accel)

# Graph 3: Attitudinal Angles (Pitch / Roll / Yaw)
graph_attitude = graph(
    title="<b>Attitude / Orientation (deg)</b>",
    xtitle="Flight Time (s)", ytitle="Degrees (°)",
    width=520, height=180, align="right", background=color.gray(0.12)
)
curve_pitch = gcurve(color=color.orange, label="Pitch", width=1.5, graph=graph_attitude)
curve_roll  = gcurve(color=color.magenta, label="Roll", width=1.5, graph=graph_attitude)
curve_yaw   = gcurve(color=color.yellow, label="Yaw", width=1.5, graph=graph_attitude)

# ==========================================
# 3. SERIAL PORT CONFIGURATION
# ==========================================
SERIAL_PORT = 'COM7'
BAUD_RATE = 115200

try:
    # Set short timeout to prevent blocking VPython rendering loop
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.02)
    print(f"[ONLINE] Ground Station linked to {SERIAL_PORT} at {BAUD_RATE} baud.")
except Exception as e:
    print(f"[WARNING] Serial port connection failed: {e}")
    ser = None

# --- STATE & FILTER VARIABLES ---
SEA_LEVEL_PRESSURE = 1013.25

pitch_deg = 0.0
roll_deg  = 0.0
yaw_deg   = 0.0

last_timestamp_ms = None
last_packet_id = None
total_received = 0
total_dropped = 0

# ==========================================
# 4. LIVE RENDER, GRAPHING & PARSING LOOP
# ==========================================
while True:
    rate(60)  # Keeps WebGL render loop alive at 60 FPS

    if ser and ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()

            if line.startswith("$CANSAT,") and line.endswith("*"):
                clean_payload = line[8:-1]
                fields = clean_payload.split(",")

                if len(fields) < 17:
                    continue

                # Parse and convert values
                packet_id        = int(fields[0])
                timestamp_ms     = int(fields[1])
                pressure         = float(fields[2])
                temperature      = float(fields[3])
                baro_alt         = float(fields[4])
                ax               = float(fields[5])
                ay               = float(fields[6])
                az               = float(fields[7])
                gx               = float(fields[8])
                gy               = float(fields[9])
                gz               = float(fields[10])
                gps_fix          = int(fields[11])
                gps_lat          = float(fields[12])
                gps_lon          = float(fields[13])
                gps_alt          = float(fields[14])
                gps_sats         = int(fields[15])
                apogee_triggered = int(fields[16])

                time_sec = timestamp_ms / 1000.0

                # Update GPS Web Map
                gps_map_server.update_gps(gps_lat, gps_lon, gps_alt, gps_fix, gps_sats, timestamp_ms)

                # Packet Loss Calculation
                total_received += 1
                if last_packet_id is not None:
                    gap = packet_id - (last_packet_id + 1)
                    if gap > 0:
                        total_dropped += gap
                last_packet_id = packet_id

                total_expected = total_received + total_dropped
                loss_percentage = (total_dropped / total_expected * 100.0) if total_expected > 0 else 0.0

                # Complementary Filter (Attitude Estimation with NaN safety)
                if last_timestamp_ms is not None:
                    dt = (timestamp_ms - last_timestamp_ms) / 1000.0
                    
                    if 0.0 < dt < 2.0:
                        denom = math.sqrt(ay**2 + az**2)
                        accel_pitch = math.degrees(math.atan2(ax, denom if denom != 0 else 0.001))
                        accel_roll  = math.degrees(math.atan2(ay, math.sqrt(ax**2 + az**2)))

                        total_accel = math.sqrt(ax**2 + ay**2 + az**2)
                        alpha = 0.98 if (0.85 < total_accel < 1.15) else 1.0

                        pitch_deg = alpha * (pitch_deg + gy * dt) + (1.0 - alpha) * accel_pitch
                        roll_deg  = alpha * (roll_deg  + gx * dt) + (1.0 - alpha) * accel_roll
                        yaw_deg  += gz * dt

                last_timestamp_ms = timestamp_ms

                # Sanity guard against NaN orientation propagation
                if not (math.isnan(pitch_deg) or math.isnan(roll_deg) or math.isnan(yaw_deg)):
                    pitch_rad = math.radians(pitch_deg)
                    roll_rad  = math.radians(roll_deg)
                    yaw_rad   = math.radians(yaw_deg)

                    # Update 3D Model Orientation
                    dir_x = math.sin(roll_rad) * math.cos(pitch_rad)
                    dir_y = -math.sin(pitch_rad)
                    dir_z = math.cos(roll_rad) * math.cos(pitch_rad)

                    orient_vec = vector(dir_x, dir_y, dir_z).norm()
                    orient_vec = rotate(orient_vec, angle=yaw_rad, axis=vector(0, 0, 1))

                    rocket.axis = orient_vec * ROCKET_LENGTH
                    rocket.pos = -0.5 * rocket.axis

                # --- REAL-TIME GRAPH PLOTTING ---
                curve_alt.plot(time_sec, baro_alt)
                curve_ax.plot(time_sec, ax)
                curve_ay.plot(time_sec, ay)
                curve_az.plot(time_sec, az)
                curve_pitch.plot(time_sec, pitch_deg)
                curve_roll.plot(time_sec, roll_deg)
                curve_yaw.plot(time_sec, yaw_deg)

                # --- ROLLING BUFFER TRIMMING ---
                if len(curve_alt.data) > MAX_GRAPH_POINTS:
                    curve_alt.data   = curve_alt.data[-MAX_GRAPH_POINTS:]
                    curve_ax.data    = curve_ax.data[-MAX_GRAPH_POINTS:]
                    curve_ay.data    = curve_ay.data[-MAX_GRAPH_POINTS:]
                    curve_az.data    = curve_az.data[-MAX_GRAPH_POINTS:]
                    curve_pitch.data = curve_pitch.data[-MAX_GRAPH_POINTS:]
                    curve_roll.data  = curve_roll.data[-MAX_GRAPH_POINTS:]
                    curve_yaw.data   = curve_yaw.data[-MAX_GRAPH_POINTS:]

                # Update HUD Overlay
                apogee_status = "ARMED / STABLE" if not apogee_triggered else "DEPLOYED!"
                telemetry_label.text = (
                    f"--- ROCKET TELEMETRY LINK ACTIVE ---\n"
                    f"Packet ID: {packet_id} | Time: {time_sec:.2f} s\n"
                    f"Rx Count: {total_received} | Dropped: {total_dropped} | Loss: {loss_percentage:.1f}%\n"
                    f"Flight Status: Apogee State -> {apogee_status}\n"
                    f"Baro Altitude: {baro_alt:.1f} m | Pressure: {pressure:.2f} hPa | Temp: {temperature:.1f} °C\n"
                    f"GPS Fix: {gps_fix} | Sats: {gps_sats} | Lat/Lon: [{gps_lat:.6f}, {gps_lon:.6f}]\n"
                    f"Pitch: {pitch_deg:.1f}° | Roll: {roll_deg:.1f}° | Yaw: {yaw_deg:.1f}°\n"
                    f"Accel [G]: [{ax:.2f}, {ay:.2f}, {az:.2f}] | Gyro [°/s]: [{gx:.1f}, {gy:.1f}, {gz:.1f}]"
                )
        except Exception as parse_error:
            print(f"[FRAME WARNING] Parse error: {parse_error}")