import serial
import math
import time
from vpython import *

# ==========================================
# 1. 3D VISUALIZATION CANVAS & SCENE SETUP
# ==========================================
scene = canvas(
    title="MRCC Rocket Telemetry Ground Station",
    width=1000,
    height=700,
    center=vector(0, 0, 0),
    background=color.gray(0.08)
)

# Set aerospace reference: Z-axis is "UP"
scene.up = vector(0, 0, 1)

# Rocket / CanSat 3D Cylinder Model
ROCKET_LENGTH = 2.5
rocket = cylinder(
    pos=vector(0, 0, -ROCKET_LENGTH / 2),
    axis=vector(0, 0, ROCKET_LENGTH),
    radius=0.4,
    color=color.orange
)

# Reference Launchpad Ground Plane
grid_plane = box(
    pos=vector(0, 0, -ROCKET_LENGTH / 2 - 0.05),
    size=vector(8, 8, 0.02),
    color=color.gray(0.2)
)

# On-screen Telemetry Overlay
telemetry_label = label(
    pos=vector(-3, 0, 3.5),
    text="Awaiting Rocket Telemetry Link...",
    xoffset=10, yoffset=10,
    space=20, height=13,
    border=4, font='sans'
)

# ==========================================
# 2. SERIAL PORT CONFIGURATION
# ==========================================
# Replace 'COM3' with your LoRa USB receiver port (e.g., '/dev/ttyUSB0' on Linux)
SERIAL_PORT = 'COM3'
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"[ONLINE] Ground Station linked to {SERIAL_PORT} at {BAUD_RATE} baud.")
except Exception as e:
    print(f"[WARNING] Serial port connection failed: {e}")
    print("Running in Standalone Visualization Mode...")
    ser = None

# Atmospheric reference constants
SEA_LEVEL_PRESSURE = 1013.25  # Standard baseline pressure (hPa)
yaw_angle = 0.0               # Yaw rotation tracker (Degrees)

# --- PACKET TIMING & LOSS METRICS ---
last_timestamp_ms = None
last_packet_id = None
total_received = 0
total_dropped = 0

# ==========================================
# 3. LIVE RENDER & TELEMETRY PARSING LOOP
# ==========================================
while True:
    rate(60)  # Maintain a smooth 60 Frames Per Second render loop
    
    current_time_sec = time.time()
    dt = current_time_sec - last_time_sec
    last_time_sec = current_time_sec

    if ser and ser.in_waiting > 0:
        try:
            # Read line from serial buffer, strip whitespace/newlines
            line = ser.readline().decode('utf-8', errors='ignore').strip()

            # Validate against packet prefix ($CANSAT) and tail (*)
            if line.startswith("$CANSAT,") and line.endswith("*"):
                
                # Strip prefix and suffix, then split CSV fields
                clean_payload = line[8:-1]
                fields = clean_payload.split(",")

                # Unpack 10 numerical telemetry fields sent by firmware
                packet_id   = int(fields[0])
                timestamp_ms= int(fields[1])
                pressure    = float(fields[2])
                temperature = float(fields[3])
                ax          = float(fields[4])
                ay          = float(fields[5])
                az          = float(fields[6])
                gx          = float(fields[7])
                gy          = float(fields[8])
                gz          = float(fields[9])

                # --- PACKET LOSS CALCULATION ---
                total_received += 1
                if last_packet_id is not None:
                    gap = packet_id - (last_packet_id + 1)
                    if gap > 0:
                        total_dropped += gap  # Gap detected in sequence
                last_packet_id = packet_id

                total_expected = total_received + total_dropped
                loss_percentage = (total_dropped / total_expected * 100.0) if total_expected > 0 else 0.0

                # ---  EXACT YAW INTEGRATION USING ONBOARD DELTA T ---
                if last_timestamp_ms is not None:
                    # Calculate true elapsed time on the rocket's hardware timer
                    dt = (timestamp_ms - last_timestamp_ms) / 1000.0
                    
                    # Sanity check: protect against negative/stale timestamps or long disconnections
                    if 0.0 < dt < 2.0:
                        yaw_angle += gz * dt
                
                last_timestamp_ms = timestamp_ms
                yaw_rad = math.radians(yaw_angle)

                # --- 1. ORIENTATION & MOTION MATH ---
                # Calculate Pitch and Roll from Accelerometer G-Forces
                denom = math.sqrt(ay**2 + az**2)
                pitch = math.atan2(ax, denom if denom != 0 else 0.001)
                roll  = math.atan2(ay, math.sqrt(ax**2 + az**2)

                # --- 2. CALCULATE ALTITUDE ---
                # Standard barometric formula to estimate altitude in meters
                altitude = 44330.0 * (1.0 - (pressure / SEA_LEVEL_PRESSURE) ** (1.0 / 5.255))

                # --- 3. UPDATE 3D MODEL AXIS & POSITION ---
                # Combine Pitch, Roll, and Yaw into directional unit vector
                dir_x = math.sin(roll) * math.cos(pitch)
                dir_y = -math.sin(pitch)
                dir_z = math.cos(roll) * math.cos(pitch)

                orient_vec = vector(dir_x, dir_y, dir_z).norm()
                
                # Rotate rocket around its vertical axis using Yaw angle
                orient_vec = rotate(orient_vec, angle=yaw_rad, axis=vector(0, 0, 1))

                # Update 3D VPython model geometry
                rocket.axis = orient_vec * ROCKET_LENGTH
                rocket.pos = -0.5 * rocket.axis

                # --- 4. UPDATE GROUND STATION TELEMETRY OVERLAY ---
                telemetry_label.text = (
                    f"--- ROCKET TELEMETRY LINK ACTIVE ---\n"
                    f"Packet ID: {packet_id} | Time: {timestamp_ms / 1000.0:.2f} s\n"
                    f"Rx Count: {total_received} | Dropped: {total_dropped} | Loss: {loss_percentage:.1f}%\n"
                    f"Altitude: {altitude:.1f} m | Pressure: {pressure:.2f} hPa | Temp: {temperature:.1f} °C\n"
                    f"Pitch: {math.degrees(pitch):.1f}° | Roll: {math.degrees(roll):.1f}° | Yaw: {yaw_angle:.1f}°\n"
                    f"Accel [G]: [{ax:.2f}, {ay:.2f}, {az:.2f}]\n"
                    f"Gyro [°/s]: [{gx:.1f}, {gy:.1f}, {gz:.1f}]"
                )

        except Exception as parse_error:
            # Drop corrupted frames without crashing the rendering process
            print(f"[FRAME WARNING] Parse error on packet: {parse_error}")