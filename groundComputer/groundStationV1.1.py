import serial
import math
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

# Rocket 3D Model
ROCKET_LENGTH = 2.5
rocket = cylinder(
    pos=vector(0, 0, -ROCKET_LENGTH / 2),
    axis=vector(0, 0, ROCKET_LENGTH),
    radius=0.4,
    color=color.orange
)

# Launchpad Plane
grid_plane = box(
    pos=vector(0, 0, -ROCKET_LENGTH / 2 - 0.05),
    size=vector(8, 8, 0.02),
    color=color.gray(0.2)
)

# Telemetry Overlay
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
SERIAL_PORT = 'COM3'
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"[ONLINE] Ground Station linked to {SERIAL_PORT} at {BAUD_RATE} baud.")
except Exception as e:
    print(f"[WARNING] Serial port connection failed: {e}")
    ser = None

# --- STATE & FILTER VARIABLES ---
SEA_LEVEL_PRESSURE = 1013.25

# Integrated Orientation Angles (Degrees)
pitch_deg = 0.0
roll_deg  = 0.0
yaw_deg   = 0.0

# Tracking Metrics
last_timestamp_ms = None
last_packet_id = None
total_received = 0
total_dropped = 0

# ==========================================
# 3. LIVE RENDER & TELEMETRY PARSING LOOP
# ==========================================
while True:
    rate(60)

    if ser and ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()

            if line.startswith("$CANSAT,") and line.endswith("*"):
                clean_payload = line[8:-1]
                fields = clean_payload.split(",")

                packet_id    = int(fields[0])
                timestamp_ms = int(fields[1])
                pressure     = float(fields[2])
                temperature  = float(fields[3])
                ax           = float(fields[4])
                ay           = float(fields[5])
                az           = float(fields[6])
                gx           = float(fields[7])
                gy           = float(fields[8])
                gz           = float(fields[9])
                rssi         = int(fields[10])   if len(fields) > 10 else -1
                snr          = float(fields[11]) if len(fields) > 11 else 0.0

                # --- 1. PACKET LOSS TRACKER ---
                total_received += 1
                if last_packet_id is not None:
                    gap = packet_id - (last_packet_id + 1)
                    if gap > 0:
                        total_dropped += gap
                last_packet_id = packet_id

                total_expected = total_received + total_dropped
                loss_percentage = (total_dropped / total_expected * 100.0) if total_expected > 0 else 0.0

                # --- 2. DYNAMIC COMPLEMENTARY SENSOR FUSION ---
                if last_timestamp_ms is not None:
                    dt = (timestamp_ms - last_timestamp_ms) / 1000.0
                    
                    if 0.0 < dt < 2.0: # Valid time step
                        # Instantaneous static tilt from Accelerometer
                        denom = math.sqrt(ay**2 + az**2)
                        accel_pitch = math.degrees(math.atan2(ax, denom if denom != 0 else 0.001))
                        accel_roll  = math.degrees(math.atan2(ay, math.sqrt(ax**2 + az**2)))

                        # Calculate total linear acceleration magnitude (|a|)
                        total_accel = math.sqrt(ax**2 + ay**2 + az**2)

                        # DYNAMIC ALPHA:
                        # If |a| is near 1.0g (coasting/pad), rely 98% on gyro, 2% on accel.
                        # If |a| > 1.3g (MOTOR BURN), ignore accel entirely (Alpha = 1.0) to prevent distortion!
                        if 0.85 < total_accel < 1.15:
                            alpha = 0.98
                        else:
                            alpha = 1.0  # Pure Gyro integration during thrust/freefall

                        # Apply Complementary Filter Equation
                        pitch_deg = alpha * (pitch_deg + gy * dt) + (1.0 - alpha) * accel_pitch
                        roll_deg  = alpha * (roll_deg  + gx * dt) + (1.0 - alpha) * accel_roll
                        yaw_deg  += gz * dt

                last_timestamp_ms = timestamp_ms

                # Radians conversion for 3D engine
                pitch_rad = math.radians(pitch_deg)
                roll_rad  = math.radians(roll_deg)
                yaw_rad   = math.radians(yaw_deg)

                # --- 3. BAROMETRIC ALTITUDE ---
                altitude = 44330.0 * (1.0 - (pressure / SEA_LEVEL_PRESSURE) ** (1.0 / 5.255))

                # --- 4. UPDATE 3D MODEL ORIENTATION ---
                dir_x = math.sin(roll_rad) * math.cos(pitch_rad)
                dir_y = -math.sin(pitch_rad)
                dir_z = math.cos(roll_rad) * math.cos(pitch_rad)

                orient_vec = vector(dir_x, dir_y, dir_z).norm()
                orient_vec = rotate(orient_vec, angle=yaw_rad, axis=vector(0, 0, 1))

                rocket.axis = orient_vec * ROCKET_LENGTH
                rocket.pos = -0.5 * rocket.axis

                # --- 5. HUD OVERLAY ---
                telemetry_label.text = (
                    f"--- ROCKET TELEMETRY LINK ACTIVE ---\n"
                    f"Packet ID: {packet_id} | Time: {timestamp_ms / 1000.0:.2f} s\n"
                    f"Rx Count: {total_received} | Dropped: {total_dropped} | Loss: {loss_percentage:.1f}%\n"
                    f"LoRa Link Quality: RSSI {rssi} dBm | SNR {snr:.1f} dB\n"
                    f"Altitude: {altitude:.1f} m | Pressure: {pressure:.2f} hPa | Temp: {temperature:.1f} °C\n"
                    f"Pitch: {pitch_deg:.1f}° | Roll: {roll_deg:.1f}° | Yaw: {yaw_deg:.1f}°\n"
                    f"Accel [G]: [{ax:.2f}, {ay:.2f}, {az:.2f}] | Gyro [°/s]: [{gx:.1f}, {gy:.1f}, {gz:.1f}]"
                )

        except Exception as parse_error:
            print(f"[FRAME WARNING] Parse error: {parse_error}")