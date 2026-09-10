import os
import sys
import csv
import time
import math
import serial
import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # Interactive GUI backend
import matplotlib.pyplot as plt

# ==========================================
# 1. CONFIGURATION & OFFLINE MAP BOUNDS
# ==========================================
SERIAL_PORT = 'COM7'
BAUD_RATE = 115200

MAP_IMAGE_PATH = 'map.png'

MAP_BOUNDS = {
    'lon_min': 100.806931,  # Left coordinate (West)
    'lon_max': 100.874914,  # Right coordinate (East)
    'lat_min': 4.038686,    # Bottom coordinate (South)
    'lat_max': 4.096436     # Top coordinate (North)
}

MAX_PLOT_POINTS = 500  # Rolling window size for performance

# ==========================================
# 2. TELEMETRY LOG FILE SETUP
# ==========================================
LOG_FILENAME = f"flight_log_{int(time.time())}.csv"
csv_file = open(LOG_FILENAME, "a", newline="", encoding="utf-8")
csv_writer = csv.writer(csv_file)

CSV_HEADERS = [
    "PACKET_ID", "TIME_MS", "PRESS_HPA", "TEMP_C", "BARO_ALT", "VERT_VEL",
    "AX", "AY", "AZ", "GX", "GY", "GZ",
    "GPS_FIX", "GPS_LAT", "GPS_LON", "GPS_ALT", "GPS_SATS",
    "APOGEE_TRIGGERED", "RSSI", "SNR"
]
csv_writer.writerow(CSV_HEADERS)
csv_file.flush()
print(f"[LOGGING] Active recording -> '{LOG_FILENAME}'")

# ==========================================
# 3. REAL-TIME 6-PANEL UI SETUP
# ==========================================
plt.style.use('seaborn-v0_8-darkgrid' if 'seaborn-v0_8-darkgrid' in plt.style.available else 'default')
plt.ion()  # Enable interactive mode for real-time updating

fig, axs = plt.subplots(3, 2, figsize=(15, 11))
fig.canvas.manager.set_window_title('Live Rocket Telemetry Ground Station')
fig.suptitle('Real-Time Rocket Flight Telemetry & Map Tracking', fontsize=16, fontweight='bold')

# Panel 1: Altitude Profile
axs[0, 0].set_title('Altitude Profile')
axs[0, 0].set_ylabel('Altitude (m)')
line_alt, = axs[0, 0].plot([], [], color='tab:blue', label='Altitude (m)')
point_apogee, = axs[0, 0].plot([], [], 'ro', label='Apogee')
axs[0, 0].legend(loc='upper left')

# Panel 2: Vertical Velocity Profile
axs[0, 1].set_title('Vertical Velocity Profile')
axs[0, 1].set_ylabel('Velocity (m/s)')
line_vel, = axs[0, 1].plot([], [], color='tab:green', label='Vert Speed (m/s)')
axs[0, 1].axhline(0, color='gray', linestyle='--')
axs[0, 1].legend(loc='upper left')

# Panel 3: Accelerations & Loads
axs[1, 0].set_title('Acceleration & Loads')
axs[1, 0].set_ylabel('Acceleration (G)')
line_accel, = axs[1, 0].plot([], [], color='tab:orange', label='Total Load (G)')
axs[1, 0].legend(loc='upper left')

# Panel 4: Airframe Tilt
axs[1, 1].set_title('Estimated Airframe Tilt')
axs[1, 1].set_ylabel('Tilt Angle (deg)')
line_tilt, = axs[1, 1].plot([], [], color='tab:olive', label='Off-Vertical Tilt (°)')
axs[1, 1].legend(loc='upper left')

# Panel 5: Angular Velocities (Gyro)
axs[2, 0].set_title('Angular Velocities')
axs[2, 0].set_xlabel('Flight Time (s)')
axs[2, 0].set_ylabel('Rate (deg/s)')
line_gx, = axs[2, 0].plot([], [], label='GX (Roll)', alpha=0.7)
line_gy, = axs[2, 0].plot([], [], label='GY (Pitch)', alpha=0.7)
line_gz, = axs[2, 0].plot([], [], label='GZ (Yaw)', alpha=0.7)
axs[2, 0].legend(loc='upper left')

# Panel 6: Live Offline GPS Map
ax_map = axs[2, 1]
ax_map.set_title('Live Ground Track (Offline Map)')
ax_map.set_xlabel('Longitude (°)')
ax_map.set_ylabel('Latitude (°)')

if os.path.exists(MAP_IMAGE_PATH):
    map_img = plt.imread(MAP_IMAGE_PATH)
    ax_map.imshow(
        map_img,
        extent=[MAP_BOUNDS['lon_min'], MAP_BOUNDS['lon_max'], MAP_BOUNDS['lat_min'], MAP_BOUNDS['lat_max']],
        aspect='auto'
    )
    print(f"[MAP] Loaded offline map image: '{MAP_IMAGE_PATH}'")
else:
    ax_map.text(0.5, 0.5, f"Map image '{MAP_IMAGE_PATH}' not found!\nPlotting coordinates on grid.",
                ha='center', va='center', transform=ax_map.transAxes, color='red')
    print(f"[WARNING] Map file '{MAP_IMAGE_PATH}' missing. Falling back to simple GPS scatter.")

line_gps_track, = ax_map.plot([], [], 'y-', linewidth=2, label='Flight Path')
point_gps_current, = ax_map.plot([], [], 'r*', markersize=12, label='Rocket Position')
ax_map.legend(loc='upper right')

plt.tight_layout()

# ==========================================
# 4. DATA BUFFERS & SERIAL INITIALIZATION
# ==========================================
time_sec_list = []
baro_alt_list = []
vert_vel_list = []
total_accel_list = []
tilt_deg_list = []
gx_list, gy_list, gz_list = [], [], []
gps_lat_list, gps_lon_list = [], []

start_time_ms = None
apogee_alt = 0.0
apogee_time = 0.0

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.02)
    print(f"[ONLINE] Serial connected to {SERIAL_PORT} @ {BAUD_RATE} baud.")
except Exception as e:
    print(f"[WARNING] Serial connection error: {e}")
    ser = None

# ==========================================
# 5. LIVE PROCESSING & RENDERING LOOP
# ==========================================
try:
    while plt.fignum_exists(fig.number):
        if ser and ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()

                if line:
                    # Print raw incoming data stream
                    print(f"[RAW] {line}")

                if line.startswith("$CANSAT,") and line.endswith("*"):
                    clean_payload = line[8:-1]
                    fields = clean_payload.split(",")

                    if len(fields) < 18:
                        continue

                    packet_id    = int(fields[0])
                    timestamp_ms = int(fields[1])
                    pressure     = float(fields[2])
                    temperature  = float(fields[3])
                    baro_alt     = float(fields[4])
                    vert_vel     = float(fields[5])  # Fused velocity sent from flight computer
                    ax, ay, az   = float(fields[6]), float(fields[7]), float(fields[8])
                    gx, gy, gz   = float(fields[9]), float(fields[10]), float(fields[11])
                    gps_fix      = int(float(fields[12]))
                    gps_lat      = float(fields[13])
                    gps_lon      = float(fields[14])
                    gps_alt      = float(fields[15])
                    gps_sats     = int(float(fields[16]))
                    apogee_trig  = int(float(fields[17]))
                    rssi         = int(float(fields[18])) if len(fields) > 18 else -1
                    snr          = float(fields[19]) if len(fields) > 19 else 0.0

                    # --- CSV RECORDING ---
                    csv_writer.writerow([
                        packet_id, timestamp_ms, pressure, temperature, baro_alt, vert_vel,
                        ax, ay, az, gx, gy, gz,
                        gps_fix, gps_lat, gps_lon, gps_alt, gps_sats,
                        apogee_trig, rssi, snr
                    ])
                    csv_file.flush()

                    # --- TIME & KINEMATIC DERIVATIONS ---
                    if start_time_ms is None:
                        start_time_ms = timestamp_ms

                    t_sec = (timestamp_ms - start_time_ms) / 1000.0
                    total_accel = math.sqrt(ax**2 + ay**2 + az**2)
                    tilt_deg = math.degrees(math.atan2(math.sqrt(ax**2 + ay**2), abs(az)))

                    # Track Apogee
                    if baro_alt > apogee_alt:
                        apogee_alt = baro_alt
                        apogee_time = t_sec

                    # Append metrics
                    time_sec_list.append(t_sec)
                    baro_alt_list.append(baro_alt)
                    vert_vel_list.append(vert_vel)
                    total_accel_list.append(total_accel)
                    tilt_deg_list.append(tilt_deg)
                    gx_list.append(gx)
                    gy_list.append(gy)
                    gz_list.append(gz)

                    if gps_fix > 0 and gps_lat != 0.0:
                        gps_lat_list.append(gps_lat)
                        gps_lon_list.append(gps_lon)

                    # Maintain memory buffers
                    if len(time_sec_list) > MAX_PLOT_POINTS:
                        time_sec_list.pop(0)
                        baro_alt_list.pop(0)
                        vert_vel_list.pop(0)
                        total_accel_list.pop(0)
                        tilt_deg_list.pop(0)
                        gx_list.pop(0)
                        gy_list.pop(0)
                        gz_list.pop(0)

                    # --- REAL-TIME UI PLOT UPDATE ---
                    line_alt.set_data(time_sec_list, baro_alt_list)
                    point_apogee.set_data([apogee_time], [apogee_alt])
                    line_vel.set_data(time_sec_list, vert_vel_list)
                    line_accel.set_data(time_sec_list, total_accel_list)
                    line_tilt.set_data(time_sec_list, tilt_deg_list)

                    line_gx.set_data(time_sec_list, gx_list)
                    line_gy.set_data(time_sec_list, gy_list)
                    line_gz.set_data(time_sec_list, gz_list)

                    if len(gps_lat_list) > 0:
                        line_gps_track.set_data(gps_lon_list, gps_lat_list)
                        point_gps_current.set_data([gps_lon_list[-1]], [gps_lat_list[-1]])

                    # Autoscale X/Y axes dynamically
                    for row in axs:
                        for ax_item in row:
                            if ax_item != ax_map:
                                ax_item.relim()
                                ax_item.autoscale_view()

                    fig.canvas.draw()
                    fig.canvas.flush_events()

            except Exception as err:
                print(f"[PARSER ERROR] {err}")

        plt.pause(0.01)

finally:
    csv_file.close()
    if ser:
        ser.close()
    print("\n[SHUTDOWN] Resources safely released.")