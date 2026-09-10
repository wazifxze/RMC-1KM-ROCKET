import os
import sys
import csv
import time
import math
import serial

# ==========================================
# 1. CONFIGURATION
# ==========================================
SERIAL_PORT = 'COM7'
BAUD_RATE = 115200

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
# 3. SERIAL INITIALIZATION
# ==========================================
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.02)
    print(f"[ONLINE] Serial connected to {SERIAL_PORT} @ {BAUD_RATE} baud.")
except Exception as e:
    print(f"[ERROR] Serial connection failed: {e}")
    sys.exit(1)

# ==========================================
# 4. HIGH-PERFORMANCE DATA PROCESSING LOOP
# ==========================================
start_time_ms = None
apogee_alt = 0.0

print("[SYSTEM] High-speed head-less logging started. Press Ctrl+C to stop.\n")

try:
    while True:
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()

                if line:
                    # Print raw incoming data stream directly to console
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
                    vert_vel     = float(fields[5])
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

                    # Track Max Altitude
                    if baro_alt > apogee_alt:
                        apogee_alt = baro_alt

            except Exception as err:
                print(f"[PARSER ERROR] {err}")

        # Lightweight sleep to prevent CPU thread starvation
        time.sleep(0.001)

except KeyboardInterrupt:
    print("\n[STOP] User interrupted logging.")

finally:
    csv_file.close()
    if ser and ser.is_open:
        ser.close()
    print("[SHUTDOWN] Serial closed and CSV log saved safely.")