import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# 1. Read raw CSV file
df = pd.read_csv("flight_log.csv", on_bad_lines="skip", low_memory=False)

# 2. Filter out repeated header lines caused by ESP32 reboots
df = df[df.iloc[:, 0] != "HEADER"].copy()

# 3. Handle column shift if '$CANSAT' is present in column 0
if df.columns[0] == "HEADER":
    # Rename columns to match actual data positions
    df.columns = [
        "PREFIX",
        "PACKET_ID",
        "TIME_MS",
        "PRESS_HPA",
        "TEMP_C",
        "AX",
        "AY",
        "AZ",
        "GX",
        "GY",
        "GZ",
        "GPS_FIX",
        "GPS_LAT",
        "GPS_LON",
        "GPS_ALT",
        "GPS_SATS",
    ]

# 4. Clean trailing '*' delimiter from the last column
df["GPS_SATS"] = df["GPS_SATS"].astype(str).str.rstrip("*")

# 5. Convert numeric columns to numeric types (forcing invalid strings to NaN)
numeric_cols = [
    "PACKET_ID",
    "TIME_MS",
    "PRESS_HPA",
    "TEMP_C",
    "AX",
    "AY",
    "AZ",
    "GX",
    "GY",
    "GZ",
    "GPS_FIX",
    "GPS_LAT",
    "GPS_LON",
    "GPS_ALT",
    "GPS_SATS",
]
for col in numeric_cols:
    df[col] = pd.to_numeric(df[col], errors="coerce")

# Drop any row that has invalid/corrupted numeric data
df.dropna(subset=numeric_cols, inplace=True)

# 6. Normalize time to seconds starting at 0
df["Time_s"] = (df["TIME_MS"] - df["TIME_MS"].iloc[0]) / 1000.0

# 7. Calculate Barometric Altitude and Total Acceleration Magnitude
SEA_LEVEL_P = 1013.25
df["Baro_Alt_m"] = 44330.0 * (
    1.0 - (df["PRESS_HPA"] / SEA_LEVEL_P) ** (1.0 / 5.255)
)
df["Accel_Mag_G"] = np.sqrt(df["AX"] ** 2 + df["AY"] ** 2 + df["AZ"] ** 2)

# 8. Render Graphs
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

ax1.plot(
    df["Time_s"], df["Baro_Alt_m"], color="tab:blue", label="Barometric Alt (m)"
)
ax1.plot(
    df["Time_s"],
    df["GPS_ALT"],
    color="tab:cyan",
    linestyle="--",
    label="GPS Alt (m)",
)
ax1.set_ylabel("Altitude [m]")
ax1.grid(True)
ax1.legend()

ax2.plot(
    df["Time_s"], df["Accel_Mag_G"], color="tab:red", label="Total Accel (g)"
)
ax2.set_ylabel("Accel [g]")
ax2.grid(True)
ax2.legend()

ax3.plot(df["Time_s"], df["GX"], label="GX")
ax3.plot(df["Time_s"], df["GY"], label="GY")
ax3.plot(df["Time_s"], df["GZ"], label="GZ")
ax3.set_ylabel("Gyro [°/s]")
ax3.set_xlabel("Flight Time [s]")
ax3.grid(True)
ax3.legend()

plt.suptitle("MRCC CanSat Flight Telemetry Profile", fontsize=14)
plt.tight_layout()
plt.show()