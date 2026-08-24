import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# 1. Load CSV file
df = pd.read_csv("flight_log.csv")

# 2. Clean trailing '*' character from the last column (GPS_SATS)
if "GPS_SATS" in df.columns:
    df["GPS_SATS"] = df["GPS_SATS"].astype(str).str.rstrip("*").astype(float)

# 3. Normalize time axis to seconds starting from t=0
df["Time_s"] = (df["TIME_MS"] - df["TIME_MS"].iloc[0]) / 1000.0

# 4. Calculate Barometric Altitude (m) from Pressure (hPa)
SEA_LEVEL_P = 1013.25
df["Baro_Alt_m"] = 44330.0 * (1.0 - (df["PRESS_HPA"] / SEA_LEVEL_P) ** (1.0 / 5.255))

# 5. Calculate Total Acceleration Vector Magnitude
df["Accel_Mag_G"] = np.sqrt(df["AX"] ** 2 + df["AY"] ** 2 + df["AZ"] ** 2)

# 6. Plot Telemetry Subplots
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

# Panel 1: Barometric & GPS Altitude
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

# Panel 2: Acceleration Profile
ax2.plot(
    df["Time_s"], df["Accel_Mag_G"], color="tab:red", label="Total Accel (g)"
)
ax2.set_ylabel("Accel [g]")
ax2.grid(True)
ax2.legend()

# Panel 3: Gyroscope Angular Velocity
ax3.plot(df["Time_s"], df["GX"], label="GX")
ax3.plot(df["Time_s"], df["GY"], label="GY")
ax3.plot(df["Time_s"], df["GZ"], label="GZ")
ax3.set_ylabel("Gyro [°/s]")
ax3.set_xlabel("Flight Duration [s]")
ax3.grid(True)
ax3.legend()

plt.suptitle("MRCC Rocket Flight Telemetry Profile", fontsize=14)
plt.tight_layout()
plt.show()