import sys
import os
import pandas as pd
import numpy as np

# Force interactive GUI backend BEFORE importing pyplot
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt


def analyze_rocket_telemetry(file_path):
    print("=== AVIONICS EXTENDED TELEMETRY ANALYSIS ===")
    
    # 1. Safely Read and Clean Raw CSV Data
    cleaned_rows = []
    try:
        with open(file_path, 'r') as f:
            for line in f:
                cleaned = line.strip().lstrip('$').rstrip('*')
                if cleaned:
                    cleaned_rows.append(cleaned.split(','))
    except FileNotFoundError:
        print(f"\n[ERROR] Could not find file: '{file_path}'")
        return False
    except Exception as e:
        print(f"\n[ERROR] Failed reading file '{file_path}': {e}")
        return False

    if not cleaned_rows:
        print("\n[ERROR] Telemetry file is empty!")
        return False

    headers = [
        "HEADER", "PACKET_ID", "TIME_MS", "PRESS_HPA", "TEMP_C", 
        "AX", "AY", "AZ", "GX", "GY", "GZ", 
        "GPS_FIX", "GPS_LAT", "GPS_LON", "GPS_ALT", "GPS_SATS"
    ]
    
    # 2. Build DataFrame and Coerce Numeric Types
    try:
        df = pd.DataFrame(cleaned_rows, columns=headers)
        num_cols = [
            "PACKET_ID", "TIME_MS", "PRESS_HPA", "TEMP_C", 
            "AX", "AY", "AZ", "GX", "GY", "GZ", 
            "GPS_FIX", "GPS_LAT", "GPS_LON", "GPS_ALT", "GPS_SATS"
        ]
        for col in num_cols:
            df[col] = pd.to_numeric(df[col], errors='coerce')
            
        df.dropna(subset=["TIME_MS"], inplace=True)
        if df.empty:
            print("\n[ERROR] No valid numerical rows found after parsing!")
            return False
    except Exception as e:
        print(f"\n[ERROR] Data parsing/formatting failed: {e}")
        return False

    # 3. Basic Dynamic Time Metrics
    df['TIME_SEC'] = (df['TIME_MS'] - df['TIME_MS'].iloc[0]) / 1000.0
    dt = np.gradient(df['TIME_SEC'])
    dt[dt == 0] = 0.001  # Prevent division by zero
    
    # Sampling frequency (Hz)
    df['FREQ_HZ'] = 1.0 / dt
    avg_freq = df['FREQ_HZ'].mean()

    # 4. Barometric Altitude & Dynamic Derivatives
    P0 = df['PRESS_HPA'].iloc[0] if df['PRESS_HPA'].iloc[0] > 0 else 1013.25
    df['BARO_ALT_M'] = 44330.0 * (1.0 - (df['PRESS_HPA'] / P0) ** 0.1903)
    df['BARO_ALT_REL'] = df['BARO_ALT_M'] - df['BARO_ALT_M'].iloc[0]
    
    # Vertical Velocity & Vertical Acceleration
    df['VERT_VELOCITY_MS'] = np.gradient(df['BARO_ALT_REL'], df['TIME_SEC'])
    df['VERT_ACCEL_MS2'] = np.gradient(df['VERT_VELOCITY_MS'], df['TIME_SEC'])

    # 5. Kinematics & Attitude Estimates
    # Accelerometer Magnitude (G) and m/s^2
    df['TOTAL_ACCEL_G'] = np.sqrt(df['AX']**2 + df['AY']**2 + df['AZ']**2)
    df['TOTAL_ACCEL_MS2'] = df['TOTAL_ACCEL_G'] * 9.80665
    
    # Angular Velocity Magnitude (deg/s)
    df['TOTAL_GYRO_DEGS'] = np.sqrt(df['GX']**2 + df['GY']**2 + df['GZ']**2)
    
    # Estimated Pitch Tilt Angle relative to vertical Z (degrees)
    df['TILT_ANGLE_DEG'] = np.degrees(np.arctan2(np.sqrt(df['AX']**2 + df['AY']**2), np.abs(df['AZ'])))

    # 6. Flight Phase & Event Analysis
    apogee_idx = df['BARO_ALT_REL'].idxmax()
    apogee_alt = df['BARO_ALT_REL'].max()
    apogee_time = df.loc[apogee_idx, 'TIME_SEC']
    
    ascent_df = df.iloc[:apogee_idx + 1]
    descent_df = df.iloc[apogee_idx:]

    max_ascent_vel = ascent_df['VERT_VELOCITY_MS'].max()
    max_descent_vel = abs(descent_df['VERT_VELOCITY_MS'].min()) if not descent_df.empty else 0.0
    avg_descent_vel = abs(descent_df['VERT_VELOCITY_MS'].mean()) if not descent_df.empty else 0.0
    descent_duration = df['TIME_SEC'].iloc[-1] - apogee_time

    # Environmental Lapse Rate (°C per 100 meters)
    alt_change = apogee_alt - df['BARO_ALT_REL'].iloc[0]
    temp_change = df.loc[apogee_idx, 'TEMP_C'] - df['TEMP_C'].iloc[0]
    lapse_rate = (temp_change / (alt_change / 100.0)) if alt_change > 10 else 0.0

    # 7. GPS Ground Distance (Haversine Formula)
    gps_fixes = df[df['GPS_FIX'] > 0].copy()
    total_ground_dist_m = 0.0
    if len(gps_fixes) > 1:
        lat = np.radians(gps_fixes['GPS_LAT'])
        lon = np.radians(gps_fixes['GPS_LON'])
        dlat = lat.diff()
        dlon = lon.diff()
        a = np.sin(dlat/2)**2 + np.cos(lat.shift(1)) * np.cos(lat) * np.sin(dlon/2)**2
        c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
        total_ground_dist_m = np.nansum(6371000.0 * c)

    # 8. Print Extended Statistics
    total_packets = len(df)
    packet_loss_est = (df['PACKET_ID'].max() - df['PACKET_ID'].min() + 1) - total_packets

    print("\n[AVIONICS & TIMING]")
    print(f"• Total Log Duration : {df['TIME_SEC'].max():.2f} s")
    print(f"• Total Packets Logged: {total_packets} (Est. Lost: {max(0, int(packet_loss_est))})")
    print(f"• Average Sampling   : {avg_freq:.1f} Hz")

    print("\n[KINEMATICS & ALTITUDE METRICS]")
    print(f"• Apogee Altitude    : {apogee_alt:.2f} m (at t = {apogee_time:.2f} s)")
    print(f"• Max Ascent Velocity: {max_ascent_vel:.2f} m/s")
    print(f"• Max Descent Rate   : {max_descent_vel:.2f} m/s (Avg: {avg_descent_vel:.2f} m/s)")
    print(f"• Descent Duration   : {descent_duration:.2f} s")
    print(f"• Max Vertical Accel : {df['VERT_ACCEL_MS2'].max():.2f} m/s²")
    print(f"• Peak Total G-Force : {df['TOTAL_ACCEL_G'].max():.2f} G (at t = {df.loc[df['TOTAL_ACCEL_G'].idxmax(), 'TIME_SEC']:.2f} s)")
    print(f"• Max Rotation Speed : {df['TOTAL_GYRO_DEGS'].max():.2f} deg/s")
    print(f"• Max Tilt Deviation : {df['TILT_ANGLE_DEG'].max():.1f}° from vertical")

    print("\n[ENVIRONMENTAL DATA]")
    print(f"• Temp Range         : {df['TEMP_C'].min():.1f} °C to {df['TEMP_C'].max():.1f} °C")
    print(f"• Temp Lapse Rate    : {lapse_rate:.2f} °C / 100m")
    print(f"• Pressure Range     : {df['PRESS_HPA'].min():.1f} hPa to {df['PRESS_HPA'].max():.1f} hPa")

    print("\n[GPS / GROUND TRACKING]")
    if len(gps_fixes) > 0:
        print(f"• GPS Fix Rate       : {(len(gps_fixes)/total_packets)*100:.1f}%")
        print(f"• Est. Ground Drift  : {total_ground_dist_m:.1f} m")
        print(f"• Landing Coordinates: Lat {gps_fixes['GPS_LAT'].iloc[-1]:.6f}, Lon {gps_fixes['GPS_LON'].iloc[-1]:.6f}")
        print(f"• Peak Satellite Count: {int(df['GPS_SATS'].max())}")
    else:
        print("• GPS Status         : No valid GPS fixes recorded.")

    # 9. Extended 6-Panel Dashboard Plot
    plt.style.use('seaborn-v0_8-darkgrid' if 'seaborn-v0_8-darkgrid' in plt.style.available else 'default')
    fig, axs = plt.subplots(3, 2, figsize=(15, 11))
    fig.canvas.manager.set_window_title('Extended Rocket Avionics Dashboard')
    fig.suptitle('Extended Flight Telemetry Analysis', fontsize=16, fontweight='bold')

    # Subplot 1: Altitude & Velocity
    axs[0, 0].plot(df['TIME_SEC'], df['BARO_ALT_REL'], color='tab:blue', label='Altitude (m)')
    axs[0, 0].scatter([apogee_time], [apogee_alt], color='red', zorder=5, label=f'Apogee ({apogee_alt:.1f}m)')
    axs[0, 0].set_title('Altitude Profile')
    axs[0, 0].set_ylabel('Altitude (m)')
    axs[0, 0].legend()

    # Subplot 2: Vertical Velocity
    axs[0, 1].plot(df['TIME_SEC'], df['VERT_VELOCITY_MS'], color='tab:green', label='Vert Speed (m/s)')
    axs[0, 1].axhline(0, color='gray', linestyle='--')
    axs[0, 1].set_title('Vertical Velocity Profile')
    axs[0, 1].set_ylabel('Velocity (m/s)')
    axs[0, 1].legend()

    # Subplot 3: Accelerations
    axs[1, 0].plot(df['TIME_SEC'], df['TOTAL_ACCEL_G'], color='tab:orange', label='Total Load (G)')
    axs[1, 0].set_title('Acceleration & Loads')
    axs[1, 0].set_ylabel('Acceleration (G)')
    axs[1, 0].legend()

    # Subplot 4: Body Tilt Angle
    axs[1, 1].plot(df['TIME_SEC'], df['TILT_ANGLE_DEG'], color='tab:olive', label='Off-Vertical Tilt (°)')
    axs[1, 1].set_title('Estimated Airframe Tilt')
    axs[1, 1].set_ylabel('Tilt Angle (deg)')
    axs[1, 1].legend()

    # Subplot 5: Gyro Rates
    axs[2, 0].plot(df['TIME_SEC'], df['GX'], label='GX (Roll)', alpha=0.7)
    axs[2, 0].plot(df['TIME_SEC'], df['GY'], label='GY (Pitch)', alpha=0.7)
    axs[2, 0].plot(df['TIME_SEC'], df['GZ'], label='GZ (Yaw)', alpha=0.7)
    axs[2, 0].set_title('Angular Velocities')
    axs[2, 0].set_xlabel('Flight Time (s)')
    axs[2, 0].set_ylabel('Rate (deg/s)')
    axs[2, 0].legend()

    # Subplot 6: Environment
    ax6 = axs[2, 1]
    ax6_temp = ax6.twinx()
    ax6.plot(df['TIME_SEC'], df['PRESS_HPA'], color='tab:purple', label='Pressure (hPa)')
    ax6_temp.plot(df['TIME_SEC'], df['TEMP_C'], color='tab:red', linestyle=':', label='Temp (°C)')
    ax6.set_title('Environmental Sensor Profile')
    ax6.set_xlabel('Flight Time (s)')
    ax6.set_ylabel('Pressure (hPa)', color='tab:purple')
    ax6_temp.set_ylabel('Temp (°C)', color='tab:red')

    plt.tight_layout()
    plt.savefig('flight_telemetry_extended.png', dpi=300)
    print("\n[OUTPUT] Dashboard plot saved to 'flight_telemetry_extended.png'")

    plt.show(block=True)
    return True


if __name__ == "__main__":
    # Select filename from terminal arg or default file
    TARGET_FILE = sys.argv[1] if len(sys.argv) > 1 else "flight_log.csv"

    print(f"[DEBUG] Current Directory : {os.getcwd()}")
    print(f"[DEBUG] Processing File   : {TARGET_FILE}")

    if not os.path.exists(TARGET_FILE):
        print(f"\n[ERROR] File '{TARGET_FILE}' not found!")
        print("Usage: python3 flightDataAnalyser.py <your_file_name.csv>")
    else:
        try:
            analyze_rocket_telemetry(TARGET_FILE)
        except Exception as fatal_err:
            print(f"\n[CRITICAL ERROR] {fatal_err}")

    print("\n" + "=" * 45)
    input("Press ENTER to exit program...")
    sys.exit()