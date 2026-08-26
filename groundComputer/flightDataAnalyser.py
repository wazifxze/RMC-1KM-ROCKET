import sys
import pandas as pd
import numpy as np

# 1. Force interactive GUI backend BEFORE importing pyplot
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt


def analyze_rocket_telemetry(file_path):
    print("=== AVIONICS TELEMETRY ANALYSIS ===")
    
    # 2. Safely Read and Clean Raw CSV Data
    cleaned_rows = []
    try:
        with open(file_path, 'r') as f:
            for line_num, line in enumerate(f, 1):
                # Strip spaces, newlines, leading '$', and trailing '*' checksum delimiters
                cleaned = line.strip().lstrip('$').rstrip('*')
                if cleaned:
                    cleaned_rows.append(cleaned.split(','))
    except FileNotFoundError:
        print(f"\n[ERROR] Could not find file: '{file_path}'")
        print("[TROUBLESHOOT] Ensure the CSV file is in the same folder as this script.")
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
    
    # 3. Build DataFrame and Coerce Numeric Types
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

    # 4. Calculate Derived Flight Metrics
    df['TIME_SEC'] = (df['TIME_MS'] - df['TIME_MS'].iloc[0]) / 1000.0
    
    # Barometric Altitude calculation relative to launch point
    P0 = 1013.25  # Standard sea-level pressure baseline (hPa)
    df['BARO_ALT_M'] = 44330.0 * (1.0 - (df['PRESS_HPA'] / P0) ** 0.1903)
    df['BARO_ALT_REL'] = df['BARO_ALT_M'] - df['BARO_ALT_M'].iloc[0]
    
    # Resultant Vectors
    df['TOTAL_ACCEL_G'] = np.sqrt(df['AX']**2 + df['AY']**2 + df['AZ']**2)
    df['TOTAL_GYRO_DEGS'] = np.sqrt(df['GX']**2 + df['GY']**2 + df['GZ']**2)
    df['VERT_VELOCITY_MS'] = np.gradient(df['BARO_ALT_REL'], df['TIME_SEC'])

    # 5. Output Summary Statistics
    duration = df['TIME_SEC'].max() - df['TIME_SEC'].min()
    apogee_idx = df['BARO_ALT_REL'].idxmax()
    apogee_alt = df['BARO_ALT_REL'].max()
    apogee_time = df.loc[apogee_idx, 'TIME_SEC']
    max_g = df['TOTAL_ACCEL_G'].max()
    max_g_time = df.loc[df['TOTAL_ACCEL_G'].idxmax(), 'TIME_SEC']
    max_speed = df['VERT_VELOCITY_MS'].max()
    
    total_packets = len(df)
    packet_loss_est = (df['PACKET_ID'].max() - df['PACKET_ID'].min() + 1) - total_packets
    
    print("\n[FLIGHT METRICS]")
    print(f"• Mission Duration   : {duration:.2f} s")
    print(f"• Packets Received   : {total_packets} (Est. Lost: {max(0, packet_loss_est)})")
    print(f"• Apogee Altitude    : {apogee_alt:.2f} m (at t = {apogee_time:.2f} s)")
    print(f"• Peak Acceleration  : {max_g:.2f} G (at t = {max_g_time:.2f} s)")
    print(f"• Max Vertical Speed : {max_speed:.2f} m/s")
    print(f"• Temperature Range  : {df['TEMP_C'].min():.1f} °C to {df['TEMP_C'].max():.1f} °C")

    print("\n[GPS / GROUND SEGMENT LOGGING]")
    gps_fixes = df[df['GPS_FIX'] > 0]
    if len(gps_fixes) > 0:
        print(f"• GPS Fix Rate       : {(len(gps_fixes)/total_packets)*100:.1f}%")
        print(f"• Last Position Log  : Lat {gps_fixes['GPS_LAT'].iloc[-1]:.6f}, Lon {gps_fixes['GPS_LON'].iloc[-1]:.6f}")
        print(f"• Max Satellites     : {int(df['GPS_SATS'].max())} satellites")
    else:
        print("• GPS Status         : No valid GPS fixes in flight log.")

    # 6. Render Telemetry Dashboard
    plt.style.use('seaborn-v0_8-darkgrid' if 'seaborn-v0_8-darkgrid' in plt.style.available else 'default')
    fig, axs = plt.subplots(2, 2, figsize=(14, 9))
    fig.canvas.manager.set_window_title('Avionics Flight Telemetry Dashboard')
    fig.suptitle('Flight Telemetry Dashboard', fontsize=16, fontweight='bold')

    # Subplot 1: Altitude & Velocity
    axs[0, 0].plot(df['TIME_SEC'], df['BARO_ALT_REL'], color='tab:blue', label='Baro Alt (m)')
    axs[0, 0].scatter([apogee_time], [apogee_alt], color='red', zorder=5, label=f'Apogee ({apogee_alt:.1f}m)')
    axs[0, 0].set_title('Altitude Profile')
    axs[0, 0].set_xlabel('Flight Time (s)')
    axs[0, 0].set_ylabel('Altitude (m)')
    axs[0, 0].legend()

    # Subplot 2: G-Force Acceleration
    axs[0, 1].plot(df['TIME_SEC'], df['TOTAL_ACCEL_G'], color='tab:orange', label='Total G-Force')
    axs[0, 1].plot(df['TIME_SEC'], df['AZ'], color='gray', alpha=0.5, linestyle='--', label='Z-Axis (Vertical)')
    axs[0, 1].set_title('Accelerometry & Loads')
    axs[0, 1].set_xlabel('Flight Time (s)')
    axs[0, 1].set_ylabel('Force (G)')
    axs[0, 1].legend()

    # Subplot 3: Gyroscope Rates
    axs[1, 0].plot(df['TIME_SEC'], df['GX'], label='GX (Roll)', alpha=0.7)
    axs[1, 0].plot(df['TIME_SEC'], df['GY'], label='GY (Pitch)', alpha=0.7)
    axs[1, 0].plot(df['TIME_SEC'], df['GZ'], label='GZ (Yaw)', alpha=0.7)
    axs[1, 0].set_title('Angular Velocity')
    axs[1, 0].set_xlabel('Flight Time (s)')
    axs[1, 0].set_ylabel('Rate (deg/s)')
    axs[1, 0].legend()

    # Subplot 4: Pressure & Temperature
    ax4 = axs[1, 1]
    ax4_temp = ax4.twinx()
    ax4.plot(df['TIME_SEC'], df['PRESS_HPA'], color='tab:purple', label='Pressure (hPa)')
    ax4_temp.plot(df['TIME_SEC'], df['TEMP_C'], color='tab:red', linestyle=':', label='Temp (°C)')
    ax4.set_title('Environmental Sensors')
    ax4.set_xlabel('Flight Time (s)')
    ax4.set_ylabel('Pressure (hPa)', color='tab:purple')
    ax4_temp.set_ylabel('Temperature (°C)', color='tab:red')

    plt.tight_layout()
    plt.savefig('flight_telemetry_analysis.png', dpi=300)
    print("\n[OUTPUT] Dashboard saved to 'flight_telemetry_analysis.png'")

    # Display plot and block main thread until closed
    plt.show(block=True)
    return True


if __name__ == "__main__":
    # Name of your target CSV/TXT file
    TARGET_FILE = "telemetry_log.csv"
    
    try:
        success = analyze_rocket_telemetry(TARGET_FILE)
    except Exception as fatal_err:
        print(f"\n[CRITICAL ERROR] Unhandled exception occurred: {fatal_err}")
    finally:
        # Prevents the terminal window from closing automatically on exit
        print("\n" + "=" * 40)
        input("Press ENTER to exit program...")
        sys.exit()