import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def analyze_rocket_telemetry(file_path):
    print("=== AVIONICS TELEMETRY ANALYSIS ===")
    
    # 1. Load and Clean Raw CSV Data
    # Strips leading '$' and trailing '*' checksum delimiters if present
    cleaned_rows = []
    with open(file_path, 'r') as f:
        for line in f:
            line = line.strip().lstrip('$').rstrip('*')
            if line:
                cleaned_rows.append(line.split(','))
                
    headers = ["HEADER", "PACKET_ID", "TIME_MS", "PRESS_HPA", "TEMP_C", 
               "AX", "AY", "AZ", "GX", "GY", "GZ", 
               "GPS_FIX", "GPS_LAT", "GPS_LON", "GPS_ALT", "GPS_SATS"]
    
    df = pd.DataFrame(cleaned_rows, columns=headers)
    
    # Convert numeric columns
    num_cols = ["PACKET_ID", "TIME_MS", "PRESS_HPA", "TEMP_C", 
                "AX", "AY", "AZ", "GX", "GY", "GZ", 
                "GPS_FIX", "GPS_LAT", "GPS_LON", "GPS_ALT", "GPS_SATS"]
    for col in num_cols:
        df[col] = pd.to_numeric(df[col], errors='coerce')
        
    df.dropna(subset=["TIME_MS"], inplace=True)
    
    # 2. Derived Metrics
    df['TIME_SEC'] = (df['TIME_MS'] - df['TIME_MS'].iloc[0]) / 1000.0
    
    # Barometric Altitude approximation from pressure (hPa)
    P0 = 1013.25  # Standard sea level pressure
    df['BARO_ALT_M'] = 44330.0 * (1.0 - (df['PRESS_HPA'] / P0) ** 0.1903)
    df['BARO_ALT_REL'] = df['BARO_ALT_M'] - df['BARO_ALT_M'].iloc[0]
    
    # Resultant Acceleration vector magnitude (G-Force)
    df['TOTAL_ACCEL_G'] = np.sqrt(df['AX']**2 + df['AY']**2 + df['AZ']**2)
    
    # Total Gyro angular velocity (deg/s)
    df['TOTAL_GYRO_DEGS'] = np.sqrt(df['GX']**2 + df['GY']**2 + df['GZ']**2)
    
    # Calculate Vertical Descent/Ascent Rate (m/s)
    df['VERT_VELOCITY_MS'] = np.gradient(df['BARO_ALT_REL'], df['TIME_SEC'])

    # 3. Print Flight Statistics Summary
    duration = df['TIME_SEC'].max() - df['TIME_SEC'].min()
    apogee_idx = df['BARO_ALT_REL'].idxmax()
    apogee_alt = df['BARO_ALT_REL'].max()
    apogee_time = df.loc[apogee_idx, 'TIME_SEC']
    max_g = df['TOTAL_ACCEL_G'].max()
    max_g_time = df.loc[df['TOTAL_ACCEL_G'].idxmax(), 'TIME_SEC']
    max_speed = df['VERT_VELOCITY_MS'].max()
    
    total_packets = len(df)
    packet_loss_est = (df['PACKET_ID'].max() - df['PACKET_ID'].min() + 1) - total_packets
    
    print(f"\n[FLIGHT METRICS]")
    print(f"• Mission Duration   : {duration:.2f} seconds")
    print(f"• Total Packets Logged: {total_packets} (Est. Lost Packets: {max(0, packet_loss_est)})")
    print(f"• Maximum Altitude   : {apogee_alt:.2f} m (Detected at t = {apogee_time:.2f} s)")
    print(f"• Peak Acceleration  : {max_g:.2f} G (Detected at t = {max_g_time:.2f} s)")
    print(f"• Peak Ascent Rate   : {max_speed:.2f} m/s")
    print(f"• Temp Range         : {df['TEMP_C'].min():.1f} °C to {df['TEMP_C'].max():.1f} °C")

    print(f"\n[GPS / GROUND SEGMENT LOGGING]")
    gps_fixes = df[df['GPS_FIX'] > 0]
    if len(gps_fixes) > 0:
        print(f"• GPS Fix Rate       : {(len(gps_fixes)/total_packets)*100:.1f}%")
        print(f"• Last Recorded Pos  : Lat {gps_fixes['GPS_LAT'].iloc[-1]:.6f}, Lon {gps_fixes['GPS_LON'].iloc[-1]:.6f}")
        print(f"• Max Satellites     : {df['GPS_SATS'].max()} satellites")
    else:
        print("• GPS Status         : No valid GPS fixes recorded in telemetry.")

    # 4. Generate Telemetry Dashboard Plots
    plt.style.use('seaborn-v0_8-darkgrid' if 'seaborn-v0_8-darkgrid' in plt.style.available else 'default')
    fig, axs = plt.subplots(2, 2, figsize=(14, 9))
    fig.suptitle('Avionics Flight Telemetry Dashboard', fontsize=16, fontweight='bold')

    # Subplot 1: Altitude & Velocity
    axs[0, 0].plot(df['TIME_SEC'], df['BARO_ALT_REL'], color='tab:blue', label='Barometric Alt (m)')
    axs[0, 0].scatter([apogee_time], [apogee_alt], color='red', zorder=5, label=f'Apogee ({apogee_alt:.1f}m)')
    axs[0, 0].set_title('Altitude Profile')
    axs[0, 0].set_xlabel('Flight Time (s)')
    axs[0, 0].set_ylabel('Altitude (m)')
    axs[0, 0].legend()

    # Subplot 2: Accelerometer (G-Force)
    axs[0, 1].plot(df['TIME_SEC'], df['TOTAL_ACCEL_G'], color='tab:orange', label='Total Accel (G)')
    axs[0, 1].plot(df['TIME_SEC'], df['AZ'], color='gray', alpha=0.5, linestyle='--', label='Z-Axis (Vertical)')
    axs[0, 1].set_title('Accelerometry & Loads')
    axs[0, 1].set_xlabel('Flight Time (s)')
    axs[0, 1].set_ylabel('Force (G)')
    axs[0, 1].legend()

    # Subplot 3: Gyroscope (Rotation Rates)
    axs[1, 0].plot(df['TIME_SEC'], df['GX'], label='GX (Roll)', alpha=0.7)
    axs[1, 0].plot(df['TIME_SEC'], df['GY'], label='GY (Pitch)', alpha=0.7)
    axs[1, 0].plot(df['TIME_SEC'], df['GZ'], label='GZ (Yaw)', alpha=0.7)
    axs[1, 0].set_title('Angular Velocity')
    axs[1, 0].set_xlabel('Flight Time (s)')
    axs[1, 0].set_ylabel('Rotation Rate (deg/s)')
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
    plt.show()

# Run script on target CSV file
# analyze_rocket_telemetry("telemetry_log.csv")