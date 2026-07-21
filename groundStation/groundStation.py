import serial
import math
from vpython import *

# 1. Initialize the 3D Canvas and the CanSat Object
scene = canvas(title="CanSat Live Flight Telemetry Tracker", width=800, height=600, center=vector(0,0,0), background=color.black)
scene.up = vector(0, 0, 1)  # Set Z-axis as "Up" for aerospace standard

# Draw a 3D cylinder representing your fiberglass CanSat body
cansat = cylinder(pos=vector(0, 0, -1), axis=vector(0, 0, 2), radius=0.5, color=color.orange)

# 2. Configure the USB Serial Port (Change 'COM3' to match your laptop's port)
# In Linux/Mac, it will look like '/dev/ttyUSB0' or '/dev/cu.usbmodem...'
ser = serial.Serial('COM3', 115200, timeout=1)

print("Ground Station Active. Listening for telemetry...")

# 3. Main Live Render Loop
while True:
    rate(60) # Limits the loop to a smooth 60 Frames Per Second (FPS)
    
    if ser.in_waiting > 0:
        try:
            # Read line from LoRa USB receiver, decode bytes to string, strip whitespace
            packet = ser.readline().decode('utf-8').strip()
            
            # Example expected packet format: $CANSAT,packet_id,ax,ay,az,gx,gy,gz*
            if packet.startswith("$CANSAT") and packet.endswith("*"):
                # Strip the markers and split by commas
                clean_data = packet.replace("$CANSAT,", "").replace("*", "")
                data_fields = clean_data.split(",")
                
                # Parse out the raw Float sensor values
                packet_id = int(data_fields[0])
                ax = float(data_fields[1])
                ay = float(data_fields[2])
                az = float(data_fields[3])
                gx = float(data_fields[4]) # Gyroscope X (deg/sec or rad/sec)
                gy = float(data_fields[5])
                gz = float(data_fields[6])
                
                # 4. Convert Raw IMU Data into 3D Vector Rotations
                # For pure raw orientation, calculate Pitch and Roll from Accelerometer G-forces:
                pitch = math.atan2(ax, math.sqrt(ay**2 + az**2))
                roll = math.atan2(ay, math.sqrt(ax**2 + az**2))
                
                # (Optional: Add your Gyro integration tracking here for Yaw rotation)
                
                # Calculate the new alignment vector for the 3D cylinder axis
                new_axis_x = math.sin(roll) * math.cos(pitch)
                new_axis_y = math.sin(pitch)
                new_axis_z = math.cos(roll) * math.cos(pitch)
                
                # 5. Update the 3D Object on screen dynamically
                cansat.axis = vector(new_axis_x, new_axis_y, new_axis_z)
                
                print(f"Packet: {packet_id} | Pitch: {math.degrees(pitch):.2f}° | Roll: {math.degrees(roll):.2f}°")
                
        except Exception as e:
            # Catches garbled packets or missing indices due to radio static without crashing the loop
            print(f"Telemetry Packet Error: {e}")