import serial
import time

COM_PORT = 'COM5'  # Change to your ground station receiver port
BAUD_RATE = 115200

def trigger_parachute():
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Allow serial connection to settle
        
        print("[ACTION] Sending Manual Parachute Deploy Command...")
        ser.write(b"TRIGGER_SERVO\n")
        ser.flush()
        
        time.sleep(1)
        print("[SUCCESS] Command sent to ground station!")
        ser.close()
    except Exception as e:
        print(f"[ERROR] Failed to send command: {e}")

if __name__ == "__main__":
    trigger_parachute()