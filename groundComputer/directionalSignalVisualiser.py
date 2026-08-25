import math
import sys
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
import serial
import serial.tools.list_ports

# ==========================================
#  CONFIGURATIONS
# ==========================================
SERIAL_PORT = "COM7"  # Change to your port (e.g., '/dev/ttyACM0' on Linux)
BAUD_RATE = 115200
MAX_POINTS = 200  # Number of historical points shown on graph


class SignalVisualizer(QtWidgets.QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("MRCC LoRa Directional Signal Tracker")
        self.resize(1000, 600)

        # Serial Setup
        self.ser = None
        self.init_serial()

        # Data Buffers
        self.time_buffer = []
        self.rssi_buffer = []
        self.quality_buffer = []
        self.packet_count = 0

        # --- UI LAYOUT ---
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QHBoxLayout(central_widget)

        # Left Panel: Real-Time Graphs
        graph_layout = QtWidgets.QVBoxLayout()

        self.win = pg.GraphicsLayoutWidget()
        graph_layout.addWidget(self.win)

        # Plot 1: RSSI (dBm)
        self.p1 = self.win.addPlot(title="Signal Strength (RSSI dBm)")
        self.p1.setLabel("left", "dBm")
        self.p1.setYRange(-125, -40)
        self.p1.showGrid(x=True, y=True)
        self.curve_rssi = self.p1.plot(pen=pg.mkPen("r", width=2))

        self.win.nextRow()

        # Plot 2: Smoothed Signal Quality (%)
        self.p2 = self.win.addPlot(title="Directional Signal Quality (%)")
        self.p2.setLabel("left", "Quality", units="%")
        self.p2.setLabel("bottom", "Packets Received")
        self.p2.setYRange(0, 100)
        self.p2.showGrid(x=True, y=True)
        self.curve_quality = self.p2.plot(pen=pg.mkPen("g", width=2))

        main_layout.addLayout(graph_layout, stretch=3)

        # Right Panel: Large Numeric Readout
        right_panel = QtWidgets.QVBoxLayout()

        self.lbl_title = QtWidgets.QLabel("SIGNAL QUALITY")
        self.lbl_title.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_title.setStyleSheet("font-size: 18px; font-weight: bold;")

        self.lbl_quality = QtWidgets.QLabel("0%")
        self.lbl_quality.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_quality.setStyleSheet(
            "font-size: 72px; font-weight: bold; color: #00FF00;"
        )

        self.lbl_metrics = QtWidgets.QLabel(
            "Raw RSSI: -- dBm\nSmoothed: -- dBm\nSNR: -- dB"
        )
        self.lbl_metrics.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_metrics.setStyleSheet("font-size: 16px; line-height: 1.4;")

        right_panel.addWidget(self.lbl_title)
        right_panel.addWidget(self.lbl_quality)
        right_panel.addWidget(self.lbl_metrics)
        right_panel.addStretch()

        main_layout.addLayout(right_panel, stretch=1)

        # Qt Update Timer (60 Hz GUI Refresh)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_loop)
        self.timer.start(16)

    def init_serial(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
            print(f"[ONLINE] Connected to {SERIAL_PORT}")
        except Exception as e:
            print(f"[WARNING] Serial connection error: {e}")

    def update_loop(self):
        if not self.ser or not self.ser.in_waiting:
            return

        try:
            line = (
                self.ser.readline().decode("utf-8", errors="ignore").strip()
            )

            # Expecting format: $SIGNAL,PacketID,RawRSSI,SmoothedRSSI,SNR,Quality%*
            if line.startswith("$SIGNAL,") and line.endswith("*"):
                clean = line[8:-1]
                fields = clean.split(",")

                packet_id = int(fields[0])
                raw_rssi = int(fields[1])
                smoothed_rssi = float(fields[2])
                snr = float(fields[3])
                quality = int(fields[4])

                # Append to Data Buffers
                self.packet_count += 1
                self.time_buffer.append(self.packet_count)
                self.rssi_buffer.append(smoothed_rssi)
                self.quality_buffer.append(quality)

                # Keep buffer size fixed
                if len(self.time_buffer) > MAX_POINTS:
                    self.time_buffer.pop(0)
                    self.rssi_buffer.pop(0)
                    self.quality_buffer.pop(0)

                # Update Plots
                self.curve_rssi.setData(self.time_buffer, self.rssi_buffer)
                self.curve_quality.setData(
                    self.time_buffer, self.quality_buffer
                )

                # Update Text Overlay
                self.lbl_quality.setText(f"{quality}%")
                self.lbl_metrics.setText(
                    f"Raw RSSI: {raw_rssi} dBm\n"
                    f"Smoothed: {smoothed_rssi:.1f} dBm\n"
                    f"SNR: {snr:.1f} dB"
                )

                # Color-code Large Quality Readout
                if quality > 70:
                    color = "#00FF00"  # Green (Strong signal direction)
                elif quality > 35:
                    color = "#FFA500"  # Orange (Medium signal direction)
                else:
                    color = "#FF0000"  # Red (Weak/Noise direction)

                self.lbl_quality.setStyleSheet(
                    f"font-size: 72px; font-weight: bold; color: {color};"
                )

        except Exception as parse_error:
            pass


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    vis = SignalVisualizer()
    vis.show()
    sys.exit(app.exec())