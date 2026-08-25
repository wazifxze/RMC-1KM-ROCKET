import math
import sys
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
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
        self.setWindowTitle(
            "MRCC LoRa Directional Signal Tracker - XIAO ESP32-S3"
        )
        self.resize(1100, 650)

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

        # Right Panel: Large Numeric Readout & Raw Serial Terminal
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
        self.lbl_metrics.setStyleSheet("font-size: 15px; line-height: 1.4;")

        # Terminal Label & Raw Log Window
        self.lbl_raw_title = QtWidgets.QLabel("RAW USB DATA STREAM:")
        self.lbl_raw_title.setStyleSheet(
            "font-size: 12px; font-weight: bold; margin-top: 15px;"
        )

        self.txt_raw_log = QtWidgets.QTextEdit()
        self.txt_raw_log.setReadOnly(True)
        self.txt_raw_log.setFont(QtGui.QFont("Courier", 9))
        self.txt_raw_log.setStyleSheet(
            "background-color: #1E1E1E; color: #00FF00; border: 1px solid #444;"
        )

        right_panel.addWidget(self.lbl_title)
        right_panel.addWidget(self.lbl_quality)
        right_panel.addWidget(self.lbl_metrics)
        right_panel.addWidget(self.lbl_raw_title)
        right_panel.addWidget(self.txt_raw_log)

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

            if not line:
                return

            # 1. Print raw string to terminal box & auto-scroll to bottom
            self.txt_raw_log.append(line)
            if self.txt_raw_log.document().blockCount() > 100:
                # Keep terminal window buffer light
                cursor = self.txt_raw_log.textCursor()
                cursor.movePosition(QtGui.QTextCursor.Start)
                cursor.select(QtGui.QTextCursor.BlockUnderCursor)
                cursor.removeSelectedText()
                cursor.deleteChar()

            sb = self.txt_raw_log.verticalScrollBar()
            sb.setValue(sb.maximum())

            # 2. Parse telemetry sentence: $SIGNAL,PacketID,RawRSSI,SmoothedRSSI,SNR,Quality%*
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

                # Keep fixed graph history length
                if len(self.time_buffer) > MAX_POINTS:
                    self.time_buffer.pop(0)
                    self.rssi_buffer.pop(0)
                    self.quality_buffer.pop(0)

                # Update Graphs
                self.curve_rssi.setData(self.time_buffer, self.rssi_buffer)
                self.curve_quality.setData(
                    self.time_buffer, self.quality_buffer
                )

                # Update Text Labels
                self.lbl_quality.setText(f"{quality}%")
                self.lbl_metrics.setText(
                    f"Raw RSSI: {raw_rssi} dBm\n"
                    f"Smoothed: {smoothed_rssi:.1f} dBm\n"
                    f"SNR: {snr:.1f} dB"
                )

                # Dynamic Color Coding
                if quality > 70:
                    color = "#00FF00"  # Strong signal
                elif quality > 35:
                    color = "#FFA500"  # Medium signal
                else:
                    color = "#FF0000"  # Weak signal

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