# Project Horizon: Dual-Core ESP32-S3 CanSat Flight Computer

A high-reliability, dual-core telemetry and recovery flight system designed for competitive CanSat operations, powered by the Seeed Studio XIAO ESP32-S3.

## 🚀 System Overview

Project Horizon is an advanced, deterministic flight computer built to handle high-frequency sensor acquisition, real-time apogee detection, thread-safe local data logging, and wireless LoRa telemetry downlinks simultaneously. By utilizing FreeRTOS task distribution across the ESP32-S3's dual cores, the system completely eliminates timing bottlenecks and bus contention during critical flight phases.

## 🛠️ Hardware & Pinout Mapping

| Component | Model / Chip | Protocol | GPIO / Pin Mapping |
| :--- | :--- | :--- | :--- |
| **Microcontroller** | Seeed Studio XIAO ESP32-S3 | Core System | Dual-Core Tensilica Xtensa LX7 |
| **Barometer** | BME280 | I2C | SDA: D4 (GPIO 5), SCL: D5 (GPIO 6) |
| **IMU** | BMI160 (6-DoF) | I2C | Shared I2C Bus (`0x69`) |
| **GPS Module** | NEO-6M | UART (Rx-only) | RX: D7 (GPIO 44), TX: Disabled (`-1`) |
| **Radio Transceiver** | Ra-02 LoRa (433MHz) | SPI | CS: D3 (GPIO 4), RST: D0 (GPIO 1), DIO0: D1 (GPIO 2) |
| **Storage** | MicroSD Card Module | SPI | CS: D2 (GPIO 3) |
| **Actuator** | Servo Motor (Parachute) | PWM | GPIO 43 (D6) |
| **Shared SPI Bus** | SCK: D8 (GPIO 7), MISO: D9 (GPIO 8), MOSI: D10 (GPIO 9) |

---

## ⚡ Software Architecture & FreeRTOS Design

The firmware breaks away from traditional single-loop blocking code by distributing responsibilities across a real-time operating system (FreeRTOS):

* **Core 1 (High-Speed Sampling Task - 50Hz):** 
  * Polls atmospheric pressure from the BME280 and calculates high-precision relative altitude.
  * Reads raw 6-DoF linear acceleration and rotational velocity vectors from the BMI160 IMU.
  * Continuously drains and parses incoming NMEA character streams from the NEO-6M GPS.
  * Runs the real-time apogee detection state machine.
* **Core 0 (Background I/O & Radio Task):** 
  * Pulls formatted telemetry packets out of a thread-safe FreeRTOS queue.
  * Manages non-blocking MicroSD CSV file writing with explicit Chip Select arbitration.
  * Broadcasts 433MHz LoRa data packets and listens asynchronously for over-the-air commands (`CMD_DEPLOY`).

---

## 🎯 Flight Logic & Safety Mechanisms

* **Dual-Condition Apogee Trigger:** To prevent false positives from boost-phase vibration or ignition spikes, parachute deployment requires clearing an arming threshold of **15 meters**, followed by a verified downward drop of **2.5 meters** from the peak recorded altitude.
* **Explicit SPI Bus Arbitration:** Because the MicroSD card and LoRa radio share a common SPI bus, the firmware forces active-LOW Chip Select isolation and a stable 1 MHz clock speed to prevent data collisions and bus lockups.
* **Manual Over-the-Air Override:** Ground station operators can remotely force parachute deployment at any time by transmitting an encrypted `CMD_DEPLOY` command packet via LoRa.

---

## 📂 Repository Structure

```text
├── src/
│   ├── main.cpp             # Core FreeRTOS initialization and task definitions
│   ├── flight_tasks.cpp     # Sensor sampling and radio/logging loops
│   └── config.h             # Pin definitions and calibration constants
├── schematics/              # Custom PCB layout and wiring diagrams
└── README.md                # Project documentation
