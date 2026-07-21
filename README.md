1. The Mission Overview
The primary goal of this aerospace payload is to launch a compact, soda-can-sized satellite (CanSat) inside a rocket, collect high-frequency flight dynamics and environmental data during ascent and descent, write that data to a local "Black Box" flight recorder, and transmit it live to a ground computer.

Instead of processing complex flight path calculations inside the rocket—which wastes precious processor speed and battery—the payload uses a Downlink-Centric Architecture. The rocket acts as a high-speed data funnel, gathering raw physical readings and beaming them down instantly over radio waves. All heavy matrix calculations, trajectory mapping, and 3D visual rendering happen safely on your team's ground station laptop.

2. Physical & Hardware Architecture
The payload is organized into a modular, 6-layer circular stack enclosed inside a radio-transparent fiberglass body tube (to allow unhindered LoRa signals) with 3D-printed ABS nosecone and fins:

                  ┌───────────────────────────────────────────┐
                  │          THE 6-LAYER CANSAT STACK         │
                  └───────────────────────────────────────────┘
                  
  [ Layer 6: Telemetry  ] ──► LoRa Ra-02 (433MHz Transceiver) + Antenna
  [ Layer 5: Motion     ] ──► BMI160 IMU (6-Axis Accelerometer & Gyroscope)
  [ Layer 4: Atmosphere ] ──► BME280 Sensor (Pressure, Temp, Humidity)
  [ Layer 3: Storage    ] ──► Micro SD Breakout Board + Class 10 Card
  [ Layer 2: Core Brain ] ──► Seeed Studio XIAO ESP32-S3 Board
  [ Layer 1: Power      ] ──► 3.7V LiPo Battery + 5V/3.3V Regulator Rails


Data Highway System:
    -The $I^2C$ Bus (SDA / SCL) runs vertically through the stack to connect both atmospheric (BME280) and motion (BMI160) sensors using only 2 shared pins.
    -The SPI Bus (SCK / MISO / MOSI) runs through the stack to allow high-speed data transfers to the SD card and the LoRa radio module, using independent Chip Select pins (SD_CS and LORA_CS) to tell them apart.

Power Distribution:
    -The LiPo battery at the bottom provides a low center of gravity. The regulators step down the voltage into steady 3.3V and 5V rails, supplying clean power up the stack header pins.

3. End-to-End Data Flow Pipeline

[PHYSICAL WORLD]
  Air Pressure, Temp, G-Forces, Spin Rates
         │
         ▼
  [ONBOARD SENSORS] ──(I2C Bus)──► [XIAO CORE 1: SENSOR TASK]
  BME280 & BMI160                  • Reads raw values @ 50Hz
                                   • Packs into TelemetryPacket Struct
                                   • Pushes to FreeRTOS Queue
                                          │
                                   (FreeRTOS Queue)
                                          │
                                          ▼
                                   [XIAO CORE 0: DOWNLINK TASK]
                                   • Pops packet from Queue
                                   • Formats string: $CANSAT,id,time...*
                                   • Writes CSV line to Micro SD
                                   • Beams packet over 433MHz LoRa
                                          │
                                     (LoRa Radio)
                                          │
                                          ▼
  [GROUND STATION] ──(USB Serial)─► [PYTHON VISUALIZER]
  Receiver LoRa Module             • Reads text stream via pyserial
                                   • Parses raw sensor floats
                                   • Calculates Pitch & Roll angles
                                   • Rotates 3D CanSat model @ 60FPS