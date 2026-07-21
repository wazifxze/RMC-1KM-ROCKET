The onboard firmware runs on the dual-core Seeed Studio XIAO ESP32-S3 inside the rocket using C++.

Core Architecture & Dual-Core Threading
The code uses FreeRTOS (an embedded real-time operating system) to split duties between the microcontroller's two processing cores:

Core 1 (High Priority - Sensor Task): 
    -Executes an uninterrupted loop at $50\text{ Hz}$ ($20\text{ms}$ delay). Its sole job is to pull raw binary data from the BME280 and BMI160 sensors, pack it into a lightweight C++ structure (TelemetryPacket), and drop it into a thread-safe memory queue (telemetryQueue).

Core 0 (Input/Output Task):
    - Continuously checks the memory queue. When a new packet arrives, Core 0 formats it into a single line of comma-separated text, appends it to the flight log on the Micro SD card, and passes it to the LoRa radio module for transmission.

Why this matters: 
    -SD card writes and radio transmissions occasionally experience tiny delays. By isolating those I/O operations on Core 0, Core 1 never lags, ensuring you capture every microsecond of violent movement during launch.

Key C++ Functions & Logic Explained

1. [struct TelemetryPacket]: 
    -A lightweight data container in memory holding packet IDs, timestamps, pressure, temperature, and 6-axis motion values ($A_x, A_y, A_z, G_x, G_y, G_z$).

2. [TaskSensorSampling()] (Core 1):
    -[bme.readPressure()] & [bme.readTemperature()]: Queries atmospheric data over $I^2C$.BMI160.
    -[readAccelerometer()] & [readGyro()]: Reads raw motion variables and divides them by hardware conversion factors (2048.0 for $\pm 16g$ mode, 16.4 for gyroscope angles) to yield standard physical units ($g$-forces and degrees/sec).
    -[xQueueSend()]: Safely transfers the data structure into the inter-core queue without blocking execution if the queue happens to be momentarily busy
    
3. [TaskRadioAndLogging()] (Core 0):
    -[xQueueReceive()]: Waits until a packet drops into the queue from Core 1.
    -[csvPacket = "$CANSAT,..."]: Formats the raw numeric values into a strict CSV string wrapped in [$CANSAT] and [*] header/footer markers (so the ground station knows where a packet begins and ends).
    -[LoRa.beginPacket()] & [LoRa.endPacket(true)]: Sends the text packet out asynchronously over the 433MHz frequency.
    -[logFile.println()] & [logFile.flush()]: Appends the string to the SD card. [flush()] forces the buffer to commit to physical flash storage every 10 packets so data isn't lost if the battery disconnects upon landing.

4. [setup()]:
    - Configures system clocks, initializes the $I^2C$ bus at $400\text{ kHz}$ (Fast Mode), sets up the SPI bus with unique Chip Select pins, mounts the file system on the SD card, boots the LoRa radio at max $20\text{ dBm}$ power, and assigns the two tasks to their respective cores using [xTaskCreatePinnedToCore()].