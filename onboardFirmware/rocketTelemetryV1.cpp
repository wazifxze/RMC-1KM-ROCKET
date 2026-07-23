#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <LoRa.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BMI160Gen.h>

// ==========================================
//  PIN DEFINITIONS (Seeed Studio XIAO ESP32-S3)
// ==========================================
#define I2C_SDA_PIN      5  // D4
#define I2C_SCL_PIN      6  // D5

#define SPI_SCK_PIN      7  // D8
#define SPI_MISO_PIN     8  // D9
#define SPI_MOSI_PIN     9  // D10

#define SD_CS_PIN        3  // D2
#define LORA_CS_PIN      4  // D3
#define LORA_RST_PIN     1  // D0
#define LORA_DIO0_PIN    2  // D1

// ==========================================
//  TELEMETRY DATA STRUCTURE
// ==========================================
struct TelemetryPacket {
    uint32_t packet_id;
    uint32_t timestamp_ms;
    float pressure;      // hPa
    float temperature;   // Deg C
    float ax, ay, az;    // G-forces (m/s^2 or g)
    float gx, gy, gz;    // Angular velocity (deg/s)
};

// FreeRTOS Queue for Thread-Safe Inter-Core Transfer
QueueHandle_t telemetryQueue;

// Peripheral Instances
Adafruit_BME280 bme;
File logFile;
bool sdInitialized = false;
bool loraInitialized = false;

// ==========================================
//  TASK 1: HIGH-SPEED SENSOR SAMPLING (CORE 1)
// ==========================================
void TaskSensorSampling(void *pvParameters) {
    uint32_t packetCounter = 0;

    for (;;) {
        TelemetryPacket packet;
        packet.packet_id = ++packetCounter;
        packet.timestamp_ms = millis();

        // 1. Read BME280 Atmospheric Data
        packet.pressure = bme.readPressure() / 100.0F; // Convert Pa to hPa
        packet.temperature = bme.readTemperature();

        // 2. Read BMI160 IMU Data
        int rawAx, rawAy, rawAz;
        int rawGx, rawGy, rawGz;
        
        BMI160.readAccelerometer(rawAx, rawAy, rawAz);
        BMI160.readGyro(rawGx, rawGy, rawGz);

        // Convert raw LSB to physical units (approx. ±16g scale & ±2000 deg/s scale)
        packet.ax = (float)rawAx / 2048.0f;
        packet.ay = (float)rawAy / 2048.0f;
        packet.az = (float)rawAz / 2048.0f;

        packet.gx = (float)rawGx / 16.4f;
        packet.gy = (float)rawGy / 16.4f;
        packet.gz = (float)rawGz / 16.4f;

        // 3. Push packet to inter-core queue (Don't block if full)
        xQueueSend(telemetryQueue, &packet, 0);

        // Run sampling at ~50 Hz (20ms delay)
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ==========================================
//  TASK 2: LOGGING & RADIO DOWNLINK (CORE 0)
// ==========================================
void TaskRadioAndLogging(void *pvParameters) {
    TelemetryPacket packet;

    for (;;) {
        // Block task until a sensor packet is available in the Queue
        if (xQueueReceive(telemetryQueue, &packet, portMAX_DELAY) == pdTRUE) {
            
            // Format packet as flat CSV string for transmission
            // Format: $CANSAT,id,time_ms,press,temp,ax,ay,az,gx,gy,gz*
            String csvPacket = "$CANSAT,";
            csvPacket += String(packet.packet_id) + ",";
            csvPacket += String(packet.timestamp_ms) + ",";
            csvPacket += String(packet.pressure, 2) + ",";
            csvPacket += String(packet.temperature, 2) + ",";
            csvPacket += String(packet.ax, 2) + ",";
            csvPacket += String(packet.ay, 2) + ",";
            csvPacket += String(packet.az, 2) + ",";
            csvPacket += String(packet.gx, 2) + ",";
            csvPacket += String(packet.gy, 2) + ",";
            csvPacket += String(packet.gz, 2);
            csvPacket += "*";

            // 1. Transmit Packet over LoRa Ra-02 (433MHz)
            if (loraInitialized) {
                LoRa.beginPacket();
                LoRa.print(csvPacket);
                LoRa.endPacket(true); // Non-blocking async packet dispatch
            }

            // 2. Log Packet to SD Card
            if (sdInitialized && logFile) {
                logFile.println(csvPacket);
                // Flush buffer to physical storage every 10 packets to prevent corruption
                if (packet.packet_id % 10 == 0) {
                    logFile.flush();
                }
            }
        }
    }
}

// ==========================================
//  INITIALIZATION & HARDWARE SETUP
// ==========================================
void setup() {
    Serial.begin(115200);
    delay(1000); // Allow hardware lines to settle

    // Initialize Thread-Safe Queue (Holds up to 20 packets)
    telemetryQueue = xQueueCreate(20, sizeof(TelemetryPacket));

    // 1. Initialize Shared I2C Bus (Sensors)
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000); // Set Fast I2C Mode (400kHz)

    if (!bme.begin(0x76, &Wire) && !bme.begin(0x77, &Wire)) {
        Serial.println("[ERROR] BME280 sensor not detected!");
    }

    BMI160.begin(BMI160GenClass::I2C_MODE, Wire, 0x68);

    // 2. Initialize Shared SPI Bus
    SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);

    // 3. Initialize SD Card Module
    pinMode(SD_CS_PIN, OUTPUT);
    digitalWrite(SD_CS_PIN, HIGH);
    
    if (SD.begin(SD_CS_PIN, SPI)) {
        sdInitialized = true;
        logFile = SD.open("/flight_log.csv", FILE_APPEND);
        if (logFile) {
            logFile.println("HEADER,PACKET_ID,TIME_MS,PRESS_HPA,TEMP_C,AX,AY,AZ,GX,GY,GZ");
            logFile.flush();
        }
    } else {
        Serial.println("[WARNING] SD Card Mount Failed!");
    }

    // 4. Initialize LoRa Ra-02 Module
    LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
    if (LoRa.begin(433E6)) { // 433MHz frequency
        loraInitialized = true;
        LoRa.setTxPower(20);          // Max transmission power (20dBm)
        LoRa.setSpreadingFactor(7);   // SF7 for fast data throughput
        LoRa.setSignalBandwidth(125E3);
        LoRa.enableCrc();
    } else {
        Serial.println("[ERROR] LoRa Ra-02 Init Failed!");
    }

    // ==========================================
    //  FREERTOS DUAL-CORE TASK PINNING
    // ==========================================
    
    // Pin Sensor Sampling to Core 1 (High priority: 2)
    xTaskCreatePinnedToCore(
        TaskSensorSampling,   // Function pointer
        "SamplingTask",       // Task name
        4096,                 // Stack size
        NULL,                 // Parameters
        2,                    // Priority
        NULL,                 // Task handle
        1                     // Core 1
    );

    // Pin IO (Radio + SD) to Core 0 (Lower priority: 1)
    xTaskCreatePinnedToCore(
        TaskRadioAndLogging,  // Function pointer
        "DownlinkTask",       // Task name
        8192,                 // Stack size
        NULL,                 // Parameters
        1,                    // Priority
        NULL,                 // Task handle
        0                     // Core 0
    );
}

void loop() {
    // Empty: Main execution loop is completely offloaded to FreeRTOS tasks!
    vTaskDelete(NULL);
}