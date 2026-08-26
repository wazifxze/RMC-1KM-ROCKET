#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <LoRa.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BMI160Gen.h>
#include <TinyGPSPlus.h>

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

#define GPS_RX_PIN  44   // D7
#define GPS_TX_PIN  43   // D6
HardwareSerial GPSSerial(1);
TinyGPSPlus gps;

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
    double gps_lat, gps_lon;
    float  gps_alt;
    uint8_t gps_satellites;
    bool   gps_fix_valid;
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

        // Convert raw LSB to physical units
        packet.ax = (float)rawAx / 2048.0f;
        packet.ay = (float)rawAy / 2048.0f;
        packet.az = (float)rawAz / 2048.0f;

        packet.gx = (float)rawGx / 16.4f;
        packet.gy = (float)rawGy / 16.4f;
        packet.gz = (float)rawGz / 16.4f;

        while (GPSSerial.available()) {
            gps.encode(GPSSerial.read());
        }
        // Read GPS module
        packet.gps_fix_valid  = gps.location.isValid();
        packet.gps_lat        = packet.gps_fix_valid ? gps.location.lat() : 0.0;
        packet.gps_lon        = packet.gps_fix_valid ? gps.location.lng() : 0.0;
        packet.gps_alt        = gps.altitude.isValid() ? gps.altitude.meters() : 0.0f;
        packet.gps_satellites = gps.satellites.value();

        // 3. Push packet to inter-core queue
        xQueueSend(telemetryQueue, &packet, 0);

        // Run sampling at ~50 Hz
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ==========================================
//  TASK 2: LOGGING & RADIO DOWNLINK (CORE 0)
// ==========================================
void TaskRadioAndLogging(void *pvParameters) {
    TelemetryPacket packet;

    for (;;) {
        if (xQueueReceive(telemetryQueue, &packet, portMAX_DELAY) == pdTRUE) {

            String csvPacket  = "$CANSAT,";
            csvPacket += String(packet.packet_id) + ",";
            csvPacket += String(packet.timestamp_ms) + ",";
            csvPacket += String(packet.pressure, 2) + ",";
            csvPacket += String(packet.temperature, 2) + ",";
            csvPacket += String(packet.ax, 2) + ",";
            csvPacket += String(packet.ay, 2) + ",";
            csvPacket += String(packet.az, 2) + ",";
            csvPacket += String(packet.gx, 2) + ",";
            csvPacket += String(packet.gy, 2) + ",";
            csvPacket += String(packet.gz, 2) + ",";
            csvPacket += String(packet.gps_fix_valid ? 1 : 0) + ",";
            csvPacket += String(packet.gps_lat, 6) + ",";
            csvPacket += String(packet.gps_lon, 6) + ",";
            csvPacket += String(packet.gps_alt, 1) + ",";
            csvPacket += String(packet.gps_satellites);
            csvPacket += "*";

            // 1. Log EVERY packet to SD Card (50 Hz)
            if (sdInitialized && logFile) {
                Serial.print("[SD] Writing ID: ");
                Serial.print(packet.packet_id);
                logFile.println(csvPacket);
                if (packet.packet_id % 10 == 0) logFile.flush();
                Serial.println(" ... Done.");
            }

            // 2. Transmit over LoRa every 7th packet (~7 Hz)
            if (loraInitialized && (packet.packet_id % 7 == 0)) {
                Serial.print("[LoRa] Transmitting ID: ");
                Serial.print(packet.packet_id);
                LoRa.beginPacket();
                LoRa.print(csvPacket);
                LoRa.endPacket(false); 
                Serial.println(" ... Done.");
            }

            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

// ==========================================
//  INITIALIZATION & HARDWARE SETUP
// ==========================================
void setup() {

    // Global offset variables
    //make sure the rocket is still and stationary when booting the board!!
    float ax_offset = 0.0, ay_offset = 0.0, az_offset = 0.0;
    float gx_offset = 0.0, gy_offset = 0.0, gz_offset = 0.0;

    void calibrateIMU() {
         const int samples = 500;
        float sum_ax = 0, sum_ay = 0, sum_az = 0;
        float sum_gx = 0, sum_gy = 0, sum_gz = 0;

        Serial.println("Calibrating IMU... Keep rocket stationary!");

        for (int i = 0; i < samples; i++) {
            // Read raw sensor values (replace with your IMU read function)
            readSensors(); 
    
            sum_ax += raw_ax;
            sum_ay += raw_ay;
            sum_az += raw_az;
            sum_gx += raw_gx;
            sum_gy += raw_gy;
            sum_gz += raw_gz;
    
            delay(5);
        }

        // Calculate average static offsets
        ax_offset = sum_ax / samples;
        ay_offset = sum_ay / samples;
  
        // Z-axis experiences 1.0G of gravity at rest (assuming vertical mounting)
        // If mounted horizontally, subtract 1.0G from whichever axis points straight UP.
        az_offset = (sum_az / samples) - 1.0; 

        gx_offset = sum_gx / samples;
        gy_offset = sum_gy / samples;
        gz_offset = sum_gz / samples;

        Serial.println("IMU Calibration Complete.");
    }

    // In your loop, subtract the offsets before transmitting telemetry:
    // ax_calibrated = raw_ax - ax_offset;
    // az_calibrated = raw_az - az_offset;
    // gx_calibrated = raw_gx - gx_offset;

    Serial.begin(115200);
    delay(1000); 

    // Explicitly deselect SPI devices before bus initialisation
    pinMode(SD_CS_PIN, OUTPUT);
    digitalWrite(SD_CS_PIN, HIGH);
    pinMode(LORA_CS_PIN, OUTPUT);
    digitalWrite(LORA_CS_PIN, HIGH);

    GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    telemetryQueue = xQueueCreate(20, sizeof(TelemetryPacket));

    // 1. Initialize Shared I2C Bus
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(100000); 

    if (!bme.begin(0x76, &Wire) && !bme.begin(0x77, &Wire)) {
        Serial.println("[ERROR] BME280 sensor not detected!");
    }

    if (BMI160.begin(BMI160GenClass::I2C_MODE, Wire, 0x69)) {
        BMI160.setFullScaleAccelRange(BMI160_ACCEL_RANGE_16G);
        BMI160.setFullScaleGyroRange(BMI160_GYRO_RANGE_2000);
        Serial.println("[INFO] BMI160 Initialized");
    } else {
        Serial.println("[ERROR] BMI160 IMU Init Failed!");
    }

    // 2. Initialize Shared SPI Bus
    SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);

    // 3. Initialize SD Card Module
    if (SD.begin(SD_CS_PIN, SPI)) {
        sdInitialized = true;
        logFile = SD.open("/flight_log.csv", FILE_APPEND);
        if (logFile) {
            logFile.println("HEADER,PACKET_ID,TIME_MS,PRESS_HPA,TEMP_C,AX,AY,AZ,GX,GY,GZ,GPS_FIX,GPS_LAT,GPS_LON,GPS_ALT,GPS_SATS");
            logFile.flush();
        }
    } else {
        Serial.println("[WARNING] SD Card Mount Failed!");
    }

    // 4. Initialize LoRa Ra-02 Module
    LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
    if (LoRa.begin(433E6)) { 
        loraInitialized = true;
        LoRa.setTxPower(20);          
        LoRa.setSpreadingFactor(7);   
        LoRa.setSignalBandwidth(125E3);
        LoRa.enableCrc();
    } else {
        Serial.println("[ERROR] LoRa Ra-02 Init Failed!");
    }

    // Pin FreeRTOS tasks to cores
    xTaskCreatePinnedToCore(
        TaskSensorSampling, "SamplingTask", 4096, NULL, 2, NULL, 1
    );

    xTaskCreatePinnedToCore(
        TaskRadioAndLogging, "DownlinkTask", 8192, NULL, 1, NULL, 0
    );
}

void loop() {
    vTaskDelete(NULL);
}