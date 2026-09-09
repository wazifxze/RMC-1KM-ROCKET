#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <LoRa.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BMI160Gen.h>
#include <TinyGPSPlus.h>
#include <ESP32Servo.h>

// ==========================================
//  PIN DEFINITIONS (Seeed Studio XIAO ESP32-S3)
// ==========================================
#define I2C_SDA_PIN      5  // D4 (GPIO 5)
#define I2C_SCL_PIN      6  // D5 (GPIO 6)

#define SPI_SCK_PIN      7  // D8 (GPIO 7)
#define SPI_MISO_PIN     8  // D9 (GPIO 8)
#define SPI_MOSI_PIN     9  // D10 (GPIO 9)

#define SD_CS_PIN        3  // D2 (GPIO 3)
#define LORA_CS_PIN      4  // D3 (GPIO 4)
#define LORA_RST_PIN     1  // D0 (GPIO 1)
#define LORA_DIO0_PIN    2  // D1 (GPIO 2)

#define GPS_RX_PIN      44  // D7 (GPIO 44)
#define GPS_TX_PIN      -1  // Disabled
#define SERVO_PIN       43  // D6 (GPIO 43)
#define ONBOARD_LED     21  // Built-in Yellow LED (Active-LOW)

HardwareSerial GPSSerial(1);
TinyGPSPlus gps;
Servo deployServo;

// ==========================================
//  TELEMETRY DATA STRUCTURE
// ==========================================
struct TelemetryPacket {
    uint32_t packet_id;
    uint32_t timestamp_ms;
    float pressure;      // hPa
    float temperature;   // Deg C
    float baro_alt;      // Relative Altitude (m)
    float ax, ay, az;    // G-forces (g)
    float gx, gy, gz;    // Angular velocity (deg/s)
    double gps_lat, gps_lon;
    float  gps_alt;
    uint8_t gps_satellites;
    bool   gps_fix_valid;
    bool   apogee_triggered;
};

QueueHandle_t telemetryQueue;

Adafruit_BME280 bme;
File logFile;
bool sdInitialized = false;
bool loraInitialized = false;

// Calibration & State Variables
float ax_offset = 0.0, ay_offset = 0.0, az_offset = 0.0;
float gx_offset = 0.0, gy_offset = 0.0, gz_offset = 0.0;
float ground_pressure_hpa = 1013.25;

float max_altitude = 0.0;
bool apogee_triggered = false;
const float APOGEE_ARM_ALT_M = 15.0;  
const float APOGEE_DROP_M    = 2.5;   

// Servo Configuration
const int SERVO_LOCKED_POS   = 0;
const int SERVO_DEPLOY_POS   = 90;
const bool INVERT_SERVO_DIR  = true; 

void writeServo(int angle) {
    int target = INVERT_SERVO_DIR ? (180 - angle) : angle;
    deployServo.write(target);
}

void calibrateIMU() {
    const int samples = 500;
    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;

    Serial.println("[INFO] Calibrating IMU...");
    for (int i = 0; i < samples; i++) {
        int rawAx, rawAy, rawAz;
        int rawGx, rawGy, rawGz;
        BMI160.readAccelerometer(rawAx, rawAy, rawAz);
        BMI160.readGyro(rawGx, rawGy, rawGz);
        sum_ax += (float)rawAx / 2048.0f;
        sum_ay += (float)rawAy / 2048.0f;
        sum_az += (float)rawAz / 2048.0f;
        sum_gx += (float)rawGx / 16.4f;
        sum_gy += (float)rawGy / 16.4f;
        sum_gz += (float)rawGz / 16.4f;
        delay(5);
    }
    ax_offset = sum_ax / samples;
    ay_offset = sum_ay / samples;
    az_offset = (sum_az / samples) - 1.0f; 
    gx_offset = sum_gx / samples;
    gy_offset = sum_gy / samples;
    gz_offset = sum_gz / samples;
}

void calibrateGroundPressure() {
    float sum_pressure = 0.0;
    const int samples = 50;
    for (int i = 0; i < samples; i++) {
        sum_pressure += bme.readPressure() / 100.0F;
        delay(20);
    }
    ground_pressure_hpa = sum_pressure / samples;
}

void TaskSensorSampling(void *pvParameters) {
    uint32_t packetCounter = 0;
    for (;;) {
        TelemetryPacket packet;
        packet.packet_id = ++packetCounter;
        packet.timestamp_ms = millis();

        packet.pressure = bme.readPressure() / 100.0F;
        packet.temperature = bme.readTemperature();
        packet.baro_alt = bme.readAltitude(ground_pressure_hpa);

        if (!apogee_triggered) {
            if (packet.baro_alt > max_altitude) max_altitude = packet.baro_alt;
            if ((max_altitude > APOGEE_ARM_ALT_M) && ((max_altitude - packet.baro_alt) >= APOGEE_DROP_M)) {
                apogee_triggered = true;
                writeServo(SERVO_DEPLOY_POS); 
                Serial.printf("[ACTION] APOGEE DETECTED AT %.2f m! Servo Deployed.\n", max_altitude);
            }
        }
        packet.apogee_triggered = apogee_triggered;

        int rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
        BMI160.readAccelerometer(rawAx, rawAy, rawAz);
        BMI160.readGyro(rawGx, rawGy, rawGz);

        packet.ax = ((float)rawAx / 2048.0f) - ax_offset;
        packet.ay = ((float)rawAy / 2048.0f) - ay_offset;
        packet.az = ((float)rawAz / 2048.0f) - az_offset;
        packet.gx = ((float)rawGx / 16.4f) - gx_offset;
        packet.gy = ((float)rawGy / 16.4f) - gy_offset;
        packet.gz = ((float)rawGz / 16.4f) - gz_offset;

        while (GPSSerial.available()) {
            gps.encode(GPSSerial.read());
        }
        packet.gps_fix_valid  = gps.location.isValid();
        packet.gps_lat        = packet.gps_fix_valid ? gps.location.lat() : 0.0;
        packet.gps_lon        = packet.gps_fix_valid ? gps.location.lng() : 0.0;
        packet.gps_alt        = gps.altitude.isValid() ? gps.altitude.meters() : 0.0f;
        packet.gps_satellites = gps.satellites.value();

        xQueueSend(telemetryQueue, &packet, 0);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void TaskRadioAndLogging(void *pvParameters) {
    TelemetryPacket packet;
    for (;;) {
        // 1. Check for incoming LoRa commands and log metrics to SD card
        if (loraInitialized) {
            int packetSize = LoRa.parsePacket();
            if (packetSize) {
                String incomingCommand = "";
                while (LoRa.available()) {
                    incomingCommand += (char)LoRa.read();
                }

                uint32_t rxTime = millis();
                int rssi = LoRa.packetRssi();
                float snr = LoRa.packetSnr();

                Serial.printf("[RX] Size: %d | RSSI: %d | SNR: %.2f | Payload: %s\n", packetSize, rssi, snr, incomingCommand.c_str());

                // Log incoming packet details to SD Card with explicit CS isolation
                if (sdInitialized) {
                    digitalWrite(LORA_CS_PIN, HIGH);
                    digitalWrite(SD_CS_PIN, LOW);
                    File loraLog = SD.open("/lora_packet_log.csv", FILE_APPEND);
                    if (loraLog) {
                        loraLog.print(rxTime); loraLog.print(",");
                        loraLog.print(packetSize); loraLog.print(",");
                        loraLog.print(rssi); loraLog.print(",");
                        loraLog.print(snr); loraLog.print(",");
                        loraLog.println(incomingCommand);
                        loraLog.flush();
                        loraLog.close();
                    }
                    digitalWrite(SD_CS_PIN, HIGH);
                }

                if ((incomingCommand.indexOf("CMD_DEPLOY") != -1 || incomingCommand.indexOf("TRIGGER_SERVO") != -1) && !apogee_triggered) {
                    apogee_triggered = true;
                    writeServo(SERVO_DEPLOY_POS);
                    Serial.println("[MANUAL OVERRIDE] Servo triggered via wireless command!");
                }
            }
        }

        // 2. Process telemetry queue for logging and throttled downlink
        if (xQueueReceive(telemetryQueue, &packet, pdMS_TO_TICKS(5)) == pdTRUE) {
            String csvPacket  = "$CANSAT,";
            csvPacket += String(packet.packet_id) + ",";
            csvPacket += String(packet.timestamp_ms) + ",";
            csvPacket += String(packet.pressure, 2) + ",";
            csvPacket += String(packet.temperature, 2) + ",";
            csvPacket += String(packet.baro_alt, 2) + ",";
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
            csvPacket += String(packet.gps_satellites) + ",";
            csvPacket += String(packet.apogee_triggered ? 1 : 0);
            csvPacket += "*";

            // SD Card Telemetry Logging
            if (sdInitialized && logFile) {
                digitalWrite(LORA_CS_PIN, HIGH);
                digitalWrite(SD_CS_PIN, LOW);
                logFile.println(csvPacket);
                if (packet.packet_id % 10 == 0) logFile.flush();
                digitalWrite(SD_CS_PIN, HIGH);
            }

            // Throttled LoRa Downlink (every 20 packets to prevent collisions)
            if (loraInitialized && (packet.packet_id % 20 == 0)) {
                digitalWrite(SD_CS_PIN, HIGH);
                digitalWrite(LORA_CS_PIN, LOW);
                LoRa.beginPacket();
                LoRa.print(csvPacket);
                LoRa.endPacket(false);
                digitalWrite(LORA_CS_PIN, HIGH);
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(ONBOARD_LED, OUTPUT);
    digitalWrite(ONBOARD_LED, LOW); 
    delay(1000); 

    pinMode(SD_CS_PIN, OUTPUT);
    digitalWrite(SD_CS_PIN, HIGH);
    pinMode(LORA_CS_PIN, OUTPUT);
    digitalWrite(LORA_CS_PIN, HIGH);

    // Allocate PWM timers for ESP32-S3 servo reliability
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    deployServo.setPeriodHertz(50);
    deployServo.attach(SERVO_PIN, 500, 2400);
    writeServo(SERVO_LOCKED_POS);

    GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    telemetryQueue = xQueueCreate(20, sizeof(TelemetryPacket));

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(100000); 

    if (bme.begin(0x76, &Wire) || bme.begin(0x77, &Wire)) {
        calibrateGroundPressure();
    }

    if (BMI160.begin(BMI160GenClass::I2C_MODE, Wire, 0x69)) {
        BMI160.setFullScaleAccelRange(BMI160_ACCEL_RANGE_16G);
        BMI160.setFullScaleGyroRange(BMI160_GYRO_RANGE_2000);
        calibrateIMU();
    }

    // Hardware Reset LoRa
    pinMode(LORA_RST_PIN, OUTPUT);
    digitalWrite(LORA_RST_PIN, HIGH);
    delay(10);
    digitalWrite(LORA_RST_PIN, LOW);
    delay(20);
    digitalWrite(LORA_RST_PIN, HIGH);
    delay(50);

    SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SD_CS_PIN);
    
    LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
    LoRa.setSPI(SPI);
    LoRa.setSPIFrequency(1000000);

    if (LoRa.begin(433E6)) { 
        loraInitialized = true;
        LoRa.setTxPower(20);          
        LoRa.setSpreadingFactor(7);   
        LoRa.setSignalBandwidth(125E3);
        LoRa.enableCrc();
    }
    digitalWrite(LORA_CS_PIN, HIGH);

    if (SD.begin(SD_CS_PIN, SPI, 1000000)) { 
        sdInitialized = true;
        
        // Initialize flight telemetry log
        logFile = SD.open("/flight_log.csv", FILE_APPEND);
        if (logFile) {
            logFile.println("HEADER,PACKET_ID,TIME_MS,PRESS_HPA,TEMP_C,REL_ALT_M,AX,AY,AZ,GX,GY,GZ,GPS_FIX,GPS_LAT,GPS_LON,GPS_ALT,GPS_SATS,APOGEE");
            logFile.flush();
        }

        // Initialize LoRa packet log header if missing
        if (!SD.exists("/lora_packet_log.csv")) {
            File loraLog = SD.open("/lora_packet_log.csv", FILE_WRITE);
            if (loraLog) {
                loraLog.println("TIMESTAMP_MS,PACKET_SIZE,RSSI,SNR,PAYLOAD");
                loraLog.close();
            }
        }
    }
    digitalWrite(SD_CS_PIN, HIGH);

    digitalWrite(ONBOARD_LED, HIGH); 

    xTaskCreatePinnedToCore(TaskSensorSampling, "SamplingTask", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(TaskRadioAndLogging, "DownlinkTask", 8192, NULL, 1, NULL, 0);
}

void loop() {
    vTaskDelete(NULL);
}