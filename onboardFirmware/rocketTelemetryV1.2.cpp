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
    float vert_vel;      // Kalman Fused Vertical Velocity (m/s)
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
bool sdInitialized   = false;
bool loraInitialized = false;
bool imuInitialized  = false;

// Calibration & State Variables
float ax_offset = 0.0, ay_offset = 0.0, az_offset = 0.0;
float gx_offset = 0.0, gy_offset = 0.0, gz_offset = 0.0;
float ground_pressure_hpa = 1013.25;

// ==========================================
//  HYBRID APOGEE & KALMAN CONFIGURATION
// ==========================================
const float MIN_ARM_ALT_M            = 15.0f;  // Safety Gate 1: Must exceed 15m AGL
const float MIN_ARM_VELOCITY_MPS     = 8.0f;   // Safety Gate 2: Must exceed +8.0 m/s ascent
const float APOGEE_VEL_TRIGGER       = 0.0f;   // Primary Trigger: Fused velocity drops <= 0 m/s
const float APOGEE_ALT_DROP_FALLBACK = 3.0f;   // Backup Trigger: Altitude drops 3m below peak

bool system_armed     = false;
bool apogee_triggered = false;
float max_altitude    = 0.0f;

// 2-State Kalman Filter (Altitude & Velocity)
float kf_alt = 0.0f;
float kf_vel = 0.0f;
float P_00 = 1.0f, P_01 = 0.0f;
float P_10 = 0.0f, P_11 = 1.0f;

// Noise Covariances (Tuning Parameters)
const float Q_ACCEL = 0.5f;   // Process noise (Accelerometer variance)
const float R_BARO  = 0.25f;  // Measurement noise (Barometer variance)

// Servo Configuration
const int SERVO_LOCKED_POS   = 0;
const int SERVO_DEPLOY_POS   = 90;
const bool INVERT_SERVO_DIR  = true; 

void writeServo(int angle) {
    int target = INVERT_SERVO_DIR ? (180 - angle) : angle;
    deployServo.write(target);
}

void calibrateIMU() {
    const int samples = 100;
    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;

    Serial.println("[INFO] Calibrating IMU...");
    for (int i = 0; i < samples; i++) {
        int rawAx = 0, rawAy = 0, rawAz = 0;
        int rawGx = 0, rawGy = 0, rawGz = 0;
        
        BMI160.readAccelerometer(rawAx, rawAy, rawAz);
        BMI160.readGyro(rawGx, rawGy, rawGz);
        
        sum_ax += (float)rawAx / 2048.0f;
        sum_ay += (float)rawAy / 2048.0f;
        sum_az += (float)rawAz / 2048.0f;
        sum_gx += (float)rawGx / 16.4f;
        sum_gy += (float)rawGy / 16.4f;
        sum_gz += (float)rawGz / 16.4f;
        delay(2);
    }
    
    ax_offset = sum_ax / samples;
    ay_offset = sum_ay / samples;
    az_offset = (sum_az / samples) - 1.0f;
    gx_offset = sum_gx / samples;
    gy_offset = sum_gy / samples;
    gz_offset = sum_gz / samples;
    Serial.println("[INFO] IMU Calibration Complete.");
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

void updateKalmanFilter(float baro_alt, float accel_z_g, float dt) {
    if (dt <= 0.001f) return;

    float a_vert = (accel_z_g - 1.0f) * 9.81f;

    kf_alt += kf_vel * dt + 0.5f * a_vert * dt * dt;
    kf_vel += a_vert * dt;

    P_00 += dt * (P_10 + P_01 + dt * P_11) + Q_ACCEL * dt * dt;
    P_01 += dt * P_11;
    P_10 += dt * P_11;
    P_11 += Q_ACCEL * dt;

    float y = baro_alt - kf_alt; 
    float S = P_00 + R_BARO;    

    float K_0 = P_00 / S; 
    float K_1 = P_10 / S; 

    kf_alt += K_0 * y;
    kf_vel += K_1 * y;

    float P00_temp = P_00;
    float P01_temp = P_01;

    P_00 -= K_0 * P00_temp;
    P_01 -= K_0 * P01_temp;
    P_10 -= K_1 * P00_temp;
    P_11 -= K_1 * P01_temp;
}

void TaskSensorSampling(void *pvParameters) {
    uint32_t packetCounter = 0;
    uint32_t last_sample_ms = millis();

    for (;;) {
        TelemetryPacket packet;
        packet.packet_id = ++packetCounter;
        packet.timestamp_ms = millis();

        float dt = (packet.timestamp_ms - last_sample_ms) / 1000.0f;
        last_sample_ms = packet.timestamp_ms;

        packet.pressure = bme.readPressure() / 100.0F;
        packet.temperature = bme.readTemperature();
        packet.baro_alt = bme.readAltitude(ground_pressure_hpa);

        if (imuInitialized) {
            int rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
            BMI160.readAccelerometer(rawAx, rawAy, rawAz);
            BMI160.readGyro(rawGx, rawGy, rawGz);

            packet.ax = ((float)rawAx / 2048.0f) - ax_offset;
            packet.ay = ((float)rawAy / 2048.0f) - ay_offset;
            packet.az = ((float)rawAz / 2048.0f) - az_offset;
            packet.gx = ((float)rawGx / 16.4f) - gx_offset;
            packet.gy = ((float)rawGy / 16.4f) - gy_offset;
            packet.gz = ((float)rawGz / 16.4f) - gz_offset;
        } else {
            packet.ax = 0; packet.ay = 0; packet.az = 1.0f;
            packet.gx = 0; packet.gy = 0; packet.gz = 0;
        }

        updateKalmanFilter(packet.baro_alt, packet.az, dt);
        packet.vert_vel = kf_vel;

        if (packet.baro_alt > max_altitude) {
            max_altitude = packet.baro_alt;
        }

        if (!apogee_triggered) {
            if (!system_armed && (packet.baro_alt >= MIN_ARM_ALT_M) && (packet.vert_vel >= MIN_ARM_VELOCITY_MPS)) {
                system_armed = true;
                Serial.printf("[ARMED] Hybrid criteria met! Alt: %.2f m | Vel: %.2f m/s\n", packet.baro_alt, packet.vert_vel);
            }

            if (system_armed) {
                bool primary_zero_vel = (packet.vert_vel <= APOGEE_VEL_TRIGGER);
                bool backup_baro_drop = ((max_altitude - packet.baro_alt) >= APOGEE_ALT_DROP_FALLBACK);

                if (primary_zero_vel || backup_baro_drop) {
                    apogee_triggered = true;
                    writeServo(SERVO_DEPLOY_POS); 
                    Serial.printf("[ACTION] APOGEE DETECTED via %s! Alt: %.2f m | Vel: %.2f m/s\n",
                                  primary_zero_vel ? "Zero Velocity (Kalman)" : "Altitude Drop Fallback",
                                  packet.baro_alt, packet.vert_vel);
                }
            }
        }
        packet.apogee_triggered = apogee_triggered;

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

        if (xQueueReceive(telemetryQueue, &packet, pdMS_TO_TICKS(5)) == pdTRUE) {
            String csvPacket  = "$CANSAT,";
            csvPacket += String(packet.packet_id) + ",";
            csvPacket += String(packet.timestamp_ms) + ",";
            csvPacket += String(packet.pressure, 2) + ",";
            csvPacket += String(packet.temperature, 2) + ",";
            csvPacket += String(packet.baro_alt, 2) + ",";
            csvPacket += String(packet.vert_vel, 2) + ",";
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

            if (sdInitialized && logFile) {
                digitalWrite(LORA_CS_PIN, HIGH);
                digitalWrite(SD_CS_PIN, LOW);
                logFile.println(csvPacket);
                if (packet.packet_id % 10 == 0) logFile.flush();
                digitalWrite(SD_CS_PIN, HIGH);
            }

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

    Serial.println("[SETUP] Initializing Servo and GPS...");
    pinMode(SD_CS_PIN, OUTPUT);
    digitalWrite(SD_CS_PIN, HIGH);
    pinMode(LORA_CS_PIN, OUTPUT);
    digitalWrite(LORA_CS_PIN, HIGH);

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    deployServo.setPeriodHertz(50);
    deployServo.attach(SERVO_PIN, 500, 2400);
    writeServo(SERVO_LOCKED_POS);

    GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    telemetryQueue = xQueueCreate(20, sizeof(TelemetryPacket));

    Serial.println("[SETUP] Initializing I2C Bus...");
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(100000); 
    Wire.setTimeOut(1000);

    Serial.println("[SETUP] Checking BME280 Barometer...");
    if (bme.begin(0x76, &Wire) || bme.begin(0x77, &Wire)) {
        calibrateGroundPressure();
        Serial.println("[SETUP] BME280 Ready.");
    } else {
        Serial.println("[WARNING] BME280 Barometer not found!");
    }

    Serial.println("[SETUP] Checking BMI160 IMU...");
    if (BMI160.begin(BMI160GenClass::I2C_MODE, Wire, 0x69) || 
        BMI160.begin(BMI160GenClass::I2C_MODE, Wire, 0x68)) {
        imuInitialized = true;
        BMI160.setFullScaleAccelRange(BMI160_ACCEL_RANGE_16G);
        BMI160.setFullScaleGyroRange(BMI160_GYRO_RANGE_2000);
        calibrateIMU(); // Completes successfully
    } else {
        Serial.println("[WARNING] BMI160 IMU not detected!");
    }

    // Explicit SPI Bus CS Guard
    digitalWrite(SD_CS_PIN, HIGH);
    digitalWrite(LORA_CS_PIN, HIGH);

    Serial.println("[SETUP] Resetting LoRa transceiver hardware...");
    pinMode(LORA_RST_PIN, OUTPUT);
    digitalWrite(LORA_RST_PIN, HIGH);
    delay(10);
    digitalWrite(LORA_RST_PIN, LOW);
    delay(20);
    digitalWrite(LORA_RST_PIN, HIGH);
    delay(50);

    Serial.println("[SETUP] Starting SPI Bus...");
    SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SD_CS_PIN);

    Serial.println("[SETUP] Testing LoRa Radio...");
    LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
    LoRa.setSPI(SPI);
    LoRa.setSPIFrequency(1000000);

    digitalWrite(SD_CS_PIN, HIGH);
    digitalWrite(LORA_CS_PIN, LOW);
    if (LoRa.begin(433E6)) { 
        loraInitialized = true;
        LoRa.setTxPower(20);          
        LoRa.setSpreadingFactor(7);   
        LoRa.setSignalBandwidth(125E3);
        LoRa.enableCrc();
        Serial.println("[SETUP] LoRa Radio Initialized Successfully.");
    } else {
        Serial.println("[WARNING] LoRa initialization failed or module disconnected.");
    }
    digitalWrite(LORA_CS_PIN, HIGH);

    Serial.println("[SETUP] Initializing SD Card Module...");
    digitalWrite(LORA_CS_PIN, HIGH);
    digitalWrite(SD_CS_PIN, LOW);
    if (SD.begin(SD_CS_PIN, SPI, 1000000)) { 
        sdInitialized = true;
        
        logFile = SD.open("/flight_log.csv", FILE_APPEND);
        if (logFile) {
            logFile.println("HEADER,PACKET_ID,TIME_MS,PRESS_HPA,TEMP_C,REL_ALT_M,VERT_VEL_MPS,AX,AY,AZ,GX,GY,GZ,GPS_FIX,GPS_LAT,GPS_LON,GPS_ALT,GPS_SATS,APOGEE");
            logFile.flush();
        }

        if (!SD.exists("/lora_packet_log.csv")) {
            File loraLog = SD.open("/lora_packet_log.csv", FILE_WRITE);
            if (loraLog) {
                loraLog.println("TIMESTAMP_MS,PACKET_SIZE,RSSI,SNR,PAYLOAD");
                loraLog.close();
            }
        }
        Serial.println("[SETUP] SD Card Log Files Initialized.");
    } else {
        Serial.println("[WARNING] SD Card failed or not present.");
    }
    digitalWrite(SD_CS_PIN, HIGH);

    digitalWrite(ONBOARD_LED, HIGH); 

    Serial.println("[SETUP] Launching FreeRTOS Flight Tasks...");
    xTaskCreatePinnedToCore(TaskSensorSampling, "SamplingTask", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(TaskRadioAndLogging, "DownlinkTask", 8192, NULL, 1, NULL, 0);

    Serial.println("[SETUP] Setup complete! Flight computer running.");
}

void loop() {
    vTaskDelete(NULL);
}