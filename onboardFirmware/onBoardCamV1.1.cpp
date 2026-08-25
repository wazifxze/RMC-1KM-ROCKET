#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"

// ==========================================
// 1. CAMERA PIN CONFIGURATION (Freenove ESP32-S3 CAM)
// ==========================================
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39

#define Y9_GPIO_NUM       38
#define Y8_GPIO_NUM       37
#define Y7_GPIO_NUM       36
#define Y6_GPIO_NUM       35
#define Y5_GPIO_NUM       34
#define Y4_GPIO_NUM       33
#define Y3_GPIO_NUM       21
#define Y2_GPIO_NUM       11
#define VSYNC_GPIO_NUM    22
#define HREF_GPIO_NUM     26
#define PCLK_GPIO_NUM     12

#define LED_INDICATOR_PIN  2 // Built-in LED pin for safe-to-disconnect status

// ==========================================
// 2. VIDEO RECORDING PARAMETERS
// ==========================================
const int RECORD_TIME_SECONDS = 10; // Video recording duration
const int TARGET_FPS = 15;          // Target Frames Per Second

void startCamera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    
    config.frame_size = FRAMESIZE_VGA; // 640x480 resolution
    config.jpeg_quality = 12;          // 0-63 scale
    config.fb_count = 2;               // Double buffering via PSRAM

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("[ERROR] Camera init failed: 0x%x\n", err);
        while (1) { delay(100); }
    }
}

void recordVideo() {
    digitalWrite(LED_INDICATOR_PIN, HIGH); // Turn LED ON during active recording
    String filename = "/video_" + String(millis()) + ".mjpeg";
    File file = SD_MMC.open(filename, FILE_WRITE);

    if (!file) {
        Serial.println("[ERROR] Failed to open file on SD card!");
        digitalWrite(LED_INDICATOR_PIN, LOW);
        return;
    }

    Serial.printf("[INFO] Recording started: %s\n", filename.c_str());

    unsigned long startTime = millis();
    unsigned long endTime = startTime + (RECORD_TIME_SECONDS * 1000);
    int frameCount = 0;
    const unsigned long frameIntervalMs = 1000 / TARGET_FPS;

    while (millis() < endTime) {
        unsigned long frameStart = millis();

        camera_fb_t * fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("[WARNING] Frame capture failed!");
            continue;
        }

        // Write JPEG payload directly to SD file
        file.write(fb->buf, fb->len);
        esp_camera_fb_return(fb);
        frameCount++;

        // Precise frame rate pacing
        unsigned long elapsed = millis() - frameStart;
        if (elapsed < frameIntervalMs) {
            delay(frameIntervalMs - elapsed);
        }
    }

    file.flush();
    file.close(); // Safely closes file stream before power loss
    
    digitalWrite(LED_INDICATOR_PIN, LOW); // Turn LED OFF when safe to unplug USB
    Serial.printf("[SUCCESS] Saved %d frames to %s\n", frameCount, filename.c_str());
    Serial.println("[SAFE] Recording complete. You can safely unplug USB now.");
}

void setup() {
    pinMode(LED_INDICATOR_PIN, OUTPUT);
    digitalWrite(LED_INDICATOR_PIN, LOW);

    // 1. Fix Native USB CDC Serial Startup
    Serial.begin(115200);
    unsigned long timeout = millis() + 3000;
    while (!Serial && millis() < timeout) { delay(10); }
    delay(1000);

    Serial.println("\n--- ESP32-S3 CAM VIDEO RECORDER ---");

    // 2. Fix SD Card Mount Failures (Set explicit pin map & pull-ups for 1-bit SD_MMC mode)
    pinMode(38, INPUT_PULLUP); // CMD
    pinMode(40, INPUT_PULLUP); // D0
    pinMode(39, INPUT_PULLUP); // CLK
    SD_MMC.setPins(39, 38, 40);

    if (!SD_MMC.begin("/sdcard", true)) { // 1-bit mode initialization
        Serial.println("[ERROR] SD Card Mount Failed!");
        Serial.println("[CHECK] Ensure card is FAT32 formatted with MBR partition scheme.");
        return;
    }

    Serial.println("[INFO] SD Card Mounted Successfully.");
    startCamera();
    recordVideo();
}

void loop() {
    delay(1000);
}