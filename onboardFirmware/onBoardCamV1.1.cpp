#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"

// ==========================================
// INTEGRATED ESP32-S3 CAM + SD PINOUT
// ==========================================
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     17  // SCCB Data line
#define SIOC_GPIO_NUM     18  // SCCB Clock line

#define Y9_GPIO_NUM       11
#define Y8_GPIO_NUM       9
#define Y7_GPIO_NUM       8
#define Y6_GPIO_NUM       6
#define Y5_GPIO_NUM       4
#define Y4_GPIO_NUM       2
#define Y3_GPIO_NUM       1
#define Y2_GPIO_NUM       5
#define VSYNC_GPIO_NUM    13
#define HREF_GPIO_NUM     12
#define PCLK_GPIO_NUM     7

#define LED_INDICATOR_PIN  2 

const int RECORD_TIME_SECONDS = 10;
const int TARGET_FPS = 15;

bool startCamera() {
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
    
    config.xclk_freq_hz = 16000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.grab_mode = CAMERA_GRAB_LATEST;

    if (psramFound()) {
        Serial.printf("[INFO] PSRAM Active (%d KB Free). Allocating PSRAM buffers...\n", ESP.getFreePsram() / 1024);
        config.frame_size = FRAMESIZE_VGA;  // 640x480
        config.jpeg_quality = 12;
        config.fb_count = 2;
        config.fb_location = CAMERA_FB_IN_PSRAM;
    } else {
        Serial.println("[WARNING] PSRAM NOT active! Falling back to DRAM...");
        config.frame_size = FRAMESIZE_QVGA; 
        config.jpeg_quality = 15;
        config.fb_count = 1;
        config.fb_location = CAMERA_FB_IN_DRAM;
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("[ERROR] Camera init failed with code: 0x%x\n", err);
        return false;
    }

    Serial.println("[INFO] Camera initialized successfully!");
    return true;
}

void recordVideo() {
    digitalWrite(LED_INDICATOR_PIN, HIGH);
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
        
        if (!fb || !fb->buf || fb->len == 0) {
            Serial.println("[WARNING] Camera frame empty! Skipping...");
            if (fb) esp_camera_fb_return(fb);
            delay(10);
            continue;
        }

        file.write(fb->buf, fb->len);
        esp_camera_fb_return(fb);
        frameCount++;

        unsigned long elapsed = millis() - frameStart;
        if (elapsed < frameIntervalMs) {
            delay(frameIntervalMs - elapsed);
        }
    }

    file.flush();
    file.close();
    
    digitalWrite(LED_INDICATOR_PIN, LOW);
    Serial.printf("[SUCCESS] Saved %d frames to %s\n", frameCount, filename.c_str());
    Serial.println("[SAFE] Recording complete. Safe to unplug USB.");
}

void setup() {
    pinMode(LED_INDICATOR_PIN, OUTPUT);
    digitalWrite(LED_INDICATOR_PIN, LOW);

    Serial.begin(115200);
    unsigned long timeout = millis() + 3000;
    while (!Serial && millis() < timeout) { delay(10); }
    delay(1000);

    Serial.println("\n--- ESP32-S3 CAM VIDEO RECORDER ---");

    // Initialize Camera first
    if (!startCamera()) {
        Serial.println("[FATAL] Camera init failed. Stopping setup.");
        return;
    }

    // Initialize Onboard SD Card Reader (1-bit SD_MMC mode)
    if (!SD_MMC.begin("/sdcard", true)) {
        Serial.println("[ERROR] SD Card Mount Failed!");
        return;
    }

    Serial.println("[INFO] SD Card Mounted Successfully.");

    // Start Recording
    recordVideo();
}

void loop() {
    delay(1000);
}