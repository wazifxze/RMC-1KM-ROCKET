#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"

// ==========================================
// 1. CAMERA PIN CONFIGURATION (ESP32-S3 CAM)
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

// ==========================================
// 2. VIDEO RECORDING PARAMETERS
// ==========================================
const int RECORD_TIME_SECONDS = 10; // Video duration
const int TARGET_FPS = 15;          // Target Frames Per Second

// AVI Header Struct Helper
struct AviHeader {
    uint32_t fps;
    uint32_t width;
    uint32_t height;
};

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
    
    // Frame size selection
    config.frame_size = FRAMESIZE_VGA; // 640x480 resolution
    config.jpeg_quality = 12;          // 0-63 (lower means higher quality)
    config.fb_count = 2;               // Double buffering enabled via PSRAM

    // Initialize Camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("[ERROR] Camera init failed with error 0x%x\n", err);
        while (1);
    }
}

void recordVideo() {
    String filename = "/video_" + String(millis()) + ".mjpeg";
    File file = SD_MMC.open(filename, FILE_WRITE);

    if (!file) {
        Serial.println("[ERROR] Failed to open file on SD card for writing!");
        return;
    }

    Serial.printf("[INFO] Recording started: %s\n", filename.c_str());

    unsigned long startTime = millis();
    unsigned long endTime = startTime + (RECORD_TIME_SECONDS * 1000);
    int frameCount = 0;

    while (millis() < endTime) {
        camera_fb_t * fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("[WARNING] Camera capture failed!");
            continue;
        }

        // Write raw JPEG buffer directly into MJPEG video file stream
        file.write(fb->buf, fb->len);
        esp_camera_fb_return(fb);

        frameCount++;
        delay(1000 / TARGET_FPS); // Rate-limiting delay
    }

    file.close();
    Serial.printf("[SUCCESS] Recording complete! Saved %d frames to %s\n", frameCount, filename.c_str());
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    // Initialize onboard SD Card via 1-bit SD_MMC mode
    if (!SD_MMC.begin("/sdcard", true)) { // true = 1-bit mode (frees GPIOs)
        Serial.println("[ERROR] SD Card Mount Failed!");
        return;
    }

    Serial.println("[INFO] SD Card Mounted Successfully.");
    startCamera();

    // Trigger video recording
    recordVideo();
}

void loop() {
    // Idle state after recording completion
    delay(1000);
}