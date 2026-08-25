#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"

// ==========================================
// OV5640 ESP32-S3 INTEGRATED BOARD PINOUT
// ==========================================
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     15
#define SIOD_GPIO_NUM      4  // SCCB Data
#define SIOC_GPIO_NUM      5  // SCCB Clock

#define Y9_GPIO_NUM       16
#define Y8_GPIO_NUM       17
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       12
#define Y5_GPIO_NUM       10
#define Y4_GPIO_NUM        8
#define Y3_GPIO_NUM        9
#define Y2_GPIO_NUM       11
#define VSYNC_GPIO_NUM     6
#define HREF_GPIO_NUM      7
#define PCLK_GPIO_NUM     13

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
    
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.grab_mode = CAMERA_GRAB_LATEST;

    if (psramFound()) {
        Serial.printf("[INFO] PSRAM Active (%d KB Free). Allocating buffers...\n", ESP.getFreePsram() / 1024);
        config.frame_size = FRAMESIZE_VGA;  
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

    sensor_t * s = esp_camera_sensor_get();
    if (s != NULL) {
        s->set_vflip(s, 0);
        s->set_hmirror(s, 0);
    }

    Serial.println("[INFO] OV5640 Camera initialized successfully!");
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
            Serial.println("[WARNING] Empty frame! Skipping...");
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

    Serial.println("\n--- ESP32-S3 OV5640 VIDEO RECORDER ---");

    // 1. Initialize Camera First
    if (!startCamera()) {
        Serial.println("[FATAL] Camera init failed. Stopping setup.");
        return;
    }

    // 2. Configure Pin Mapping & Internal Pull-ups for Integrated SD Card Slot
    pinMode(38, INPUT_PULLUP); // CMD
    pinMode(40, INPUT_PULLUP); // D0
    pinMode(39, INPUT_PULLUP); // CLK
    SD_MMC.setPins(39, 38, 40);

    // 3. Mount SD Card in 1-bit SD_MMC mode
    if (!SD_MMC.begin("/sdcard", true)) {
        Serial.println("[ERROR] SD Card Mount Failed!");
        Serial.println("[CHECK] Ensure card is formatted to FAT32 with MBR scheme (32GB or smaller).");
        return;
    }

    Serial.println("[INFO] SD Card Mounted Successfully.");

    // 4. Begin Video Recording
    recordVideo();
}

void loop() {
    delay(1000);
}