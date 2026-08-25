#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"

// ==========================================
// 1. PIN CONFIGURATION & HARDWARE
// ==========================================
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     15
#define SIOD_GPIO_NUM      4  
#define SIOC_GPIO_NUM      5  

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
#define BUTTON_PIN         0  // Onboard BOOT Button (GPIO 0)

const int TARGET_FPS = 15;
const unsigned long FRAME_INTERVAL_MS = 1000 / TARGET_FPS;

bool isRecording = false;

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
        config.frame_size = FRAMESIZE_VGA;  
        config.jpeg_quality = 12;          
        config.fb_count = 2;
        config.fb_location = CAMERA_FB_IN_PSRAM;
    } else {
        config.frame_size = FRAMESIZE_QVGA; 
        config.jpeg_quality = 15;
        config.fb_count = 1;
        config.fb_location = CAMERA_FB_IN_DRAM;
    }

    esp_err_t err = esp_camera_init(&config);
    return (err == ESP_OK);
}

void recordContinuousVideo() {
    digitalWrite(LED_INDICATOR_PIN, HIGH); // LED ON = Active Recording
    String filename = "/video_" + String(millis()) + ".mjpeg";
    File file = SD_MMC.open(filename, FILE_WRITE);

    if (!file) {
        Serial.println("[ERROR] Failed to open file on SD card!");
        digitalWrite(LED_INDICATOR_PIN, LOW);
        return;
    }

    Serial.printf("[INFO] Continuous Recording Started: %s\n", filename.c_str());
    Serial.println("[INFO] Press BOOT button (GPIO 0) anytime to STOP recording.");

    int frameCount = 0;
    isRecording = true;

    // Wait until button is released if pressed to start
    while (digitalRead(BUTTON_PIN) == LOW) { delay(10); }

    while (isRecording) {
        unsigned long frameStart = millis();

        // Check if BOOT button is pressed to STOP
        if (digitalRead(BUTTON_PIN) == LOW) {
            delay(50); // Debounce
            if (digitalRead(BUTTON_PIN) == LOW) {
                Serial.println("\n[ACTION] Stop button detected!");
                isRecording = false;
                break;
            }
        }

        camera_fb_t * fb = esp_camera_fb_get();
        if (!fb || !fb->buf || fb->len == 0) {
            if (fb) esp_camera_fb_return(fb);
            continue;
        }

        file.write(fb->buf, fb->len);
        esp_camera_fb_return(fb);
        frameCount++;

        unsigned long elapsed = millis() - frameStart;
        if (elapsed < FRAME_INTERVAL_MS) {
            delay(FRAME_INTERVAL_MS - elapsed);
        }
    }

    file.flush();
    file.close(); // Crucial: Finalize AVI/MJPEG container on SD card
    
    digitalWrite(LED_INDICATOR_PIN, LOW); // LED OFF = Safe to disconnect
    Serial.printf("[SUCCESS] Recording stopped! Saved %d frames to %s\n", frameCount, filename.c_str());
    Serial.println("[SAFE] File closed. Safe to power off or unplug SD card.");
}

void setup() {
    pinMode(LED_INDICATOR_PIN, OUTPUT);
    digitalWrite(LED_INDICATOR_PIN, LOW);

    // Set up onboard BOOT button with internal pull-up resistor
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    Serial.begin(115200);
    unsigned long timeout = millis() + 3000;
    while (!Serial && millis() < timeout) { delay(10); }
    delay(1000);

    Serial.println("\n--- CONTINUOUS VIDEO RECORDER ---");

    if (!startCamera()) {
        Serial.println("[FATAL] Camera init failed!");
        return;
    }

    pinMode(38, INPUT_PULLUP);
    pinMode(40, INPUT_PULLUP);
    pinMode(39, INPUT_PULLUP);
    SD_MMC.setPins(39, 38, 40);

    if (!SD_MMC.begin("/sdcard", true)) {
        Serial.println("[ERROR] SD Card Mount Failed!");
        return;
    }

    Serial.println("[INFO] Hardware Ready. Starting recording loop...");
    
    // Begin continuous recording
    recordContinuousVideo();
}

void loop() {
    // Idle state after recording is stopped via button
    delay(1000);
}