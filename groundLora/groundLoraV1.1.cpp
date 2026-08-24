#include <SPI.h>
#include <LoRa.h>

// ==========================================
//  PIN CONFIGURATION FOR GROUND ESP32
// ==========================================
#define LORA_CS_PIN      4   // D3 (GPIO 4)
#define LORA_RST_PIN     1   // D0 (GPIO 1)
#define LORA_DIO0_PIN    2   // D1 (GPIO 2)

#define LORA_SCK_PIN     7   // D8 (GPIO 7 - Hardware SPI SCK)
#define LORA_MISO_PIN    8   // D9 (GPIO 8 - Hardware SPI MISO)
#define LORA_MOSI_PIN    9   // D10 (GPIO 9 - Hardware SPI MOSI)

#define LORA_FREQUENCY_HZ 433E6 // Must match rocket frequency (433MHz)

void setup() {
    Serial.begin(115200);
    delay(1000); // Allow UART to stabilize

    // Initialize SPI explicitly for standard ESP32 VSPI
    SPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_CS_PIN);

    // Configure LoRa module pins
    LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);

    // Initialize Radio Hardware
    if (!LoRa.begin(LORA_FREQUENCY_HZ)) {
        Serial.println("[ERROR] Receiver LoRa Init Failed! Check SPI wiring and 3.3V power.");
        while (1) {
            delay(100); // Feed watchdog to prevent TG1WDT reboot loop
        }
    }

    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);
    LoRa.enableCrc();

    Serial.println("[INFO] Ground Receiver Ready. Waiting for telemetry...");
}

void loop() {
    int packetSize = LoRa.parsePacket();

    if (packetSize) {
        String incomingPacket = "";

        while (LoRa.available()) {
            incomingPacket += (char)LoRa.read();
        }

        int rssi = LoRa.packetRssi();
        float snr = LoRa.packetSnr();

        if (incomingPacket.endsWith("*")) {
            incomingPacket.remove(incomingPacket.length() - 1);
            incomingPacket += "," + String(rssi) + "," + String(snr, 1) + "*";
        }

        Serial.println(incomingPacket);
    }
}