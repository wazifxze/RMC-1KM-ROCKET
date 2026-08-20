#include <SPI.h>
#include <LoRa.h>

// ==========================================
//  PIN CONFIGURATION FOR GROUND ESP32
// ==========================================
#define LORA_CS_PIN      5   // Chip Select (NSS)
#define LORA_RST_PIN     14  // Hardware Reset
#define LORA_DIO0_PIN    26  // DIO0 Interrupt (Moved off strapping GPIO 2)

#define LORA_SCK_PIN     18  // VSPI Clock
#define LORA_MISO_PIN    19  // VSPI MISO
#define LORA_MOSI_PIN    23  // VSPI MOSI

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