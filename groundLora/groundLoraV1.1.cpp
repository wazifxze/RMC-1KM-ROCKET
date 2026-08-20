#include <SPI.h>
#include <LoRa.h>

// ==========================================
//  PIN CONFIGURATION FOR GROUND ESP32
// ==========================================
#define LORA_CS_PIN      5   // Chip Select (NSS)
#define LORA_RST_PIN     14  // Hardware Reset (Avoid GPIO 1)
#define LORA_DIO0_PIN    2   // DIO0 Interrupt

#define LORA_SCK_PIN     18  // SPI Clock
#define LORA_MISO_PIN    19  // SPI Master In Slave Out
#define LORA_MOSI_PIN    23  // SPI Master Out Slave In

#define LORA_FREQUENCY_HZ 433E6 // Must match rocket frequency (433MHz)

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    // Explicitly initialize SPI pins for Standard ESP32 VSPI
    SPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_CS_PIN);

    // Configure LoRa module pins
    LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);

    // Initialize radio hardware
    if (!LoRa.begin(LORA_FREQUENCY_HZ)) {
        Serial.println("[ERROR] Receiver LoRa Init Failed!");
        while (1); 
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