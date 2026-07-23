#include <SPI.h>
#include <LoRa.h>

// ==========================================
//  PIN CONFIGURATION FOR GROUND RECEIVER
//  (Adjust these to match your receiver board)
// ==========================================
#define LORA_CS_PIN      4   // Chip Select (NSS)
#define LORA_RST_PIN     1   // Reset
#define LORA_DIO0_PIN    2   // DIO0 Interrupt

#define LORA_FREQUENCY_HZ 433E6 // Must match rocket frequency (433MHz)

void setup() {
    // 1. Initialize USB Serial to PC/Laptop
    Serial.begin(115200);
    while (!Serial && millis() < 3000); // Wait for USB connection

    // 2. Configure LoRa Module Pins
    LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);

    // 3. Initialize Radio at 433MHz
    if (!LoRa.begin(LORA_FREQUENCY_HZ)) {
        Serial.println("[ERROR] Receiver LoRa Init Failed!");
        while (1); // Halt if hardware connection fails
    }

    // Match radio settings with the rocket transmitter for maximum link reliability
    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);

    // enable CRC check for data intergrity check
    LoRa.enableCrc();
}

void loop() {
    // Check if a LoRa packet has arrived over the air
    int packetSize = LoRa.parsePacket();

    if (packetSize) {
        String incomingPacket = "";

        // Read all bytes out of the radio buffer
        while (LoRa.available()) {
            incomingPacket += (char)LoRa.read();
        }

        int rssi = LoRa.packetRssi(); //read recieved signal strength indicator
        float snr = LoRa.packetSnr(); //read signal noise ratio

        //append rsii and snr to the packet
        if (incomingPacket.endsWith("*")) {
            incomingPacket.remove(incomingPacket.length() - 1); // Strip trailing '*'
            incomingPacket += "," + String(rssi) + "," + String(snr, 1) + "*";
        }

        // Forward the RAW telemetry line over USB directly to the Python script
        Serial.println(incomingPacket);
    }
}