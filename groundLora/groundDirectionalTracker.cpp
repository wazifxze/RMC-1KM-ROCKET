#include <SPI.h>
#include <LoRa.h>

// ==========================================
//  PIN CONFIGURATION FOR XIAO ESP32-S3
// ==========================================
#define LORA_CS_PIN      4   // D3 (GPIO 4)
#define LORA_RST_PIN     1   // D0 (GPIO 1)
#define LORA_DIO0_PIN    2   // D1 (GPIO 2)

#define LORA_SCK_PIN     7   // D8 (GPIO 7)
#define LORA_MISO_PIN    8   // D9 (GPIO 8)
#define LORA_MOSI_PIN    9   // D10 (GPIO 9)

#define LORA_FREQUENCY_HZ 433E6 // Must match rocket frequency (433MHz)

// --- SIGNAL MAPPING CONFIGURATION ---
// Expected RSSI Range for 433MHz LoRa
const int RSSI_MIN = -120; // Weak/Noise Floor (0%)
const int RSSI_MAX = -50;  // Strong Signal (100%)

// Exponential Moving Average (EMA) Filter Factor (0.1 = Very Smooth, 1.0 = Raw)
const float EMA_ALPHA = 0.3;
float smoothed_rssi = -120.0;

void setup() {
    // Native USB CDC Initialization
    Serial.begin(115200);
    delay(2000); 

    // Explicit SPI Bus Setup
    SPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_CS_PIN);
    LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);

    if (!LoRa.begin(LORA_FREQUENCY_HZ)) {
        Serial.println("[ERROR] Receiver LoRa Init Failed!");
        while (1) {
            delay(100);
        }
    }

    // Match Rocket Transmitter Parameters
    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);
    LoRa.enableCrc();

    Serial.println("[INFO] Directional Signal Tracker Ready.");
    Serial.println("Format: $SIGNAL,PacketID,RawRSSI,SmoothedRSSI,SNR,SignalQuality%*");
}

void loop() {
    int packetSize = LoRa.parsePacket();

    if (packetSize) {
        String incomingPacket = "";
        while (LoRa.available()) {
            incomingPacket += (char)LoRa.read();
        }

        // 1. Capture Raw Signal Metrics
        int raw_rssi = LoRa.packetRssi();
        float snr = LoRa.packetSnr();

        // 2. Parse Packet ID from incoming payload ($CANSAT,ID,...)
        uint32_t packet_id = 0;
        if (incomingPacket.startsWith("$CANSAT,")) {
            int firstComma = incomingPacket.indexOf(',');
            int secondComma = incomingPacket.indexOf(',', firstComma + 1);
            if (firstComma != -1 && secondComma != -1) {
                packet_id = incomingPacket.substring(firstComma + 1, secondComma).toInt();
            }
        }

        // 3. Apply Exponential Moving Average (EMA) Filter
        smoothed_rssi = (EMA_ALPHA * raw_rssi) + ((1.0 - EMA_ALPHA) * smoothed_rssi);

        // 4. Map RSSI to a 0-100% Signal Quality Percentage
        int signal_quality = map(constrain((int)smoothed_rssi, RSSI_MIN, RSSI_MAX), RSSI_MIN, RSSI_MAX, 0, 100);

        // 5. Output Clean Signal Telemetry Stream to USB Type-C
        Serial.print("$SIGNAL,");
        Serial.print(packet_id);
        Serial.print(",");
        Serial.print(raw_rssi);
        Serial.print(",");
        Serial.print(smoothed_rssi, 1);
        Serial.print(",");
        Serial.print(snr, 1);
        Serial.print(",");
        Serial.print(signal_quality);
        Serial.println("*");
    }
}