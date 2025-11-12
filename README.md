# Wii_Nunchuck_ESP_NOW_TX
Wii Nunchuck ESP-NOW Transmitter based on Xiao ESP32C6 (RiscV)

# ESP-NOW Nunchuk Transmitter (XIAO ESP32-C6)

This project turns a Wii Nunchuk controller into a low-latency, low-power wireless controller using ESP-NOW. It's specifically designed for the **Seeed Studio XIAO ESP32-C6** board, taking advantage of its external antenna and compact form factor.

[Image of XIAO ESP32-C6 Nunchuk controller]

## 🚀 Features

* **Wireless Communication:** Uses ESP-NOW for low-latency, low-power communication without needing to connect to a Wi-Fi network.
* **Specific Hardware:** Optimized for the XIAO ESP32-C6 board.
* **External Antenna:** Configured to use the U.FL (external) antenna connector for maximum range.
* **Custom I2C Pins:** Uses `D4 (GPIO 22)` and `D5 (GPIO 23)` for I2C, avoiding conflicts with the C6's USB serial port.
* **Battery Powered:** Designed to run on a LiPo battery. *(See important power note)*.

---

## 🛠️ Hardware Components

* **Controller:** Seeed Studio XIAO ESP32-C6
* **Input:** A Wii Nunchuk (original or compatible).
* **Connector:** Nunchuk adapter (or wires soldered directly).
* **Power:** 3.7V LiPo Battery (e.g., 500mAh).
* **Critical Component:** 1x **220uF to 1000uF** Electrolytic Capacitor (to stabilize the battery supply).

---

## 🔌 Wiring

| Nunchuk (Wire) | XIAO Pin (Function) | Physical Pin |
| :--- | :--- | :--- |
| **GND** (White) | `GND` | `GND` |
| **+3.3V** (Red) | `3V3` | `3V3` |
| **SDA** (Green) | `GPIO 23` | `D5` |
| **SCL** (Yellow) | `GPIO 22` | `D4` |

### 🔋 Important Note on Battery Power!

The ESP32 draws high current spikes (300-400mA) when transmitting via Wi-Fi/ESP-NOW. A LiPo battery cannot handle these instant spikes, causing a voltage drop (*brownout*) that reboots the chip.

**MANDATORY FIX:** You must solder an **electrolytic capacitor (220uF to 1000uF)** between the `3.3V` and `GND` pins of the XIAO board. This acts as a reservoir to supply power during those spikes.

* Capacitor **Negative (-) leg** ➔ **GND** Pin
* Capacitor **Positive (+) leg** ➔ **3.3V** Pin

---

## ⚙️ Software Setup

### 1. Environment (Arduino IDE)

Ensure you have the Espressif ESP32 board manager installed and select "Seeed Studio XIAO ESP32C6" as your board.

### 2. Libraries

This project requires a custom Nunchuk library (`MinimalNunchuk.h`) to allow manual I2C initialization on specific pins.

* `WiFi.h` (Included with ESP32)
* `esp_now.h` (Included with ESP32)
* `Wire.h` (Included)
* `MinimalNunchuk.h` (You must add this custom library to your project)

---

## 📈 Power Consumption (Benchmark)

The average consumption of this transmitter (sending data ~50 times per second) is approx. **63 mA**.

* **500mAh Battery:** ~8 hours of runtime.

This can be drastically improved by reducing the send frequency (increasing the `delay()`) or by implementing Light Sleep.

---

## 🔧 Troubleshooting

* **TX constantly reboots on battery:** You have not installed the 220uF capacitor. This is the most common problem.
* **"Nunchuk not detected":** Check your wiring. SDA must go to D5 (23) and SCL to D4 (22). Verify 3.3V and GND.
* **Poor range:** Ensure `USE_EXTERNAL_ANTENNA` is defined and you have an antenna connected to the U.FL port. If not, comment out that line to use the internal PCB antenna.
* **Receiver gets nothing:** Verify the MAC address in the TX is correct and that both devices are on the same Wi-Fi channel (Channel 1 by default).

---
---

## Project Source Code

### `Transmitter_Nunchuk.ino` (Main Sketch)

```cpp
/*
 * NUNCHUK TRANSMITTER (XIAO ESP32-C6)
 * ----------------------------------
 * Board: XIAO ESP32-C6
 * I2C: Pins D4 (GPIO 22) & D5 (GPIO 23)
 * Antenna: External (U.FL)
 * Library: "MinimalNunchuk.h" (custom)
 */

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Wire.h>
#include "MinimalNunchuk.h" // <-- Our "mini-library"

// --- I2C Configuration (XIAO ESP32-C6 on pins D4/D5) ---
#define I2C_SDA_PIN 23 // Pin D5 (GPIO 23) green wire
#define I2C_SCL_PIN 22 // Pin D4 (GPIO 22) yellow wire

// --- Antenna Configuration (C6 Specific) ---
#define RF_SWITCH_ENABLE_PIN 3  // GPIO 3: Pin to enable the switch control
#define ANTENNA_SELECT_PIN 14 // GPIO 14: Pin to control the RF switch
// Comment this out to use the INTERNAL (PCB) antenna
#define USE_EXTERNAL_ANTENNA 

// --- ESP-NOW Channel ---
#define ESP_NOW_CHANNEL 1

// --- Receiver's UNIQUE MAC Address ---
uint8_t receiver_mac[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // <-- REPLACE WITH YOUR RECEIVER'S MAC

// --- Data Structure (Must match RX) ---
typedef struct {
  int x; // Joystick X (0-255)
  int y; // Joystick Y (0-255)
  bool c; // Button C
  bool z; // Button Z
  int accX; // Accelerometer X
  int accY; // Accelerometer Y
  int accZ; // Accelerometer Z
} NunchukData;

NunchukData myData; // Packet to send

// --- Objects ---
MinimalNunchuk nunchuk; // <-- Object of our new library
esp_now_peer_info_t peerInfo;

// --- Send Callback ---
void OnDataSent(const wifi_tx_info_t *wifi_info, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("Error sending the packet.");
  }
}
  
void setup() {
  Serial.begin(115200);
  delay(1000); 
  Serial.println("Starting Transmitter (XIAO C6, I2C on D4/D5)...");

  // --- Antenna Selection (C6 Specific) ---
  pinMode(RF_SWITCH_ENABLE_PIN, OUTPUT);
  digitalWrite(RF_SWITCH_ENABLE_PIN, LOW); 
  pinMode(ANTENNA_SELECT_PIN, OUTPUT);
#ifdef USE_EXTERNAL_ANTENNA
  digitalWrite(ANTENNA_SELECT_PIN, HIGH); 
  Serial.println("Using EXTERNAL antenna.");
#else
  digitalWrite(ANTENNA_SELECT_PIN, LOW); 
  Serial.println("Using INTERNAL antenna.");
#endif
  delay(10); 
  
  // --- Initialize I2C on the working pins (22 & 23) ---
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN); 
  
  // --- Initialize Nunchuk ---
  if (!nunchuk.begin()) {
    Serial.println("Error: Nunchuk not detected on D4/D5!");
  } else {
    Serial.println("Nunchuk initialized.");
  }
  
  // Initialize Wi-Fi and ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_wifi_set_channel(ESP_NOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_now_register_send_cb(OnDataSent);
  
  memcpy(peerInfo.peer_addr, receiver_mac, 6);
  peerInfo.channel = ESP_NOW_CHANNEL;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
  Serial.println("Nunchuk Transmitter ready.");
}
  
void loop() {
  
  if (!nunchuk.update()) {
    Serial.println("Nunchuk disconnected!");
    delay(100);
    return; 
  }
    
  // --- Simple send logic ---
  myData.x = nunchuk.joyX;
  myData.y = nunchuk.joyY;
  myData.c = nunchuk.buttonC;
  myData.z = nunchuk.buttonZ;
  myData.accX = nunchuk.accelX;
  myData.accY = nunchuk.accelY;
  myData.accZ = nunchuk.accelZ;
  
  esp_now_send(receiver_mac, (uint8_t *) &myData, sizeof(myData));
  
  delay(20); 
}
