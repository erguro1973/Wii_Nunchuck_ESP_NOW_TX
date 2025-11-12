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
MinimalNunchuk.h (Custom Library)
C++

/*
 * MinimalNunchuk.h
 * Minimal library for reading a Wii Nunchuk.
 * IMPORTANT: This library does NOT call Wire.begin().
 * The user must call Wire.begin(SDA, SCL) BEFORE calling nunchuk.begin().
 */

#ifndef MINIMAL_NUNCHUK_H
#define MINIMAL_NUNCHUK_H

#include <Wire.h>

class MinimalNunchuk {
public:
    // Public variables for direct access
    int joyX, joyY;
    bool buttonC, buttonZ;
    int accelX, accelY, accelZ;

    MinimalNunchuk() {
        // Constructor (empty)
    }

    // Initializes the communication (DOES NOT call Wire.begin())
    bool begin() {
        Wire.beginTransmission(0x52); // Nunchuk I2C address
        Wire.write(0xF0);
        Wire.write(0x55);
        Wire.endTransmission();
        delay(1);
        Wire.beginTransmission(0x52);
        Wire.write(0xFB);
        Wire.write(0x00);
        return Wire.endTransmission() == 0;
    }

    // Reads data from the Nunchuk
    bool update() {
        Wire.requestFrom(0x52, 6); // Request 6 bytes of data
        if (Wire.available() < 6) {
            return false; // Read error
        }

        uint8_t data[6];
        for (int i = 0; i < 6; i++) {
            data[i] = Wire.read();
        }

        // Send the command for the next read
        Wire.beginTransmission(0x52);
        Wire.write(0x00);
        Wire.endTransmission();
        delay(1);

        // Decode the data
        joyX = data[0];
        joyY = data[1];
        accelX = (data[2] << 2) | ((data[5] >> 2) & 0x03);
        accelY = (data[3] << 2) | ((data[5] >> 4) & 0x03);
        accelZ = (data[4] << 2) | ((data[5] >> 6) & 0x03);
        
        buttonC = !(data[5] & 0x02);
        buttonZ = !(data[5] & 0x01);

        return true;
    }
};

#endif
Receiver.ino (Example Receiver Sketch)
C++

/*
 * ESP-NOW RECEIVER
 * ----------------
 * Receives data from the Nunchuk transmitter and prints it.
 * Make sure this receiver's MAC matches the one
 * in the transmitter code.
 */

#include <esp_now.h>
#include <WiFi.h>

// --- Data Structure (MUST BE IDENTICAL to the TX) ---
typedef struct {
  int x; // Joystick X (0-255)
  int y; // Joystick Y (0-255)
  bool c; // Button C
  bool z; // Button Z
  int accX; // Accelerometer X
  int accY; // Accelerometer Y
  int accZ; // Accelerometer Z
} NunchukData;

NunchukData incomingData;

// --- Receive Callback ---
void OnDataRecv(const uint8_t * mac, const uint8_t *incoming_data, int len) {
  if (len == sizeof(NunchukData)) {
    memcpy(&incomingData, incoming_data, sizeof(incomingData));
    
    Serial.print("Joy X: "); Serial.print(incomingData.x);
    Serial.print(" | Joy Y: "); Serial.print(incomingData.y);
    Serial.print(" | C: "); Serial.print(incomingData.c);
    Serial.print(" | Z: "); Serial.println(incomingData.z);
    
    // You can add code here to move servos, motors, etc.
    
  } else {
    Serial.println("Error: Packet of incorrect size.");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Initializing ESP-NOW Receiver...");

  // Print this receiver's MAC address (to put in the TX code)
  Serial.print("My MAC address is: ");
  Serial.println(WiFi.macAddress());

  // Initialize Wi-Fi and ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the receive callback
  esp_now_register_recv_cb(OnDataRecv);
  
  Serial.println("Receiver ready. Awaiting data...");
}

void loop() {
  // Nothing needed here, everything happens in the callback
  delay(1000);
}
