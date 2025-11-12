# Wii_Nunchuck_ESP_NOW_TX
Wii Nunchuck ESP-NOW Transmitter based on Xiao ESP32C6 (RiscV)

# ESP-NOW Nunchuk Transmitter (XIAO ESP32-C6)

This project turns a Wii Nunchuk controller into a low-latency, low-power wireless controller using ESP-NOW. It's specifically designed for the **Seeed Studio XIAO ESP32-C6** board, taking advantage of its external antenna and compact form factor.



## 🚀 Features

* **Wireless Communication:** Uses ESP-NOW for low-latency, low-power communication without needing to connect to a Wi-Fi network.
* **Dual-Mode Control:** Standard joystick control plus a **Gesture Mode** (using the accelerometer) activated by holding the **Z button**.
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
