# Wii_Nunchuck_ESP_NOW_TX_For_SINOBOT_Project
Wii Nunchuck ESP-NOW Transmitter based on Xiao ESP32C6 (RISC-V)

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

# Wii_Nunchuck_ESP_NOW_TX_para_proyecto_SINOBOT
Transmisor ESP-NOW para Nunchuk de Wii basado en Xiao ESP32C6 (RISC-V)

# Transmisor Nunchuk con ESP-NOW (XIAO ESP32-C6)

Este proyecto convierte un mando Nunchuk de Wii en un controlador inalámbrico de baja latencia y bajo consumo usando ESP-NOW. Está diseñado específicamente para la placa **Seeed Studio XIAO ESP32-C6**, aprovechando su antena externa y su formato compacto.

## 🚀 Características

* **Comunicación Inalámbrica:** Usa ESP-NOW para una comunicación de baja latencia y bajo consumo sin necesidad de conectarse a una red Wi-Fi.
* **Control de Modo Dual:** Control estándar con joystick más un **Modo de Gestos** (usando el acelerómetro) que se activa manteniendo presionado el **botón Z**.
* **Hardware Específico:** Optimizado para la placa XIAO ESP32-C6.
* **Antena Externa:** Configurado para usar el conector de antena externa U.FL para un rango máximo.
* **Pines I2C Personalizados:** Usa `D4 (GPIO 22)` y `D5 (GPIO 23)` para I2C, evitando conflictos con el puerto serie USB del C6.
* **Alimentado por Batería:** Diseñado para funcionar con una batería LiPo. *(Ver nota importante sobre la alimentación)*.

---

## 🛠️ Componentes de Hardware

* **Controlador:** Seeed Studio XIAO ESP32-C6
* **Entrada:** Un Nunchuk de Wii (original o compatible).
* **Conector:** Adaptador para Nunchuk (o cables soldados directamente).
* **Alimentación:** Batería LiPo de 3.7V (ej: 500mAh).
* **Componente Crítico:** 1x Condensador Electrolítico de **220uF a 1000uF** (para estabilizar la alimentación de la batería).

---

## 🔌 Cableado

| Nunchuk (Cable) | Pin XIAO (Función) | Pin Físico |
| :--- | :--- | :--- |
| **GND** (Blanco) | `GND` | `GND` |
| **+3.3V** (Rojo) | `3V3` | `3V3` |
| **SDA** (Verde) | `GPIO 23` | `D5` |
| **SCL** (Amarillo)| `GPIO 22` | `D4` |

### 🔋 ¡Nota Importante sobre la Alimentación por Batería!

El ESP32 consume picos de alta corriente (300-400mA) al transmitir con Wi-Fi/ESP-NOW. Una batería LiPo no puede manejar estos picos instantáneos, causando una caída de voltaje (*brownout*) que reinicia el chip.

**SOLUCIÓN OBLIGATORIA:** Debes soldar un **condensador electrolítico (220uF a 1000uF)** entre los pines `3.3V` y `GND` de la placa XIAO. Esto actúa como una reserva para suministrar energía during esos picos.

* Pata **Negativa (-)** del condensador ➔ Pin **GND**
* Pata **Positiva (+)** del condensador ➔ Pin **3.3V**

---

## ⚙️ Configuración de Software

### 1. Entorno (Arduino IDE)

Asegúrate de tener instalado el gestor de placas de Espressif ESP32 y selecciona "Seeed Studio XIAO ESP32C6" como tu placa.

### 2. Librerías

Este proyecto requiere una librería de Nunchuk personalizada (`MinimalNunchuk.h`) para permitir la inicialización manual de I2C en pines específicos.

* `WiFi.h` (Incluida con ESP32)
* `esp_now.h` (Incluida con ESP32)
* `Wire.h` (Incluida)
* `MinimalNunchuk.h` (Debes añadir esta librería personalizada a tu proyecto)

---

## 📈 Consumo de Energía (Prueba)

El consumo medio de este transmisor (enviando datos ~50 veces por segundo) es de aprox. **63 mA**.

* **Batería de 500mAh:** ~8 horas de autonomía.

Esto se puede mejorar drásticamente reduciendo la frecuencia de envío (aumentando el `delay()`) o implementando Light Sleep.

---

## 🔧 Solución de Problemas

* **El TX se reinicia constantemente con batería:** No has instalado el condensador de 220uF. Es el problema más común.
* **"Nunchuk no detectado":** Revisa tu cableado. SDA debe ir a D5 (23) y SCL a D4 (22). Verifica 3.3V y GND.
* **Alcance pobre:** Asegúrate de que `USE_EXTERNAL_ANTENNA` esté definido y tengas una antena conectada al puerto U.FL. Si no, comenta esa línea para usar la antena interna de la PCB.
* **El receptor no recibe nada:** Verifica que la dirección MAC en el TX es correcta y que ambos dispositivos están en el mismo canal Wi-Fi (Canal 1 por defecto).

---
---
