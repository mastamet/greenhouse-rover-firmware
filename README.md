# Greenhouse Rover Drivetrain Firmware

Precision stepper motor locomotion, real-time UI menu control, and motion-profile firmware developed for an autonomous greenhouse rover using the **ESP32** and **ESP-IDF**.

---

## 🚀 Features Achieved So Far

- **500-Step Targeted Move Routine:** Includes a dedicated button trigger that executes a precise 500-step motion cycle with an automatic deceleration ramp to 0 Hz before bringing the motor to a smooth, controlled halt.
- **Hardware Step Counting & Odometer Tracking:** Implements real-time pulse tracking/step counting to monitor exact motor displacement, rover positioning, and distance traveled.
- **Non-Blocking User Interface & Button Polling:** Refactored input polling logic into non-blocking FreeRTOS state loops, eliminating input lag (1–3s delays) and enabling smooth, instant menu interactions.
- **SH1107 OLED Display Integration:** Integrated I2C driver support for the SH1107 display controller to render live system status, step count telemetry, and UI menus.
- **Dynamic Speed Control:** Translates analog inputs into frequency-modulated square waves via the ESP32 `LEDC` peripheral.
- **Latching Direction Control:** Utilizes debounced edge detection on push-button inputs to toggle travel direction cleanly.
- **Internal Hardware Pull-Down Configuration:** Configures internal pull-down resistors to eliminate floating input states safely without external components.
- **Advanced Motion Ramping (Acceleration/Deceleration):** Smoothly ramps motor step frequency up and down to prevent mechanical shock, gear slip, or stepper stalls.
- **Safe Directional Reversal State Machine:** Implements a zero-crossing decay algorithm that decelerates the motor to 0 Hz before safely toggling the physical `DIR` pin and accelerating to target speed in reverse.

---

## 🛠️ Hardware Wiring Map

| Component Pin | Peripheral / Driver | ESP32 Pin / Rail | Notes |
| :--- | :--- | :--- | :--- |
| **STEP** | A4988 `STEP` | `GPIO 2` | Driven by LEDC PWM clock |
| **DIR** | A4988 `DIR` | `GPIO 4` | Digital output for direction control |
| **Button 1** | Direction Toggle | `GPIO 5` | Latching direction switch (internal pull-down) |
| **Button 2** | 500-Step Routine | Configured GPIO | Triggers 500-step auto-deceleration move |
| **I2C SDA** | SH1107 OLED | `GPIO 21` | Real-time display data line |
| **I2C SCL** | SH1107 OLED | `GPIO 22` | Real-time display clock line |
| **Logic Power** | `VDD` / Display `VCC` | `3.3V` | Powers driver logic and OLED screen |
| **Logic Ground** | `GND` | `GND` | Common ground rail |
| **Safety Jumpers**| A4988 `SLEEP` & `RESET` | Jumpered together | Tied HIGH to keep driver active |
| **Coil 1** | `1A` & `1B` | Motor Coil A+ / A- | Stepper phase A |
| **Coil 2** | `2A` & `2B` | Motor Coil B+ / B- | Stepper phase B |

---

## 💻 Software Stack

- **Framework:** ESP-IDF (v5.x)
- **OS:** FreeRTOS
- **Peripherals:** LEDC (PWM timer/channel), PCNT / Pulse Counter, I2C Driver, ADC Oneshot, GPIO
- **Display Driver:** SH1107 OLED Library (I2C interface)
