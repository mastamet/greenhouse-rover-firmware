# Greenhouse Rover Drivetrain Firmware

Precision stepper motor locomotion and motion-profile control firmware developed for an autonomous greenhouse rover using the **ESP32** and **ESP-IDF**.

## 🚀 Features Achieved So Far
* **Dynamic Speed Control:** Translates analog inputs (potentiometer) into frequency-modulated square waves via the ESP32 `LEDC` peripheral.
* **Latching Direction Control:** Utilizes rising-edge detection on a push-button input to toggle travel direction cleanly.
* **Internal Pull-Down Configuration:** Configures internal hardware pull-down resistors to eliminate floating input states safely without external components.
* **Advanced Motion Ramping (Acceleration/Deceleration):** Smoothly ramps motor frequency up and down to prevent mechanical shock, gear slip, or motor stalls.
* **Safe Directional Reversal State Machine:** Implements a zero-crossing decay algorithm that decelerates the motor to `0 Hz` before safely flipping the physical `DIR` pin and accelerating in the reverse direction.

---

## 🛠️ Hardware Wiring Map

| Component Pin | A4988 / Peripheral | ESP32 Pin / Rail | Notes |
| :--- | :--- | :--- | :--- |
| **STEP** | `STEP` | `GPIO 2` | Driven by LEDC PWM clock |
| **DIR** | `DIR` | `GPIO 4` | Digital output for direction |
| **Button** | `BUTTON_PIN` | `GPIO 5` | Configured with internal pull-down |
| **Logic Power** | `VDD` | `3.3V` | Powers driver logic side |
| **Logic Ground** | `GND` | `GND` | Common ground |
| **Safety Jumpers**| `SLEEP` & `RESET` | Jumpered together | Forced permanently HIGH |
| **Coil 1** | `1A` & `1B` | Motor Coil A+ / A- | Stepper winding 1 |
| **Coil 2** | `2A` & `2B` | Motor Coil B+ / B- | Stepper winding 2 |

---

## 💻 Software Stack
* **Framework:** ESP-IDF (v5.x recommended)
* **OS:** FreeRTOS
* **Peripherals:** LEDC (PWM timer/channel), ADC Oneshot, GPIO
