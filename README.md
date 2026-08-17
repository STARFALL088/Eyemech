# EyeMech 👁️🤖

**EyeMech** is an open-source, animatronic eye mechanism designed for robotics, wearable tech, and interactive art installations. It features multi-axis movement (pan, tilt, and realistic eyelid blinking/tracking) driven by micro servos and microcontrollers.

---

## 📌 Features

- **2-DOF Eyeball Movement:** Independent Pitch (up/down) and Yaw (left/right) tracking.
- **Synchronized Eyelid Blinking:** Dual-eyelid linkages capable of realistic blinks and squinting.
- **Modular Hardware:** Optimized for standard FDM 3D printers with zero support requirements on key linkages.
- **Multiple Control Modes:**
  - Joystick / Potentiometer manual override
  - Automated saccade & blink simulation routines
  - Vision-based face tracking (via OpenCV / Serial bridge)

---

## 🛠️ Hardware Requirements

| Component | Specification | Quantity |
| :--- | :--- | :--- |
| **Micro Servos** | SG90 / MG90S (Metal gear recommended) | 4–6 |
| **Microcontroller** | Arduino Nano / ESP32 / Raspberry Pi Pico | 1 |
| **Linkages & Hardware** | M2 / M3 Screws, Ball-joint linkages | 1 Set |
| **Power Supply** | External 5V 2A–3A DC Power Supply | 1 |
| **Structure** | 3D Printed Chassis (PLA or PETG) | 1 Set |

---

## 🔌 Wiring & Pinout

> ⚠️ **Important:** Do not power servos directly from the microcontroller 5V pin. Use a dedicated 5V power supply and share a common Ground (GND).

| Servo Channel | Function | Default Pin (Arduino) |
| :--- | :--- | :--- |
| **Servo 1** | Horizontal Pan (Yaw) | `D9` |
| **Servo 2** | Vertical Tilt (Pitch) | `D10` |
| **Servo 3** | Upper Eyelid | `D11` |
| **Servo 4** | Lower Eyelid | `D12` |

---

## 🚀 Quick Start

### 1. Hardware Setup
1. Print all STL files located in `/hardware/cad_models`.
2. Assemble the gimbal base and attach the pitch/yaw linkages to the servo horns.
3. Center all servos to **90°** before mounting linkage arms to ensure symmetrical travel.

### 2. Firmware Installation
1. Clone the repository:
   ```bash
   git clone [https://github.com/](https://github.com/)<your-username>/EyeMech.git
