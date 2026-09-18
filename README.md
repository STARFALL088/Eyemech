# EyeMech

**EyeMech** is an open-source animatronic eye: pan/tilt eyeball servos plus
eyelid blink servos, driven from a PC over USB-serial.

## Control modes (current)

| Mode | Script | Input |
|------|--------|--------|
| Mouse pad | `software/direct_control.py` | Tkinter circle pad → degrees over serial |
| Iris / pupil | `software/pupil_control.py` | Webcam + MediaPipe iris → same serial protocol |

Firmware: **`software/firmware/firmware.ino`**.

## Protocol (new firmware)

- Baud: **9600**
- Line: `"<right_deg>,<up_deg>\n"` (ASCII), center look = `0,0`
- Firmware clamps each axis with `safe_turn()` to **±MAX_TURN_ANGLE** (15° default)

## Hardware (new firmware pins)

> Do not power servos from the MCU 5V pin. Use a dedicated 5V supply and common GND.

| Name | Pin | Function |
|------|-----|----------|
| `PIN_SERVO_X` | D2 | Horizontal (rightward +) |
| `PIN_SERVO_Y` | D3 | Vertical (upward +) |
| `PIN_BLINK_UL` | D4 | Upper-left eyelid |
| `PIN_BLINK_LL` | D5 | Lower-left eyelid |
| `PIN_BLINK_UR` | D6 | Upper-right eyelid |
| `PIN_BLINK_LR` | D7 | Lower-right eyelid |

## Quick start

```bash
git clone https://github.com/STARFALL088/Eyemech.git
cd Eyemech/software
python3 -m venv .venv && source .venv/bin/activate   # or: uv venv .venv
pip install -r requirements.txt
# desktop preview also needs:  pip install "opencv-python>=4.7,<5"
# Linux Tk UI:  sudo apt install python3-tk
```

1. Flash `software/firmware/firmware.ino` (Arduino IDE / CLI, Servo library).
2. Mouse control:
   ```bash
   python direct_control.py
   # pick port → Connect → move mouse on the pad
   ```
3. Iris / pupil control:
   ```bash
   python pupil_control.py --list-ports
   python pupil_control.py --port /dev/ttyUSB0
   # or without hardware:
   python pupil_control.py --mock
   ```

More detail: [`software/README.md`](software/README.md).
