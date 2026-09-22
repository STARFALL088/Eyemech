# EyeMech 👁️

**EyeMech** is an open-source animatronic eye: servo-driven eyeballs with
independent eyelids, driven from a PC over USB-serial.

Move it with your mouse, or let it mirror your own gaze via webcam +
MediaPipe iris tracking — including realistic blink / wink from your eyelids.

Built by students at KUET with
[Hardware Acceleration Club of KUET (HACK)](https://github.com/STARFALL088/Eyemech).

## Features

- 👀 Pan/tilt eyeballs + 4x eyelid servos (independent left/right blink & wink)
- 🖱️ Mouse-pad control via Tkinter circle pad (`direct_control.py`)
- 👁️ Iris tracking via webcam + MediaPipe Face Landmarker (`pupil_control.py`)
- 📏 Arrow-only calibrators for mechanism limits (`calibrate.py`) and iris
  travel (`eye_calibrate.py`)
- 🛡️ Calibrated safe ranges — firmware + PC both clamp per axis
- 🏠 `HOME` park command on clean disconnect / app exit
- 📦 Fully open-source: Python + Arduino, auto-downloaded MediaPipe model

## How it works

```
Mouse pad ─┐
           ├─> host degrees (right, up) + lid openness ──serial 9600──> Arduino ──> 6 servos
Webcam + MediaPipe iris + EAR ─┘
```

Angle math is shared everywhere: unit-disk `θ = s / r` (pupil offset / eye
radius). `θ = ±1` at the rim maps to full host-degree limits. See
[`software/README.md`](software/README.md) for details.

## Control modes

| Mode | Script | Input | Output |
|------|--------|-------|--------|
| Mouse pad | `software/direct_control.py` | Tkinter circle pad + arrows/Home | gaze + lids open |
| Iris / pupil + blink | `software/pupil_control.py` | Webcam + MediaPipe iris + EAR | gaze + L/R lid openness |
| Mechanism calibrator | `software/calibrate.py` | Arrows only, no pad | `calib_limits_*.txt` |
| Iris calibrator | `software/eye_calibrate.py` | Webcam only, no Arduino | `eye_calib_limits_*.txt` |

> `pupil_control.py` starts in arrows-only mode. Click **Connect**, then
> **Start** to enable iris follow. Lids follow the webcam whenever connected.

## Protocol (firmware `software/firmware/firmware.ino`)

- Baud: **9600**
- Gaze: `"<right_deg>,<up_deg>\n"` — `0,0` = look straight, lids unchanged
- Gaze + lids: `"<right_deg>,<up_deg>,<left_open>,<right_open>\n"`
  - `left_open` / `right_open` ∈ `[0,1]`, person's L/R, `1` = open
- Park: `"HOME\n"` — mechanical park (experimental `v=0`), sent on clean
  Disconnect / app exit
- Host limits (from mechanism calibration): **±14.0° X**, **±4.5° Y**
- Firmware maps host degrees onto calibrated servo ranges and clamps per axis
  with `safe_turn_axis()`

## Hardware (matches wired firmware)

> ⚠️ Do not power servos from the MCU 5V pin. Use a dedicated 5V supply with
> common GND.

| Name | Pin | Function | Calibrated range |
|------|-----|----------|------------------|
| `PIN_SERVO_X` | D3 | Horizontal (rightward +) | 0–80, mid 40 |
| `PIN_SERVO_Y` | D5 | Vertical (upward +) | 60–120, mid 90 |
| `PIN_BLINK_UL` | D6 | Upper-left eyelid | open 130 / closed 180 |
| `PIN_BLINK_LL` | D9 | Lower-left eyelid | open 50 / closed 0 |
| `PIN_BLINK_UR` | D10 | Upper-right eyelid | open 50 / closed 0 |
| `PIN_BLINK_LR` | D11 | Lower-right eyelid | open 130 / closed 180 |

On boot the firmware **assumes the mechanism is already at HOME** and does
not drive there. It prints `Eyemech firmware ready (gaze+lids)`.

Requires Arduino + Servo library.

## Quick start

```bash
git clone https://github.com/STARFALL088/Eyemech.git
cd Eyemech/software
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
# Linux Tk UI:
sudo apt install python3-tk
# Desktop preview with OpenCV windows needs the GUI build:
# pip install "opencv-python>=4.7,<5"  (instead of -headless)
```

1. Flash `software/firmware/firmware.ino` (Arduino IDE / CLI + Servo library).
2. Mouse control:
   ```bash
   python direct_control.py
   # pick port → Connect → move on the pad, arrows nudge, Home re-zeroes
   ```
3. Iris / pupil + blink control:
   ```bash
   python pupil_control.py --list-ports
   python pupil_control.py --port /dev/ttyUSB0
   # or without hardware:
   python pupil_control.py --mock
   # still image test:
   python pupil_control.py --image test_face.jpg
   ```
   First run auto-downloads `models/face_landmarker.task`. In the app:
   **Connect → arrows work + lids follow → Start for iris follow → `q` quits.**
4. (Optional) Calibrate your hardware / eyes:
   ```bash
   python calibrate.py      # Arduino connected, arrows to safe edges
   python eye_calibrate.py  # webcam only, look hard L/R/U/D
   ```

More flags and math: [`software/README.md`](software/README.md).

## Project structure

```
Eyemech/
├── software/
│   ├── direct_control.py   # Tkinter mouse-pad → serial gaze + open lids
│   ├── pupil_control.py    # MediaPipe iris + EAR → serial gaze + lids
│   ├── calibrate.py        # Arrow-only mechanism range recorder
│   ├── eye_calibrate.py    # Webcam-only iris θ range recorder
│   ├── eye_calib_limits.txt
│   ├── firmware/
│   │   └── firmware.ino    # 6-servo Arduino (X/Y + 4 lids, HOME, clamp)
│   ├── models/             # face_landmarker.task (auto-downloaded)
│   ├── requirements.txt
│   └── README.md
├── README.md
└── .gitignore
```

Roadmap: `hardware/` (schematics/wiring), `mechanical/` (3D models),
`docs/`, `images/`, `tests/` — contributions welcome.

## Calibration notes

- Mechanism extremes live in firmware as `MAX_TURN_X = 14.0`, `MAX_TURN_Y = 4.5`
  (from `calibrate.py` sweeps). Pad rim / full iris look = those edges.
- Iris `θ` extremes live in `pupil_control.py` as
  `R 0.3098 / L 0.3226 / U 0.2106 / D 0.1200` (from `eye_calib_limits.txt`).
  Full natural look in each direction hits the host limits above.
- Looking down narrows EAR, so `pupil_control.py` adds a down-gaze lid bias
  (up to +0.35 openness at full down) to avoid false blinks.
- No face for > `--lose-timeout` frames (default 15) while tracking → commands
  `0,0` with lids open.

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `pyserial missing` / `No Port` | `pip install -r requirements.txt`, check USB cable + permissions (`dialout` group on Linux) |
| `tkinter required` | `sudo apt install python3-tk` |
| Eyes move mirrored vertically | Run with `--invert-y` |
| Blinks feel sticky / twitchy | Tune `EAR_CLOSED` / `EAR_OPEN` / `LID_SMOOTH` in `pupil_control.py` |
| Servos jitter / brown-out | Dedicated 5V supply + common GND, never MCU 5V pin |
| Glasses / low light / profile | Expected MediaPipe degradation — use frontal, well-lit face |

## Team

Built by SR Prantor, Mohosin Alam, Khalid Bin Atik, Al Fahad with guidance
from seniors at Hardware Acceleration Club of KUET (HACK).

Contributions, issues, and forks welcome — star the repo if you build your own!
