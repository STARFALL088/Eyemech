#!/usr/bin/env python3
"""
Eyemech — realtime pupil / iris control (MediaPipe → Arduino)
=============================================================

Opens the webcam, runs MediaPipe Face Landmarker (478 points + iris), and
draws for each eye:
  * a larger white circle fitted to the periocular orbit (halo x3; θ ref)
  * a smaller red circle on the iris (practical "pupil" marker)
  * faint green eyelid contour (visual only; not used for θ)

Angle math matches ``direct_control.py``: white orbit = sphere disk of radius
``r``, pupil center = point; ``s = r * theta`` so ``theta = s / r``
(+right / +up). Angles are printed to the terminal, then degrees are sent
with the same protocol as the mouse pad:

  "<right_deg>,<up_deg>\\n"   @ 9600 baud

Firmware (``firmware/firmware.ino``) clamps per axis
(±14° X, ±4.5° Y from range calibrator).

Honest note on reliability
--------------------------
MediaPipe tracks the **iris** (and eye contour) well — much better than
Haar + dark-blob OpenCV. Glasses, extreme profile, and very low light still
hurt. For animatronic eyes this is usually enough.

Usage
-----
  python pupil_control.py
  python pupil_control.py --port /dev/ttyUSB0   # optional auto-connect
  python pupil_control.py --mock
  python pupil_control.py --camera 1 --invert-y
  python pupil_control.py --image still.jpg

Use Port / Update / Connect in the control window (same as direct_control).
Press 'q' in the webcam window to quit.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
import urllib.request
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

try:
    import tkinter as tk
    from tkinter import ttk
except ImportError:  # pragma: no cover
    tk = None  # type: ignore
    ttk = None  # type: ignore

try:
    import mediapipe as mp
    from mediapipe.tasks import python as mp_python
    from mediapipe.tasks.python import vision as mp_vision
except ImportError as e:  # pragma: no cover
    raise SystemExit(
        "mediapipe is required. From software/:  uv pip install mediapipe\n"
        f"Import error: {e}"
    ) from e

try:
    import serial
    from serial.tools.list_ports import comports
except ImportError:  # pragma: no cover
    serial = None  # type: ignore
    comports = None  # type: ignore


# Face Mesh style indices (Face Landmarker outputs the same 478 topology).
LEFT_EYE: Sequence[int] = (
    33, 7, 163, 144, 145, 153, 154, 155, 133, 173, 157, 158, 159, 160, 161, 246,
)
RIGHT_EYE: Sequence[int] = (
    362, 382, 381, 380, 374, 373, 390, 249, 263, 466, 388, 387, 386, 385, 384, 398,
)
# Periocular “halo x3” — stable socket outer (not eyelid / glasses).
LEFT_ORBIT: Sequence[int] = (
    33, 133,
    226, 31, 228, 229, 230, 231, 232, 233, 244,
    113, 225, 224, 223, 222, 221, 189,
)
RIGHT_ORBIT: Sequence[int] = (
    263, 362,
    446, 261, 448, 449, 450, 451, 452, 453, 464,
    342, 445, 444, 443, 442, 441, 413,
)
LEFT_IRIS: Sequence[int] = (468, 469, 470, 471, 472)
RIGHT_IRIS: Sequence[int] = (473, 474, 475, 476, 477)

MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/"
    "face_landmarker/face_landmarker/float16/1/face_landmarker.task"
)
HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MODEL = os.path.join(HERE, "models", "face_landmarker.task")

BAUD = 9600
SEND_INTERVAL_S = 0.02  # ~50 Hz — same idea as direct_control.py
# Host-degree edges (Arduino / calibrate.py) — full servo travel.
MAX_TURN_X = 14.0
MAX_TURN_Y = 4.5
NUDGE_DEG = 1.0

# Iris θ extremes from eye_calibrate.py vs white periocular orbit (unit-disk s/r).
# Full natural look in each direction maps to ±MAX_TURN_*.
IRIS_MAX_RIGHT = 0.3098
IRIS_MAX_LEFT = 0.3226   # magnitude of left extreme (θ was -0.3226)
IRIS_MAX_UP = 0.2106
IRIS_MAX_DOWN = 0.1200   # magnitude of down extreme (θ was -0.1200)


Point = Tuple[float, float]


def ensure_model(path: str) -> str:
    """Download the Face Landmarker .task file if missing."""
    if os.path.isfile(path) and os.path.getsize(path) > 0:
        return path
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    print(f"[pupil_control] downloading model -> {path}", file=sys.stderr)
    urllib.request.urlretrieve(MODEL_URL, path)
    return path


def landmark_xy(
    landmarks, index: int, width: int, height: int
) -> Point:
    lm = landmarks[index]
    return lm.x * width, lm.y * height


def points_from_indices(
    landmarks, indices: Sequence[int], width: int, height: int
) -> List[Point]:
    return [landmark_xy(landmarks, i, width, height) for i in indices]


def fit_circle(pts: Sequence[Point]) -> Tuple[Point, float]:
    """Circle from point cloud: center = mean, radius = mean distance."""
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    cx = sum(xs) / len(pts)
    cy = sum(ys) / len(pts)
    radii = [math.hypot(x - cx, y - cy) for x, y in pts]
    r = sum(radii) / len(radii)
    return (cx, cy), max(1.0, r)


def pupil_sphere_angles(
    eye_c: Point,
    eye_r: float,
    pupil_c: Point,
    gain: float = 1.0,
    invert_y: bool = False,
) -> Tuple[float, float]:
    """
    Same geometry as ``direct_control.py``: big circle = eye, point = pupil.

    Center of the eye circle is (0, 0). Arc-length angles:
      s = r * theta  =>  theta = s / r
      +theta_right from +x (pupil right of eye center)
      +theta_up    from +y (pupil above eye center; screen y is flipped)

    Returns (theta_right, theta_up) in **radians**. At the eye rim, |theta| ≈ 1.
    """
    if eye_r <= 0:
        return 0.0, 0.0
    sx = (pupil_c[0] - eye_c[0]) * gain   # +right, pixels (= arc length s_x)
    sy = (eye_c[1] - pupil_c[1]) * gain   # +up,    pixels (= arc length s_y)
    th_right = sx / eye_r
    th_up = sy / eye_r
    if invert_y:
        th_up = -th_up
    return th_right, th_up


def average_sphere_angles(
    eyes: Sequence[dict],
    gain: float = 1.0,
    invert_y: bool = False,
) -> Tuple[float, float]:
    """Mean L/R eye sphere angles (radians)."""
    if not eyes:
        return 0.0, 0.0
    rs, us = [], []
    for e in eyes:
        th_r, th_u = pupil_sphere_angles(
            e["eye_c"], e["eye_r"], e["pupil_c"], gain=gain, invert_y=invert_y
        )
        rs.append(th_r)
        us.append(th_u)
    return sum(rs) / len(rs), sum(us) / len(us)


def map_iris_theta_to_degrees(th_r: float, th_u: float) -> Tuple[float, float]:
    """
    Scale measured iris θ (unit disk) so each calibrated extreme hits the
    full host-degree limit for that axis (asymmetric L/R and U/D).
    """
    if th_r >= 0.0:
        n_r = th_r / IRIS_MAX_RIGHT if IRIS_MAX_RIGHT > 1e-6 else 0.0
    else:
        n_r = th_r / IRIS_MAX_LEFT if IRIS_MAX_LEFT > 1e-6 else 0.0
    if th_u >= 0.0:
        n_u = th_u / IRIS_MAX_UP if IRIS_MAX_UP > 1e-6 else 0.0
    else:
        n_u = th_u / IRIS_MAX_DOWN if IRIS_MAX_DOWN > 1e-6 else 0.0
    # Allow slight overshoot from noisy frames, then clamp.
    n_r = max(-1.0, min(1.0, n_r))
    n_u = max(-1.0, min(1.0, n_u))
    return n_r * MAX_TURN_X, n_u * MAX_TURN_Y


def clamp_degrees(right_deg: float, up_deg: float) -> Tuple[float, float]:
    """PC-side safety clamp (firmware also clamps per axis)."""
    return (
        max(-MAX_TURN_X, min(MAX_TURN_X, right_deg)),
        max(-MAX_TURN_Y, min(MAX_TURN_Y, up_deg)),
    )


def format_angles(th_right: float, th_up: float) -> str:
    """Unit-disk θ + mapped host degrees."""
    deg_r, deg_u = map_iris_theta_to_degrees(th_right, th_up)
    return (
        f"θ=({th_right:+.3f},{th_up:+.3f})  "
        f"deg=({deg_r:+.1f},{deg_u:+.1f})"
    )


def create_landmarker(model_path: str) -> mp_vision.FaceLandmarker:
    options = mp_vision.FaceLandmarkerOptions(
        base_options=mp_python.BaseOptions(model_asset_path=model_path),
        running_mode=mp_vision.RunningMode.VIDEO,
        num_faces=1,
        min_face_detection_confidence=0.5,
        min_face_presence_confidence=0.5,
        min_tracking_confidence=0.5,
    )
    return mp_vision.FaceLandmarker.create_from_options(options)


class FirmwareSender:
    """Sends ``right_deg,up_deg\\n`` to Arduino (or stdout in mock mode)."""

    def __init__(self, baud: int = BAUD, mock: bool = False) -> None:
        self.mock = mock
        self.baud = baud
        self.port: Optional[str] = None
        self._ser = None
        self._last_send_t = 0.0
        self._last_sent: Optional[Tuple[float, float]] = None

    @property
    def connected(self) -> bool:
        return self.mock or self._ser is not None

    def connect(self, port: str) -> None:
        if serial is None:
            raise RuntimeError("pyserial missing — `uv pip install pyserial`")
        if self._ser is not None:
            self.disconnect(park_home=False)
        self._ser = serial.Serial(port, self.baud, timeout=0.1)
        self.port = port
        time.sleep(2.0)  # Arduino resets on open
        try:
            self._ser.reset_input_buffer()
        except Exception:
            pass
        self._last_sent = None
        print(f"[pupil_control] serial open {port} @ {self.baud}", file=sys.stderr)

    def send(self, right_deg: float, up_deg: float, force: bool = False) -> None:
        if not self.connected:
            return
        now = time.monotonic()
        if not force and (now - self._last_send_t) < SEND_INTERVAL_S:
            return
        sample = (round(right_deg, 2), round(up_deg, 2))
        if not force and sample == self._last_sent:
            return
        line = f"{sample[0]:.2f},{sample[1]:.2f}\n"
        if self.mock:
            sys.stdout.write(line)
            sys.stdout.flush()
        elif self._ser is not None:
            try:
                self._ser.write(line.encode("ascii"))
                self._ser.flush()
            except Exception as exc:
                print(f"[pupil_control] serial write failed: {exc}", file=sys.stderr)
                self.disconnect(park_home=False)
                return
        self._last_send_t = now
        self._last_sent = sample

    def home(self) -> None:
        """Park mechanism at calibrated zero before a clean shutdown."""
        if self.mock:
            sys.stdout.write("HOME\n")
            sys.stdout.flush()
            return
        if self._ser is None:
            return
        try:
            self._ser.write(b"HOME\n")
            self._ser.flush()
            time.sleep(0.4)
        except Exception as exc:
            print(f"[pupil_control] HOME failed: {exc}", file=sys.stderr)

    def disconnect(self, park_home: bool = True) -> None:
        if park_home and self._ser is not None:
            self.home()
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        self.port = None

    def close(self, park_home: bool = True) -> None:
        self.disconnect(park_home=park_home)


def list_ports() -> None:
    if comports is None:
        print("pyserial not installed; cannot list ports.", file=sys.stderr)
        return
    ports = list(comports())
    if not ports:
        print("No serial ports found.")
        return
    for p in ports:
        print(f"  {p.device:18} {p.description}")


def draw_eye(
    frame: np.ndarray,
    landmarks,
    orbit_idx: Sequence[int],
    eyelid_idx: Sequence[int],
    iris_idx: Sequence[int],
    width: int,
    height: int,
) -> Optional[dict]:
    """
    White periocular orbit = θ reference; red iris unchanged.
    Green eyelid polyline is visual-only.
    """
    orbit_pts = points_from_indices(landmarks, orbit_idx, width, height)
    eyelid_pts = points_from_indices(landmarks, eyelid_idx, width, height)
    iris_pts = points_from_indices(landmarks, iris_idx, width, height)

    (ex, ey), er = fit_circle(orbit_pts)
    (px, py), pr = fit_circle(iris_pts)

    poly = np.array(
        [(int(round(x)), int(round(y))) for x, y in eyelid_pts], dtype=np.int32
    )
    cv2.polylines(frame, [poly], isClosed=True, color=(0, 180, 0), thickness=1)
    cv2.circle(
        frame, (int(round(ex)), int(round(ey))), int(round(er)), (255, 255, 255), 2
    )

    cv2.circle(frame, (int(round(px)), int(round(py))), int(round(pr)), (0, 0, 255), 2)
    cv2.circle(frame, (int(round(px)), int(round(py))), 2, (0, 0, 255), -1)

    return {
        "eye_c": (ex, ey),
        "eye_r": er,
        "pupil_c": (px, py),
        "pupil_r": pr,
        "ox": (px - ex) / er if er else 0.0,
        "oy": (py - ey) / er if er else 0.0,
    }


def process_frame(
    landmarker: mp_vision.FaceLandmarker,
    frame_bgr: np.ndarray,
    timestamp_ms: int,
) -> Tuple[np.ndarray, List[dict]]:
    h, w = frame_bgr.shape[:2]
    frame_bgr = cv2.flip(frame_bgr, 1)
    rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

    result = landmarker.detect_for_video(mp_image, timestamp_ms)
    info: List[dict] = []

    if result.face_landmarks:
        lms = result.face_landmarks[0]
        left = draw_eye(frame_bgr, lms, LEFT_ORBIT, LEFT_EYE, LEFT_IRIS, w, h)
        right = draw_eye(frame_bgr, lms, RIGHT_ORBIT, RIGHT_EYE, RIGHT_IRIS, w, h)
        if left:
            info.append({"side": "L", **left})
        if right:
            info.append({"side": "R", **right})

        cv2.putText(
            frame_bgr,
            "white orbit + iris",
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
    else:
        cv2.putText(
            frame_bgr,
            "no face",
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
        )

    return frame_bgr, info


def run_image(path: str, model_path: str, out: Optional[str]) -> int:
    img = cv2.imread(path)
    if img is None:
        print(f"Cannot read image: {path}", file=sys.stderr)
        return 1
    model_path = ensure_model(model_path)
    with create_landmarker(model_path) as landmarker:
        annotated, info = process_frame(landmarker, img, timestamp_ms=0)
    for e in info:
        th_r, th_u = pupil_sphere_angles(e["eye_c"], e["eye_r"], e["pupil_c"])
        print(
            f"{e['side']}: eye=({e['eye_c'][0]:.1f},{e['eye_c'][1]:.1f}) r={e['eye_r']:.1f}  "
            f"pupil=({e['pupil_c'][0]:.1f},{e['pupil_c'][1]:.1f})  "
            f"{format_angles(th_r, th_u)}"
        )
    if info:
        th_r, th_u = average_sphere_angles(info)
        cmd_r, cmd_u = clamp_degrees(*map_iris_theta_to_degrees(th_r, th_u))
        print(f"avg: {format_angles(th_r, th_u)}  -> send {cmd_r:+.2f},{cmd_u:+.2f}")
    else:
        print("no face / iris detected", file=sys.stderr)
    dest = out or os.path.splitext(path)[0] + "_pupil.jpg"
    cv2.imwrite(dest, annotated)
    print(f"saved -> {dest}")
    return 0 if info else 2


class CalibState:
    """
    Manual pose (arrows) + optional iris gaze once tracking is started.

    Wire command = base + zero + gaze (then clamped).
    Home sets base to the current pose and clears zero/gaze so the UI is
    (0,0) without moving the eye (caller must not send a new command).
    """

    def __init__(self) -> None:
        self.base_r = 0.0
        self.base_u = 0.0
        self.zero_r = 0.0
        self.zero_u = 0.0
        self.gaze_r = 0.0
        self.gaze_u = 0.0
        self.tracking = False

    def clamp(self, r: float, u: float) -> Tuple[float, float]:
        return clamp_degrees(r, u)

    def nudge(self, dr: float, du: float) -> Tuple[float, float]:
        self.gaze_r = 0.0
        self.gaze_u = 0.0
        self.zero_r, self.zero_u = self.clamp(self.zero_r + dr, self.zero_u + du)
        return self.command()

    def set_home_zero(self) -> Tuple[float, float]:
        """Regard current pose as home; coordinates become (0,0); no motion."""
        self.base_r, self.base_u = self.command()
        self.zero_r = 0.0
        self.zero_u = 0.0
        self.gaze_r = 0.0
        self.gaze_u = 0.0
        return self.command()

    def set_gaze(self, right_deg: float, up_deg: float) -> Tuple[float, float]:
        if not self.tracking:
            return self.command()
        self.gaze_r, self.gaze_u = self.clamp(right_deg, up_deg)
        return self.command()

    def command(self) -> Tuple[float, float]:
        return self.clamp(
            self.base_r + self.zero_r + self.gaze_r,
            self.base_u + self.zero_u + self.gaze_u,
        )


class ControlPanel:
    """Tk control strip: Port / Update / Connect + calib arrows + Home."""

    def __init__(
        self,
        calib: CalibState,
        sender: FirmwareSender,
        initial_port: Optional[str] = None,
    ) -> None:
        if tk is None:
            raise SystemExit(
                "tkinter required for the control panel (sudo apt install python3-tk)"
            )
        self.calib = calib
        self.sender = sender
        self.root = tk.Tk()
        self.root.title("Eyemech pupil control")
        self.root.resizable(False, False)
        self.root.attributes("-topmost", True)

        # --- serial chrome (same idea as direct_control) ---
        top = ttk.Frame(self.root, padding=(8, 6))
        top.pack(fill=tk.X)

        ttk.Label(top, text="Port:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(
            top, textvariable=self.port_var, width=16, state="readonly"
        )
        self.port_box.pack(side=tk.LEFT, padx=(4, 6))

        ttk.Button(top, text="Update", command=self._refresh_ports).pack(
            side=tk.LEFT, padx=(0, 6)
        )
        self.connect_btn = ttk.Button(
            top, text="Connect", command=self._toggle_connect
        )
        self.connect_btn.pack(side=tk.LEFT, padx=(0, 6))

        self.start_btn = ttk.Button(
            top, text="Start", command=self._toggle_tracking
        )
        self.start_btn.pack(side=tk.LEFT, padx=(0, 8))

        self.status_var = tk.StringVar(value="Not Connected | arrows only")
        ttk.Label(top, textvariable=self.status_var).pack(side=tk.LEFT)

        # --- calib nudges ---
        bar = ttk.Frame(self.root, padding=(8, 4))
        bar.pack(fill=tk.X)
        ttk.Label(bar, text="Calib:").pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(bar, text="←", width=3, command=lambda: self._nudge(-NUDGE_DEG, 0)).pack(
            side=tk.LEFT, padx=1
        )
        ttk.Button(bar, text="→", width=3, command=lambda: self._nudge(+NUDGE_DEG, 0)).pack(
            side=tk.LEFT, padx=1
        )
        ttk.Button(bar, text="↑", width=3, command=lambda: self._nudge(0, +NUDGE_DEG)).pack(
            side=tk.LEFT, padx=1
        )
        ttk.Button(bar, text="↓", width=3, command=lambda: self._nudge(0, -NUDGE_DEG)).pack(
            side=tk.LEFT, padx=1
        )
        ttk.Button(bar, text="Home", command=self._home).pack(side=tk.LEFT, padx=(8, 4))
        self.zero_var = tk.StringVar(value="zero=(+0.0,+0.0)°")
        ttk.Label(bar, textvariable=self.zero_var).pack(side=tk.LEFT)

        self._refresh_ports()
        if initial_port:
            self.port_var.set(initial_port)
        elif self.sender.mock:
            self.status_var.set("Mock (stdout)")
            self.connect_btn.configure(state=tk.DISABLED)

        self._refresh_zero()

        # Keyboard arrows (same as buttons) — focus this window first.
        self.root.bind("<Left>", lambda e: self._nudge(-NUDGE_DEG, 0))
        self.root.bind("<Right>", lambda e: self._nudge(+NUDGE_DEG, 0))
        self.root.bind("<Up>", lambda e: self._nudge(0, +NUDGE_DEG))
        self.root.bind("<Down>", lambda e: self._nudge(0, -NUDGE_DEG))
        self.root.focus_force()

    def _refresh_ports(self) -> None:
        ports = []
        if comports is not None:
            ports = [p.device for p in comports()]
        self.port_box["values"] = ports
        cur = self.port_var.get()
        if ports and cur not in ports:
            self.port_var.set(ports[0])
        elif not ports and not self.sender.mock:
            self.port_var.set("")

    def _status_text(self) -> str:
        link = "Connected" if self.sender.connected else "Not Connected"
        mode = "TRACKING" if self.calib.tracking else "arrows only"
        return f"{link} | {mode}"

    def _refresh_status(self) -> None:
        self.status_var.set(self._status_text())
        self.start_btn.configure(
            text="Stop" if self.calib.tracking else "Start"
        )

    def _toggle_tracking(self) -> None:
        self.calib.tracking = not self.calib.tracking
        if self.calib.tracking:
            print(
                "[pupil_control] Start — iris tracking drives the eye",
                file=sys.stderr,
            )
        else:
            # Freeze on current zero; clear live gaze so arrows stay clean.
            self.calib.gaze_r = 0.0
            self.calib.gaze_u = 0.0
            print(
                "[pupil_control] Stop — arrows only (webcam does not drive)",
                file=sys.stderr,
            )
            if self.sender.connected:
                cmd = self.calib.command()
                self.sender.send(cmd[0], cmd[1], force=True)
        self._refresh_status()

    def _toggle_connect(self) -> None:
        if self.sender.mock:
            return
        if self.sender.connected:
            self.sender.disconnect(park_home=True)
            self.calib.tracking = False
            self.connect_btn.configure(text="Connect")
            self._refresh_status()
            print("[pupil_control] disconnected", file=sys.stderr)
            return

        port = self.port_var.get()
        if not port:
            self.status_var.set("No Port")
            return
        self.status_var.set("Connecting…")
        self.root.update_idletasks()
        try:
            self.sender.connect(port)
        except Exception as exc:
            self.status_var.set("Open failed")
            print(f"[pupil_control] serial open failed: {exc}", file=sys.stderr)
            return

        self.connect_btn.configure(text="Disconnect")
        self.calib.tracking = False  # arrows-only until Start
        self._refresh_status()
        cmd = self.calib.command()
        self.sender.send(cmd[0], cmd[1], force=True)
        print(
            "[pupil_control] connected — use arrows now; click Start for iris follow",
            file=sys.stderr,
        )

    def _refresh_zero(self) -> None:
        self.zero_var.set(
            f"rel=({self.calib.zero_r:+.1f},{self.calib.zero_u:+.1f})°  "
            f"base=({self.calib.base_r:+.1f},{self.calib.base_u:+.1f})°"
        )

    def _nudge(self, dr: float, du: float) -> None:
        if not self.sender.connected:
            self.status_var.set("Connect first!")
            print("[calib] nudge ignored — not connected", file=sys.stderr)
            return
        cmd = self.calib.nudge(dr, du)
        self._refresh_zero()
        print(f"[calib] nudge → cmd=({cmd[0]:+.2f},{cmd[1]:+.2f})°", flush=True)
        self.sender.send(cmd[0], cmd[1], force=True)

    def _home(self) -> None:
        # Redefine origin only — do not send (eye must not move).
        before = self.calib.command()
        after = self.calib.set_home_zero()
        self._refresh_zero()
        print(
            f"[calib] Home — pose unchanged "
            f"({before[0]:+.2f},{before[1]:+.2f}) → coords (0,0) "
            f"[base=({self.calib.base_r:+.2f},{self.calib.base_u:+.2f})]",
            flush=True,
        )
        assert abs(before[0] - after[0]) < 1e-6 and abs(before[1] - after[1]) < 1e-6

    def pump(self) -> None:
        try:
            self.root.update_idletasks()
            self.root.update()
        except tk.TclError:
            pass

    def destroy(self) -> None:
        try:
            self.root.destroy()
        except Exception:
            pass


def run_camera(args: argparse.Namespace) -> int:
    model_path = ensure_model(args.model)
    sender = FirmwareSender(baud=args.baud, mock=bool(args.mock))
    calib = CalibState()

    panel: Optional[ControlPanel] = None
    try:
        panel = ControlPanel(calib, sender, initial_port=args.port)
    except SystemExit as e:
        print(e, file=sys.stderr)
        panel = None

    # Optional auto-connect from CLI --port (panel Connect still works either way).
    if args.port and not args.mock and panel is not None:
        panel.port_var.set(args.port)
        try:
            sender.connect(args.port)
            panel.connect_btn.configure(text="Disconnect")
            panel.calib.tracking = False
            panel._refresh_status()
        except Exception as exc:
            print(f"[pupil_control] auto-connect failed: {exc}", file=sys.stderr)

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"Cannot open camera {args.camera}", file=sys.stderr)
        if panel:
            panel.destroy()
        return 1
    if args.width > 0:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    if args.height > 0:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    print(
        f"[pupil_control] camera={args.camera}  "
        f"iris θ→deg (R{IRIS_MAX_RIGHT}/L{IRIS_MAX_LEFT}/U{IRIS_MAX_UP}/D{IRIS_MAX_DOWN}) "
        f"→ ±{MAX_TURN_X}°/±{MAX_TURN_Y}° — "
        f"Connect → arrows; Start → iris follow; 'q' quits",
        file=sys.stderr,
    )

    win = "Eyemech pupil_control"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE | cv2.WINDOW_GUI_NORMAL)

    t0 = time.time()
    no_face = 0
    last_print_t = 0.0
    with create_landmarker(model_path) as landmarker:
        try:
            while True:
                if panel is not None:
                    panel.pump()

                ok, frame = cap.read()
                if not ok:
                    continue
                ts = int((time.time() - t0) * 1000)
                annotated, info = process_frame(landmarker, frame, ts)

                cmd_r, cmd_u = calib.command()
                th_r = th_u = 0.0

                if info:
                    no_face = 0
                    th_r, th_u = average_sphere_angles(
                        info, gain=args.gain, invert_y=args.invert_y
                    )
                    right_deg, up_deg = map_iris_theta_to_degrees(th_r, th_u)
                    # Only updates gaze when tracking==True (after Start).
                    cmd_r, cmd_u = calib.set_gaze(right_deg, up_deg)
                else:
                    no_face += 1
                    if (
                        calib.tracking
                        and sender.connected
                        and no_face > args.lose_timeout
                    ):
                        cmd_r, cmd_u = calib.set_gaze(0.0, 0.0)

                now = time.monotonic()
                if (now - last_print_t) >= SEND_INTERVAL_S:
                    link = "TX" if sender.connected else "no-serial"
                    mode = "TRACK" if calib.tracking else "ARROWS"
                    print(
                        f"{format_angles(th_r, th_u)}  "
                        f"cmd=({cmd_r:+.1f},{cmd_u:+.1f})  [{link}/{mode}]",
                        flush=True,
                    )
                    last_print_t = now

                cv2.putText(
                    annotated,
                    format_angles(th_r, th_u),
                    (10, 56),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 255),
                    2,
                )
                if sender.connected:
                    mode = "TRACKING" if calib.tracking else "ARROWS ONLY"
                    color = (0, 255, 0) if calib.tracking else (255, 200, 0)
                else:
                    mode = "Not Connected"
                    color = (0, 0, 255)
                cv2.putText(
                    annotated,
                    f"send {cmd_r:+.1f},{cmd_u:+.1f}  {mode}",
                    (10, 84),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    color,
                    2,
                )

                # Iris serial only after Start; arrows always send via button handlers.
                if sender.connected and calib.tracking:
                    sender.send(cmd_r, cmd_u)

                cv2.imshow(win, annotated)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
        finally:
            cap.release()
            cv2.destroyAllWindows()
            if panel is not None:
                panel.destroy()
            sender.close(park_home=True)
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="MediaPipe iris/pupil overlay + firmware serial (same as direct_control)."
    )
    p.add_argument("--camera", type=int, default=0, help="Webcam index.")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--image", help="Still image instead of webcam.")
    p.add_argument("--out", help="Annotated output path for --image.")
    p.add_argument(
        "--model",
        default=DEFAULT_MODEL,
        help="Path to face_landmarker.task (auto-downloaded if missing).",
    )
    p.add_argument(
        "--port",
        default=None,
        help="Serial port to Arduino (e.g. /dev/ttyUSB0, COM5).",
    )
    p.add_argument("--baud", type=int, default=BAUD, help="Must match firmware (9600).")
    p.add_argument(
        "--mock",
        action="store_true",
        help="Print right_deg,up_deg lines to stdout instead of serial.",
    )
    p.add_argument("--gain", type=float, default=1.0, help="Amplify iris offsets.")
    p.add_argument(
        "--invert-y",
        action="store_true",
        help="Flip vertical axis if the robot looks the wrong way.",
    )
    p.add_argument(
        "--lose-timeout",
        type=int,
        default=15,
        help="Frames without face before commanding center (0,0).",
    )
    p.add_argument("--list-ports", action="store_true", help="List serial ports and exit.")
    return p


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.list_ports:
        list_ports()
        return 0
    if args.image:
        return run_image(args.image, args.model, args.out)
    return run_camera(args)


if __name__ == "__main__":
    raise SystemExit(main())
