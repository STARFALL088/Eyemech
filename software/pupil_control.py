#!/usr/bin/env python3
"""
Eyemech — realtime pupil / iris control (MediaPipe → Arduino)
=============================================================

Opens the webcam, runs MediaPipe Face Landmarker (478 points + iris), and
draws for each eye:
  * a larger circle fitted to the eye opening
  * a smaller circle on the iris (used as the practical "pupil" marker)

Angle math matches ``direct_control.py``: eye circle = sphere disk of radius
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
  python pupil_control.py --port /dev/ttyUSB0
  python pupil_control.py --mock                 # print degrees to stdout
  python pupil_control.py --camera 1 --invert-y
  python pupil_control.py --image still.jpg      # single-frame smoke test

Press 'q' to quit live mode.
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
# Match firmware host-degree edges (from range calibrator).
MAX_TURN_X = 14.0
MAX_TURN_Y = 4.5
NUDGE_DEG = 0.5


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


def clamp_degrees(right_deg: float, up_deg: float) -> Tuple[float, float]:
    """PC-side safety clamp (firmware also clamps per axis)."""
    return (
        max(-MAX_TURN_X, min(MAX_TURN_X, right_deg)),
        max(-MAX_TURN_Y, min(MAX_TURN_Y, up_deg)),
    )


def format_angles(th_right: float, th_up: float) -> str:
    """Unit-disk fractions + mapped host degrees."""
    return (
        f"right={th_right:+.4f} ({th_right * MAX_TURN_X:+.1f}°)  "
        f"up={th_up:+.4f} ({th_up * MAX_TURN_Y:+.1f}°)"
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
    """Sends ``right_deg,up_deg\\n`` to Arduino or stdout (--mock)."""

    def __init__(
        self,
        port: Optional[str] = None,
        baud: int = BAUD,
        mock: bool = False,
    ) -> None:
        self.mock = mock
        self.port = port
        self.baud = baud
        self._ser = None
        self._last_send_t = 0.0
        self._last_sent: Optional[Tuple[float, float]] = None
        if mock:
            return
        if not port:
            raise ValueError("port required unless mock=True")
        if serial is None:
            raise SystemExit("pyserial missing — `uv pip install pyserial` or use --mock")
        self._ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2.0)  # Arduino resets on open
        try:
            self._ser.reset_input_buffer()
        except Exception:
            pass
        print(f"[pupil_control] serial open {port} @ {baud}", file=sys.stderr)

    def send(self, right_deg: float, up_deg: float, force: bool = False) -> None:
        now = time.monotonic()
        if not force and (now - self._last_send_t) < SEND_INTERVAL_S:
            return
        sample = (round(right_deg, 2), round(up_deg, 2))
        if not force and sample == self._last_sent:
            return
        line = f"{sample[0]:.2f},{sample[1]:.2f}\n"
        if self.mock or self._ser is None:
            sys.stdout.write(line)
            sys.stdout.flush()
        else:
            try:
                self._ser.write(line.encode("ascii"))
            except Exception as exc:
                print(f"[pupil_control] serial write failed: {exc}", file=sys.stderr)
                self.close(park_home=False)
                raise
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

    def close(self, park_home: bool = True) -> None:
        if park_home:
            self.home()
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None


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
    eye_idx: Sequence[int],
    iris_idx: Sequence[int],
    width: int,
    height: int,
) -> Optional[dict]:
    """Draw eye circle + iris/pupil circle. Returns pixel centers/radii."""
    eye_pts = points_from_indices(landmarks, eye_idx, width, height)
    iris_pts = points_from_indices(landmarks, iris_idx, width, height)

    (ex, ey), er = fit_circle(eye_pts)
    (px, py), pr = fit_circle(iris_pts)

    poly = np.array([(int(round(x)), int(round(y))) for x, y in eye_pts], dtype=np.int32)
    cv2.polylines(frame, [poly], isClosed=True, color=(0, 200, 0), thickness=1)
    cv2.circle(frame, (int(round(ex)), int(round(ey))), int(round(er)), (0, 255, 0), 2)

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
        left = draw_eye(frame_bgr, lms, LEFT_EYE, LEFT_IRIS, w, h)
        right = draw_eye(frame_bgr, lms, RIGHT_EYE, RIGHT_IRIS, w, h)
        if left:
            info.append({"side": "L", **left})
        if right:
            info.append({"side": "R", **right})

        cv2.putText(
            frame_bgr,
            "MediaPipe iris lock",
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
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
        cmd_r, cmd_u = clamp_degrees(th_r * MAX_TURN_X, th_u * MAX_TURN_Y)
        print(f"avg: {format_angles(th_r, th_u)}  -> send {cmd_r:+.2f},{cmd_u:+.2f}")
    else:
        print("no face / iris detected", file=sys.stderr)
    dest = out or os.path.splitext(path)[0] + "_pupil.jpg"
    cv2.imwrite(dest, annotated)
    print(f"saved -> {dest}")
    return 0 if info else 2


class CalibState:
    """Shared zero-offset for arrow nudges + Home (set zero)."""

    def __init__(self) -> None:
        self.zero_r = 0.0
        self.zero_u = 0.0
        self.gaze_r = 0.0
        self.gaze_u = 0.0

    def clamp(self, r: float, u: float) -> Tuple[float, float]:
        return clamp_degrees(r, u)

    def nudge(self, dr: float, du: float) -> Tuple[float, float]:
        self.zero_r, self.zero_u = self.clamp(self.zero_r + dr, self.zero_u + du)
        return self.command()

    def set_home_zero(self) -> Tuple[float, float]:
        abs_r = self.zero_r + self.gaze_r
        abs_u = self.zero_u + self.gaze_u
        self.zero_r, self.zero_u = self.clamp(abs_r, abs_u)
        self.gaze_r = 0.0
        self.gaze_u = 0.0
        return self.command()

    def set_gaze(self, right_deg: float, up_deg: float) -> Tuple[float, float]:
        self.gaze_r, self.gaze_u = self.clamp(right_deg, up_deg)
        return self.command()

    def command(self) -> Tuple[float, float]:
        return self.clamp(self.zero_r + self.gaze_r, self.zero_u + self.gaze_u)


class CalibPanel:
    """Small always-on-top tk bar: ← → ↑ ↓ and Home (set zero)."""

    def __init__(self, calib: CalibState, on_change) -> None:
        if tk is None:
            raise SystemExit("tkinter required for calib buttons (sudo apt install python3-tk)")
        self.calib = calib
        self.on_change = on_change
        self.root = tk.Tk()
        self.root.title("Eyemech calib")
        self.root.resizable(False, False)
        self.root.attributes("-topmost", True)

        bar = ttk.Frame(self.root, padding=8)
        bar.pack()
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
        self._refresh()

    def _refresh(self) -> None:
        self.zero_var.set(f"zero=({self.calib.zero_r:+.1f},{self.calib.zero_u:+.1f})°")

    def _nudge(self, dr: float, du: float) -> None:
        cmd = self.calib.nudge(dr, du)
        self._refresh()
        print(
            f"[calib] nudge → zero=({self.calib.zero_r:+.2f},{self.calib.zero_u:+.2f})°",
            flush=True,
        )
        self.on_change(cmd, force=True)

    def _home(self) -> None:
        cmd = self.calib.set_home_zero()
        self._refresh()
        print(
            f"[calib] Home → set zero=({self.calib.zero_r:+.2f},{self.calib.zero_u:+.2f})°",
            flush=True,
        )
        self.on_change(cmd, force=True)

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
    sender: Optional[FirmwareSender] = None
    if args.mock:
        sender = FirmwareSender(mock=True)
    elif args.port:
        sender = FirmwareSender(port=args.port, baud=args.baud, mock=False)

    calib = CalibState()

    def on_calib_change(cmd: Tuple[float, float], force: bool = False) -> None:
        if sender is not None:
            sender.send(cmd[0], cmd[1], force=force)

    panel: Optional[CalibPanel] = None
    try:
        panel = CalibPanel(calib, on_calib_change)
    except SystemExit as e:
        print(e, file=sys.stderr)
        # Continue without buttons if tk missing
        panel = None

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

    mode = "mock" if args.mock else (args.port or "preview-only")
    print(
        f"[pupil_control] camera={args.camera} serial={mode} "
        f"edges ±{MAX_TURN_X}° X / ±{MAX_TURN_Y}° Y  nudge={NUDGE_DEG}° — press 'q' to quit",
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

                if info:
                    no_face = 0
                    th_r, th_u = average_sphere_angles(
                        info, gain=args.gain, invert_y=args.invert_y
                    )
                    # Unit-disk iris offset → calibrated host degrees.
                    right_deg = th_r * MAX_TURN_X
                    up_deg = th_u * MAX_TURN_Y
                    cmd_r, cmd_u = calib.set_gaze(right_deg, up_deg)

                    now = time.monotonic()
                    if (now - last_print_t) >= SEND_INTERVAL_S:
                        print(
                            f"{format_angles(th_r, th_u)}  "
                            f"zero=({calib.zero_r:+.1f},{calib.zero_u:+.1f})",
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
                    cv2.putText(
                        annotated,
                        f"send {cmd_r:+.1f},{cmd_u:+.1f}  zero=({calib.zero_r:+.1f},{calib.zero_u:+.1f})",
                        (10, 84),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (255, 200, 0),
                        2,
                    )
                    if sender is not None:
                        sender.send(cmd_r, cmd_u)
                else:
                    no_face += 1
                    if sender is not None and no_face > args.lose_timeout:
                        cmd = calib.set_gaze(0.0, 0.0)
                        sender.send(cmd[0], cmd[1])

                cv2.imshow(win, annotated)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
        finally:
            cap.release()
            cv2.destroyAllWindows()
            if panel is not None:
                panel.destroy()
            if sender is not None:
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
    if not args.mock and not args.port:
        print(
            "[pupil_control] no --port; preview only (use --port or --mock to drive firmware)",
            file=sys.stderr,
        )
    return run_camera(args)


if __name__ == "__main__":
    raise SystemExit(main())
