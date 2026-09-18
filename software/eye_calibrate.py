#!/usr/bin/env python3
"""
Eyemech — eye / iris range calibrator (webcam only, no Arduino)
===============================================================

Opens the webcam, runs the same MediaPipe geometry as pupil_control
(white periocular orbit + red iris; theta = s/r on the unit disk), and
records peak offsets:

  max_right, max_left, max_up, max_down   (unit-disk fractions, typically |x|<1)

Look hard L/R/U/D so the extremes update. Quit and share the .txt so we can
scale pupil tracking to your real iris travel.

Usage
-----
  python eye_calibrate.py
  python eye_calibrate.py --camera 1

Press 'q' in the video window to quit. Optional Reset clears extremes.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from datetime import datetime
from typing import Optional

import cv2

try:
    import tkinter as tk
    from tkinter import ttk
except ImportError:  # pragma: no cover
    tk = None  # type: ignore
    ttk = None  # type: ignore

from pupil_control import (
    DEFAULT_MODEL,
    average_sphere_angles,
    create_landmarker,
    ensure_model,
    format_angles,
    process_frame,
)

HERE = os.path.dirname(os.path.abspath(__file__))


class ThetaExtremeLog:
    """Peak unit-disk iris offsets (theta = s/r vs white orbit)."""

    def __init__(self, path: str) -> None:
        self.path = path
        self.max_right = 0.0
        self.max_left = 0.0
        self.max_up = 0.0
        self.max_down = 0.0
        self._write(initial=True)

    def reset(self) -> None:
        self.max_right = self.max_left = self.max_up = self.max_down = 0.0
        self._write(initial=False)
        print(f"[eye_calibrate] extremes reset → {self.summary()}", flush=True)

    def observe(self, th_r: float, th_u: float) -> bool:
        changed = False
        if th_r > self.max_right:
            self.max_right = th_r
            changed = True
        if th_r < self.max_left:
            self.max_left = th_r
            changed = True
        if th_u > self.max_up:
            self.max_up = th_u
            changed = True
        if th_u < self.max_down:
            self.max_down = th_u
            changed = True
        if changed:
            self._write(initial=False)
        return changed

    def summary(self) -> str:
        return (
            f"R={self.max_right:+.4f} L={self.max_left:+.4f} "
            f"U={self.max_up:+.4f} D={self.max_down:+.4f}"
        )

    def _write(self, initial: bool) -> None:
        stamp = datetime.now().isoformat(timespec="seconds")
        body = (
            f"# Eyemech eye_calib_limits — unit-disk iris theta (s/r)\n"
            f"# outer_ref=white_periocular_halo_x3 (not green eyelid)\n"
            f"# updated={stamp}\n"
            f"# Look to extremes; hand this file back to scale pupil_control mapping.\n"
            f"max_right={self.max_right:.6f}\n"
            f"max_left={self.max_left:.6f}\n"
            f"max_up={self.max_up:.6f}\n"
            f"max_down={self.max_down:.6f}\n"
        )
        with open(self.path, "w", encoding="utf-8") as f:
            f.write(body)
        tag = "created" if initial else "updated"
        print(f"[eye_calibrate] {tag} {self.path}  ({self.summary()})", flush=True)


class StatusPanel:
    """Tiny always-on-top bar: live extremes + Reset."""

    def __init__(self, log: ThetaExtremeLog) -> None:
        if tk is None:
            self.root = None
            return
        self.log = log
        self.root = tk.Tk()
        self.root.title("Eyemech eye calibrate")
        self.root.resizable(False, False)
        self.root.attributes("-topmost", True)

        bar = ttk.Frame(self.root, padding=8)
        bar.pack()
        ttk.Label(bar, text="Iris θ extremes (white orbit):").pack(side=tk.LEFT)
        self.limits_var = tk.StringVar(value=log.summary())
        ttk.Label(bar, textvariable=self.limits_var).pack(side=tk.LEFT, padx=(6, 8))
        ttk.Button(bar, text="Reset", command=self._reset).pack(side=tk.LEFT)

    def _reset(self) -> None:
        self.log.reset()
        self.limits_var.set(self.log.summary())

    def update_label(self) -> None:
        if self.root is not None:
            self.limits_var.set(self.log.summary())

    def pump(self) -> None:
        if self.root is None:
            return
        try:
            self.root.update_idletasks()
            self.root.update()
        except tk.TclError:
            pass

    def destroy(self) -> None:
        if self.root is None:
            return
        try:
            self.root.destroy()
        except Exception:
            pass


def run(camera: int, width: int, height: int, model: str) -> int:
    model = ensure_model(model)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(HERE, f"eye_calib_limits_{ts}.txt")
    log = ThetaExtremeLog(path)
    panel = StatusPanel(log)

    cap = cv2.VideoCapture(camera)
    if not cap.isOpened():
        print(f"Cannot open camera {camera}", file=sys.stderr)
        panel.destroy()
        return 1
    if width > 0:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    if height > 0:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    print(
        f"[eye_calibrate] white periocular outer; logging → {path}\n"
        f"  Look hard LEFT / RIGHT / UP / DOWN. Press 'q' to quit.",
        file=sys.stderr,
    )

    win = "Eyemech eye_calibrate"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE | cv2.WINDOW_GUI_NORMAL)

    t0 = time.time()
    last_print = 0.0
    with create_landmarker(model) as landmarker:
        try:
            while True:
                panel.pump()
                ok, frame = cap.read()
                if not ok:
                    continue
                ts_ms = int((time.time() - t0) * 1000)
                annotated, info, _lids = process_frame(landmarker, frame, ts_ms)

                th_r = th_u = 0.0
                if info:
                    th_r, th_u = average_sphere_angles(info)
                    if log.observe(th_r, th_u):
                        panel.update_label()

                now = time.monotonic()
                if now - last_print >= 0.2:
                    print(
                        f"{format_angles(th_r, th_u)}  peaks {log.summary()}",
                        flush=True,
                    )
                    last_print = now

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
                    f"peaks {log.summary()}",
                    (10, 84),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 200, 0),
                    2,
                )
                cv2.imshow(win, annotated)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
        finally:
            cap.release()
            cv2.destroyAllWindows()
            panel.destroy()
            log._write(initial=False)
            print(
                f"[eye_calibrate] done. Share this file:\n  {path}\n"
                f"  finals: {log.summary()}",
                flush=True,
            )
    return 0


def main(argv: Optional[list] = None) -> int:
    p = argparse.ArgumentParser(
        description="Record max iris θ vs white periocular outer (no Arduino)."
    )
    p.add_argument("--camera", type=int, default=0)
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--model", default=DEFAULT_MODEL)
    args = p.parse_args(argv)
    return run(args.camera, args.width, args.height, args.model)


if __name__ == "__main__":
    raise SystemExit(main())
