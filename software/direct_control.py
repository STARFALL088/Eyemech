#!/usr/bin/env python3
"""
Eyemech — Direct mouse control pad (no webcam)
==============================================

Tkinter UI:
  * Port / Connect bar
  * Arrow nudges + Home (set current pose as gaze zero)
  * Circle pad: theta = s/r → degrees over serial

Protocol (9600 baud), same as firmware:
  "<right_deg>,<up_deg>,1.00,1.00\\n"   gaze + lids open (no webcam)
  "HOME\\n"                              park mechanical v=0 on clean disconnect
"""

from __future__ import annotations

import math
import time
import tkinter as tk
from tkinter import ttk
from typing import Optional, Tuple

try:
    import serial
    from serial.tools.list_ports import comports
except ImportError:  # pragma: no cover
    serial = None  # type: ignore
    comports = None  # type: ignore


CANVAS_SIZE = 420
MARGIN = 28
CIRCLE_DIAMETER = CANVAS_SIZE - 2 * MARGIN
CIRCLE_RADIUS = CIRCLE_DIAMETER / 2.0
DOT_RADIUS = 5

BAUD = 9600
SEND_INTERVAL_S = 0.02
NUDGE_DEG = 0.5
# Match firmware host-degree edges (from calib_limits).
MAX_TURN_X = 14.0
MAX_TURN_Y = 4.5


class DirectControlApp:
    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title("EyeMech Control Interface")
        self.root.resizable(False, False)

        self._cx = CANVAS_SIZE / 2.0
        self._cy = CANVAS_SIZE / 2.0
        self._dot_pos: Tuple[float, float] = (self._cx, self._cy)

        self._ser: Optional["serial.Serial"] = None
        self._connected = False
        self._last_send_t = 0.0
        self._last_sent: Optional[Tuple[float, float]] = None

        # Gaze zero: absolute degrees sent when pad/pupil reports (0,0).
        self._zero_r = 0.0
        self._zero_u = 0.0
        self._gaze_r = 0.0
        self._gaze_u = 0.0

        self._build_chrome()
        self._build_calib_bar()
        self._build_canvas()
        self._refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_chrome(self) -> None:
        bar = ttk.Frame(self.root, padding=(8, 6))
        bar.pack(fill=tk.X)

        ttk.Label(bar, text="Port:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(
            bar, textvariable=self.port_var, width=14, state="readonly"
        )
        self.port_box.pack(side=tk.LEFT, padx=(4, 6))

        ttk.Button(bar, text="Update", command=self._refresh_ports).pack(
            side=tk.LEFT, padx=(0, 6)
        )
        self.connect_btn = ttk.Button(
            bar, text="Connect", command=self._toggle_connect
        )
        self.connect_btn.pack(side=tk.LEFT, padx=(0, 8))

        self.status_var = tk.StringVar(value="Not Connected")
        ttk.Label(bar, textvariable=self.status_var).pack(side=tk.LEFT)

    def _build_calib_bar(self) -> None:
        bar = ttk.Frame(self.root, padding=(8, 0))
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
        ttk.Button(bar, text="Home", command=self._set_zero).pack(side=tk.LEFT, padx=(8, 4))
        self.zero_var = tk.StringVar(value="zero=(+0.0,+0.0)°")
        ttk.Label(bar, textvariable=self.zero_var).pack(side=tk.LEFT)

    def _build_canvas(self) -> None:
        self.canvas = tk.Canvas(
            self.root,
            width=CANVAS_SIZE,
            height=CANVAS_SIZE,
            bg="white",
            highlightthickness=1,
            highlightbackground="#cccccc",
        )
        self.canvas.pack(padx=8, pady=(6, 8))

        x0 = self._cx - CIRCLE_RADIUS
        y0 = self._cy - CIRCLE_RADIUS
        x1 = self._cx + CIRCLE_RADIUS
        y1 = self._cy + CIRCLE_RADIUS
        self.canvas.create_oval(x0, y0, x1, y1, outline="black", width=2)
        self.canvas.create_line(x0, self._cy, x1, self._cy, fill="#9a9a9a", dash=(4, 4))
        self.canvas.create_line(self._cx, y0, self._cx, y1, fill="#9a9a9a", dash=(4, 4))

        self.dot = self.canvas.create_oval(
            self._cx - DOT_RADIUS,
            self._cy - DOT_RADIUS,
            self._cx + DOT_RADIUS,
            self._cy + DOT_RADIUS,
            fill="red",
            outline="red",
        )
        self.canvas.bind("<Motion>", self._on_motion)
        self.canvas.bind("<Leave>", self._on_leave)

    def _refresh_zero_label(self) -> None:
        self.zero_var.set(f"zero=({self._zero_r:+.1f},{self._zero_u:+.1f})°")

    def _clamp_cmd(self, r: float, u: float) -> Tuple[float, float]:
        return (
            max(-MAX_TURN_X, min(MAX_TURN_X, r)),
            max(-MAX_TURN_Y, min(MAX_TURN_Y, u)),
        )

    def _nudge(self, dr: float, du: float) -> None:
        """Tiny manual move; adjusts the zero offset and sends immediately."""
        self._zero_r, self._zero_u = self._clamp_cmd(
            self._zero_r + dr, self._zero_u + du
        )
        self._refresh_zero_label()
        print(
            f"[calib] nudge → zero=({self._zero_r:+.2f},{self._zero_u:+.2f})°",
            flush=True,
        )
        self._send_gaze(force=True)

    def _set_zero(self) -> None:
        """Current absolute pose becomes gaze (0,0); recenter the pad."""
        abs_r = self._zero_r + self._gaze_r
        abs_u = self._zero_u + self._gaze_u
        self._zero_r, self._zero_u = self._clamp_cmd(abs_r, abs_u)
        self._gaze_r = 0.0
        self._gaze_u = 0.0
        self._dot_pos = (self._cx, self._cy)
        self.canvas.coords(
            self.dot,
            self._cx - DOT_RADIUS,
            self._cy - DOT_RADIUS,
            self._cx + DOT_RADIUS,
            self._cy + DOT_RADIUS,
        )
        self._refresh_zero_label()
        print(
            f"[calib] Home → set zero=({self._zero_r:+.2f},{self._zero_u:+.2f})°",
            flush=True,
        )
        self._send_gaze(force=True)

    def _send_gaze(self, force: bool = False) -> None:
        r, u = self._clamp_cmd(self._zero_r + self._gaze_r, self._zero_u + self._gaze_u)
        self._send_angles(r, u, force=force)

    # ----------------------------------------------------------- serial
    def _refresh_ports(self) -> None:
        ports = []
        if comports is not None:
            ports = [p.device for p in comports()]
        self.port_box["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        elif not ports:
            self.port_var.set("")

    def _disconnect(self, park_home: bool = True) -> None:
        if self._ser is not None:
            if park_home:
                try:
                    self._ser.write(b"HOME\n")
                    self._ser.flush()
                    time.sleep(0.4)
                except Exception:
                    pass
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        self._connected = False
        self.status_var.set("Not Connected")
        self.connect_btn.configure(text="Connect")

    def _toggle_connect(self) -> None:
        if self._connected:
            self._disconnect(park_home=True)
            return

        port = self.port_var.get()
        if not port:
            self.status_var.set("No Port")
            return
        if serial is None:
            self.status_var.set("pyserial missing")
            return

        try:
            self._ser = serial.Serial(port, BAUD, timeout=0.1)
            time.sleep(2.0)
            try:
                self._ser.reset_input_buffer()
            except Exception:
                pass
        except Exception as exc:
            self._ser = None
            self.status_var.set("Open failed")
            print(f"[direct_control] serial open failed: {exc}", flush=True)
            return

        self._connected = True
        self.status_var.set("Connected")
        self.connect_btn.configure(text="Disconnect")
        self._send_gaze(force=True)

    def _send_angles(self, right_deg: float, up_deg: float, force: bool = False) -> None:
        if not self._connected or self._ser is None:
            return
        now = time.monotonic()
        if not force and (now - self._last_send_t) < SEND_INTERVAL_S:
            return
        sample = (round(right_deg, 2), round(up_deg, 2))
        if not force and sample == self._last_sent:
            return
        line = f"{sample[0]:.2f},{sample[1]:.2f},1.00,1.00\n"
        try:
            self._ser.write(line.encode("ascii"))
            self._last_send_t = now
            self._last_sent = sample
        except Exception as exc:
            print(f"[direct_control] serial write failed: {exc}", flush=True)
            self._disconnect(park_home=False)

    def _on_close(self) -> None:
        self._disconnect(park_home=True)
        self.root.destroy()

    # ----------------------------------------------------------- mouse / math
    def _clamp_to_circle(self, px: float, py: float) -> Tuple[float, float]:
        dx = px - self._cx
        dy = py - self._cy
        dist = math.hypot(dx, dy)
        if dist > CIRCLE_RADIUS and dist > 0:
            scale = CIRCLE_RADIUS / dist
            dx *= scale
            dy *= scale
        return self._cx + dx, self._cy + dy

    def _pixel_to_angles(self, px: float, py: float) -> Tuple[float, float]:
        x = px - self._cx
        y = self._cy - py
        return x / CIRCLE_RADIUS, y / CIRCLE_RADIUS

    def _move_dot(self, px: float, py: float) -> None:
        px, py = self._clamp_to_circle(px, py)
        self._dot_pos = (px, py)
        self.canvas.coords(
            self.dot,
            px - DOT_RADIUS,
            py - DOT_RADIUS,
            px + DOT_RADIUS,
            py + DOT_RADIUS,
        )
        th_r, th_u = self._pixel_to_angles(px, py)
        # Unit disk (±1 at rim) → calibrated host degrees (±MAX_TURN_*).
        self._gaze_r = th_r * MAX_TURN_X
        self._gaze_u = th_u * MAX_TURN_Y
        print(
            f"right={th_r:+.4f} ({self._gaze_r:+.1f}°)  "
            f"up={th_u:+.4f} ({self._gaze_u:+.1f}°)  "
            f"zero=({self._zero_r:+.1f},{self._zero_u:+.1f})",
            flush=True,
        )
        self._send_gaze()

    def _on_motion(self, event: tk.Event) -> None:
        self._move_dot(float(event.x), float(event.y))

    def _on_leave(self, _event: tk.Event) -> None:
        self._move_dot(self._cx, self._cy)

    def run(self) -> None:
        print(
            f"[direct_control] circle d={CIRCLE_DIAMETER:.0f}px  "
            f"edges ±{MAX_TURN_X}° X / ±{MAX_TURN_Y}° Y  "
            f"nudge={NUDGE_DEG}°  serial {BAUD} baud",
            flush=True,
        )
        self.root.mainloop()


def main() -> None:
    DirectControlApp().run()


if __name__ == "__main__":
    main()
