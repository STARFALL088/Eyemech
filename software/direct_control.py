#!/usr/bin/env python3
"""
Eyemech — Direct mouse control pad (no webcam)
==============================================

Tkinter UI matching the EyeMech Control Interface sketch:
  * large circle = orthographic disk of a sphere with diameter d
  * red marker follows the mouse (clamped to the disk)
  * center = (0, 0); +x = right, +y = up
  * arc-length angles:  s = r * theta  =>  theta = s / r
    so theta_right = x / r,  theta_up = y / r  (radians)

When Connected, each gaze sample is also sent to the Arduino firmware as:
  "<right_deg>,<up_deg>\\n"
matching software/firmware/firmware.ino (9600 baud). Firmware clamps with
safe_turn() to ±MAX_TURN_ANGLE.
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


# Canvas / circle layout (pixels)
CANVAS_SIZE = 420
MARGIN = 28
CIRCLE_DIAMETER = CANVAS_SIZE - 2 * MARGIN  # d
CIRCLE_RADIUS = CIRCLE_DIAMETER / 2.0        # r
DOT_RADIUS = 5

BAUD = 9600
# Don't flood the Arduino USB-serial buffer on every pixel of mouse motion.
SEND_INTERVAL_S = 0.02  # 50 Hz


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

        self._build_chrome()
        self._build_canvas()
        self._refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ------------------------------------------------------------------ UI
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

    def _build_canvas(self) -> None:
        self.canvas = tk.Canvas(
            self.root,
            width=CANVAS_SIZE,
            height=CANVAS_SIZE,
            bg="white",
            highlightthickness=1,
            highlightbackground="#cccccc",
        )
        self.canvas.pack(padx=8, pady=(0, 8))

        x0 = self._cx - CIRCLE_RADIUS
        y0 = self._cy - CIRCLE_RADIUS
        x1 = self._cx + CIRCLE_RADIUS
        y1 = self._cy + CIRCLE_RADIUS
        self.canvas.create_oval(x0, y0, x1, y1, outline="black", width=2)

        self.canvas.create_line(
            x0, self._cy, x1, self._cy, fill="#9a9a9a", dash=(4, 4)
        )
        self.canvas.create_line(
            self._cx, y0, self._cx, y1, fill="#9a9a9a", dash=(4, 4)
        )

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

    def _disconnect(self) -> None:
        if self._ser is not None:
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
            self._disconnect()
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
            time.sleep(2.0)  # Arduino auto-reset on open
            try:
                self._ser.reset_input_buffer()
            except Exception:
                pass
        except Exception as exc:
            self._ser = None
            self.status_var.set(f"Open failed")
            print(f"[direct_control] serial open failed: {exc}", flush=True)
            return

        self._connected = True
        self.status_var.set("Connected")
        self.connect_btn.configure(text="Disconnect")
        # Sync firmware to current (usually center) pose.
        th_r, th_u = self._pixel_to_angles(*self._dot_pos)
        self._send_angles(math.degrees(th_r), math.degrees(th_u), force=True)

    def _send_angles(self, right_deg: float, up_deg: float, force: bool = False) -> None:
        if not self._connected or self._ser is None:
            return
        now = time.monotonic()
        if not force and (now - self._last_send_t) < SEND_INTERVAL_S:
            return
        sample = (round(right_deg, 2), round(up_deg, 2))
        if not force and sample == self._last_sent:
            return
        line = f"{sample[0]:.2f},{sample[1]:.2f}\n"
        try:
            self._ser.write(line.encode("ascii"))
            self._last_send_t = now
            self._last_sent = sample
        except Exception as exc:
            print(f"[direct_control] serial write failed: {exc}", flush=True)
            self._disconnect()

    def _on_close(self) -> None:
        self._disconnect()
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
        """
        Screen → sphere angles (radians).
          +theta_right : rightward
          +theta_up    : upward  (screen y grows downward, so flip)
        s = r * theta  =>  theta = s / r
        """
        x = px - self._cx
        y = self._cy - py
        theta_right = x / CIRCLE_RADIUS
        theta_up = y / CIRCLE_RADIUS
        return theta_right, theta_up

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
        right_deg = math.degrees(th_r)
        up_deg = math.degrees(th_u)
        print(
            f"right={th_r:+.4f} rad ({right_deg:+.1f}°)  "
            f"up={th_u:+.4f} rad ({up_deg:+.1f}°)",
            flush=True,
        )
        self._send_angles(right_deg, up_deg)

    def _on_motion(self, event: tk.Event) -> None:
        self._move_dot(float(event.x), float(event.y))

    def _on_leave(self, _event: tk.Event) -> None:
        self._move_dot(self._cx, self._cy)

    def run(self) -> None:
        print(
            f"[direct_control] circle d={CIRCLE_DIAMETER:.0f}px  r={CIRCLE_RADIUS:.1f}px  "
            f"edge = ±1.0000 rad (±{math.degrees(1.0):.1f}°)  "
            f"serial {BAUD} baud  line='right_deg,up_deg'",
            flush=True,
        )
        self.root.mainloop()


def main() -> None:
    DirectControlApp().run()


if __name__ == "__main__":
    main()
