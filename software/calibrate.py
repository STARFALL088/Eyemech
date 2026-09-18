#!/usr/bin/env python3
"""
Eyemech — range calibrator (arrow-only, records host-degree extremes)
=====================================================================

UI: serial port bar + ← → ↑ ↓ + Home. No circle / mouse pad.

  max_right / max_left / max_up / max_down
are the farthest commands relative to the last Home (set-zero) pose.

On start: create ``calib_limits_*.txt``.
On Home: set current pose as zero AND reset all extremes to 0.
Whenever an extreme is beaten, rewrite the file.

Usage
-----
  python calibrate.py
  # Connect → Home → nudge to safe edges → quit → share the .txt
"""

from __future__ import annotations

import os
import time
import tkinter as tk
from datetime import datetime
from tkinter import ttk
from typing import Optional, Tuple

try:
    import serial
    from serial.tools.list_ports import comports
except ImportError:  # pragma: no cover
    serial = None  # type: ignore
    comports = None  # type: ignore


BAUD = 9600
SEND_INTERVAL_S = 0.02
NUDGE_DEG = 0.5
# Match firmware host-degree edges.
MAX_TURN_X = 14.0
MAX_TURN_Y = 4.5

HERE = os.path.dirname(os.path.abspath(__file__))


class ExtremeLog:
    """Track axis extremes (relative to Home) and persist to a .txt file."""

    def __init__(self, path: str) -> None:
        self.path = path
        self.max_right = 0.0
        self.max_left = 0.0
        self.max_up = 0.0
        self.max_down = 0.0
        self._write(initial=True)

    def reset(self) -> None:
        self.max_right = 0.0
        self.max_left = 0.0
        self.max_up = 0.0
        self.max_down = 0.0
        self._write(initial=False)
        print(f"[calibrate] extremes reset → {self.summary()}", flush=True)

    def observe(self, right_deg: float, up_deg: float) -> bool:
        """``right_deg`` / ``up_deg`` are offsets from the Home pose."""
        changed = False
        if right_deg > self.max_right:
            self.max_right = right_deg
            changed = True
        if right_deg < self.max_left:
            self.max_left = right_deg
            changed = True
        if up_deg > self.max_up:
            self.max_up = up_deg
            changed = True
        if up_deg < self.max_down:
            self.max_down = up_deg
            changed = True
        if changed:
            self._write(initial=False)
        return changed

    def summary(self) -> str:
        return (
            f"R={self.max_right:+.2f} L={self.max_left:+.2f} "
            f"U={self.max_up:+.2f} D={self.max_down:+.2f}"
        )

    def _write(self, initial: bool) -> None:
        stamp = datetime.now().isoformat(timespec="seconds")
        body = (
            f"# Eyemech calib_limits — degrees relative to last Home\n"
            f"# updated={stamp}\n"
            f"# Arrow-only calibrator; map full pad radius to these edges.\n"
            f"max_right={self.max_right:.4f}\n"
            f"max_left={self.max_left:.4f}\n"
            f"max_up={self.max_up:.4f}\n"
            f"max_down={self.max_down:.4f}\n"
        )
        with open(self.path, "w", encoding="utf-8") as f:
            f.write(body)
        tag = "created" if initial else "updated"
        print(f"[calibrate] {tag} {self.path}  ({self.summary()})", flush=True)


class CalibrateApp:
    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title("EyeMech Range Calibrator")
        self.root.resizable(False, False)

        self._ser: Optional["serial.Serial"] = None
        self._connected = False
        self._last_send_t = 0.0
        self._last_sent: Optional[Tuple[float, float]] = None

        # Absolute command on the wire (Home defines the zero for recording).
        self._pos_r = 0.0
        self._pos_u = 0.0
        self._home_r = 0.0
        self._home_u = 0.0

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._log_path = os.path.join(HERE, f"calib_limits_{ts}.txt")
        self._log = ExtremeLog(self._log_path)

        self._build_chrome()
        self._build_calib_bar()
        self._build_limits_bar()
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
        ttk.Button(bar, text="Home", command=self._set_home).pack(side=tk.LEFT, padx=(8, 4))
        self.pos_var = tk.StringVar(value="pos=(+0.0,+0.0)°")
        ttk.Label(bar, textvariable=self.pos_var).pack(side=tk.LEFT)

    def _build_limits_bar(self) -> None:
        bar = ttk.Frame(self.root, padding=(8, 4))
        bar.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(bar, text="Recorded extremes (vs Home):").pack(side=tk.LEFT)
        self.limits_var = tk.StringVar(value=self._log.summary())
        ttk.Label(bar, textvariable=self.limits_var).pack(side=tk.LEFT, padx=(6, 0))

    def _refresh_pos_label(self) -> None:
        rel_r = self._pos_r - self._home_r
        rel_u = self._pos_u - self._home_u
        self.pos_var.set(
            f"pos=({self._pos_r:+.1f},{self._pos_u:+.1f})°  "
            f"rel=({rel_r:+.1f},{rel_u:+.1f})°"
        )

    def _clamp_cmd(self, r: float, u: float) -> Tuple[float, float]:
        return (
            max(-MAX_TURN_X, min(MAX_TURN_X, r)),
            max(-MAX_TURN_Y, min(MAX_TURN_Y, u)),
        )

    def _nudge(self, dr: float, du: float) -> None:
        self._pos_r, self._pos_u = self._clamp_cmd(self._pos_r + dr, self._pos_u + du)
        self._refresh_pos_label()
        print(
            f"[calib] nudge → pos=({self._pos_r:+.2f},{self._pos_u:+.2f})°",
            flush=True,
        )
        self._send_pos(force=True)

    def _set_home(self) -> None:
        """Current pose becomes zero; clear recorded extremes."""
        self._home_r = self._pos_r
        self._home_u = self._pos_u
        self._log.reset()
        self.limits_var.set(self._log.summary())
        self._refresh_pos_label()
        print(
            f"[calib] Home → zero at ({self._home_r:+.2f},{self._home_u:+.2f})°; "
            f"extremes cleared",
            flush=True,
        )
        self._send_pos(force=True)

    def _send_pos(self, force: bool = False) -> None:
        rel_r = self._pos_r - self._home_r
        rel_u = self._pos_u - self._home_u
        if self._log.observe(rel_r, rel_u):
            self.limits_var.set(self._log.summary())
        self._send_angles(self._pos_r, self._pos_u, force=force)

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
            print(f"[calibrate] serial open failed: {exc}", flush=True)
            return

        self._connected = True
        self.status_var.set("Connected")
        self.connect_btn.configure(text="Disconnect")
        self._send_pos(force=True)

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
            print(f"[calibrate] serial write failed: {exc}", flush=True)
            self._disconnect(park_home=False)

    def _on_close(self) -> None:
        self._log._write(initial=False)
        print(
            f"[calibrate] done. Share this file:\n  {self._log_path}\n"
            f"  finals: {self._log.summary()}",
            flush=True,
        )
        self._disconnect(park_home=True)
        self.root.destroy()

    def run(self) -> None:
        print(
            f"[calibrate] logging → {self._log_path}\n"
            f"  Connect → Home (clears extremes) → arrows to safe edges → quit.\n"
            f"  Firmware clamps ±{MAX_TURN_X}° X / ±{MAX_TURN_Y}° Y.",
            flush=True,
        )
        self.root.mainloop()


def main() -> None:
    CalibrateApp().run()


if __name__ == "__main__":
    main()
