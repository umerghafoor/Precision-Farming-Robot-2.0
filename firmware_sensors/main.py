#!/usr/bin/env python3
"""
Sensor Node Monitor — Arduino Nano (MPU-9250 + Laser)

Reads IMU data printed by the Arduino over USB Serial and renders a
live curses dashboard. Laser is toggled ON/OFF with SPACE.

Arduino serial format (one line per sample):
  A:<ax>,<ay>,<az> G:<gx>,<gy>,<gz> M:<mx>,<my>,<mz>

Controls:
  SPACE  — toggle laser (sends "LASER_ON\n" or "LASER_OFF\n")
  ESC/Q  — quit
"""

import curses
import re
import sys
import threading
import time
from collections import deque

import serial

# ── Serial config ──────────────────────────────────────────────────────────────
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE   = 115200

# ── IMU scaling (must match firmware) ──────────────────────────────────────────
# Firmware stores:  ax = accel_g * 1000  → divide by 1000 for g
#                   gx = gyro_dps * 10   → divide by 10   for dps
#                   mx = mag_uT * 10     → divide by 10   for µT
ACC_SCALE = 1000.0
GYR_SCALE = 10.0
MAG_SCALE = 10.0

# ── History length for sparkline bars ─────────────────────────────────────────
HISTORY = 40

_IMU_RE = re.compile(
    r"A:(-?\d+),(-?\d+),(-?\d+)\s+G:(-?\d+),(-?\d+),(-?\d+)\s+M:(-?\d+),(-?\d+),(-?\d+)"
)


class SerialReader:
    """Background thread: reads IMU lines from the Arduino."""

    def __init__(self, port: str, baud: int):
        self.port  = port
        self.baud  = baud
        self._ser  = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._latest = None          # (ax,ay,az,gx,gy,gz,mx,my,mz) raw ints
        self._error  = None          # last connect error string
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._ser and self._ser.is_open:
            try:
                self._ser.close()
            except Exception:
                pass

    def latest(self):
        with self._lock:
            return self._latest

    def error(self):
        with self._lock:
            return self._error

    def send(self, text: str):
        with self._lock:
            if self._ser and self._ser.is_open:
                try:
                    self._ser.write((text + "\n").encode())
                    self._ser.flush()
                except Exception:
                    pass

    def _run(self):
        while not self._stop.is_set():
            try:
                self._ser = serial.Serial(self.port, self.baud, timeout=1)
                time.sleep(2)          # wait for Arduino reset
                self._ser.reset_input_buffer()
                with self._lock:
                    self._error = None

                while not self._stop.is_set():
                    line = self._ser.readline().decode("utf-8", errors="ignore").strip()
                    m = _IMU_RE.search(line)
                    if m:
                        vals = tuple(int(x) for x in m.groups())
                        with self._lock:
                            self._latest = vals

            except serial.SerialException as e:
                with self._lock:
                    self._error = str(e)
                    self._latest = None
                time.sleep(2)
            except Exception as e:
                with self._lock:
                    self._error = str(e)
                time.sleep(2)


# ── Sparkline helpers ──────────────────────────────────────────────────────────

SPARK_CHARS = " ▁▂▃▄▅▆▇█"

def _sparkline(history: deque, width: int) -> str:
    """Return a sparkline string of `width` characters from the deque."""
    data = list(history)[-width:]
    if not data:
        return " " * width
    lo, hi = min(data), max(data)
    rng = hi - lo or 1
    chars = [SPARK_CHARS[int((v - lo) / rng * (len(SPARK_CHARS) - 1))] for v in data]
    # Pad left
    pad = width - len(chars)
    return " " * pad + "".join(chars)


def _bar(value: float, full_scale: float, width: int) -> str:
    """Signed bar centred at zero. Positive fills right, negative fills left."""
    half = width // 2
    filled = int(abs(value) / full_scale * half)
    filled = min(filled, half)
    if value >= 0:
        return " " * half + "█" * filled + " " * (half - filled)
    else:
        return " " * (half - filled) + "█" * filled + " " * half


# ── Main curses UI ─────────────────────────────────────────────────────────────

def run_ui(stdscr, reader: SerialReader):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(True)

    # Colour pairs
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_CYAN,    -1)  # header
    curses.init_pair(2, curses.COLOR_GREEN,   -1)  # value OK
    curses.init_pair(3, curses.COLOR_YELLOW,  -1)  # label
    curses.init_pair(4, curses.COLOR_RED,     -1)  # laser ON / error
    curses.init_pair(5, curses.COLOR_MAGENTA, -1)  # bar

    HDR   = curses.color_pair(1) | curses.A_BOLD
    VAL   = curses.color_pair(2)
    LBL   = curses.color_pair(3)
    ERR   = curses.color_pair(4) | curses.A_BOLD
    BAR   = curses.color_pair(5)
    LASER_ON_ATTR  = curses.color_pair(4) | curses.A_BOLD | curses.A_REVERSE
    LASER_OFF_ATTR = curses.color_pair(3) | curses.A_DIM

    # History deques  (ax ay az | gx gy gz | mx my mz)
    hist = [deque(maxlen=HISTORY) for _ in range(9)]

    laser_on  = False
    sample_count = 0
    last_sample  = None

    def safe_addstr(row, col, text, attr=0):
        h, w = stdscr.getmaxyx()
        if row >= h or col >= w:
            return
        try:
            stdscr.addstr(row, col, text[: w - col], attr)
        except curses.error:
            pass

    while True:
        # ── Input ──────────────────────────────────────────────────────────
        key = stdscr.getch()
        if key in (27, ord("q"), ord("Q")):          # ESC or Q → quit
            break
        if key == ord(" "):
            laser_on = not laser_on
            reader.send("LASER_ON" if laser_on else "LASER_OFF")

        # ── Pull latest IMU sample ─────────────────────────────────────────
        raw = reader.latest()
        err = reader.error()

        if raw and raw != last_sample:
            last_sample = raw
            sample_count += 1
            for i, v in enumerate(raw):
                hist[i].append(v)

        # ── Draw ───────────────────────────────────────────────────────────
        stdscr.erase()
        h, w = stdscr.getmaxyx()
        bar_w = max(20, min(40, w - 30))

        row = 0
        title = "  Precision Farming Robot — Sensor Node Monitor  "
        safe_addstr(row, max(0, (w - len(title)) // 2), title, HDR)
        row += 1
        safe_addstr(row, 0, "─" * w, HDR)
        row += 1

        # Laser status
        if laser_on:
            laser_str = "  LASER  [ ON  ]  (SPACE to toggle)"
            safe_addstr(row, 0, laser_str, LASER_ON_ATTR)
        else:
            laser_str = "  LASER  [ OFF ]  (SPACE to toggle)"
            safe_addstr(row, 0, laser_str, LASER_OFF_ATTR)
        row += 1

        # Connection status
        if err:
            safe_addstr(row, 0, f"  Serial ERROR: {err}", ERR)
        else:
            safe_addstr(row, 0, f"  Port: {SERIAL_PORT} @ {BAUD_RATE}  samples: {sample_count}", VAL)
        row += 2

        # ── Sections: Accel / Gyro / Mag ───────────────────────────────────
        sections = [
            ("Accelerometer", ["Ax", "Ay", "Az"], 0, ACC_SCALE, 2.0,   "g"),
            ("Gyroscope",     ["Gx", "Gy", "Gz"], 3, GYR_SCALE, 500.0, "dps"),
            ("Magnetometer",  ["Mx", "My", "Mz"], 6, MAG_SCALE, 500.0, "µT"),
        ]

        for title_s, labels, base, scale, full, unit in sections:
            if row + 6 >= h:
                break
            safe_addstr(row, 2, f"── {title_s} ──", HDR)
            row += 1

            for i, lbl in enumerate(labels):
                if row >= h:
                    break
                idx  = base + i
                raw_v = list(hist[idx])[-1] if hist[idx] else 0
                phys  = raw_v / scale
                spark = _sparkline(hist[idx], HISTORY)
                bar   = _bar(phys, full, bar_w)

                col = 4
                safe_addstr(row, col, f"{lbl}:", LBL)
                col += 4
                val_str = f"{phys:+8.3f} {unit}"
                safe_addstr(row, col, val_str, VAL)
                col += len(val_str) + 1
                safe_addstr(row, col, bar, BAR)
                col += bar_w + 1
                safe_addstr(row, col, spark[:max(0, w - col - 1)], curses.color_pair(1))
                row += 1

            row += 1

        # Footer
        if row < h - 1:
            safe_addstr(h - 1, 0, "  SPACE=laser toggle   Q/ESC=quit", LBL)

        stdscr.refresh()
        time.sleep(0.05)


def main():
    reader = SerialReader(SERIAL_PORT, BAUD_RATE)
    reader.start()
    try:
        curses.wrapper(run_ui, reader)
    finally:
        reader.stop()


if __name__ == "__main__":
    main()
