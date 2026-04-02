"""
Live Plot — Digger Control Monitor (Nano R4)
=============================================
Real-time plot for the Arduino Nano R4 digger controller.
Reads CSV from USB serial and displays all inputs + outputs.

Panels:
  1. RC1  — CH1 Left Motor      (blue)
  2. RC2  — CH2 Right Motor     (red)
  3. RC4  — CH4 Control Mode    (green)
  4. RC5  — CH5 Override        (orange)
  5. JoyY — Joystick Throttle   (purple)
  6. JoyX — Joystick Steering   (cyan)
  7. OutL — Left ESC Output     (darkorange)
  8. OutR — Right ESC Output    (saddlebrown)

CSV format from sketch:
  RC1,RC2,RC4,RC5,JoyY,JoyX,OutL,OutR,CH1ok,CH2ok

Usage:
  python live_plot.py
"""

from collections import deque

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import serial

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
PORT   = "COM8"
BAUD   = 115200
WINDOW = 200

# Channel definitions: (key, label, color, y_range, center)
CHANNELS = [
    ("rc1",  "RC CH1 Left Motor [D2]",       "blue",        (800, 2200),  1500),
    ("rc2",  "RC CH2 Right Motor [D4]",      "red",         (800, 2200),  1500),
    ("rc4",  "RC CH4 Control Mode [D3]",     "green",       (800, 2200),  1500),
    ("rc5",  "RC CH5 Override Switch [D7]",  "darkorange",  (800, 2200),  1000),
    ("jy",   "Joystick Y Throttle [A0]",     "purple",      (0, 16383),   8192),
    ("jx",   "Joystick X Steering [A1]",     "darkcyan",    (0, 16383),   8192),
    ("outl", "Left ESC Output [D9]",         "orangered",   (1200, 1800), 1500),
    ("outr", "Right ESC Output [D10]",       "saddlebrown", (1200, 1800), 1500),
]

# CSV column indices matching sketch output
# RC1,RC2,RC4,RC5,JoyY,JoyX,OutL,OutR,CH1ok,CH2ok
CSV_MAP = [0, 1, 2, 3, 4, 5, 6, 7]  # maps channel index to CSV column

# -----------------------------------------------------------------------------
# Data Buffers
# -----------------------------------------------------------------------------
buffers = []
for _, _, _, _, center in CHANNELS:
    buffers.append(deque([center] * WINDOW, maxlen=WINDOW))

# -----------------------------------------------------------------------------
# Plot Setup
# -----------------------------------------------------------------------------
fig, axes = plt.subplots(len(CHANNELS), 1, figsize=(10, 12), sharex=True)
fig.suptitle("Digger Control — Live Monitor (Nano R4)", fontsize=14, fontweight="bold")

plot_lines = []
for i, (key, label, color, ylim, center) in enumerate(CHANNELS):
    ax = axes[i]
    line, = ax.plot([], [], color=color, linewidth=1.5)
    plot_lines.append(line)
    ax.set_ylim(*ylim)
    ax.set_title(label, fontsize=10, color=color, loc="left")
    ax.axhline(y=center, color="gray", linestyle="--", alpha=0.5)
    ax.grid(True, alpha=0.3)

# Servo range reference lines
for i in [0, 1, 2, 3, 6, 7]:
    axes[i].axhline(y=1000, color="gray", linestyle=":", alpha=0.3)
    axes[i].axhline(y=2000, color="gray", linestyle=":", alpha=0.3)

# ESC power limit lines (50% = 1250-1750us)
for i in [6, 7]:
    axes[i].axhline(y=1250, color="red", linestyle="--", alpha=0.4)
    axes[i].axhline(y=1750, color="red", linestyle="--", alpha=0.4)

axes[-1].set_xlabel("Samples")
plt.tight_layout()

# -----------------------------------------------------------------------------
# Serial Connection
# -----------------------------------------------------------------------------
ser = serial.Serial(PORT, BAUD, timeout=0.1)

# -----------------------------------------------------------------------------
# Animation Update
# -----------------------------------------------------------------------------
def update(_frame):
    try:
        while ser.in_waiting:
            try:
                raw = ser.readline().decode("utf-8", errors="ignore").strip()
                if raw.startswith("#") or not raw:
                    continue
                parts = raw.split(",")
                if len(parts) >= 10:
                    for ch_idx, csv_col in enumerate(CSV_MAP):
                        buffers[ch_idx].append(int(parts[csv_col]))
            except (ValueError, UnicodeDecodeError, IndexError):
                pass
    except (serial.SerialException, OSError):
        pass

    x = range(WINDOW)
    for line, buf in zip(plot_lines, buffers):
        line.set_data(x, buf)

    for ax in axes:
        ax.set_xlim(0, WINDOW)

    for i, (key, label, color, _, _) in enumerate(CHANNELS):
        value = buffers[i][-1]
        axes[i].set_title(f"{label}: {value}", fontsize=10, color=color, loc="left")

    return plot_lines


ani = animation.FuncAnimation(
    fig, update, interval=50, blit=False, cache_frame_data=False
)
plt.show()
ser.close()
