"""
Live Plot — Tank Mixer Monitor
===============================
Real-time 7-panel plot for the Arduino tank mixer.
Reads serial data from the Arduino and displays all inputs + outputs.

Panels:
  1. D2  — RC CH1 Throttle     (blue)
  2. D4  — RC CH2 Steering     (red)
  3. D7  — RC CH5 Override     (green)
  4. A0  — Joystick Y          (purple)
  5. A1  — Joystick X          (cyan)
  6. L   — Left ESC output     (orange)
  7. R   — Right ESC output    (brown)

Usage:
  python live_plot.py
  (or Ctrl+Shift+B in VS Code -> "Live Plot")

Expects serial format:
  D4=1500  D2=1500  D7=1000  A0=512  A1=512  L=1500  R=1500
"""

import re
from collections import deque

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import serial

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
PORT   = "COM7"
BAUD   = 9600
WINDOW = 200  # Number of data points visible on screen

# Channel definitions: (key, label, color, y_range, center, unit)
CHANNELS = [
    ("d2",   "D2 — CH1 Throttle",    "blue",        (800, 2200), 1500, "us"),
    ("d4",   "D4 — CH2 Steering",    "red",         (800, 2200), 1500, "us"),
    ("d7",   "D7 — CH5 Override",    "green",       (800, 2200), 1500, "us"),
    ("a0",   "A0 — Joystick Y",      "purple",      (0, 1023),   512,  "ADC"),
    ("a1",   "A1 — Joystick X",      "darkcyan",    (0, 1023),   512,  "ADC"),
    ("lesc", "L — Left ESC Out",     "darkorange",  (800, 2200), 1500, "us"),
    ("resc", "R — Right ESC Out",    "saddlebrown", (800, 2200), 1500, "us"),
]

# Indices of servo-range channels that get 1000/2000 reference lines
SERVO_CHANNELS = [0, 1, 2, 5, 6]

# Serial parse pattern (must match Arduino output format)
PATTERN = re.compile(
    r"D4=(\d+)\s+D2=(\d+)\s+D7=(\d+)\s+A0=(\d+)\s+A1=(\d+)\s+L=(\d+)\s+R=(\d+)"
)

# Regex group index -> buffer index mapping (D4 is sent first but displayed second)
PARSE_ORDER = [1, 0, 2, 3, 4, 5, 6]  # group1->d4, group2->d2, etc.

# -----------------------------------------------------------------------------
# Data Buffers
# -----------------------------------------------------------------------------
buffers = {}
for key, _, _, _, center, _ in CHANNELS:
    buffers[key] = deque([center] * WINDOW, maxlen=WINDOW)

buffer_list = [buffers[ch[0]] for ch in CHANNELS]

# -----------------------------------------------------------------------------
# Plot Setup
# -----------------------------------------------------------------------------
fig, axes = plt.subplots(len(CHANNELS), 1, figsize=(10, 11), sharex=True)
fig.suptitle("Tank Mixer Monitor — Live", fontsize=14, fontweight="bold")

plot_lines = []
for i, (key, label, color, ylim, center, unit) in enumerate(CHANNELS):
    ax = axes[i]
    line, = ax.plot([], [], color=color, linewidth=1.5)
    plot_lines.append(line)

    ax.set_ylabel(unit)
    ax.set_ylim(*ylim)
    ax.set_title(label, fontsize=10, color=color, loc="left")
    ax.axhline(y=center, color="gray", linestyle="--", alpha=0.5)
    ax.grid(True, alpha=0.3)

for i in SERVO_CHANNELS:
    axes[i].axhline(y=1000, color="gray", linestyle=":", alpha=0.3)
    axes[i].axhline(y=2000, color="gray", linestyle=":", alpha=0.3)

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
    """Read all available serial lines and update plot data."""
    while ser.in_waiting:
        try:
            raw = ser.readline().decode("utf-8", errors="ignore").strip()
            match = PATTERN.search(raw)
            if match:
                for group_idx, buf_idx in enumerate(PARSE_ORDER):
                    buffer_list[buf_idx].append(int(match.group(group_idx + 1)))
        except (ValueError, UnicodeDecodeError):
            pass

    x = range(WINDOW)
    for line, buf in zip(plot_lines, buffer_list):
        line.set_data(x, buf)

    for ax in axes:
        ax.set_xlim(0, WINDOW)

    for i, (key, label, color, _, _, unit) in enumerate(CHANNELS):
        value = buffer_list[i][-1]
        suffix = "us" if unit == "us" else ""
        axes[i].set_title(
            f"{label}: {value}{suffix}", fontsize=10, color=color, loc="left"
        )

    return plot_lines


ani = animation.FuncAnimation(
    fig, update, interval=50, blit=False, cache_frame_data=False
)
plt.show()
ser.close()
