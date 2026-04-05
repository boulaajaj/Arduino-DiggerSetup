"""
Live Plot — ESC Telemetry Packet Decoder
=========================================
Displays byte0 and byte1 from each packet type alongside ESC output.
Correlate visually: which lines move when motor speeds up?

CSV: OutL,OutR, T0_b0,T0_b1, T1_b0,T1_b1, T2_b0,T2_b1, T3_b0,T3_b1,
     T4_b0,T4_b1, T5_b0,T5_b1, T6_b0,T6_b1, T7_b0,T7_b1

Usage: python live_plot_telem.py
"""

from collections import deque
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import serial

PORT   = "COM8"
BAUD   = 115200
WINDOW = 300

# Show ESC output + byte0 for each packet type (byte0 is the primary data carrier)
CHANNELS = [
    ("outl",  "ESC Output Left",               "orangered",    (1200, 1800), 1500, 0),
    ("outr",  "ESC Output Right",              "saddlebrown",  (1200, 1800), 1500, 1),
    ("t0b0",  "Type0 [91 48 F0] b0 (status?)", "lime",         (0, 255),     0,    2),
    ("t1b0",  "Type1 [23 91 F0] b0 (RPM?)",    "deepskyblue",  (0, 255),     0,    4),
    ("t2b0",  "Type2 [47 22 C2] b0 (voltage?)", "gold",        (0, 255),     0,    6),
    ("t3b0",  "Type3 [8C 45 84] b0 (temp?)",   "red",          (0, 255),     0,    8),
    ("t4b0",  "Type4 [48 24 F8] b0",           "violet",       (0, 255),     0,   10),
    ("t5b0",  "Type5 [1D 8A 09] b0 (current?)", "salmon",      (0, 255),     0,   12),
    ("t6b0",  "Type6 [8F 45 84] b0 (temp R?)", "darkred",      (0, 255),     0,   14),
    ("t7b0",  "Type7 [A4 49 FE] b0",           "gray",         (0, 255),     0,   16),
]

buffers = []
for _, _, _, _, center, _ in CHANNELS:
    buffers.append(deque([center] * WINDOW, maxlen=WINDOW))

fig, axes = plt.subplots(len(CHANNELS), 1, figsize=(14, 16), sharex=True)
fig.suptitle("ESC Telemetry — Packet Type Analysis", fontsize=14, fontweight="bold")

plot_lines = []
for i, (key, label, color, ylim, center, _) in enumerate(CHANNELS):
    ax = axes[i]
    line, = ax.plot([], [], color=color, linewidth=1.5)
    plot_lines.append(line)
    ax.set_ylim(*ylim)
    ax.set_title(label, fontsize=9, color=color, loc="left")
    ax.axhline(y=center, color="gray", linestyle="--", alpha=0.3)
    ax.grid(True, alpha=0.3)

axes[-1].set_xlabel("Samples")
plt.tight_layout()

ser = serial.Serial(PORT, BAUD, timeout=0.1)

def update(_frame):
    try:
        while ser.in_waiting:
            try:
                raw = ser.readline().decode("utf-8", errors="ignore").strip()
                if raw.startswith("#") or not raw:
                    continue
                parts = raw.split(",")
                if len(parts) >= 18:
                    for ch_idx, (_, _, _, _, _, csv_col) in enumerate(CHANNELS):
                        try:
                            buffers[ch_idx].append(float(parts[csv_col]))
                        except (ValueError, IndexError):
                            pass
            except (ValueError, UnicodeDecodeError, IndexError):
                pass
    except (serial.SerialException, OSError):
        pass

    x = range(WINDOW)
    for line, buf in zip(plot_lines, buffers):
        line.set_data(x, buf)
    for ax in axes:
        ax.set_xlim(0, WINDOW)
    for i, (key, label, color, _, _, _) in enumerate(CHANNELS):
        value = buffers[i][-1]
        axes[i].set_title(f"{label}: {value:.0f}", fontsize=9, color=color, loc="left")
    return plot_lines

ani = animation.FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
plt.show()
ser.close()
