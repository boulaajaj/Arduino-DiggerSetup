import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import re

PORT = "COM7"
BAUD = 9600
WINDOW = 200  # data points visible on screen

# Circular buffers for each channel
d2_data = deque([1500] * WINDOW, maxlen=WINDOW)
d4_data = deque([1500] * WINDOW, maxlen=WINDOW)
d7_data = deque([1500] * WINDOW, maxlen=WINDOW)

# Set up plot
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
fig.suptitle("RC Channel Monitor — Live", fontsize=14, fontweight="bold")

line1, = ax1.plot([], [], "b-", linewidth=1.5)
line2, = ax2.plot([], [], "r-", linewidth=1.5)
line3, = ax3.plot([], [], "g-", linewidth=1.5)

for ax, label, color in [
    (ax1, "D2 — CH1 Throttle", "blue"),
    (ax2, "D4 — CH2 Steering", "red"),
    (ax3, "D7 — CH5 Override (3-pos)", "green"),
]:
    ax.set_ylabel("μs")
    ax.set_ylim(800, 2200)
    ax.set_title(label, fontsize=11, color=color, loc="left")
    ax.axhline(y=1500, color="gray", linestyle="--", alpha=0.5, label="center")
    ax.axhline(y=1000, color="gray", linestyle=":", alpha=0.3)
    ax.axhline(y=2000, color="gray", linestyle=":", alpha=0.3)
    ax.grid(True, alpha=0.3)

ax3.set_xlabel("Samples")
plt.tight_layout()

# Serial connection
ser = serial.Serial(PORT, BAUD, timeout=0.1)

# Parse pattern: D4=1500  D2=1500  D7=1500 (D4 read first to avoid blocking)
pattern = re.compile(r"D4=(\d+)\s+D2=(\d+)\s+D7=(\d+)")

def update(frame):
    # Read all available lines
    while ser.in_waiting:
        try:
            raw = ser.readline().decode("utf-8", errors="ignore").strip()
            m = pattern.search(raw)
            if m:
                d4_data.append(int(m.group(1)))
                d2_data.append(int(m.group(2)))
                d7_data.append(int(m.group(3)))
        except:
            pass

    x = range(len(d2_data))
    line1.set_data(x, d2_data)
    line2.set_data(x, d4_data)
    line3.set_data(x, d7_data)

    for ax in (ax1, ax2, ax3):
        ax.set_xlim(0, WINDOW)

    # Show current values in title
    ax1.set_title(f"D2 — CH1 Throttle: {d2_data[-1]}μs", fontsize=11, color="blue", loc="left")
    ax2.set_title(f"D4 — CH2 Steering: {d4_data[-1]}μs", fontsize=11, color="red", loc="left")
    ax3.set_title(f"D7 — CH5 Override: {d7_data[-1]}μs", fontsize=11, color="green", loc="left")

    return line1, line2, line3

ani = animation.FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
plt.show()
ser.close()
