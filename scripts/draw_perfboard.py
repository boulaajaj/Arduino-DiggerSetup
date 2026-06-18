"""Render the interface-board perfboard layout (breadboard-style) to PNG.

Two-sided layout:
  LEFT  side = INPUTS  (ESC-L tel, ESC-R tel, receiver S.BUS)
  RIGHT side = OUTPUTS to Arduino (D0 RX, D1 TX, D12 inv. S.BUS)
  MIDDLE     = components (R1, R2, R3, R4, Q1) — signal flow left -> right.

Run: python scripts/draw_perfboard.py
See: docs/INTERFACE-BOARD-PERFBOARD.md (the written guide).
"""
from PIL import Image, ImageDraw, ImageFont

# ---------------- geometry ----------------
X0, PITCH, R = 120, 70, 11
COLS = 24
ROWY = {'P': 150, 'A': 290, 'B': 360, 'C': 430, 'D': 500, 'E': 570, 'G': 700}
W, H = X0 + COLS * PITCH + 70, 1040


def cx(c):   return X0 + (c - 1) * PITCH
def pos(c, row): return (cx(c), ROWY[row])

img = Image.new("RGB", (W, H), "white")
d = ImageDraw.Draw(img)


def font(sz, bold=False):
    try:
        return ImageFont.truetype(
            "C:\\Windows\\Fonts\\arialbd.ttf" if bold else "C:\\Windows\\Fonts\\arial.ttf",
            sz,
        )
    except OSError:
        return ImageFont.load_default()


F  = font(20)
FB = font(22, True)
FS = font(16)
FXS = font(14)
FT = font(34, True)
FH = font(26, True)


def text(xy, s, fnt=F, fill="black", anchor="mm"):
    d.text(xy, s, font=fnt, fill=fill, anchor=anchor)


def hole(c, row, fill="#d9d9d9", outline="#888"):
    x, y = pos(c, row)
    d.ellipse((x - R, y - R, x + R, y + R), fill=fill, outline=outline, width=2)


def wire(a, b, color, w=6, dots=True):
    d.line((pos(*a), pos(*b)), fill=color, width=w)
    if dots:
        for p in (a, b):
            x, y = pos(*p)
            d.ellipse((x - 5, y - 5, x + 5, y + 5), fill=color)


def join_dot(c, row, color):
    x, y = pos(c, row)
    d.ellipse((x - 6, y - 6, x + 6, y + 6), fill=color)


# ---------------- zone shading (input / middle / output) ----------------
# Input zone: cols 1-9. Middle: 10-15. Output: 16-24.
ZONE_INPUT_X1 = cx(1) - 50
ZONE_INPUT_X2 = cx(9) + 40
ZONE_MID_X1 = cx(10) - 30
ZONE_MID_X2 = cx(15) + 30
ZONE_OUT_X1 = cx(16) - 40
ZONE_OUT_X2 = cx(24) + 50
ZONE_Y1 = ROWY['A'] - 105
ZONE_Y2 = ROWY['E'] + 100

d.rounded_rectangle((ZONE_INPUT_X1, ZONE_Y1, ZONE_INPUT_X2, ZONE_Y2), radius=18,
                    fill="#fffaec", outline="#d6b85a", width=3)
d.rounded_rectangle((ZONE_MID_X1, ZONE_Y1, ZONE_MID_X2, ZONE_Y2), radius=18,
                    fill="#f4f6f9", outline="#bcc3cf", width=2)
d.rounded_rectangle((ZONE_OUT_X1, ZONE_Y1, ZONE_OUT_X2, ZONE_Y2), radius=18,
                    fill="#eef6fb", outline="#7aa3c0", width=3)


# ---------------- title and zone labels ----------------
text((W // 2, 38), "Interface Board — Perfboard Layout (breadboard-style)", FT)
text((W // 2, 74),
     "INPUTS (from ESCs + receiver) on the LEFT   |   COMPONENTS in the MIDDLE   |   OUTPUTS (to Arduino) on the RIGHT",
     FS, "#444")

text(((ZONE_INPUT_X1 + ZONE_INPUT_X2) // 2, ZONE_Y1 + 22), "← INPUTS (from ESCs + Receiver)", FH, "#8a6d10")
text(((ZONE_MID_X1 + ZONE_MID_X2) // 2, ZONE_Y1 + 22), "COMPONENTS", FH, "#444")
text(((ZONE_OUT_X1 + ZONE_OUT_X2) // 2, ZONE_Y1 + 22), "OUTPUTS (to Arduino) →", FH, "#235684")


# ---------------- column shading + numbers + row letters ----------------
for c in range(1, COLS + 1):
    x = cx(c)
    d.rounded_rectangle((x - 22, ROWY['A'] - 22, x + 22, ROWY['E'] + 22),
                        radius=16, fill="#ffffff", outline="#dde2ea", width=1)
    text((x, ROWY['A'] - 44), str(c), FS, "#666")
for row in "ABCDE":
    text((X0 - 70, ROWY[row]), row, FS, "#666")


# ---------------- power rails ----------------
d.rounded_rectangle((X0 - 50, ROWY['P'] - 26, cx(COLS) + 50, ROWY['P'] + 26),
                    radius=18, fill="#fdecec", outline="#c0392b", width=3)
d.rounded_rectangle((X0 - 50, ROWY['G'] - 26, cx(COLS) + 50, ROWY['G'] + 26),
                    radius=18, fill="#e9eef7", outline="#274690", width=3)
text((cx(1) - 80, ROWY['P']), "+5V", FB, "#c0392b", "rm")
text((cx(1) - 80, ROWY['G']), "GND", FB, "#274690", "rm")
for c in range(1, COLS + 1):
    hole(c, 'P', "#f7c6c6", "#c0392b")
    hole(c, 'G', "#c9d4ea", "#274690")


# ---------------- terminal holes ----------------
for c in range(1, COLS + 1):
    for row in "ABCDE":
        hole(c, row)


# ---------------- colors ----------------
RED, BLK, BLU, GRN, TAN = "#c0392b", "#222", "#2c6fb0", "#1f9d55", "#8a6d3b"


# ---------------- the X.BUS "bus highway" ----------------
# Bus net members: J1.sig (col 3), J2.sig (col 6), bus anchor (col 10), J4.sig (col 16).
# Draw a single visible blue "highway" at row D from col 3 to col 16 with solder
# dots at each joining column.
for a, b in [(3, 6), (6, 10), (10, 16)]:
    wire((a, 'D'), (b, 'D'), BLU, w=7, dots=False)
for c in (3, 6, 10, 16):
    join_dot(c, 'D', BLU)
# label above the bus
text(((cx(3) + cx(16)) // 2, ROWY['D'] - 24), "X.BUS bus", FS, BLU)


# ---------------- jumper wires ----------------
# +5V feeds (header +5V pins -> +5V rail). +5V pins at cols 8 (J5), 17 (J4), 20 (J3), 23 (J6).
for c in (8, 17, 20, 23):
    wire((c, 'A'), (c, 'P'), RED)
# GND feeds (header GND pins -> GND rail). GND pins at cols 1, 4, 7 (left)  and 18, 21, 24 (right).
for c in (1, 4, 7, 18, 21, 24):
    wire((c, 'E'), (c, 'G'), BLK)
# Q1 emitter (col 13) -> GND rail
wire((13, 'E'), (13, 'G'), BLK)
# XBUS_TX jumper: R2 right (col 11) -> J3.sig (col 19)
wire((11, 'C'), (19, 'C'), BLU)
join_dot(11, 'C', BLU)
join_dot(19, 'C', BLU)
# SBUS_IN jumper: J5.sig (col 9) -> R3 left anchor (col 12)
wire((9, 'C'), (12, 'C'), "#c97c2a")
join_dot(9, 'C', "#c97c2a")
join_dot(12, 'C', "#c97c2a")
# SBUS_OUT jumper: Q1.C / R4 anchor (col 15) -> J6.sig (col 22)
wire((15, 'C'), (22, 'C'), "#7a3f8a")
join_dot(15, 'C', "#7a3f8a")
join_dot(22, 'C', "#7a3f8a")


# ---------------- resistors ----------------
def resistor(a, b, label, val):
    """Draw resistor; the value box contains BOTH ref and value (no separate ref text)."""
    (x1, y1), (x2, y2) = pos(*a), pos(*b)
    d.line((x1, y1, x2, y2), fill=TAN, width=5)
    vertical = (x1 == x2)
    box_w = 58
    if vertical:
        # Sit the value box just above row A so it doesn't sit on top of the
        # column number or the net-name label up near the +5V rail.
        by, bx = max(y1, y2) - 22, x1
    else:
        bx, by = (x1 + x2) // 2, (y1 + y2) // 2
    d.rounded_rectangle((bx - box_w, by - 13, bx + box_w, by + 13),
                        radius=8, fill="#f5e6c8", outline=TAN, width=2)
    text((bx, by), f"{label}  {val}", FB, "#5a4520")
    for p in (a, b):
        xx, yy = pos(*p)
        d.ellipse((xx - 5, yy - 5, xx + 5, yy + 5), fill=TAN)


# R1: BUS anchor (col 10) -> +5V rail.
resistor((10, 'A'), (10, 'P'), "R1", "4.7k")
# R2: BUS anchor (col 10) -> col 11 at row B.
resistor((10, 'B'), (11, 'B'), "R2", "1k")
# R3: col 12 (SBUS_IN anchor) -> col 14 (Q1 base / NPN_BASE).
resistor((12, 'B'), (14, 'B'), "R3", "10k")
# R4: SBUS_OUT (col 15) -> +5V rail.
resistor((15, 'A'), (15, 'P'), "R4", "10k")


# ---------------- Q1 transistor body (legs at row C, body below) ----------------
# Q1 legs at cols 13 (E), 14 (B), 15 (C). Pinout 2N3904 = E-B-C with flat side
# facing viewer, leads pointing down.
chip_top, chip_bot = ROWY['C'] + 22, ROWY['C'] + 70
d.rounded_rectangle((cx(13) - 32, chip_top, cx(15) + 32, chip_bot), radius=10,
                    fill="#dff3e6", outline=GRN, width=3)
text(((cx(13) + cx(15)) // 2, (chip_top + chip_bot) // 2),
     "Q1  2N3904 (flat side toward you)", FXS, GRN)
for c, lab in ((13, 'E'), (14, 'B'), (15, 'C')):
    xx, yy = pos(c, 'C')
    d.ellipse((xx - 6, yy - 6, xx + 6, yy + 6), fill=GRN)
    d.line((xx, yy, xx, chip_top), fill=GRN, width=4)
    text((xx, yy - 26), lab, FB, GRN)


# ---------------- net labels (above the column they live in) ----------------
NETS = {
    3: "BUS", 6: "BUS", 10: "BUS",
    11: "TX",
    9: "S.IN", 12: "S.IN",
    14: "BASE",
    15: "S.OUT",
    16: "BUS",
    19: "TX",
    22: "S.OUT",
}
for c, n in NETS.items():
    text((cx(c), ROWY['A'] - 70), n, FXS, "#1f6f8b")


# ---------------- headers J1..J6 ----------------
# Input side: J1 (cols 1-3), J2 (cols 4-6), J5 (cols 7-9).
# Pin order (left-to-right): GND, +5V/NC, SIG  (SIG closest to middle).
# Output side: J4 (cols 16-18), J3 (cols 19-21), J6 (cols 22-24).
# Pin order (left-to-right): SIG, +5V, GND  (SIG closest to middle).
HEADERS = [
    # name, col_start, sig_left?  +5V?  (sig_left True => SIG on leftmost col)
    ("J1 ESC-L  (yellow=SIG)",       1, False, False),  # GND, NC,  SIG (right)
    ("J2 ESC-R  (yellow=SIG)",       4, False, False),
    ("J5 SBUS-IN  (from receiver)",  7, False, True),   # GND, +5V, SIG
    ("J4 XBUS-RX → D0",             16, True,  True),   # SIG, +5V, GND
    ("J3 XBUS-TX → D1",             19, True,  True),
    ("J6 SBUS-OUT → D12",           22, True,  True),
]
for name, c0, sig_left, powered in HEADERS:
    # bracket spans the 3 cols
    d.rounded_rectangle((cx(c0) - 30, ROWY['E'] + 26,
                         cx(c0 + 2) + 30, ROWY['E'] + 60),
                        radius=10, outline="#444", width=2)
    text(((cx(c0) + cx(c0 + 2)) // 2, ROWY['E'] + 43), name, FS, "#222")
    # pin labels: derive from sig_left and powered
    if sig_left:
        pin_cols = [(c0, "SIG"), (c0 + 1, "+5V" if powered else "NC"), (c0 + 2, "GND")]
        sig_col = c0
    else:
        pin_cols = [(c0, "GND"), (c0 + 1, "+5V" if powered else "NC"), (c0 + 2, "SIG")]
        sig_col = c0 + 2
    for c, lab in pin_cols:
        color = "#1f6f8b" if lab == "SIG" else (RED if lab == "+5V" else ("#c0392b" if lab == "NC" else BLK))
        text((cx(c), ROWY['E'] + 80), lab, FS, color)
        if lab == "SIG":
            hole(c, 'E', "#bfe3ee", "#1f6f8b")
        elif lab == "NC":
            xx, yy = pos(c, 'E')
            d.line((xx - 9, yy - 9, xx + 9, yy + 9), fill="#c0392b", width=3)
            d.line((xx - 9, yy + 9, xx + 9, yy - 9), fill="#c0392b", width=3)


# (Inter-zone flow arrows were overlapping the resistor boxes. The zone titles
#  "← INPUTS" and "OUTPUTS →" already convey direction, and the X.BUS bus
#  highway at row D visibly crosses the zone boundary, so the flow is clear.)


# ---------------- legend ----------------
ly = H - 60
items = [
    ("red = +5V wire", RED),
    ("black = GND wire", BLK),
    ("blue = X.BUS bus / TX", BLU),
    ("orange = S.BUS in jumper", "#c97c2a"),
    ("purple = S.BUS out jumper", "#7a3f8a"),
    ("green = Q1 2N3904", GRN),
    ("tan = resistor", TAN),
]
lx = 90
for s, col in items:
    d.rectangle((lx, ly - 8, lx + 22, ly + 12), fill=col)
    text((lx + 30, ly + 2), s, FS, "#333", "lm")
    lx += 220

text((W // 2, H - 28),
     "Build to the NETLIST in INTERFACE-BOARD-PERFBOARD.md — this image is the visual aid.   ESC red/BEC wires (J1·J2 mid pin) = NC.",
     FS, "#666")


img.save("docs/interface-board-perfboard-layout.png")
print("saved docs/interface-board-perfboard-layout.png", img.size)
