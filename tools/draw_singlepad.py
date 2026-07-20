"""Single-pad perfboard layout for the interface board.

Board: 6 cols x 28 rows, each hole isolated (no copper strips).
Top half (rows 1-13)  = X.BUS subsystem (J1, J2, J3, J4 + R1, R2)
Bottom half (rows 15-28) = S.BUS inverter (J5, J6 + R3, R4, Q1)

All 6 headers on the LEFT side (cols 1-3), horizontal orientation.
Pin order (cols 1->3): GND, +5V/NC, SIG.

Bare-wire rails replace the missing copper:
 - +5V rail   = horizontal bare wire at row 1
 - GND rail   = horizontal bare wire at row 28
 - X.BUS bus  = vertical bare wire in col 3, rows 3-7

J1 and J2 have their middle pin BROKEN OFF so the +5V vertical wire in col 2
can pass through rows 3 and 5 without shorting (J1/J2 BEC red wire is NC).
"""
from PIL import Image, ImageDraw, ImageFont

PITCH = 52
R = 9
X0, Y0 = 220, 120
COLS, ROWS = 6, 28
W = X0 + COLS * PITCH + 380
H = Y0 + ROWS * PITCH + 90

img = Image.new("RGB", (W, H), "white")
d = ImageDraw.Draw(img)


def fnt(sz, b=False):
    try:
        return ImageFont.truetype(
            "C:\\Windows\\Fonts\\arialbd.ttf" if b else "C:\\Windows\\Fonts\\arial.ttf", sz)
    except OSError:
        return ImageFont.load_default()


F, FB, FS, FXS, FT = fnt(16), fnt(18, True), fnt(13), fnt(11), fnt(22, True)


def text(xy, s, f=F, fill="black", a="mm"):
    d.text(xy, s, font=f, fill=fill, anchor=a)


def xy(c, r):
    return (X0 + (c - 1) * PITCH, Y0 + (r - 1) * PITCH)


def hole(c, r, fill="#dddddd", outline="#888"):
    x, y = xy(c, r)
    d.ellipse((x - R, y - R, x + R, y + R), fill=fill, outline=outline, width=2)


def wire(a, b, color, w=5):
    d.line((xy(*a), xy(*b)), fill=color, width=w)
    for p in (a, b):
        x, y = xy(*p)
        d.ellipse((x - 5, y - 5, x + 5, y + 5), fill=color)


RED, BLK, BLU, GRN, TAN, ORN, PUR = "#c0392b", "#222", "#2c6fb0", "#1f9d55", "#8a6d3b", "#d6841d", "#7a3f8a"

# Title
text((W // 2, 30), "Single-Pad Perfboard — 6 cols x 28 rows", FT)
text((W // 2, 55), "Each hole isolated. Bare-wire rails form the strips. Joint count ~30.", FS, "#444")

# Zone shading (X.BUS top, S.BUS bottom)
zx_left, zx_right = X0 - 50, X0 + COLS * PITCH + 50
d.rounded_rectangle((zx_left, Y0 - 20, zx_right, Y0 + 13 * PITCH + 15),
                    radius=16, fill="#fffaec", outline="#d6b85a", width=3)
d.rounded_rectangle((zx_left, Y0 + 14 * PITCH - 10, zx_right, Y0 + 28 * PITCH + 15),
                    radius=16, fill="#eef6fb", outline="#7aa3c0", width=3)

text((zx_right - 12, Y0 + 7 * PITCH - 100), "X.BUS", FB, "#8a6d10", "rm")
text((zx_right - 12, Y0 + 7 * PITCH - 80), "subsystem", FXS, "#8a6d10", "rm")
text((zx_right - 12, Y0 + 21 * PITCH - 100), "S.BUS inverter", FB, "#235684", "rm")

# Col numbers along top
for c in range(1, COLS + 1):
    text((xy(c, 1)[0], Y0 - 35), str(c), FXS, "#666")
# Row numbers along left (every odd + key rows) — moved further left so they don't collide with pin labels
for r in list(range(1, 29, 2)) + [22, 23, 24]:
    text((X0 - 85, xy(1, r)[1]), str(r), FXS, "#666")

# Draw all holes
for c in range(1, COLS + 1):
    for r in range(1, ROWS + 1):
        hole(c, r)

# -------- BARE WIRES --------
# +5V rail horizontal across row 1
wire((1, 1), (6, 1), RED, w=6)
# GND rail horizontal across row 28
wire((1, 28), (6, 28), BLK, w=6)
# Col 1 GND vertical wire (ties all header GND pins to GND rail) — long bare wire
wire((1, 3), (1, 28), BLK, w=4)
# Col 2 +5V vertical wire (ties J3, J4, J5, J6 +5V to rail) — PASSES through col 2 rows 3,5 (J1/J2 NC pins broken off)
wire((2, 1), (2, 18), RED, w=4)
# Col 3 BUS vertical wire (ties J1.SIG, J2.SIG, J4.SIG)
wire((3, 3), (3, 7), BLU, w=4)
# Col 4 SBUS_OUT extension (J6.SIG -> Q1.C)
wire((4, 18), (4, 22), PUR, w=4)
# Col 4 GND extension (Q1.E -> GND rail)
wire((4, 24), (4, 28), BLK, w=4)
# Col 5 SBUS_IN extension (J5.SIG jumper end -> R3 left)
wire((5, 16), (5, 23), ORN, w=4)
# Col 6 +5V extension (rail -> R4 right)
wire((6, 1), (6, 22), RED, w=4)

# Horizontal jumpers
wire((3, 3), (4, 3), BLU, w=4)        # BUS to R1 lower
wire((3, 16), (5, 16), ORN, w=4)      # J5.SIG to SBUS_IN ext
wire((3, 18), (4, 18), PUR, w=4)      # J6.SIG to SBUS_OUT ext

# -------- COMPONENTS --------

def res_v(c, r1, r2, ref_val):
    x1, y1 = xy(c, r1)
    x2, y2 = xy(c, r2)
    d.line((x1, y1, x2, y2), fill=TAN, width=4)
    cy = (y1 + y2) // 2
    d.rounded_rectangle((x1 - 38, cy - 13, x1 + 38, cy + 13), radius=6,
                        fill="#f5e6c8", outline=TAN, width=2)
    text((x1, cy), ref_val, FXS, "#5a4520")


def res_h(c1, c2, r, ref_val):
    x1, y1 = xy(c1, r)
    x2, y2 = xy(c2, r)
    d.line((x1, y1, x2, y2), fill=TAN, width=4)
    cx = (x1 + x2) // 2
    d.rounded_rectangle((cx - 38, y1 - 13, cx + 38, y1 + 13), radius=6,
                        fill="#f5e6c8", outline=TAN, width=2)
    text((cx, y1), ref_val, FXS, "#5a4520")


res_v(4, 1, 3, "R1 4.7k")
res_v(3, 7, 9, "R2 1k")
res_h(4, 5, 23, "R3 10k")
res_h(4, 6, 22, "R4 10k")

# Q1 transistor: vertical in col 4, rows 22 (C), 23 (B), 24 (E)
# Leg labels go to the LEFT (col 3 area is free at these rows) to avoid R3/R4 boxes.
for r, lab in ((22, 'C'), (23, 'B'), (24, 'E')):
    x, y = xy(4, r)
    d.ellipse((x - 6, y - 6, x + 6, y + 6), fill=GRN)
    text((x - 22, y), lab, FB, GRN, "rm")
# Q1 name label — placed to the right of the legs in the empty col 5-6 area at row 25
text((xy(5, 25)[0], xy(5, 25)[1]), "Q1  2N3904", FB, GRN, "lm")
text((xy(5, 26)[0], xy(5, 26)[1]), "(flat side toward you)", FXS, GRN, "lm")

# -------- HEADERS --------
# Each header: 3 pins horizontal at one row. Pin labels GND / +5V (or NC) / SIG.
HEADERS = [
    # name,                  row, powered(mid=+5V?), note
    ("J1 ESC-L",              3, False, "NC pin BROKEN"),
    ("J2 ESC-R",              5, False, "NC pin BROKEN"),
    ("J4 XBUS-RX -> D0",      7, True,  ""),
    ("J3 XBUS-TX -> D1",      9, True,  ""),
    ("J5 SBUS-IN (receiver)",16, True,  ""),
    ("J6 SBUS-OUT -> D12",   18, True,  ""),
]
for name, r, powered, note in HEADERS:
    # bracket around 3 pins
    x1, y1 = xy(1, r)
    x3, y3 = xy(3, r)
    d.rounded_rectangle((x1 - 22, y1 - 22, x3 + 22, y3 + 22), radius=10,
                        outline="#222", width=2)
    text((x3 + 45, y1 - 12), name, FS, "#222", "lm")
    if note:
        text((x3 + 45, y1 + 12), note, FXS, "#c0392b", "lm")
    # pin labels under each col
    pin_labels = ("GND", "+5V" if powered else "NC", "SIG")
    pin_colors = (BLK, RED if powered else RED, "#1f6f8b")
    for col, (lab, col_color) in enumerate(zip(pin_labels, pin_colors), start=1):
        x, y = xy(col, r)
        text((x, y + 26), lab, FXS, col_color)
        if lab == "SIG":
            hole(col, r, "#bfe3ee", "#1f6f8b")
        if lab == "NC":
            d.line((x - 10, y - 10, x + 10, y + 10), fill="#c0392b", width=3)
            d.line((x - 10, y + 10, x + 10, y - 10), fill="#c0392b", width=3)

# -------- RAIL LABELS --------
text((X0 - 110, xy(1, 1)[1]), "+5V rail", FB, RED, "rm")
text((X0 - 110, xy(1, 28)[1]), "GND rail", FB, BLK, "rm")

# -------- LEGEND on the right --------
lx = X0 + COLS * PITCH + 60
ly = Y0 + 20
text((lx, ly), "Legend", FB, "#222", "lm")
ly += 28
items = [
    ("+5V wire / rail",         RED),
    ("GND wire / rail",         BLK),
    ("X.BUS bus (blue)",        BLU),
    ("S.BUS in (orange)",       ORN),
    ("S.BUS out (purple)",      PUR),
    ("Q1 2N3904 (green)",       GRN),
    ("Resistor (tan)",          TAN),
]
for s, c in items:
    d.rectangle((lx, ly - 8, lx + 24, ly + 12), fill=c)
    text((lx + 34, ly + 2), s, FS, "#333", "lm")
    ly += 26

# Build notes
ly += 18
text((lx, ly), "Build notes", FB, "#222", "lm")
ly += 24
notes = [
    "* Lay the bare-wire rails FIRST",
    "  (+5V along row 1, GND along row 28).",
    "* Then col 1 (GND), col 2 (+5V),",
    "  col 3 (BUS) vertical wires.",
    "* Snap off pin 2 of J1 and J2 with pliers",
    "  so the col 2 +5V wire can pass through",
    "  rows 3 and 5 without touching them.",
    "* Solder all header GND pins to col 1 wire.",
    "* Mount components last.",
    "* Estimated joints: ~30.",
]
for s in notes:
    text((lx, ly), s, FXS, "#333", "lm")
    ly += 18

img.save("docs/interface-board-singlepad-layout.png")
print("saved docs/interface-board-singlepad-layout.png", img.size)
