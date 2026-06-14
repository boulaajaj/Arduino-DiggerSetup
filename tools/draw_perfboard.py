"""Render the interface-board perfboard layout (breadboard-style) to PNG.
Each net = one 5-hole column; +5V/GND on the rails. See INTERFACE-BOARD-PERFBOARD.md.
Run: python tools/draw_perfboard.py
"""
from PIL import Image, ImageDraw, ImageFont

# ---- geometry ----
X0, PITCH, R = 110, 70, 11
COLS = 20
ROWY = {'P': 140, 'A': 285, 'B': 355, 'C': 425, 'D': 495, 'E': 565, 'G': 695}
W, H = X0 + COLS * PITCH + 60, 1035

def cx(c): return X0 + (c - 1) * PITCH
def pos(c, row): return (cx(c), ROWY[row])

img = Image.new("RGB", (W, H), "white")
d = ImageDraw.Draw(img)

def font(sz, bold=False):
    for p in (("C:\\Windows\\Fonts\\arialbd.ttf" if bold else "C:\\Windows\\Fonts\\arial.ttf"),):
        try: return ImageFont.truetype(p, sz)
        except OSError: pass
    return ImageFont.load_default()

F  = font(20); FB = font(22, True); FS = font(16); FT = font(34, True)

def text(xy, s, fnt=F, fill="black", anchor="mm"):
    d.text(xy, s, font=fnt, fill=fill, anchor=anchor)

def hole(c, row, fill="#d9d9d9", outline="#888"):
    x, y = pos(c, row)
    d.ellipse((x-R, y-R, x+R, y+R), fill=fill, outline=outline, width=2)

def wire(a, b, color, w=6):
    d.line((pos(*a), pos(*b)), fill=color, width=w)
    for p in (a, b):
        x, y = pos(*p); d.ellipse((x-5, y-5, x+5, y+5), fill=color)

# ---- title ----
text((W//2, 38), "Interface Board — Perfboard Layout (breadboard-style)", FT)
text((W//2, 74), "Each numbered column = 5 connected holes (A-E).  Top rail = +5V,  bottom rail = GND.", FS, "#444")

# ---- column connectivity shading + numbers + row letters ----
for c in range(1, COLS+1):
    x = cx(c)
    d.rounded_rectangle((x-22, ROWY['A']-22, x+22, ROWY['E']+22), radius=16, fill="#f3f6fb", outline="#e0e6ef")
    text((x, ROWY['A']-40), str(c), FS, "#666")
for row in "ABCDE":
    text((X0-58, ROWY[row]), row, FS, "#666")

# ---- rails ----
d.rounded_rectangle((X0-40, ROWY['P']-26, cx(COLS)+40, ROWY['P']+26), radius=18, fill="#fdecec", outline="#e06666", width=3)
d.rounded_rectangle((X0-40, ROWY['G']-26, cx(COLS)+40, ROWY['G']+26), radius=18, fill="#e9eef7", outline="#5b78b0", width=3)
text((X0-95, ROWY['P']), "+5V", FB, "#c0392b", "lm") if False else None
text((cx(1)-70, ROWY['P']), "+5V", FB, "#c0392b", "rm")
text((cx(1)-70, ROWY['G']), "GND", FB, "#274690", "rm")
for c in range(1, COLS+1):
    hole(c, 'P', "#f7c6c6", "#c0392b")
    hole(c, 'G', "#c9d4ea", "#274690")

# ---- terminal holes ----
for c in range(1, COLS+1):
    for row in "ABCDE":
        hole(c, row)

# ---- net column tints (label the signal nets) ----
NETS = {3:"XBUS_BUS",6:"XBUS_BUS",9:"XBUS_BUS",12:"XBUS_TX",15:"SBUS_IN",18:"SBUS_OUT",19:"NPN_BASE"}
# ---- jumpers (drawn first, under parts) ----
RED, BLK, BLU, GRN = "#c0392b", "#222", "#2c6fb0", "#1f9d55"
# bus ties (col 3-6-9 common) at row D
wire((3,'D'),(6,'D'), BLU); wire((6,'D'),(9,'D'), BLU)
# +5V feeds (header + pins to +5V rail) col 8,11,14,17 via row A
for c in (8,11,14,17): wire((c,'A'),(c,'P'), RED)
# GND feeds (header G pins to GND rail) col 1,4,7,10,13,16 from row E
for c in (1,4,7,10,13,16): wire((c,'E'),(c,'G'), BLK)
# Q1 emitter (col 20) to GND — from row E (below the chip body)
wire((20,'E'),(20,'G'), BLK)

# ---- components ----
def resistor(a, b, label, val):
    (x1,y1),(x2,y2) = pos(*a), pos(*b)
    d.line((x1,y1,x2,y2), fill="#8a6d3b", width=5)
    vertical = (x1 == x2)
    if vertical:                       # value box near the lower hole, ref to the side
        by = max(y1, y2) - 40
        bx = x1
        d.rounded_rectangle((bx-34,by-15,bx+34,by+15), radius=7, fill="#f5e6c8", outline="#8a6d3b", width=2)
        text((bx,by), val, FS, "#5a4520")
        text((bx+58, by), label, FB, "#8a6d3b")
    else:
        mx,my = (x1+x2)//2,(y1+y2)//2
        d.rounded_rectangle((mx-34,my-15,mx+34,my+15), radius=7, fill="#f5e6c8", outline="#8a6d3b", width=2)
        text((mx,my), val, FS, "#5a4520")
        text((mx, my-30), label, FB, "#8a6d3b")
    for p in (a,b):
        xx,yy=pos(*p); d.ellipse((xx-5,yy-5,xx+5,yy+5), fill="#8a6d3b")

resistor((3,'A'),(3,'P'), "R1", "4.7k")     # bus -> +5V
resistor((9,'B'),(12,'B'), "R2", "1k")       # bus -> TX
resistor((15,'B'),(19,'B'), "R3", "10k")     # SBUS_IN -> base
resistor((18,'A'),(18,'P'), "R4", "10k")     # SBUS_OUT -> +5V

# Q1 (2N3904): legs C=18, B=19, E=20 at row C; body drawn BELOW the legs
chip_top, chip_bot = ROWY['C']+20, ROWY['C']+66
d.rounded_rectangle((cx(18)-32, chip_top, cx(20)+32, chip_bot), radius=10, fill="#dff3e6", outline=GRN, width=3)
text(((cx(18)+cx(20))//2, (chip_top+chip_bot)//2), "Q1  2N3904", FS, GRN)
for c,lab in ((18,'C'),(19,'B'),(20,'E')):
    xx,yy=pos(c,'C'); d.ellipse((xx-6,yy-6,xx+6,yy+6), fill=GRN)
    d.line((xx,yy,xx,chip_top), fill=GRN, width=4)
    text((xx, yy-26), lab, FB, GRN)   # C / B / E leg label above its hole

# ---- headers J1..J6 at row E (pins) ----
HEADERS = [("J1 ESC-L",1,False),("J2 ESC-R",4,False),("J4 XBUS-RX>D0",7,True),
           ("J3 XBUS-TX>D1",10,True),("J5 SBUS-IN",13,True),("J6 SBUS-OUT>D12",16,True)]
for name,c0,powered in HEADERS:
    # bracket under the 3 columns
    d.rounded_rectangle((cx(c0)-30, ROWY['E']+24, cx(c0+2)+30, ROWY['E']+58), radius=10, outline="#444", width=2)
    text(((cx(c0)+cx(c0+2))//2, ROWY['E']+41), name, FS, "#222")
    # pin labels: G / + / S
    text((cx(c0),   ROWY['E']+78), "GND", FS, BLK)
    text((cx(c0+1), ROWY['E']+78), ("+5V" if powered else "NC"), FS, (RED if powered else "#c0392b"))
    text((cx(c0+2), ROWY['E']+78), "SIG", FS, "#1f6f8b")
    # SIG pin emphasis
    hole(c0+2,'E', "#bfe3ee", "#1f6f8b")
    if not powered:  # ESC +5V (red wire) — leave NC, mark
        xx,yy = pos(c0+1,'E'); d.line((xx-9,yy-9,xx+9,yy+9), fill="#c0392b", width=3); d.line((xx-9,yy+9,xx+9,yy-9), fill="#c0392b", width=3)

# ---- net labels at signal columns (above the column numbers) ----
for c,n in {3:"BUS",6:"BUS",9:"BUS",12:"TX",15:"S.IN",18:"S.OUT",19:"BASE"}.items():
    text((cx(c), ROWY['A']-78), n, FS, "#1f6f8b")

# ---- legend ----
ly = H-70
items = [("red = +5V jumper",RED),("black = GND jumper",BLK),("blue = bus tie (3-6-9)",BLU),
         ("green = Q1 2N3904",GRN),("tan = resistor","#8a6d3b")]
lx = 120
for s,col in items:
    d.rectangle((lx,ly-8,lx+22,ly+12), fill=col); text((lx+30, ly+2), s, FS, "#333", "lm"); lx += 300
text((W//2, H-30), "Build to the NETLIST in INTERFACE-BOARD-PERFBOARD.md - this image is the visual aid. ESC red/BEC wires = NC.", FS, "#666")

img.save("docs/interface-board-perfboard-layout.png")
print("saved docs/interface-board-perfboard-layout.png", img.size)
