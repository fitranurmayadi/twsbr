"""Generate docs/images/control-system-{light,dark}.svg (cascade PID block diagram).

    python docs/diagrams/make_control_diagram.py

Edit the layout constants or labels below and re-run; no dependencies needed.
"""
import pathlib
import sys

OUT = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else pathlib.Path(__file__).resolve().parent.parent / "images"

THEMES = {
    "light": dict(bg="#ffffff", fg="#1f2328", muted="#59636e", nfill="#f6f8fa", nstroke="#57606a",
                  speed="#9a6700", speed_fill="#fff8c5", bal="#0969da", bal_fill="#ddf4ff",
                  line="#1a7f37", line_fill="#dafbe1"),
    "dark": dict(bg="#0d1117", fg="#e6edf3", muted="#9198a1", nfill="#161b22", nstroke="#8b949e",
                 speed="#d29922", speed_fill="#2b2111", bal="#4493f8", bal_fill="#0f2742",
                 line="#3fb950", line_fill="#11281a"),
}

# ---- layout -------------------------------------------------------------
X0 = 24                       # setpoint arrows start
SX = 160                      # summing junctions of speed + line loops
R = 16
PW, PH = 124, 54              # PID block size
P1 = SX + R + 22              # speed / line PID left edge
SX2 = P1 + PW + 66            # balance summing junction
P2 = SX2 + R + 22             # balance PID left edge
MX, MW = P2 + PW + 56, 126    # mixer
RX, RW = MX + MW + 74, 136    # robot
W = RX + RW + 22
Y0, Y1 = 128, 250             # line row, main (speed -> balance) row
MY0, MY1 = Y0 - 38, Y1 + 38
RY0, RY1 = 106, 272
YL, YR = 168, 232             # PWM left / right rows
QTR = (MX + 16, 10, W - 22, 58)
IMU = (MX, 310, RX + 44, 358)
ENC = (MX, 384, W - 22, 432)
H = 478

FONT = "-apple-system, BlinkMacSystemFont, 'Segoe UI', Helvetica, Arial, sans-serif"


def svg(theme):
    c = THEMES[theme]
    e = []
    add = e.append

    def line(points, color, dashed=False, arrow=True):
        d = " ".join(f"{x},{y}" for x, y in points)
        dash = ' stroke-dasharray="7 5"' if dashed else ""
        mk = f' marker-end="url(#a-{color.strip("#")})"' if arrow else ""
        add(f'<polyline points="{d}" fill="none" stroke="{color}" stroke-width="1.8"{dash}{mk}/>')

    def text(x, y, s, size=14, color=None, weight=400, anchor="middle", italic=False):
        style = ' font-style="italic"' if italic else ""
        add(f'<text x="{x}" y="{y}" font-size="{size}" fill="{color or c["fg"]}" font-weight="{weight}" '
            f'text-anchor="{anchor}"{style}>{s}</text>')

    def sub(s):
        # subscript, then a zero-width space to bring the baseline back for any following text
        return f'<tspan font-size="0.72em" dy="4">{s}</tspan><tspan dy="-4">​</tspan>'

    def box(x0, y0, x1, y1, fill, stroke, rx=8):
        add(f'<rect x="{x0}" y="{y0}" width="{x1 - x0}" height="{y1 - y0}" rx="{rx}" '
            f'fill="{fill}" stroke="{stroke}" stroke-width="1.6"/>')

    def junction(x, y, color):
        add(f'<circle cx="{x}" cy="{y}" r="{R}" fill="{c["bg"]}" stroke="{color}" stroke-width="1.8"/>')
        text(x, y + 5, "Σ", 15, color, 600)

    def pid(x, y, title, subtitle, k):
        box(x, y - PH / 2, x + PW, y + PH / 2, c[f"{k}_fill"], c[k])
        text(x + PW / 2, y - 3, title, 14.5, c["fg"], 600)
        text(x + PW / 2, y + 16, subtitle, 12, c[k])

    def sensor(rect, title, subtitle, k):
        x0, y0, x1, y1 = rect
        box(x0, y0, x1, y1, c[f"{k}_fill"], c[k])
        text((x0 + x1) / 2, y0 + 20, title, 13.5, c["fg"], 600)
        text((x0 + x1) / 2, y0 + 38, subtitle, 12, c["muted"])

    colors = {c["fg"], c["speed"], c["bal"], c["line"]}
    defs = "".join(
        f'<marker id="a-{col.strip("#")}" viewBox="0 0 10 10" refX="10" refY="5" markerWidth="9" '
        f'markerHeight="9" markerUnits="userSpaceOnUse" orient="auto">'
        f'<path d="M0,0 L10,5 L0,10 z" fill="{col}"/></marker>' for col in sorted(colors))

    add(f'<rect width="{W}" height="{H}" fill="{c["bg"]}"/>')

    # ---- setpoints ----
    for y, label, desc in ((Y0, f"line{sub('sp')} = 0", "line setpoint"),
                           (Y1, f"v{sub('sp')} = 1 m/s", "speed setpoint")):
        line([(X0, y), (SX - R, y)], c["fg"])
        text(X0 + 2, y - 9, label, 13.5, anchor="start")
        text(X0 + 2, y + 19, desc, 12, c["muted"], anchor="start")

    # ---- line loop (row 0) ----
    junction(SX, Y0, c["line"])
    text(SX - 25, Y0 - 7, "+", 14, c["fg"], 600)
    text(SX - 13, Y0 - 22, "−", 15, c["line"], 600)
    line([(SX + R, Y0), (P1, Y0)], c["fg"])
    pid(P1, Y0, "Line PID", "steering", "line")
    line([(P1 + PW, Y0), (MX, Y0)], c["fg"])
    text((P1 + PW + MX) / 2, Y0 - 9, f"u{sub('steer')}", 14, italic=True)
    text((P1 + PW + MX) / 2, Y0 + 19, "±100", 12, c["muted"])

    # ---- speed -> balance cascade (row 1) ----
    junction(SX, Y1, c["speed"])
    text(SX - 25, Y1 - 7, "+", 14, c["fg"], 600)
    text(SX - 13, Y1 + 34, "−", 15, c["speed"], 600)
    line([(SX + R, Y1), (P1, Y1)], c["fg"])
    pid(P1, Y1, "Speed PID", "outer loop", "speed")
    line([(P1 + PW, Y1), (SX2 - R, Y1)], c["fg"])
    text((P1 + PW + SX2 - R) / 2 - 10, Y1 - 9, f"θ{sub('sp')}", 14, italic=True)
    text((P1 + PW + SX2 - R) / 2 - 10, Y1 + 19, "±2°", 12, c["muted"])
    junction(SX2, Y1, c["bal"])
    text(SX2 - 25, Y1 - 7, "+", 14, c["fg"], 600)
    text(SX2 - 13, Y1 + 34, "−", 15, c["bal"], 600)
    line([(SX2 + R, Y1), (P2, Y1)], c["fg"])
    pid(P2, Y1, "Balance PID", "inner loop", "bal")
    line([(P2 + PW, Y1), (MX, Y1)], c["fg"])
    text((P2 + PW + MX) / 2, Y1 - 9, f"u{sub('bal')}", 14, italic=True)

    # ---- mixer and robot ----
    box(MX, MY0, MX + MW, MY1, c["nfill"], c["nstroke"])
    mx = MX + MW / 2
    text(mx, (MY0 + MY1) / 2 - 12, "Motor mixer", 14.5, c["fg"], 600)
    text(mx, (MY0 + MY1) / 2 + 10, f"u{sub('bal')} ∓ u{sub('steer')}", 12.5, c["muted"])
    text(mx, (MY0 + MY1) / 2 + 30, "clamp ±255", 12, c["muted"])
    for y, s in ((YL, "L"), (YR, "R")):
        line([(MX + MW, y), (RX, y)], c["fg"])
        text((MX + MW + RX) / 2, y - 9, f"PWM{sub(s)}", 13)
    box(RX, RY0, RX + RW, RY1, c["nfill"], c["nstroke"])
    text(RX + RW / 2, (RY0 + RY1) / 2 - 4, "Robot", 16, c["fg"], 600)
    text(RX + RW / 2, (RY0 + RY1) / 2 + 17, "2 × BLDC motor", 12, c["muted"])

    # ---- sensors + feedback ----
    sensor(QTR, "QTR-8A line sensor (8 × IR)", "weighted average → line position", "line")
    qx = RX + RW - 34
    line([(qx, RY0), (qx, QTR[3])], c["line"], dashed=True)
    qy = (QTR[1] + QTR[3]) / 2
    line([(QTR[0], qy), (SX, qy), (SX, Y0 - R)], c["line"], dashed=True)
    text((SX + QTR[0]) / 2, qy - 7, "line position", 12.5, c["line"])

    sensor(IMU, "MPU6050 IMU", "gyro + accel → Kalman filter", "bal")
    ix = RX + 22
    line([(ix, RY1), (ix, IMU[1])], c["bal"], dashed=True)
    iy = (IMU[1] + IMU[3]) / 2
    line([(IMU[0], iy), (SX2, iy), (SX2, Y1 + R)], c["bal"], dashed=True)
    text((SX2 + IMU[0]) / 2, iy - 7, "tilt angle θ", 12.5, c["bal"])

    sensor(ENC, "Wheel encoders (left, right)", f"v = (v{sub('L')} + v{sub('R')}) / 2", "speed")
    ex = RX + RW - 18
    line([(ex, RY1), (ex, ENC[1])], c["speed"], dashed=True)
    ey = (ENC[1] + ENC[3]) / 2
    line([(ENC[0], ey), (SX, ey), (SX, Y1 + R)], c["speed"], dashed=True)
    text((SX + ENC[0]) / 2, ey - 7, "wheel speed v", 12.5, c["speed"])

    # ---- legend ----
    ly = H - 16
    x = X0
    add(f'<line x1="{x}" y1="{ly - 4}" x2="{x + 26}" y2="{ly - 4}" stroke="{c["fg"]}" stroke-width="1.8"/>')
    text(x + 34, ly, "control signal", 12, c["muted"], anchor="start")
    x += 140
    for k, label in (("speed", "speed feedback"), ("bal", "angle feedback"), ("line", "line feedback")):
        add(f'<line x1="{x}" y1="{ly - 4}" x2="{x + 26}" y2="{ly - 4}" stroke="{c[k]}" '
            f'stroke-width="1.8" stroke-dasharray="7 5"/>')
        text(x + 34, ly, label, 12, c["muted"], anchor="start")
        x += 145
    text(W - 22, ly, "all loops run every 10 ms", 12, c["muted"], anchor="end")

    body = "\n  ".join(e)
    return (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" '
            f'font-family="{FONT}" role="img" aria-labelledby="t d">\n'
            f'  <title id="t">TWSBR cascade PID control system</title>\n'
            f'  <desc id="d">Speed PID sets the tilt-angle setpoint for the balance PID; the line PID adds '
            f'differential steering; a mixer turns both into left and right motor PWM. Encoders, the MPU6050 '
            f'with a Kalman filter and the QTR-8A line sensor close the three loops.</desc>\n'
            f'  <defs>{defs}</defs>\n  {body}\n</svg>\n')


for theme in THEMES:
    path = OUT / f"control-system-{theme}.svg"
    path.write_text(svg(theme), encoding="utf-8", newline="\n")
    print(path.name, W, "x", H)
