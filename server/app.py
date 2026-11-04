"""TWSBR telemetry server.

Receives the run log that firmware v2 (``twsbr_v2_tuning_telemetry``) POSTs
after each 1000-sample run and saves, per run:

* ``<out>/speed_<timestamp>.png``          - speed vs. speed setpoint
* ``<out>/self_balancing_<timestamp>.png`` - tilt angle vs. angle setpoint
* ``<out>/line_following_<timestamp>.png`` - line position vs. line setpoint
* ``<out>/run_<timestamp>.csv``            - the raw samples, for later analysis

Run it on a PC in the same network as the robot:

    python app.py --port 5000

and point ``serverName`` in the firmware's ``secrets.h`` to
``http://<pc-ip>:5000/data``.
"""

import argparse
import csv
import datetime
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # headless backend; must be set before importing pyplot
import matplotlib.pyplot as plt  # noqa: E402
from flask import Flask, jsonify, request  # noqa: E402

REQUIRED_KEYS = (
    "measurements_time",
    "angle_adjustment",
    "measurements_speed",
    "measurements_angle",
    "measurements_line",
    "setpoint_time",
    "setpoint_speed",
    "setpoint_angle",
    "setpoint_line",
    "pid_speed",
    "pid_angle",
    "pid_line",
)

app = Flask(__name__)
app.config["OUTPUT_DIR"] = Path(__file__).resolve().parent / "statistic"


def _r(values, ndigits=3):
    return [round(v, ndigits) for v in values]


def _gains(pid):
    return {k: round(pid[k], 3) for k in ("kp", "ki", "kd")}


def _fmt(g):
    return f"KP={g['kp']}, KI={g['ki']}, KD={g['kd']}"


def _plot(path, time, label, values, setpoint, title, ylabel, ylim):
    """Plot one measurement against its setpoint (a constant or a series)."""
    plt.figure(figsize=(10, 5))
    if isinstance(setpoint, list):
        plt.plot(time, setpoint, color="r", label="Setpoint")
    else:
        plt.axhline(setpoint, color="r", linestyle="--", label="Setpoint")
    plt.plot(time, values, label=label)
    plt.title(title)
    plt.xlabel("Time (s)")
    plt.ylabel(ylabel)
    plt.ylim(*ylim)
    plt.legend(loc="upper right", fancybox=True, shadow=True)
    plt.savefig(path)
    plt.close()


@app.get("/")
def status():
    out = app.config["OUTPUT_DIR"]
    runs = sorted(p.stem[len("run_"):] for p in out.glob("run_*.csv")) if out.exists() else []
    return jsonify({"status": "ok", "runs": len(runs), "latest": runs[-1] if runs else None})


@app.post("/data")
def receive_data():
    data = request.get_json(silent=True)
    if not isinstance(data, dict):
        return jsonify({"message": "Body must be a JSON object"}), 400

    missing = [k for k in REQUIRED_KEYS if k not in data]
    if missing:
        return jsonify({"message": "Invalid data format", "missing": missing}), 400

    try:
        time = _r(data["measurements_time"])
        angle_sp = _r(data["angle_adjustment"])
        speed = _r(data["measurements_speed"])
        angle = _r(data["measurements_angle"])
        line = _r(data["measurements_line"])
        speed_sp = data["setpoint_speed"]
        line_sp = data["setpoint_line"]
        pid_speed = _gains(data["pid_speed"])
        pid_angle = _gains(data["pid_angle"])
        pid_line = _gains(data["pid_line"])
    except (TypeError, KeyError) as exc:
        return jsonify({"message": f"Invalid data format: {exc}"}), 400

    out = app.config["OUTPUT_DIR"]
    out.mkdir(parents=True, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    files = {
        "speed": out / f"speed_{stamp}.png",
        "self_balancing": out / f"self_balancing_{stamp}.png",
        "line_following": out / f"line_following_{stamp}.png",
        "csv": out / f"run_{stamp}.csv",
    }

    _plot(files["speed"], time, "Speed Measurement", speed, speed_sp,
          f"Speed Control: Setpoint={speed_sp}, {_fmt(pid_speed)}", "Speed (m/s)", (-5, 5))
    _plot(files["self_balancing"], time, "Angle Measurement", angle, angle_sp,
          f"Self-Balancing: {_fmt(pid_angle)}", "Angle (degrees)", (-15, 15))
    _plot(files["line_following"], time, "Line Measurement", line, line_sp,
          f"Line Following: Setpoint={line_sp}, {_fmt(pid_line)}", "Position Error", (-10, 10))

    with files["csv"].open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time_s", "speed_mps", "speed_setpoint", "angle_deg",
                         "angle_setpoint", "line_pos", "line_setpoint"])
        for t, v, a, a_sp, ln in zip(time, speed, angle, angle_sp, line):
            writer.writerow([t, v, speed_sp, a, a_sp, ln, line_sp])

    print(f"[{stamp}] run saved: {len(time)} samples, balance {_fmt(pid_angle)}")
    return jsonify({"message": "Data received successfully",
                    "files": {k: p.name for k, p in files.items()}}), 200


def main():
    parser = argparse.ArgumentParser(description="TWSBR telemetry server")
    parser.add_argument("--host", default="0.0.0.0", help="interface to listen on (default: all)")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--out", type=Path, default=app.config["OUTPUT_DIR"],
                        help="folder for plots and CSV logs (default: ./statistic)")
    args = parser.parse_args()
    app.config["OUTPUT_DIR"] = args.out.resolve()
    app.run(host=args.host, port=args.port)


if __name__ == "__main__":
    main()
