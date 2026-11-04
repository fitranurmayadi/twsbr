"""Send a synthetic run to the telemetry server - test it without the robot.

    python send_test_payload.py                       # http://127.0.0.1:5000/data
    python send_test_payload.py http://192.168.1.10:5000/data

The payload has the same shape as the one firmware v2 sends: 1000 samples at
10 ms with a damped tilt-angle response, a slowly converging speed and a
line position that wanders around the centre. Standard library only.
"""

import json
import math
import random
import sys
import urllib.request

N = 1000
DT = 0.01


def build_payload(seed=1):
    rng = random.Random(seed)
    time = [round((i + 1) * DT, 3) for i in range(N)]
    angle_sp = [0.0 if t < 1.0 else 2.0 for t in time]
    angle = [
        sp - 2.0 * math.exp(-1.5 * t) * math.cos(6 * t) + rng.gauss(0, 0.15)
        for t, sp in zip(time, angle_sp)
    ]
    speed = [1.0 * (1 - math.exp(-0.8 * t)) + rng.gauss(0, 0.05) for t in time]
    line = [1.5 * math.sin(0.7 * t) * math.exp(-0.2 * t) + rng.gauss(0, 0.1) for t in time]
    return {
        "setpoint_time": DT,
        "measurements_time": time,
        "setpoint_speed": 1.0,
        "measurements_speed": speed,
        "setpoint_angle": angle_sp[-1],
        "angle_adjustment": angle_sp,
        "measurements_angle": angle,
        "setpoint_line": 0.0,
        "measurements_line": line,
        "pid_speed": {"kp": 1.0, "ki": 0.001, "kd": 0.001},
        "pid_angle": {"kp": 25.0, "ki": 0.001, "kd": 0.5},
        "pid_line": {"kp": 5.0, "ki": 0.001, "kd": 20.0},
    }


def main():
    url = sys.argv[1] if len(sys.argv) > 1 else "http://127.0.0.1:5000/data"
    body = json.dumps(build_payload()).encode()
    req = urllib.request.Request(url, data=body, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=30) as resp:
        print(resp.status, resp.read().decode())


if __name__ == "__main__":
    main()
