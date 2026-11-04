# Firmware

Arduino sketches for the ESP32-S3, in the order they were written. Each folder is a complete
sketch (the folder name matches its main `.ino`, as the Arduino IDE requires).

| Sketch | Loop | What it does |
|---|---|---|
| [`twsbr_v0_balance_only`](twsbr_v0_balance_only) | 1 ms | Single-file prototype. One PID on the tilt angle keeps the robot upright. Battery warning on the buzzer. |
| [`twsbr_v1_cascade_pid`](twsbr_v1_cascade_pid) | 1 ms | Split into modules. Balance PID + line PID; the speed loop is stubbed (encoders not read yet). |
| [`twsbr_v2_tuning_telemetry`](twsbr_v2_tuning_telemetry) | 10 ms | Full cascade (speed → angle → PWM, line → steering). Logs 1000 samples, then POSTs them to [`server/app.py`](../server/app.py) for plotting. **Use this one to tune.** |
| [`twsbr_v3_continuous`](twsbr_v3_continuous) | 10 ms | Latest. Same cascade running forever in `loop()`, angle setpoint clamped to ±2°, reflectance-weighted line position. No telemetry. |

## Modules (v1 – v3)

| File | Contents |
|---|---|
| `Parameters.h` | Pin map, PID gains, loop period, shared state |
| `Control.ino` | `computeSpeedPID()`, `computeBalancePID()`, `computeLinePID()` |
| `IMU_Process.ino` | MPU6050 setup + gyro calibration, 1-D Kalman filter, `readIMUAngle()` |
| `LineSensor.ino` | `readLinePosition()` from the QTR-8A |
| `Motor_N_Control.ino` | Motor drive, encoder ISRs, `readWheelSpeed()`, buzzer and battery helpers |
| `Communication_N_Data.ino` | v2 only: Wi-Fi + JSON POST of the run log |
| `Memory.ino` | v1 only: save/load gains in EEPROM (not called yet) |

## Build

1. **Arduino IDE 2** with the **esp32 by Espressif** board package **3.x**
   (the sketches use the 3.x LEDC API: `ledcAttach(pin, freq, bits)` / `ledcWrite(pin, duty)`).
2. Board: **ESP32S3 Dev Module**.
3. v2 only: install **ArduinoJson 7.x** (Library Manager), then
   ```bash
   cp twsbr_v2_tuning_telemetry/secrets.example.h twsbr_v2_tuning_telemetry/secrets.h
   ```
   and put your Wi-Fi SSID/password and the PC's address (`http://<pc-ip>:5000/data`) in `secrets.h`.
   `secrets.h` is git-ignored; without it the sketch builds with the placeholder values.
4. Open the sketch folder, upload, and open the Serial Monitor at **115200 baud**.

## Before running on the robot

- Hold the robot still while it powers up: `angle_setup()` averages 2000 gyro readings for the bias.
- `robot_angle = -(KalmanAnglePitch + 2.3)` - the `2.3` is this robot's mounting offset. Re-measure
  it for a different frame (it should read about 0° when the robot is balanced by hand).
- The motors only run while the tilt is within ±45°; outside that the brake line is pulled low.
- Check motor direction at low gains first - the sign conventions (`DIR` levels, the `-` in the PWM
  mix) are specific to this wiring.
