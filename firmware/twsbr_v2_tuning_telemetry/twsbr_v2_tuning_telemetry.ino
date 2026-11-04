/*
 * TWSBR v2 - tuning build with Wi-Fi telemetry
 * Full cascade every 10 ms:
 *   speed PID (encoders)  -> angle setpoint
 *   balance PID (IMU)     -> common motor PWM
 *   line PID (QTR-8A)     -> differential steering PWM
 *
 * The control loop runs inside setup() for 1000 samples (~10 s), then leaves the
 * loop and POSTs the whole log as JSON to server/app.py, which
 * plots speed, angle and line position so the gains can be tuned run by run.
 *
 * Wi-Fi credentials and the server URL go in secrets.h (see secrets.example.h).
 */

#include <EEPROM.h>
#include <Wire.h>
#include "Parameters.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>


void setup() {
  Serial.begin(115200);

  pinMode(BUZZER, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  digitalWrite(BRAKE, HIGH);

  pinMode(DIR_L, OUTPUT);
  ledcAttach(PWM_L, BASE_FREQ, TIMER_BIT);
  Motor_L_control(0);

  pinMode(DIR_R, OUTPUT);
  ledcAttach(PWM_R, BASE_FREQ, TIMER_BIT);
  Motor_R_control(0);

  pinMode(ENC_L_A, INPUT);
  pinMode(ENC_L_B, INPUT);
  pinMode(ENC_R_A, INPUT);
  pinMode(ENC_R_B, INPUT);

  angle_setup();
  beep();
  //test();

  // Attach interrupts for encoder counting
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), updateEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), updateEncoderRight, CHANGE);
  running();
}

void running() {
  bool running = true;
  int i = 0;
  float last_dt = 0;
  Serial.println("RUNNNN!!!!");
  while (running) {
    unsigned long current_time = millis();
    float dt = (current_time - previous_time) / 1000.0;  // Convert to seconds

    if (dt >= desired_loop_time) {
      previous_time = current_time;
      if (!vertical) {
        Serial.println("NOT VERTICAL!!!");
        delay(100);
      }
      if (vertical) {
        if (!start) {
          Serial.println("NOT STARTED!!!");
          current_angle = readIMUAngle();
          if (current_angle > -30 && current_angle < 30) start = true;
          else start = false;
          delay(100);
        }
        if (start) {

          digitalWrite(BRAKE, HIGH);
          // 1. Read Sensors
          current_angle = readIMUAngle();  // Tilt angle from the IMU


          float leftSpeed = readWheelSpeed(encoderCount_L, dt);   // in meters per second (m/s)
          float rightSpeed = readWheelSpeed(encoderCount_R, dt);  // in meters per second (m/s)
          encoderCount_L = 0;
          encoderCount_R = 0;

          current_speed = (leftSpeed + rightSpeed) / 2;  // Calculate current wheel speed
          line_position = readLinePosition();            // Line position from the QTR-8A

          // 2. Compute Speed Control (Middle Loop)
          angle_setpoint = computeSpeedPID(speed_setpoint, current_speed, dt);

          // 3. Compute Balance Control (Inner Loop)
          float balance_output = computeBalancePID(angle_setpoint, current_angle, dt);

          // 4. Compute Line Following Control (Outer Loop)
          float steering_adjustment = computeLinePID(line_setpoint, line_position, dt);

          // 5. Compute Motor PWM Outputs
          float motor_pwm_left = balance_output - steering_adjustment;
          float motor_pwm_right = balance_output + steering_adjustment;

          // 6. Apply PWM Limits
          motor_pwm_left = constrain(-motor_pwm_left, -255, 255);
          motor_pwm_right = constrain(-motor_pwm_right, -255, 255);

          // 7. Send PWM Signals to Motors
          Motor_L_control(motor_pwm_left);
          Motor_R_control(motor_pwm_right);

          // 8. Optional: Debugging Output
          //displayData(current_angle, line_position, motor_pwm_left, motor_pwm_right);

          timeSamplingDt[i] = last_dt + dt;
          last_dt = last_dt + dt;
          angleAdjustments[i] = angle_setpoint;
          speedMeasurements[i] = current_speed;
          angleMeasurements[i] = current_angle;
          lineMeasurements[i] = line_position;

          Serial.print("Angle SP: ");
          Serial.print(angle_setpoint);
          Serial.print("Angle MV: ");
          Serial.print(current_angle);
          Serial.print(" | Speed: ");
          Serial.print(current_speed);
          Serial.print(" | Line Pos: ");
          Serial.print(line_position);
          Serial.print(" | PWM L: ");
          Serial.print(motor_pwm_left);
          Serial.print(" | PWM R: ");
          Serial.println(motor_pwm_right);
          i++;
          if (i > 999) {
            running = false;
          }
        }
      }

      else {
        Motor_L_control(0);
        Motor_R_control(0);
        digitalWrite(BRAKE, LOW);
        Serial.println("BRAKE!!");
        current_angle = readIMUAngle();
      }
    }
  }
  Serial.println("SEND DATA!!!!");
  sendDataToServer();
  beep();
  beep();
  beep();
}

void loop() {
}
