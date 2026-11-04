/*
 * TWSBR v1 - first cascade-PID structure
 * Balance PID (inner loop) + line-following PID (steering), every 1 ms.
 *
 * The speed loop is still disabled here: the wheel encoders are not read yet
 * (readWheelSpeed() is a stub), so angle_setpoint stays at its initial value.
 */

#include <EEPROM.h>
#include <Wire.h>
#include "Parameters.h"


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

  angle_setup();
  beep();
}

void loop() {
  unsigned long current_time = millis();
  float dt = (current_time - previous_time) / 1000.0;  // Convert to seconds

  if (dt >= desired_loop_time) {
    previous_time = current_time;

    if (vertical) {

        digitalWrite(BRAKE, HIGH);
        // 1. Read Sensors
        current_angle = readIMUAngle();      // Tilt angle from the IMU
        current_speed = readWheelSpeed(dt);  // Wheel speed from the encoders
        line_position = readLinePosition();  // Line position from the QTR-8A

        // 2. Compute Speed Control (Middle Loop)
        //angle_setpoint = computeSpeedPID(speed_setpoint, current_speed, dt);

        // 3. Compute Balance Control (Inner Loop)
        float balance_output = computeBalancePID(angle_setpoint, current_angle, dt);

        // 4. Compute Line Following Control (Outer Loop)
        float steering_adjustment = computeLinePID(line_setpoint, line_position, dt);

        // 5. Compute Motor PWM Outputs
        float motor_pwm_left = balance_output + steering_adjustment;
        float motor_pwm_right = balance_output - steering_adjustment;

        // 6. Apply PWM Limits
        motor_pwm_left = constrain(-motor_pwm_left, -255, 255);
        motor_pwm_right = constrain(-motor_pwm_right, -255, 255);


        // 7. Send PWM Signals to Motors
        Motor_L_control(motor_pwm_left);
        Motor_R_control(motor_pwm_right);

        // 8. Optional: Debugging Output
        //displayData(current_angle, line_position, motor_pwm_left, motor_pwm_right);

        Serial.print("Angle: ");
        Serial.print(current_angle);
        Serial.print(" | Speed: ");
        Serial.print(current_speed);
        Serial.print(" | Line Pos: ");
        Serial.print(line_position);
        Serial.print(" | PWM L: ");
        Serial.print(motor_pwm_left);
        Serial.print(" | PWM R: ");
        Serial.println(motor_pwm_right);

    } else {
      current_angle = readIMUAngle();
      if (current_angle > -2 && current_angle < 2) start = true;
      else start = false;
      Motor_L_control(0);
      Motor_R_control(0);
      digitalWrite(BRAKE, LOW);
      Serial.println("BRAKE!!");
    }
  }
}
