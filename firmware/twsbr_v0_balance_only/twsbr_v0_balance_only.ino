/*
 * TWSBR v0 - balance-only prototype
 * Two-wheeled self-balancing robot: ESP32-S3 + MPU6050 + BLDC motors.
 *
 * A single PID loop keeps the robot upright using the Kalman-filtered tilt
 * angle, running every 1 ms. The battery is checked every 500 ms and the
 * buzzer sounds when it drops into the 8.0-9.5 V range.
 *
 * Serial commands "p+", "p-", "i+", "i-", "s+", "s-" change X1/X2/X3. These
 * were a tuning experiment and are not wired into the PID (it uses Kp/Ki/Kd).
 */

#include <Wire.h>
#include <EEPROM.h>
//#include "BluetoothSerial.h"
//BluetoothSerial Serial;
// BLDC motors (driver with DIR / PWM / BRAKE inputs, quadrature encoder)
#define BRAKE 36

#define DIR_L 17
#define PWM_L 18
#define ENC_L_A 16
#define ENC_L_B 15

#define DIR_R 11
#define PWM_R 10
#define ENC_R_A 13
#define ENC_R_B 12


#define TIMER_BIT 8
#define BASE_FREQ 20000
#define VBAT 21

// Buzzer and battery sense
#define BUZZER 14


// MPU6050 IMU (I2C)

#define SDA_PIN 35
#define SCL_PIN 9

int pwm_s = 0;
byte dir;
int32_t motor_speed;
uint32_t timer;

// MPU6050 IMU (I2C)
#define MPU6050 0x68       // Device address
#define ACCEL_CONFIG 0x1C  // Accelerometer configuration address
#define GYRO_CONFIG 0x1B   // Gyro configuration address
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C

#define accSens 0          // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 1         // 0 = 250 deg/s, 1 = 500 deg/s, 2 = 1000 deg/s, 3 = 2000 deg/s
#define Gyro_amount 0.996  // percent of gyro in complementary filter

uint8_t i2cData[14];  // Buffer for I2C data

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;

uint32_t LoopTimer;

float alpha = 0.40;

int16_t AcX, AcY, AcZ;
int16_t GyX, GyY, GyZ;

float robot_angle;
float Acc_angle;
bool vertical = false;

//Control
float Kp = 25;
float Ki = 0.001;
float Kd = 0.5;

float P = 0, I = 0, D = 0;
float prev_error = 0;
float integral = 0;

int16_t current_error = 0, last_error = 0;

float X1 = 15.0;
float X2 = 0.01;
float X3 = 0.001;


float loop_time = 1;

long currentT, previousT_1, previousT_2 = 0;

// 1-D Kalman filter state
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = { 0, 0 };

void beep() {
  digitalWrite(BUZZER, HIGH);
  delay(70);
  digitalWrite(BUZZER, LOW);
  delay(80);
}

void battVoltage(double voltage) {
  //Serial.print("batt: "); Serial.println(voltage); //debug
  if (voltage > 8 && voltage <= 9.5) {
    digitalWrite(BUZZER, HIGH);
  } else {
    digitalWrite(BUZZER, LOW);
  }
}
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + 0.004 * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}
void pwmSet(uint8_t channel, uint32_t value) {
  ledcWrite(channel, value);
}

void Motor_L_control(int sp) {
  if (sp < 0) {
    digitalWrite(DIR_L, LOW);
    sp = -sp;
  } else {
    digitalWrite(DIR_L, HIGH);
  }
  pwmSet(PWM_L, sp > 255 ? 255 : 255 - sp);
}

void Motor_R_control(int sp) {
  if (sp < 0) {
    digitalWrite(DIR_R, LOW);
    sp = -sp;
  } else {
    digitalWrite(DIR_R, HIGH);
  }
  pwmSet(PWM_R, sp > 255 ? 255 : 255 - sp);
}

void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

void angle_setup() {
  Wire.begin();
  delay(100);
  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3);  // Specifying output scaling of accelerometer
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3);  // Specifying output scaling of gyroscope
  delay(100);
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    angle_calc();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;
  beep();
  beep();
  //Serial.print("Offset: "); Serial.println(RateCalibrationRoll);
}

void angle_calc() {
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true);
  AcX = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  Wire.beginTransmission(MPU6050);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 6, true);
  GyX = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  RateRoll = (float)GyX / 65.536;
  RatePitch = (float)GyY / 65.536;
  RateYaw = (float)GyZ / 65.536;

  AccX = (float)AcX / 4096 - 0.16;
  AccY = (float)AcY / 4096 + 0.08;
  AccZ = (float)AcZ / 4096 - 3.41;

  //Roll = x, Pitch=y, Yaw = Z
  AngleRoll = atan(AcY / sqrt(AcX * AcX + AcZ * AcZ)) * 1 / (3.142 / 180);
  AnglePitch = -atan(AcX / sqrt(AcY * AcY + AcZ * AcZ)) * 1 / (3.142 / 180);


  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  robot_angle = -(KalmanAnglePitch + 3.3);

  if (abs(robot_angle) > 45) vertical = false;
  if (abs(robot_angle) < 45) vertical = true;
}

int Tuning() {
  if (!Serial.available()) return 0;
  delay(2);
  char param = Serial.read();  // get parameter byte
  if (!Serial.available()) return 0;
  char cmd = Serial.read();  // get command byte
  Serial.flush();
  switch (param) {
    case 'p':
      if (cmd == '+') X1 += 1;
      if (cmd == '-') X1 -= 1;
      printValues();
      break;
    case 'i':
      if (cmd == '+') X2 += 0.01;
      if (cmd == '-') X2 -= 0.01;
      printValues();
      break;
    case 's':
      if (cmd == '+') X3 += 0.005;
      if (cmd == '-') X3 -= 0.005;
      printValues();
      break;
  }
  return 0;
}

void printValues() {
  Serial.print("X1: ");
  Serial.print(X1);
  Serial.print(" X2: ");
  Serial.print(X2);
  Serial.print(" X3: ");
  Serial.println(X3, 3);
}


float calculatePID(float setpoint, float measured_value, float& prev_error, float& integral, float kp, float ki, float kd, float dt) {
  // Error
  float error = setpoint - measured_value;

  // Proportional term
  float P = kp * error;

  // Integral term
  integral += error * dt;
  float I = ki * integral;

  // Derivative term
  float derivative = (error - prev_error) / dt;
  float D = kd * derivative;

  // Keep the error for the next derivative
  prev_error = error;

  // PID output
  float output = P + I + D;

  return output;
}


void setup() {
  Serial.begin(115200);
  //Serial.begin("ESP32-Cube-blue");  //Bluetooth device name
  //EEPROM.begin(EEPROM_SIZE);

  //pinMode(BUZZER, OUTPUT);
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
  currentT = millis();
  if (currentT - previousT_1 >= loop_time) {
    Tuning();
    angle_calc();
    if (vertical) {
      digitalWrite(BRAKE, HIGH);

      float dt = loop_time / 1000.0;  // Loop time in seconds
      pwm_s = calculatePID(-0.27, robot_angle, prev_error, integral, Kp, Ki, Kd, dt);

      pwm_s = constrain(pwm_s, -255, 255);  // Clamp PWM

      Motor_L_control(pwm_s);
      Motor_R_control(-pwm_s);
      //motor_speed += pwm_s;
      Serial.print("Angle: ");
      Serial.print(robot_angle);
      Serial.print("\tSpeed: ");
      Serial.println(pwm_s);
      //Serial.print("\tMotorSpeed: ");
      //Serial.println(motor_speed);

    } else {
      Motor_L_control(0);
      Motor_R_control(0);
      digitalWrite(BRAKE, LOW);
      Serial.println("Here!!");
      motor_speed = 0;
    }
    previousT_1 = currentT;
  }
  if (currentT - previousT_2 >= 500) {
    battVoltage((double)analogRead(VBAT) / 207);  // VBAT is read through a 47k/12k voltage divider
    previousT_2 = currentT;
  }
}
