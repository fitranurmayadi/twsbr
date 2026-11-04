// MPU6050 driver + 1-D Kalman filter fusing gyro rate and accelerometer angle into the tilt angle.

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + 0.004 * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

void angle_setup() {
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);
  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, accSens << 3);  // Specifying output scaling of accelerometer
  writeTo(MPU6050, GYRO_CONFIG, gyroSens << 3);  // Specifying output scaling of gyroscope
  delay(100);
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    readIMUAngle();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;
  //Serial.print("Offset: "); Serial.println(RateCalibrationRoll);
}

float readIMUAngle() {
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

  robot_angle = -(KalmanAnglePitch + 2.3);

  if (abs(robot_angle) > 45) vertical = false;
  else if (abs(robot_angle) < 45) vertical = true;
  if (robot_angle > 0 && robot_angle < 6) line_following_mode = true;
  if (robot_angle <= 0 && robot_angle > 6) line_following_mode = false;

  float angle = robot_angle;
  return angle;
}
