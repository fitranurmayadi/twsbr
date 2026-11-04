// Pin map, PID gains and shared state.

// Wi-Fi credentials and telemetry server URL (ssid, password, serverName).
// Copy secrets.example.h to secrets.h and fill in your own values;
// secrets.h is git-ignored. Without it the placeholders are used.
#if __has_include("secrets.h")
#include "secrets.h"
#else
#include "secrets.example.h"
#endif

// Run log sent to the server (only the first 1000 samples are used)
float timeSamplingDt[1000];

float speedMeasurements[3000];
float angleAdjustments[3000];
float angleMeasurements[3000];
float lineMeasurements[3000];

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

#define MPU6050 0x68       // Device address
#define ACCEL_CONFIG 0x1C  // Accelerometer configuration address
#define GYRO_CONFIG 0x1B   // Gyro configuration address
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C

#define accSens 0          // 0 = 2g, 1 = 4g, 2 = 8g, 3 = 16g
#define gyroSens 1         // 0 = 250 deg/s, 1 = 500 deg/s, 2 = 1000 deg/s, 3 = 2000 deg/s
#define Gyro_amount 0.996  // percent of gyro in complementary filter

uint8_t i2cData[14];  // Buffer for I2C data

// Speed PID Parameters
float Kp_speed = 1.0;
float Ki_speed = 0.001;
float Kd_speed = 0.001;

// Variables
float speed_setpoint = 1.0;  // Desired speed in m/s (can be set dynamically)

float current_speed = 0.0;   // Read from wheel encoders
float previous_speed_error = 0.0;
float integral_speed = 0.0;

// Balance PID Parameters
float Kp_balance = 25.0;
float Ki_balance = 0.001;
float Kd_balance = 0.5;

// Variables
float angle_setpoint = 0.0;   // Set by Speed Control Loop

float current_angle = 0.0;  // Read from IMU
float previous_angle_error = 0.0;
float integral_angle = 0.0;

// Line Following PID Parameters
float Kp_line = 5.0;
float Ki_line = 0.001;
float Kd_line = 20.0;

// Variables
float line_setpoint = 0.0;  // Desired position (centered on line)
float line_position = 0.0;  // Read from line sensors (e.g., -4 to +4)
float previous_line_error = 0.0;
float integral_line = 0.0;

float previous_position = 0;

static unsigned long previous_time = 0;
float desired_loop_time = 0.01;  // 10 ms

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float alpha = 0.40;

int16_t AcX, AcY, AcZ;
int16_t GyX, GyY, GyZ;

float robot_angle;
float Acc_angle;
bool vertical = false;
bool start = false;
bool line_following_mode = false;

// 1-D Kalman filter state
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = { 0, 0 };

// Encoder setup
volatile long encoderCount_L = 0;  // Left encoder pulse count
volatile long encoderCount_R = 0;  // Right encoder pulse count
int pulsesPerRevolution = 100;     // Number of pulses per revolution for the encoder
float wheelDiameter = 0.09;         // Diameter of the wheel in meters (change according to your setup)
float wheelCircumference = 3.1416 * wheelDiameter; // Wheel circumference in meters
