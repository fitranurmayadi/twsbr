// Buzzer, battery warning, BLDC motor drive (DIR + inverted PWM) and wheel encoders.

void beep() {
  digitalWrite(BUZZER, HIGH);
  delay(100);
  digitalWrite(BUZZER, LOW);
  delay(80);
  digitalWrite(BUZZER, HIGH);
  delay(100);
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

void pwmSet(uint8_t channel, uint32_t value) {
  ledcWrite(channel, value);
}

void Motor_L_control(int sp) {
  if (sp < 0) {
    digitalWrite(DIR_L, HIGH);
    sp = -sp;
  } else {
    digitalWrite(DIR_L, LOW);
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

// Interrupt service routines for encoders
void updateEncoderLeft() {
  int state = digitalRead(ENC_L_A);
  if (digitalRead(ENC_L_B) != state) {
    encoderCount_L = encoderCount_L + 1;  // Forward direction
  } else {
    encoderCount_L = encoderCount_L - 1;  // Reverse direction
  }
}

void updateEncoderRight() {
  int state = digitalRead(ENC_R_A);
  if (digitalRead(ENC_R_B) != state) {
    encoderCount_R = encoderCount_R + 1;  // Forward direction
  } else {
    encoderCount_R = encoderCount_R - 1;  // Reverse direction
  }
}

// Function to calculate wheel speed based on encoder count and time delta
float readWheelSpeed(volatile long &encoderCount, float dt) {
  // Calculate the distance traveled in meters
  float revolutions = (float)encoderCount / pulsesPerRevolution;
  float distance = revolutions * wheelCircumference;

  // Calculate speed (distance / time)
  float speed = distance / dt;
  return speed;  // Speed in meters per second
}
