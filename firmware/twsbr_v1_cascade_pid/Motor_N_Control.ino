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

float readWheelSpeed(float dt) {
    // Wheel speed from the encoders (stub in v1: the encoder is not read yet)
    static long previous_encoder_count = 0;
    long current_encoder_count = 10;// TODO: readEncoder()
    float speed = (current_encoder_count - previous_encoder_count) / dt; // dt from the main loop
    previous_encoder_count = current_encoder_count;
    return speed;
}
