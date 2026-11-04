// Save/load PID gains in EEPROM (not called yet).

void setPIDParams() {
  EEPROM.put(0, Kp_balance);
  EEPROM.put(4, Ki_balance);
  EEPROM.put(8, Kd_balance);
  EEPROM.put(12, Kp_line);
  EEPROM.put(16, Ki_line);
  EEPROM.put(20, Kd_line);
  EEPROM.commit();
}

void getPIDParams() {
  EEPROM.get(0, Kp_balance);
  EEPROM.get(4, Ki_balance);
  EEPROM.get(8, Kd_balance);
  EEPROM.get(12, Kp_line);
  EEPROM.get(16, Ki_line);
  EEPROM.get(20, Kd_line);
}
