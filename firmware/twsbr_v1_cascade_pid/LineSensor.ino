// QTR-8A line position from a thresholded bit pattern (-5 ... +5).

float readLinePosition() {
  // QTR-8A analog outputs on GPIO 1..8
  const int line_sensor_pin[] = { 1, 2, 3, 4, 5, 6, 7, 8 };
  int line_sensor_data[8];
  int threshold[] = { 3500,  3500,  3500,  3500,  3500,  3500,  3500,  3500 };
  float position = 0;
  byte sensor_state = 0;
  // Read sensor values and compare with threshold
  for (int i = 0; i < 8; i++) {
    line_sensor_data[i] = analogRead(line_sensor_pin[i]) > threshold[i] ? 1 : 0;

    // Set the corresponding bit in sensor_state if line is detected
    sensor_state |= line_sensor_data[i] << i;  // Bit i = sensor i (pin 1 ... pin 8)
  }

  // Determine error based on sensor_state pattern
  switch (sensor_state) {
    case 0b00011000: position = 0; break;
    case 0b00111000: position = -1; break;
    case 0b00110000: position = -1; break;
    case 0b01110000: position = -2; break;
    case 0b01100000: position = -2; break;
    case 0b11100000: position = -3; break;
    case 0b11110000: position = -3; break;
    case 0b11000000: position = -4; break;
    case 0b10000000: position = -5; break;
    case 0b00011100: position = 1; break;
    case 0b00001100: position = 1; break;
    case 0b00001110: position = 2; break;
    case 0b00000110: position = 2; break;
    case 0b00000111: position = 3; break;
    case 0b00001111: position = 3; break;
    case 0b00000011: position = 4; break;
    case 0b00000001: position = 5; break;
    case 0b00000000:
      if (previous_position < 0 || previous_position > 0) position = previous_position;
      else position = 0;
    default:
      position = 0;
      break;
  }
  previous_position = position;
  return position;
}
