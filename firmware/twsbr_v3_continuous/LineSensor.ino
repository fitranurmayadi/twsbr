// QTR-8A line position: reflectance-weighted average of the sensors above threshold (-4 ... +4).

float readLinePosition() {
  // QTR-8A analog outputs on GPIO 1..8
  const int line_sensor_pin[] = { 1, 2, 3, 4, 5, 6, 7, 8 };
  int line_sensor_data[8];
  int threshold[] = { 3500, 3500, 3500, 3500, 3500, 3500, 3500, 3500 };
  float weights[] = { -4, -3, -2, -1, 1, 2, 3, 4 };  // Weight of each sensor
  float weighted_sum = 0;
  float sensor_sum = 0;
  float position = 0;
  // Read sensor values and compare with threshold
  for (int i = 0; i < 8; i++) {
    line_sensor_data[i] = analogRead(line_sensor_pin[i]);

    // Only sensors that see the line take part in the average
    if (line_sensor_data[i] > threshold[i]) {
      weighted_sum += line_sensor_data[i] * weights[i];  // Reflectance x weight
      sensor_sum += line_sensor_data[i];  // Sum of reflectance
    }
  }

  // If any sensor sees the line, take the weighted average
  if (sensor_sum > 0) {
    position = weighted_sum / sensor_sum;  // Weighted-average position
  }
  previous_position = position;
  return position;
}
