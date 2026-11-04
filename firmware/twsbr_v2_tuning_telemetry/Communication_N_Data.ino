// Wi-Fi and HTTP POST of the logged run to server/app.py as JSON.

// Function to connect to WiFi
void connectToWiFi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("Connected");
}

void sendDataToServer() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, trying to reconnect...");
    connectToWiFi();
  }

  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverName);
    http.addHeader("Content-Type", "application/json");
    http.setTimeout(5000);  // 5 s timeout

    // Create JSON object with a dynamic buffer
    JsonDocument doc;  // Adjust size as needed

    doc["setpoint_time"] = desired_loop_time;

    // Add time measurements
    JsonArray times = doc["measurements_time"].to<JsonArray>();
    for (int i = 0; i < 1000; i++) {
      times.add(timeSamplingDt[i]);
    }

    // Add speed measurements
    doc["setpoint_speed"] = speed_setpoint;
    JsonArray speeds = doc["measurements_speed"].to<JsonArray>();
    for (int i = 0; i < 1000; i++) {
      speeds.add(speedMeasurements[i]);
    }

    // Add angle measurements
    doc["setpoint_angle"] = angle_setpoint;

    JsonArray angles_adjust = doc["angle_adjustment"].to<JsonArray>();
    for (int i = 0; i < 1000; i++) {
      angles_adjust.add(angleAdjustments[i]);
    }

    JsonArray angles = doc["measurements_angle"].to<JsonArray>();
    for (int i = 0; i < 1000; i++) {
      angles.add(angleMeasurements[i]);
    }

    // Add line-following measurements
    doc["setpoint_line"] = line_setpoint;
    JsonArray lines = doc["measurements_line"].to<JsonArray>();
    for (int i = 0; i < 1000; i++) {
      lines.add(lineMeasurements[i]);
    }

    // Add PID values
    JsonObject pidSpeed = doc["pid_speed"].to<JsonObject>();
    pidSpeed["kp"] = Kp_speed;
    pidSpeed["ki"] = Ki_speed;
    pidSpeed["kd"] = Kd_speed;

    JsonObject pidAngle = doc["pid_angle"].to<JsonObject>();
    pidAngle["kp"] = Kp_balance;
    pidAngle["ki"] = Ki_balance;
    pidAngle["kd"] = Kd_balance;

    JsonObject pidLine = doc["pid_line"].to<JsonObject>();
    pidLine["kp"] = Kp_line;
    pidLine["ki"] = Ki_line;
    pidLine["kd"] = Kd_line;

    // Serialize JSON to string
    String jsonData;
    serializeJson(doc, jsonData);

    // Send HTTP POST request
    int httpResponseCode = http.POST(jsonData);

    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println(httpResponseCode);
      Serial.println(response);
    } else {
      Serial.print("Error on sending POST: ");
      Serial.println(httpResponseCode);
      if (httpResponseCode == -1) {
        Serial.println("Connection failed");
      }
    }

    http.end();
  }
}
