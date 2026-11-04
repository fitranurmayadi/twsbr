// PID controllers: speed (outer) -> tilt-angle setpoint, balance (inner) -> motor PWM,
// line position -> differential steering.

float computeSpeedPID(float speed_setpoint, float current_speed, float dt) {
    float speed_error = speed_setpoint - current_speed;
    integral_speed += speed_error * dt;
    float derivative_speed = (speed_error - previous_speed_error) / dt;
    previous_speed_error = speed_error;
    //float angle_setpoint = (Kp_speed * speed_error) + (Ki_speed * integral_speed) + (Kd_speed * derivative_speed);
    // Limit angle_setpoint to prevent excessive tilting
    //angle_setpoint = constrain(angle_setpoint, -3.0, 3.0); // degrees
    //return angle_setpoint;
    float speed_adjusment = (Kp_speed * speed_error) + (Ki_speed * integral_speed) + (Kd_speed * derivative_speed);
    // Limit angle_setpoint to prevent excessive tilting
    return speed_adjusment;
}

float computeBalancePID(float angle_setpoint, float current_angle, float dt) {
    float angle_error = angle_setpoint - current_angle;
    integral_angle += angle_error * dt;
    float derivative_angle = (angle_error - previous_angle_error) / dt;
    previous_angle_error = angle_error;
    float balance_output = (Kp_balance * angle_error) + (Ki_balance * integral_angle) + (Kd_balance * derivative_angle);
    return balance_output;
}

float computeLinePID(float line_setpoint, float line_position, float dt) {
    float line_error = line_setpoint - line_position;
    integral_line += line_error * dt;
    float derivative_line = (line_error - previous_line_error) / dt;
    previous_line_error = line_error;
    float steering_adjustment = (Kp_line * line_error) + (Ki_line * integral_line) + (Kd_line * derivative_line);
    // Limit steering adjustment
    steering_adjustment = constrain(steering_adjustment, -100, 100); // Adjust as needed
    return steering_adjustment;
}
