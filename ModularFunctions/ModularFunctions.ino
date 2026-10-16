void clamp(float &voltage, float &accumTheta, float deltaTheta, float desired_Ts_ms) {
  if (voltage >= 8.0) {
    voltage = 8.0;
    accumTheta -= deltaTheta * desired_Ts_ms;
  } else if (voltage <= -8.0) {
    voltage = -8.0;
    accumTheta -= deltaTheta * desired_Ts_ms;
  }
}

void calc_odo(&phi, &odometry_0, &odometry_1, ){
      phi = ((theta[0] - theta[1]) / 2);
      //calculates x
    odometry[0] += cos(phi) * (robotLinearVelocity* dt);
      //calculates y
    odometry[1] += sin(phi) * (robotLinearVelocity * dt);
}

void voltageDirection(float voltage, int SIGN_PIN, int PWM_PIN, int i){
      if (voltage > 0) {
        digitalWrite(SIGN_PIN, !i ? LOW : HIGH);
        analogWrite(PWM_PIN, (abs(voltage) / 8.0 * 255));
      } else if (voltage < 0) {
        digitalWrite(SIGN_PIN, !i ? HIGH : LOW);
        analogWrite(PWM_PINs, (abs(voltage) / 8.0 * 255));
      }
      else{
        analogWrite(PWM_PIN, 0);
      }
}