
// Return current time in microseconds
unsigned long current_time()
{
  return micros();
}

// Delay the task until untilTime has occured. untilTime is in microseconds.
void wait_until(unsigned long untilTime)
{
  delayMicroseconds(700);

  //  unsigned long currentTime = micros();
  //  unsigned int waitTime = (unsigned int)(untilTime - currentTime);
  //  if (waitTime > 0) {
  //    delayMicroseconds(waitTime);
  //  }
}

double getTheta() {

  /* Calculate pitch */
  while (i2cRead(0x3D, i2cBuffer, 8));
  int16_t accY = ((i2cBuffer[0] << 8) | i2cBuffer[1]);
  int16_t accZ = ((i2cBuffer[2] << 8) | i2cBuffer[3]);
  int16_t gyroX = ((i2cBuffer[6] << 8) | i2cBuffer[7]);

  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  double accAngle = (atan2((double)accY - cfg.accYzero, (double)accZ - cfg.accZzero) + PI) * RAD_TO_DEG;

  uint32_t timer = micros();
  // This fixes the 0-360 transition problem when the accelerometer angle jumps between 0 and 360 degrees
  if ((accAngle < 90 && pitch > 270) || (accAngle > 270 && pitch < 90)) {
    kalman.setAngle(accAngle);
    pitch = accAngle;
    gyroAngle = accAngle;
  } else {
    double gyroRate = ((double)gyroX - gyroXzero) / 131.0; // Convert to deg/s
    double dt = (double)(timer - kalmanTimer) / 1000000.0;
    gyroAngle += gyroRate * dt; // Gyro angle is only used for debugging
    if (gyroAngle < 0 || gyroAngle > 360)
      gyroAngle = pitch; // Reset the gyro angle when it has drifted too much
    pitch = kalman.getAngle(accAngle, gyroRate, dt); // Calculate the angle using a Kalman filter
    pitch_dot = kalman.getRate();       // Get the angular speed
  }
  kalmanTimer = timer;
  // Serial.print(accAngle);Serial.print('\t');Serial.print(gyroAngle);Serial.print('\t');Serial.println(pitch);

  // Convert the angle from the Kalman fiter to radians and shift zero position
  double theta = (3.141592 / 180.0) * (180.0 - pitch);

  return theta;
}

double getSpeed(double dt) {
  int32_t wheelPosition1 = getWheelsPosition();
  double wheelVelocity1 = wheelPosition1 - lastWheelPosition1;
  lastWheelPosition1 = wheelPosition1;
  double speed_m_s = (float) 0.5 * 2.0 * 3.141592 * wheel_radius * wheelVelocity1 / EncoderRes / dt;
  //double speed_m_s = (float) 0.5 * wheel_radius * wheelVelocity1 / EncoderRes / dt;
  return speed_m_s;
}

double getRotSpeed(double dt) {
  int32_t wheelPosition1 = readLeftEncoder();
  double wheelVelocity1 = wheelPosition1 - lastWheelPosition1;
  lastWheelPosition1 = wheelPosition1;
  double speed_rad_s = (float) 2.0 * 3.141592 * wheelVelocity1 / EncoderRes / dt;
  return speed_rad_s;
}

double getPosition() {
  int32_t wheelPosition1 = getWheelsPosition();
  double pos_rad = (float) 0.5 * 2.0 * 3.141592 * wheelPosition1 / EncoderRes;
  return pos_rad;
}

void updateEncoders() {
  /* Update encoders */
  timer_us = millis();
  if (timer_us - encoderTimer >= 100) { // Update encoder values every 100ms
    encoderTimer = timer_us;
    int32_t wheelPosition = getWheelsPosition();
    wheelVelocity = wheelPosition - lastWheelPosition;
    lastWheelPosition = wheelPosition;
    //Serial.print(wheelPosition);Serial.print('\t');Serial.print(targetPosition);Serial.print('\t');Serial.println(wheelVelocity);
    if (abs(wheelVelocity) <= 40 && !stopped) { // Set new targetPosition if braking
      targetPosition = wheelPosition;
      stopped = true;
    }
  }
  // Will also update the battery voltage measurement
  batteryCounter++;
  if (batteryCounter >= 10) { // Measure battery every 1s
    batteryCounter = 0;
    batteryVoltage = (double)analogRead(VBAT) / 63.050847458; // VBAT is connected to analog input 5 which is not broken out. This is then connected to a 47k-12k voltage divider - 1023.0/(3.3/(12.0/(12.0+47.0))) = 63.050847458
    if (batteryVoltage < 10.2 && batteryVoltage > 5) // Equal to 3.4V per cell - don't turn on if it's below 5V, this means that no battery is connected
      buzzer::Set();
    else
      buzzer::Clear();
  }
}

void checkMotors() {
  if (!leftDiag::IsSet() || !rightDiag::IsSet()) { // Motor driver will pull these low on error
    buzzer::Set();
    stopMotor(left);
    stopMotor(right);
    while (1);
  }
}

void actuateControlSignal(double u) {
  // Control signal constraints - Motor voltage is between -12 V and + 12V
  u = constrain(u, -12.0, 12.0);

  // Convert from voltage to PWM duty cycle 0-100 % (sign indicates the direction)
  u = 100.0 * (u / 12.0);

  // Compensate for friction (i.e. small control signals will not move the motor)
//  if ((u < 3.0) && (u > 0.0)) {
//    u = 3.0;
//  }
//  if ((u > -3.0) && (u < 0.0)) {
//    u = -3.0;
//  }

  // Set the control signal - Change sign on u to send correct voltage to the motors
  double PIDValue = -u;

  double PIDLeft   = PIDValue;
  double PIDRight  = PIDValue;

  PIDLeft *= cfg.leftMotorScaler; // Compensate for difference in some of the motors
  PIDRight *= cfg.rightMotorScaler;

  /* Set PWM Values */
  if (PIDLeft >= 0)
    moveMotor(left, forward, PIDLeft);
  else
    moveMotor(left, backward, -PIDLeft);
  if (PIDRight >= 0)
    moveMotor(right, forward, PIDRight);
  else
    moveMotor(right, backward, -PIDRight);
}

void setup_setpoint_generator_pulse(double low, double high, unsigned long period_us)
{
    setpoint_generator_pulse_attributes.low = low;   
    setpoint_generator_pulse_attributes.high = high;   
    setpoint_generator_pulse_attributes.period_us = period_us;   
    setpoint_generator_pulse_attributes.next_update_us = period_us; 
    setpoint_generator_pulse_attributes.state = 0;   
}

double setpoint_generator_pulse()
{
  unsigned long timer_us = micros();
    
  // Is it time to update
  if (timer_us >= setpoint_generator_pulse_attributes.next_update_us)
  {
    if (setpoint_generator_pulse_attributes.state == 0)
      setpoint_generator_pulse_attributes.state = 1;
    else
      setpoint_generator_pulse_attributes.state = 0;
      
     setpoint_generator_pulse_attributes.next_update_us = setpoint_generator_pulse_attributes.next_update_us + setpoint_generator_pulse_attributes.period_us;  
  }
  
  if (setpoint_generator_pulse_attributes.state == 0)
    return setpoint_generator_pulse_attributes.low;
  else
    return setpoint_generator_pulse_attributes.high;      
}


