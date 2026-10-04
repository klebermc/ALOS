void initialize_controllers()
{
  //Define the controllers' gains
  Kp_X = 0;
  Ki_X = 0;
  Kd_X = 0;

  Kp_Y = 0;
  Ki_Y = 0;
  Kd_Y = 0;

  Kp_Z = 32;
  Ki_Z = 5*0.02;
  Kd_Z = 10/0.02;

}


void X_controller()
{
  // Inertial X Position
  error_X = desired_pos_I[0] - pos_I[0];
  dXdt = pos_I[0] - previous_pos[0];
  cumul_error[0] = cumul_error[0] + error_X;            // Somatory (integral)
  SATURATION(cumul_error[0],100,-100);                   //Saturation
  X_out = (Kp_X * error_X) + (Ki_X * cumul_error[0]) + (Kd_X * dXdt);
  previous_pos[0] = pos_I[0];

  Serial.print(ANG2uSEC(X_out));
}

void Y_controller()
{
  // Inertial Y Position
  error_Y = desired_pos_I[1] - pos_I[1];
  dYdt = pos_I[0] - previous_pos[1];
  cumul_error[1] = cumul_error[1] + error_Y;            // Somatory (integral)
  SATURATION(cumul_error[1],100,-100);                   //Saturation
  Y_out = (Kp_Y * error_Y) + (Ki_Y * cumul_error[1]) + (Kd_Y * dYdt);
  previous_pos[1] = pos_I[1];

  Serial.println(ANG2uSEC(Y_out));
}

void Z_controller()
{
  //Read the IR sensor
  read_IR_sensor();
  
  // Inertial Z Position
  error_Z = desired_pos_I[2] - pos_I[2];                // Proportional
  dZdt = pos_I[2] - previous_pos[2];                    // Derivative
  cumul_error[2] = cumul_error[2] + error_Z;            // Somatory (integral)
  SATURATION(cumul_error[2],500,0);                     // Saturation
  Z_out = (Kp_Z * error_Z) + (Ki_Z * cumul_error[2]) + (Kd_Z * dZdt);
  previous_pos[2] = pos_I[2];

  Serial.print("Vertical force [N]:"); Serial.println(Z_out);
  Serial.print("Vertical PWM  [us]:"); Serial.println(THRUST2THROTTLE(Z_out));
  SET_PWM_PIN2(THRUST2THROTTLE(Z_out));
}

void HEADING_controller()
{
  // Heading correction based on magnetometer measurement
  error_HEADING = desired_HEADING - actual_heading;
  dHDdt = actual_heading - previous_HEADING;
  cumul_error_HEADING = cumul_error_HEADING + error_HEADING;      // Somatory (integral)
  cumul_error_HEADING = max(min(cumul_error_HEADING, 100), -100); // saturarion
  HD_out = (Kp_HD * error_HEADING) + (Ki_HD * cumul_error_HEADING) + (Kd_HD * dHDdt);
  previous_HEADING = actual_heading;
}

