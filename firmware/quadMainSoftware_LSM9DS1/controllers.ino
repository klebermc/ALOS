void initialize_controllers()
{
  //Define the controllers' gains
  Kp_X = 9.2;
  Ki_X = 0.16*0.1;
  Kd_X = 0;    //3.45/0.1;

  Kp_Y = 8.9;
  Ki_Y = 0.04*0.1;
  Kd_Y = 0;   //3.5/0.1;
    
  Kp_Z = 32;
  Ki_Z = 5*0.02;
  Kd_Z = 2.5/0.02;

/*CARLA
51.074 
43.066 
11.795 
*/
  Kp_HD = 10/4.4937865280331;
  Ki_HD = 0.003; //*0.02;
  Kd_HD = 2; ///0.02;
}


void X_controller()
{
  // Inertial X Position
  error_X = desired_pos_I[0] - pos_I[0];
  dXdt = pos_I[0] - previous_pos[0];
  cumul_error[0] = cumul_error[0] + error_X;            // Somatory (integral)
  SATURATION(cumul_error[0],100,-100);                   //Saturation
  X_out = (Kp_X * error_X) + (Ki_X * cumul_error[0]) - (Kd_X * dXdt);
  previous_pos[0] = pos_I[0];
  
  X_out = -1*X_out;
  SATURATION(X_out,2,-2);
  
  //Serial.print(error_X); Serial.print(" ");  Serial.println(ANG2uSEC(X_out));
  SET_PWM_PIN5(ANG2uSEC(X_out));//ELEVATOR
}

void Y_controller()
{
  // Inertial Y Position
  error_Y = desired_pos_I[1] - pos_I[1];                 // Proportional
  dYdt = pos_I[1] - previous_pos[1];                     // Derivative
  cumul_error[1] = cumul_error[1] + error_Y;             // Somatory (integral)
  SATURATION(cumul_error[1],100,-100);                   // Saturation
  Y_out = (Kp_Y * error_Y) + (Ki_Y * cumul_error[1]) - (Kd_Y * dYdt);
  previous_pos[1] = pos_I[1];

  Y_out = -1*Y_out;
  SATURATION(Y_out,2,-2);
  
  //Serial.print(error_Y); Serial.print(" ");  Serial.println(ANG2uSEC(Y_out));
  SET_PWM_PIN3(ANG2uSEC(Y_out));//AILERON
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
  Z_out = (Kp_Z * error_Z) + (Ki_Z * cumul_error[2]) - (Kd_Z * dZdt);
  previous_pos[2] = pos_I[2];
  
  SATURATION(Z_out,33,0);
  
  //Serial.print(desired_pos_I[2]*100);  Serial.print(" ");  Serial.print(distance2ground);  Serial.print(" ");  Serial.println(Z_out);
  SET_PWM_PIN2(THRUST2THROTTLE(Z_out));
}

void HEADING_controller()
{
  // Heading correction based on magnetometer measurement
  error_HEADING = desired_HEADING - actual_heading;
  dHDdt = actual_heading - previous_HEADING;
  cumul_error_HEADING = cumul_error_HEADING + error_HEADING;      // Somatory (integral)
  cumul_error_HEADING = max(min(cumul_error_HEADING, 100), -100); // saturarion
  HD_out = Kp_HD *( error_HEADING + (Ki_HD*cumul_error_HEADING) - (Kd_HD*dHDdt) );
  //HD_out = 30 * HD_out; //from force output (controller vrep) to angle velocity output
  HD_out = 0.5 * HD_out;
  previous_HEADING = actual_heading;
  
  SATURATION(HD_out,10,-10);

//  Serial.print(actual_heading); Serial.print(" "); Serial.println(VEL_ANG2uSEC(HD_out));
  SET_PWM_PIN6(VEL_ANG2uSEC(HD_out));//RUDDER
}

