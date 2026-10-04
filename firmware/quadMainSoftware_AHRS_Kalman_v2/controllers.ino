void initialize_controllers()
{
  //Lead gains
  Kp_X=0.6;
  Kp_Y = Kp_X;
//  //Define the controllers' gains
//  
//  //X PID controller gains
//  Kp_X = 9.2;
//  Ki_X = 0.16*0.1;
//  Kd_X = (3.45/0.1) * 0.001;
//  
//  //Y PID controller gains
//  Kp_Y = Kp_X;
//  Ki_Y = Ki_X;
//  Kd_Y = Kd_X;
////  Kp_Y = 8.9;
////  Ki_Y = 0.04*0.1;
////  Kd_Y = (3.5/0.1) * 0.001;
//
//  //Z PID controller gains  
//  //Kp_Z = 20;
//  //Ki_Z = 2.5*0.02;
//  //Kd_Z = 2/0.02;

// *** Trained using height model with non linearities and noise ***
  Kp_Z=31.87;
  Ki_Z=1.92*0.02;
  Kd_Z=(8.42/4)/0.02;
// ***

  //HEADING PID controller gains
  Kp_HD = 2.22; //2.22;
  Ki_HD = 0.01*0.1;
  Kd_HD = 0.05/0.1;
}


void X_controller()
{
  //Lead Controller
  error_X = desired_pos_I[0] - pos_I[0];
  //X_out = Kp_X*error_X - Kp_X*previous_error_X + 0.9048*X_out;
  //X_out = Kp_X*(100*error_X - 99.37*previous_error_X) + 0.3679*X_out; // 10 s + 1 /  0.1 s + 1
  X_out = Kp_X*(20*error_X - 19.61*previous_error_X) + 0.6065*X_out;    //  4 s + 1 /  0.2 s + 1
  previous_error_X=error_X;
  
  //  // Inertial X Position
//  error_X = desired_pos_I[0] - pos_I[0];
//  dXdt = pos_I[0] - previous_pos[0];
//  cumul_error[0] = cumul_error[0] + error_X;            // Somatory (integral)
//  SATURATION(cumul_error[0],100,-100);                   //Saturation
//  X_out = (Kp_X * error_X) + (Ki_X * cumul_error[0]) - (Kd_X * dXdt);
//  previous_pos[0] = pos_I[0];
//  
//  X_out = X_out*0.5;
//
//  // ***************************
//  //Anti windup filter
//  if (X_out>2)        cumul_error[0] = previous_cumul_error[0];
//  else if (X_out<-2)  cumul_error[0] = previous_cumul_error[0];
//    
//  previous_cumul_error[0] = cumul_error[0];
//  // ***************************  

  SATURATION(X_out,2,-2);
  
  //Serial.print(error_X); Serial.print(" "); Serial.print(cumul_error[0]); Serial.print(" ");  Serial.println(ANG2uSEC(X_out));
  SET_PWM_PIN5(ANG2uSEC(-X_out));//ELEVATOR
}

void Y_controller()
{
    //Lead Controller
  error_Y = desired_pos_I[1] - pos_I[1];
  //Y_out = Kp_Y*error_Y - Kp_Y*previous_error_Y + 0.9048*Y_out;
  //Y_out = Kp_Y*(100*error_Y - 99.37*previous_error_Y) + 0.3679*Y_out;
  Y_out = Kp_Y*(20*error_Y - 19.61*previous_error_Y) + 0.6065*Y_out;    //  4 s + 1 /  0.2 s + 1
  previous_error_Y=error_Y;

//u(k) =kp * (100e(k) - 99.37e(k-1)) + 0.3679u(k-1)
   
//  // Inertial Y Position
//  error_Y = desired_pos_I[1] - pos_I[1];                 // Proportional
//  dYdt = pos_I[1] - previous_pos[1];                     // Derivative
//  cumul_error[1] = cumul_error[1] + error_Y;             // Somatory (integral)
//  SATURATION(cumul_error[1],100,-100);                   // Saturation
//  Y_out = (Kp_Y * error_Y) + (Ki_Y * cumul_error[1]) - (Kd_Y * dYdt);
//  previous_pos[1] = pos_I[1];
//
//  Y_out = Y_out*0.5;
//
//  // ***************************  
//  //Anti windup filter
//  if (Y_out>2)        cumul_error[1] = previous_cumul_error[1];
//  else if (Y_out<-2)  cumul_error[1] = previous_cumul_error[1];
//    
//  previous_cumul_error[1] = cumul_error[1];
//  // ***************************  

  SATURATION(Y_out,2,-2);
  
  //Serial.print(error_Y); Serial.print(" ");  Serial.println(ANG2uSEC(Y_out));
  SET_PWM_PIN3(ANG2uSEC(-Y_out));//AILERON
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
  
  SATURATION(Z_out,25,0);
  
//  Serial.print(desired_pos_I[2]*100);  Serial.print(" ");  Serial.print(distance2ground);  Serial.print(" ");  Serial.println(Z_out);
  SET_PWM_PIN2(THRUST2THROTTLE(Z_out));
}

void HEADING_controller()
{
  // Heading correction based on magnetometer measurement
  error_HEADING = desired_HEADING - actual_HEADING;
  dHDdt = actual_HEADING - previous_HEADING;
  cumul_error_HEADING = cumul_error_HEADING + error_HEADING;      // Somatory (integral)
  cumul_error_HEADING = max(min(cumul_error_HEADING, 1000), -1000); // saturarion
  HD_out = Kp_HD *( error_HEADING + (Ki_HD*cumul_error_HEADING) - (Kd_HD*dHDdt) );
  HD_out = 0.5 * HD_out; //from force output (controller vrep) to angle velocity output
  previous_HEADING = actual_HEADING;
  
  SATURATION(HD_out,30,-30);

//  Serial.print(filtered_HEADING); Serial.print(" "); Serial.print(HD_out); Serial.print(" "); Serial.println(VEL_ANG2uSEC(HD_out));
  SET_PWM_PIN6(VEL_ANG2uSEC(HD_out));//RUDDER
}

