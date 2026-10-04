//This procedure ARM the KK215 board, allowing the flight
void arm_kk215()
{
  SET_PWM_PIN2(1000);//THROTTLE
  SET_PWM_PIN6(1000);//RUDDER

  delay(3000);
  
  SET_PWM_PIN2(1000);//THROTTLE
  SET_PWM_PIN6(1500);//RUDDER
}

//This procedure DISARM the KK215 board, stopping the flight
void disarm_kk215()
{
  SET_PWM_PIN2(1000);//THROTTLE
  SET_PWM_PIN6(2000);//RUDDER

  delay(4000);
  
  SET_PWM_PIN2(1000);//THROTTLE
  SET_PWM_PIN6(1500);//RUDDER
}

//This method is called whenever the quadrotor identified a problem, in order to stop the system
void smoth_landing()
{
    idle_state_PWMs();
    //Lets get the quadrotor down
    while (distance2ground>20)
    {
      desired_pos_I[2] = desired_pos_I[2] - 0.0001;
      SATURATION(desired_pos_I[2],1,0);
      Z_controller();
      delay(20);
    }
    idle_state_PWMs();
    while(true);
}

//
void print_position_I(char newline)
{
  Serial.print(pos_I[0]);    Serial.print(" ");
  Serial.print(pos_I[1]);    Serial.print(" ");
  Serial.print(pos_I[2]);    Serial.print(newline);
}

void print_desired_position(char newline)
{
  Serial.print(desired_pos_I[0]);    Serial.print(" ");
  Serial.print(desired_pos_I[1]);    Serial.print(" ");
  Serial.print(desired_pos_I[2]);    Serial.print(newline);
}

void print_IMU_data(char newline)
{
  Serial.print(accel_b[0]);    Serial.print(" ");
  Serial.print(accel_b[1]);    Serial.print(" ");
  Serial.print(accel_b[2]);    
  Serial.print(" ");
  Serial.print(actual_roll);    Serial.print(" ");
  Serial.print(actual_pitch);   Serial.print(" ");
  Serial.print(actual_HEADING); 
  Serial.print(newline);
}

void verify_IMU_connection(float degree_range_startup_error)
{
  //The IMU must be connect, and the startup heading value must be inside of X degrees range
  //otherwise, the code will get stuck here, stopping the test
  digitalWrite(offBoardLED,LOW);
  float startup_heading=0;
  
  for(short counterFor=0;counterFor<50;counterFor++)
  {
    while(!new_data_ahrs)
    {
      // Holds waiting for AHRS message to start the program (accel x y and mag);
      receive_data_ahrs(); // Try to read data comming from the AHRS 
      delay(10);
    }
    parse_msg_ahrs();   //Retrieve accel and heading information from the image
    new_data_ahrs = false;
    startup_heading += actual_HEADING/50;
//    Serial.println(startup_heading);
  }
  // at the startup of the system, the magnetometer measurement is more than X degrees wrong, do not proceed
  if(abs(startup_heading)>degree_range_startup_error)
  {
    while(true)
    {
      blink_offboard_led(2);// "Morse code" for the user to know the problem was at IMU
    }
  }
}

void blink_offboard_led(int number_of_blink)
{
  delay(1000-(number_of_blink*200));
  for(;number_of_blink>0;number_of_blink--)
  {
    digitalWrite(offBoardLED, !digitalRead(offBoardLED)); delay(100);
    digitalWrite(offBoardLED, !digitalRead(offBoardLED)); delay(100);
  }
}


void rotate_body2inertial()
{
  //filtered values are always in [degrees]
  float  roll = deg2rad(actual_roll);
  float pitch = deg2rad(actual_pitch);
        
  //this routine takes 370us
  float ROT[3][3];
  ROT[0][0]= 1   ;ROT[0][1]= 0           ;ROT[0][2]= 0;
  ROT[1][0]= 0   ;ROT[1][1]= cos(roll)   ;ROT[1][2]= -sin(roll);
  ROT[2][0]= 0   ;ROT[2][1]= sin(roll)   ;ROT[2][2]=  cos(roll);

  Matrix.Multiply((float*)ROT, (float*)accel_b, 3, 3, 1, (float*)t1);

  ROT[0][0]=  cos(pitch)   ;ROT[0][1]= 0    ;ROT[0][2]= sin(pitch); 
  ROT[1][0]=      0        ;ROT[1][1]= 1    ;ROT[1][2]=     0;
  ROT[2][0]= -sin(pitch)   ;ROT[2][1]= 0    ;ROT[2][2]= cos(pitch);

  Matrix.Multiply((float*)ROT, (float*)t1, 3, 3, 1, (float*)U);
}

//Integration, find X_INS[k+1] from X_INS[k] and U[k]
void euler_integration2()
{
  //this routine takes 100us 
  SATURATION(U[0], 2, -2);
  SATURATION(U[1], 2, -2); // +-2 m/s^2 max acceleration
  SATURATION(U[2], deg2rad(45), deg2rad(-45)); // saturation at +-45º/s angle speed 
   
  X_INS[4] = X_INS[4] + U[2]*dt; //psi in X_INS[4] is in [radians]
  X_INS[0] = X_INS[0] + cos(X_INS[4])*U[0]*dt - sin(X_INS[4])*U[1]*dt; //velocity in [m/s]
  X_INS[1] = X_INS[1] + sin(X_INS[4])*U[0]*dt + cos(X_INS[4])*U[1]*dt; //velocity in [m/s]
  X_INS[2] = X_INS[2] + X_INS[0] * dt; //position in [m]
  X_INS[3] = X_INS[3] + X_INS[1] * dt; //position in [m]

  SATURATION(X_INS[0], 1, -1);
  SATURATION(X_INS[1], 1, -1);
  SATURATION(X_INS[2], 3, 0);
  SATURATION(X_INS[3], 3, 0);
  SATURATION(X_INS[4], deg2rad(90), deg2rad(-90));
}

void clear_temporary_matrices()
{
  //this routine takes 440us
  byte i,j;
  for(i=0;i<8;i++){ for(j=0;j<8;j++){  t1[i][j]=0; } }
  for(i=0;i<8;i++){ for(j=0;j<8;j++){  t2[i][j]=0; } }
  for(i=0;i<8;i++){ for(j=0;j<8;j++){  t3[i][j]=0; } }
  for(i=0;i<8;i++){ for(j=0;j<8;j++){  t4[i][j]=0; } }
  for(i=0;i<8;i++){ for(j=0;j<8;j++){  t5[i][j]=0; } }
  for(i=0;i<8;i++){ for(j=0;j<8;j++){  t6[i][j]=0; } }
}
