
//Integration, from acceleration to velocity and position, in inertial coordinate system
void euler_integration()
{
  SATURATION(accel_I[0], 2, -2);
  SATURATION(accel_I[1], 2, -2);
  
  vel_I[0] = vel_I[0] + accel_I[0] * dt;
  vel_I[1] = vel_I[1] + accel_I[1] * dt;
  //vel_I[2] = vel_I[2] + accel_I[2] * dt;

  //Saturation of velocity
  SATURATION(vel_I[0],0.10,-0.10);
  SATURATION(vel_I[1],0.10,-0.10);
  
  pos_I[0] = pos_I[0] + vel_I[0] * dt;
  pos_I[1] = pos_I[1] + vel_I[1] * dt;
  //pos_I[2] = pos_I[2] + vel_I[2] * dt;
  
  //Saturation of position
  SATURATION(pos_I[0],3,0);
  SATURATION(pos_I[1],3,0);
  //SATURATION(pos_I[2],3,0); 
}

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
void print_position(char newline)
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
  Serial.print(accel_I[0]);    Serial.print(" ");
  Serial.print(accel_I[1]);    Serial.print(" ");
  Serial.print(accel_I[1]);    Serial.print(" ");
  Serial.print(actual_roll);    Serial.print(" ");
  Serial.print(actual_pitch);   Serial.print(" ");
  Serial.print(actual_HEADING); Serial.print(newline);
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
    Serial.println(startup_heading);
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
