
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

void print_estimated_states(char newline)
{
  Serial.print(X_INS[0]);    Serial.print(" ");
  Serial.print(X_INS[1]);    Serial.print(" ");
  Serial.print(X_INS[2]);    Serial.print(" ");
  Serial.print(X_INS[3]);    Serial.print(" ");
  Serial.print(X_INS[4]);    Serial.print(newline);
}

void print_IMU_data(char newline)
{
  Serial.print(accel_b[0]);    Serial.print(" ");
  Serial.print(accel_b[1]);    Serial.print(" ");
  Serial.print(accel_b[2]);    Serial.print(" ");
  Serial.print(actual_roll);    Serial.print(" ");
  Serial.print(actual_pitch);   Serial.print(" ");
  Serial.print(actual_HEADING); Serial.print(" ");
  Serial.print(wz_b);
  Serial.print(newline);
}

float verify_IMU_connection()
{
  //The IMU must be connect, and the startup heading value must be inside of X degrees range
  //otherwise, the code will get stuck here, stopping the test
  digitalWrite(offBoardLED,LOW);
  float startup_heading=0;
  char in_char;
  while (true)
  {
    in_char = Serial1.read();
    if (in_char == 'C') //begging of the mag calibration
    {
        sprintf(msg,"i%d,%df",(int)-250,(int)250);
        Serial.print("<");Serial.print(msg);Serial.println(">");
        byte i=0;  
        Wire.beginTransmission(9); // transmit to device #9  
        while(msg[i]!=0)
        {
          Wire.write(msg[i]);              // sends x 
          i++;
          if (i>=20) break;
        }
        Wire.endTransmission();    // stop transmitting 
    }
    if (in_char == 'c') //ending of mag calibration
    {
        sprintf(msg,"i%d,%df",(int)0,(int)0);
        Serial.print("<");Serial.print(msg);Serial.println(">");
        byte i=0;  
        Wire.beginTransmission(9); // transmit to device #9  
        while(msg[i]!=0)
        {
          Wire.write(msg[i]);              // sends x 
          i++;
          if (i>=20) break;
        }
        Wire.endTransmission();    // stop transmitting 
        break;
    }
  }

  delay(5000);
  for(short counterFor=0;counterFor<10;counterFor++)
  {
    while(!new_data_ahrs)
    {
      // Holds waiting for AHRS message to start the program (accel x y and mag);
      receive_data_ahrs(); // Try to read data comming from the AHRS 
      delay(10);
    }
    parse_msg_ahrs();   //Retrieve accel and heading information from the image
    new_data_ahrs = false;
    startup_heading += actual_HEADING/10;
    //Serial.println(startup_heading);
  }
  return startup_heading;
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
  float  roll = deg2rad(0);
  float pitch = deg2rad(0);
        
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
  SATURATION(U[0], 10, -10);
  SATURATION(U[1], 10, -10); // +-2 m/s^2 max acceleration
  SATURATION(U[2], deg2rad(90), deg2rad(-90)); // saturation at +-90º/s angle speed 
   
  X_INS[4] = X_INS[4] + U[2]*dt; //psi in X_INS[4] is in [radians]
  X_INS[0] = X_INS[0] + cos(X_INS[4])*U[0]*dt - sin(X_INS[4])*U[1]*dt; //velocity in [m/s]
  X_INS[1] = X_INS[1] + sin(X_INS[4])*U[0]*dt + cos(X_INS[4])*U[1]*dt; //velocity in [m/s]
  X_INS[2] = X_INS[2] + X_INS[0] * dt; //position in [m]
  X_INS[3] = X_INS[3] + X_INS[1] * dt; //position in [m]

  SATURATION(X_INS[0], 5, -5); //X inertial frame velocity
  SATURATION(X_INS[1], 5, -5); //Y inertial frame velocity
  SATURATION(X_INS[2], 3, 0); //test area size
  SATURATION(X_INS[3], 3, 0); //test area size
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

void try_comm_with_embeeded_arduino()
{
  sprintf(msg,"i%d,%df",0,0);
  //Serial.print("<");Serial.print(msg);Serial.println(">");
  
  byte i=0;  
  Wire.beginTransmission(9); // transmit to device #9  
  while(msg[i]!=0)
  {
    Wire.write(msg[i]);              // sends x 
    i++;
    if (i>=20) break;
  }
  Wire.endTransmission();    // stop transmitting 
}

