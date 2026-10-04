void initialize_controllers()
{
  //Communication with the robot Arduino
    Wire.begin(); // join i2c bus (address optional for master)
}
 

void POS_controller()
{
  //P controller, from position to desired speed
  desired_velX = (0.1)*(desired_pos_I[0]-X_INS[2]);
  desired_velY = (0.1)*(desired_pos_I[1]-X_INS[3]);
  desired_wz   = (1)*(deg2rad(desired_pos_I[2])-X_INS[4]);

  SATURATION(desired_velX, 0.5, -0.5);
  SATURATION(desired_velY, 0.5, -0.5);
  SATURATION(desired_wz,     1,   -1);

//  Serial.print(desired_velX); Serial.print(" "); 
//  Serial.print(desired_velY); Serial.print(" "); 
//  Serial.print(rad2deg(desired_wz)); Serial.println();

  // ------ Sending the desired vels to the robot -------
  float dvel_I[3];
  dvel_I[0] = desired_velX*1000; // [mm/s]
  dvel_I[1] = desired_velY*1000; // [mm/s]
  dvel_I[2] = desired_wz*1000;   // [krad/s]

  float R_I2b[2][2];
  R_I2b[0][0]= cos(X_INS[4]);R_I2b[0][1]=sin(X_INS[4]);
  R_I2b[1][0]=-sin(X_INS[4]);R_I2b[1][1]=cos(X_INS[4]);

  float dvel_b[3];
  Matrix.Multiply((float*)R_I2b, (float*)dvel_I, 2, 2, 1, (float*)dvel_b);
  dvel_b[2]=dvel_I[2];
  
  sprintf(msg,"i%d,%d,%df",(int)dvel_b[0],(int)dvel_b[1], (int)dvel_b[2]);
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

