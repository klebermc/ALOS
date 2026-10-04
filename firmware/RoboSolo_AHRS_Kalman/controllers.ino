void initialize_controllers()
{
  //Communication with the robot Arduino
    Wire.begin(); // join i2c bus (address optional for master)
}
 
  
void POS_controller()
{
//  float LOS = (atan2((desired_pos_I[1]-pos_LS[1]),(desired_pos_I[0]-pos_LS[0]))); // Navigation using SILA data only
  LOS = (atan2((desired_pos_I[1]-X_INS[3]),(desired_pos_I[0]-X_INS[2]))); // Navigation using Kalman filter estimation

//  float error_psi = LOS-deg2rad(actual_HEADING);// Navigation using compass
  error_psi = LOS-X_INS[4];
  
  error_psi -= deg2rad(360) * floor((error_psi + deg2rad(180)) / deg2rad(360));
  
//  if(desired_pos_I[0]==1.25 && desired_pos_I[1]==2.40) error_psi=abs(error_psi);
  //if(error_psi>deg2rad(90))error_psi=deg2rad(170);
  //if(error_psi<deg2rad(-90))error_psi=deg2rad(-170);
  speed_left = (0.1)*((1-tan((error_psi)/2)) + (1-sin(error_psi)));
  speed_right = (0.1)*((1+tan((error_psi)/2)) + (1+sin(error_psi)));

  SATURATION(speed_right, 0.3, -0.3);
  SATURATION(speed_left, 0.3, -0.3);

//if( desired_pos_I[2]== 1 ){
//    speed_left=0.1;  
//    speed_right=0.1;
//    digitalWrite(offBoardLED, !digitalRead(offBoardLED));
//}
//if( desired_pos_I[2]== 0 ){
//    speed_left=0;  
//    speed_right=0;
//}
//  Serial.print(rad2deg(LOS)); Serial.print(" "); 
//  Serial.print(rad2deg(X_INS[4])); Serial.print(" "); 
//  Serial.print(rad2deg(error_psi)); Serial.print(" "); 
//  Serial.print(speed_right); Serial.print(" "); 
//  Serial.println(speed_left);

   // ------ Sending the desired vels to the robot -------
  speed_right*=1000; // [mm/s]
  speed_left*=1000;  // [mm/s]
  sprintf(msg,"i%d,%df",(int)speed_right,(int)speed_left);
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

