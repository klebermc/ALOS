void receive_data_xbee ()
{
  if (Serial2.available())
  {
    length_msg_xbee = 0;
    in_msg_xbee[0] = 0;
    actual_time_xbee = millis();
    while ( (actual_time_xbee + TIMEOUT_MSG_XBEE) > millis()) //If there is data to be received, stay in the loop while: All the data was received, a limit time passes
    {
      in_msg_xbee[length_msg_xbee + 1] = 0;
      if (length_msg_xbee < MAX_MSG_SIZE)
      {
        if (Serial2.available())
        {
          in_msg_xbee[length_msg_xbee] = Serial2.read();
          if (in_msg_xbee[length_msg_xbee] == '\n') //this carachter indicates the end of the message
          {
            in_msg_xbee[length_msg_xbee] = 0;
            msg_xbeeOK = true;
            //Serial.print("-"); Serial.print(count); Serial.println("-");
            break;
          }
          //Serial.print("["); Serial.print(in_msg_xbee[length_msg_xbee]); Serial.print("]");
          length_msg_xbee++;
        }
      }
      else
      {
        length_msg_xbee = 0;
        msg_xbeeOK = false;
        break;
      }
      delayMicroseconds(100);
    }
    //Serial.println(millis() - actual_time_xbee);
  }
}


//This function retrieve from the message the actual accel posX, posY and posZ in the inertial reference system, calculated by the Location System
void parse_msg_xbee() {
  char *token_from_msg;
  char delimiters[] = ",";
  uint8_t iterator = 0;

  token_from_msg = strtok(in_msg_xbee, delimiters); //This initializes strtok with our string to tokenize
  //Serial.print("XBEE INSIDE Parse: ");
  while (token_from_msg != NULL) {
    //Serial.print(iterator);    Serial.print("->");
    //Serial.print(token_from_msg);    Serial.print(" ");

    switch (iterator) {
      case 0:
        pos_LS[0] = atof (token_from_msg)/1000; //first value is the X position
        break;
      case 1:
        pos_LS[1] = atof (token_from_msg)/1000; //second value is the Y position
        break;
      case 2:
        pos_LS[2] = atof (token_from_msg)/1000; //third value is the Z position
        break;
      case 3:
        desired_pos_I[0] = atof (token_from_msg)/1000; //first value is the X position
        break;
      case 4:
        desired_pos_I[1] = atof (token_from_msg)/1000; //second value is the Y position
        break;
      case 5:
        desired_pos_I[2] = atof (token_from_msg)/1000; //third value is the Z position
        break;
    }
    //the third value is the yaw, should be similar to the magnetometer heading measurement, but it is derived from the DCM computed internally at AHRS

    token_from_msg = strtok(NULL, delimiters);    //Here we pass in a NULL value, which tells strtok to continue working with the previous string
    iterator++;
  }
  //Serial.println();
}
