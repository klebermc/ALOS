void receive_data_ahrs()
{
  while (Serial1.available())
  {
    in_char_ahrs = Serial1.read();
    if (in_char_ahrs == 'i') //begging of the message
    {
      pos_vmsg = 0;
    }
    else if (in_char_ahrs == 'f') // end of the message
    {
      size_msg = pos_vmsg + 1;
      in_msg_ahrs[pos_vmsg] = 0;
      new_data_ahrs = true;
    }
    else if (in_char_ahrs == '\n')
    {
      break; //stops the while when it recognizes an end of message given by a new line '\n'
    }
    else
    {
      in_msg_ahrs[pos_vmsg] = in_char_ahrs;
      pos_vmsg++;
    }
  }
}

//This function retrieve from the message the actual accel aN, aE, aD, and the haeading read by the magnetometer
void parse_msg_ahrs() {
  char *token_from_msg;
  char delimiters[] = " ";
  uint8_t iterator = 0;
  //Serial.println(in_msg_ahrs);
  
  token_from_msg = strtok(in_msg_ahrs, delimiters); //This initializes strtok with our string to tokenize
  //Serial.print("AHRS INSIDE Parse: ");
  while (token_from_msg != NULL) {
    //Serial.print(iterator);    Serial.print("->");
    //Serial.print(token_from_msg);    Serial.print(" ");

    switch (iterator) {
      case 0:
        pitch = atof (token_from_msg); //first value is the accel N
        break;
      case 1:
        roll = atof (token_from_msg); //second value is the accel E
        break;
      case 2:
        yaw = atof (token_from_msg); //third value is the accel D
        break;
    }
    //the third value is the yaw, should be similar to the magnetometer heading measurement, but it is derived from the DCM computed internally at AHRS

    token_from_msg = strtok(NULL, delimiters);    //Here we pass in a NULL value, which tells strtok to continue working with the previous string
    iterator++;
  }
  //Serial.println();
}

