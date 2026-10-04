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
    }
    else if (in_char_ahrs == '\n')
    {
      new_data_ahrs = true;
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
  
  token_from_msg = strtok(in_msg_ahrs, delimiters); //This initializes strtok with our string to tokenize
  while (token_from_msg != NULL) {
    switch (iterator) {
      case 0:
        accel_I[0] = atof (token_from_msg); //first value is the accel N
        break;
      case 1:
        accel_I[1] = atof (token_from_msg); //second value is the accel E
        break;
      case 2:
        accel_I[2] = atof (token_from_msg); //third value is the accel D
        break;
      case 4:
        actual_heading = atof (token_from_msg); //fifth value is the heading by the magnetometer
        break;
    }
    //the third value is the yaw, should be similar to the magnetometer heading measurement, but it is derived from the DCM computed internally at AHRS

    token_from_msg = strtok(NULL, delimiters);    //Here we pass in a NULL value, which tells strtok to continue working with the previous string
    iterator++;
  }
}

