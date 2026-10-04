//***********************************************************************
//      Este algoritmo utiliza  para determinar a orientação da plataforma
//      em função dos valores de intensidade do campo magneticos medidos
//      nos eixos X e Y do magnetômetro.
//***********************************************************************


void Compass_Heading() // Função utilizada para calcular o angulo de orientação
{
//  Serial.print( "Compass_Head \t= " );
  float MAG_X;  
  float MAG_Y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;
   
  cos_roll = cos(roll); // Calcula cosseno de roll em função do valor ângulo de roll atual. 
  sin_roll = sin(roll);  // Calcula seno de roll em função do valor ângulo de roll atual.
  cos_pitch = cos(pitch); // Calcula cosseno de pitch em função do valor ângulo de pitch atual.
  sin_pitch = sin(pitch); // Calcula seno de pitch em função do valor ângulo de pitch atual. 
  
  MAG_X = magnetom_x*cos_pitch + magnetom_y*sin_roll*sin_pitch + magnetom_z*cos_roll*sin_pitch; // Calcula a componente magnetica em X
  MAG_Y = magnetom_y*cos_roll - magnetom_z*sin_roll; // Calcula a componente magnetica em y.
 
  MAG_Heading = atan2(-MAG_Y,MAG_X); // Calcula o valor de orientação
  MAG_Heading = MAG_Heading - ToRad(Angle_Xinertial2MagNorth);

//  Serial.print( MAG_X ); Serial.print(" ");
//  Serial.print( MAG_Y ); Serial.print(" ");
//  Serial.print(ToDeg(MAG_Heading)); Serial.print("\n");
}

//Essa função encontra o valor da diferença entre o norte geográfico e o eixo X da área de teste 
//o valor lido durante o experimento, com essa correção, 
//a bussola deverá "ler" 0 graus quando o X do corpo apontar para o X da área de teste
void findCompassOffsetValue()
{
  
  float temp_offset_value=0;
  int iter;
  for (iter = 0; iter < N_CALIB_DATA; iter++)
  {
    Read_Compass();
    Compass_Heading();
    temp_offset_value += MAG_Heading / N_CALIB_DATA; //calculating the mean value
    digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
    delay(25);
//    Serial.print(temp_offset_value); Serial.print(" "); Serial.println(ToDeg(temp_offset_value));
  }
  Angle_Xinertial2MagNorth=ToDeg(temp_offset_value);
  Read_Compass();
  Compass_Heading();
//  Serial.println(Angle_Xinertial2MagNorth);
}
