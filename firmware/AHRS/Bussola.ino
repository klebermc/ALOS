//***********************************************************************
//      Este algoritmo utiliza  para determinar a orientação da plataforma
//      em função dos valores de intensidade do campo magneticos medidos
//      nos eixos X e Y do magnetômetro.
//***********************************************************************


void Compass_Heading() // Função utilizada para calcular o angulo de orientação
{
  float MAG_X;  
  float MAG_Y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;
   
  magnetom_x=((magnetom_x-mx_bias)*100)/mx_ganhoescala; // medidas corrigidas do magnetometro usando bias e ganho de escala
  magnetom_y=((magnetom_y-my_bias)*100)/my_ganhoescala; // 100 significa que a escala do magnetometro agora vai de -100 a 100
  magnetom_z=((magnetom_z-mz_bias)*100)/mz_ganhoescala; 
  /*magnetom_x=(magnetom_x-mx_bias);
  magnetom_y=(magnetom_y-my_bias);
  magnetom_z=(magnetom_z-mz_bias);*/
  
  cos_roll = cos(roll); // Calcula cosseno de roll em função do valor ângulo de roll atual. 
  sin_roll = sin(roll);  // Calcula seno de roll em função do valor ângulo de roll atual.
  cos_pitch = cos(pitch); // Calcula cosseno de pitch em função do valor ângulo de pitch atual.
  sin_pitch = sin(pitch); // Calcula seno de pitch em função do valor ângulo de pitch atual. 
  MAG_X = magnetom_x*cos_pitch + magnetom_y*sin_roll*sin_pitch + magnetom_z*cos_roll*sin_pitch; // Calcula a componente magnetica em X
  MAG_Y = magnetom_y*cos_roll - magnetom_z*sin_roll; // Calcula a componente magnetica em y.
 
  MAG_Heading = atan2(-MAG_Y,MAG_X); // Calcula o valor de orientação
  MAG_Heading = MAG_Heading - ToRad(Angle_Xinertial2NORTH);
  //Serial.println(ToDeg(MAG_Heading)); //leitura do magnetometro, usando pitch e roll da DCM para traduzir de mx,my,mz para o angulo de fato
}

