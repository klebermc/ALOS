//************************************************************************
//    Este programa é usado para transmite atitude da plataforma
//    para a interface de visualização gráfica desenvolvida no Python
//    ou para o algoritmo de plotagem das resposta, desenvolvido no 
//    Matlab.
//************************************************************************

void printdata(void)
{    
  char inByte = 0; // Variavel de controle para o envio serial do dado de saida 
  
  uint8_t PRINT_DATA = 1; // Variavel de Seleção. 
                      // PRINT_DATA = 0, não envia nada
                      // PRINT_DATA = 1, envia os dados formatados no formato esperado pelo Arduino (controlador de posição do quadrirrotor)

  // Envia dados para o programa de interface gráfica Python    
  if (PRINT_DATA == 1)
  {
    counter_print_output = (counter_print_output + 1) % numReadings;
    
    //accel x
    history_accelx[posVectAccelx] = accel_b[0];
    posVectAccelx = (posVectAccelx + 1) % numReadings;
    //accel y
    history_accely[posVectAccely] = accel_b[1];
    posVectAccely = (posVectAccely + 1) % numReadings;
    //accel z
    history_accelz[posVectAccelz] = accel_b[2];
    posVectAccelz = (posVectAccelz + 1) % numReadings;
    //roll
    history_Roll[posVectRoll] = roll_output;
    posVectRoll = (posVectRoll + 1) % numReadings;
    //pitch
    history_Pitch[posVectPitch] = pitch_output;
    posVectPitch = (posVectPitch + 1) % numReadings;
    //yaw
    history_Yaw[posVectHD] = yaw_output;
    posVectHD = (posVectHD + 1) % numReadings;
    //gyro z
    history_accelwz[posVectAccelwz] = gyro_b[2];
    posVectAccelwz = (posVectAccelwz + 1) % numReadings;

    if (counter_print_output==0)
    {
      float filtered_accel_b[3];
      // -------  Filtering the IMU data ----------
      filtered_accel_b[0] = median(history_accelx);
      filtered_accel_b[1] = median(history_accely);
      filtered_accel_b[2] = median(history_accelz);
  
      float filtered_wz_b = median(history_accelwz);
  
      //incoming values are in [degrees]
      filtered_Roll   = median(history_Roll); 
      filtered_Pitch  = median(history_Pitch);
      filtered_Yaw    = median(history_Yaw);

      // --- Filtering spikes--
      //if there is a small diference between the last measured accel and the filtered value, send the last one
      if (abs( filtered_accel_b[0] - accel_b[0] ) < 0.5 ) filtered_accel_b[0] = accel_b[0];
      if (abs( filtered_accel_b[1] - accel_b[1] ) < 0.5 ) filtered_accel_b[1] = accel_b[1];
      if (abs( filtered_accel_b[2] - accel_b[2] ) < 0.5 ) filtered_accel_b[2] = accel_b[2];
      if (abs( filtered_Roll - roll_output ) < 5 ) filtered_Roll = roll_output;
      if (abs( filtered_Pitch - pitch_output ) < 5 ) filtered_Pitch = pitch_output;
      if (abs( filtered_Yaw - yaw_output ) < 5 ) filtered_Yaw = yaw_output;
      if ((abs(filtered_wz_b - gyro_b[2] ) / filtered_wz_b) < 1 ) filtered_wz_b = gyro_b[2];
      // ------- --------------------- -----------
            
      Serial.write("i");
      //------------------------------------
      Serial.print(filtered_accel_b[0],3);
      Serial.print(" ");
      Serial.print(filtered_accel_b[1],3);//O eixo y do body aponta para a esquerda
      Serial.print(" ");
      Serial.print(filtered_accel_b[2],3);//O eixo z do body aponta para cima
      Serial.print(" ");
      Serial.print(filtered_Roll,3); // angulo de yaw, retirado da DCM, corrigido/encontrado usando o magnetometro
      Serial.print(" ");
      Serial.print(filtered_Pitch,3); // angulo de yaw, retirado da DCM, corrigido/encontrado usando o magnetometro
      Serial.print(" ");
      Serial.print(filtered_Yaw,3); // angulo de yaw, retirado da DCM, corrigido/encontrado usando o magnetometro
      Serial.print(" ");
      Serial.print(filtered_wz_b,3); // girometro eixo z
      //------------------------------------ 
      Serial.write("f");
      Serial.println();

   }
  }
//   if (Serial.available() > 0) // Verifica se uma palavra de solicitação de envio foi recebida
//    {
//       inByte = Serial.read();  // Ler o dado recebido
//       if(inByte == 7) // Verifica se o dado recebido era o esperado
//          {
//             delay(10);
//             transmite_dado(); // Chama a função transmite
//             inByte = 0;
//          }
//    }   

}

void transmite_dado()
{
  
  memcpy(&buffer_pitch,&pitch_output,4); // Converte a leitura do eixo x de 32 bits tipo float para 4 bytes em tipo char
   
  Serial.write(buffer_pitch[0]); //  Escreve dado no barramento serial
  Serial.write(buffer_pitch[1]);
  Serial.write(buffer_pitch[2]);
  Serial.write(buffer_pitch[3]); 
  delay(5);
     
  memcpy(&buffer_roll,&roll_output,4); // Converte a leitura do eixo y de 32 bits tipo float para 4 bytes em tipo char
   
  Serial.write(buffer_roll[0]);  // Escreve dado no barramento serial
  Serial.write(buffer_roll[1]);
  Serial.write(buffer_roll[2]);
  Serial.write(buffer_roll[3]);
  delay(5);
      
  memcpy(&buffer_yaw,&yaw_output,4); // Converte a leitura do eixo z de 32 bits tipo float para 4 bytes em tipo char
 
  Serial.write(buffer_yaw[0]);  // Escreve dado no barramento serial
  Serial.write(buffer_yaw[1]);
  Serial.write(buffer_yaw[2]);
  Serial.write(buffer_yaw[3]);
  delay(5);
   
}
