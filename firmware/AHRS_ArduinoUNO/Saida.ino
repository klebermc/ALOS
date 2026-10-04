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
    history_yaw[posVectHD] = yaw_output;
    posVectHD = (posVectHD + 1) % numReadings;
    //gyro z
    history_accelwz[posVectAccelwz] = wz_b;
    posVectAccelwz = (posVectAccelwz + 1) % numReadings;
    
      Serial.write("i");

      //O eixo z do sistema inercial aponta para cima
       Serial.print(accel_b[0],3);
       Serial.print(" ");
       Serial.print(accel_b[1],3);
       Serial.print(" ");
       Serial.print(accel_b[2],3); //Removing the gravity value from the measured az
       Serial.print(" ");

//       float accel_INERTIAL[3]={0,0,0};
//       Multiplica_Matriz_Vetor(Matrix_rotacao,accel_b, accel_INERTIAL);
//       //O eixo z do sistema inercial aponta para cima
//       Serial.print(accel_INERTIAL[0],3);
//       Serial.print(" ");
//       Serial.print(accel_INERTIAL[1],3);
//       Serial.print(" ");
//       Serial.print(accel_INERTIAL[2]-9.80665,3); //Removing the gravity value from the measured az
//       Serial.print(" ");

       Serial.print(roll_output,3); // angulo de yaw, retirado da DCM, corrigido/encontrado usando o magnetometro
       Serial.print(" ");
       Serial.print(pitch_output,3); // angulo de yaw, retirado da DCM, corrigido/encontrado usando o magnetometro
       Serial.print(" ");
       Serial.print(yaw_output,3); // angulo de yaw, retirado da DCM, corrigido/encontrado usando o magnetometro
       Serial.print(" ");
//       Serial.print(ToDeg(MAG_Heading),3); //leitura do magnetometro, usando pitch e roll da DCM para traduzir de mx,my,mz para o angulo de fato
       
       Serial.print(gyro_b[2],3); // girometro eixo z
       
       Serial.write("f");
       Serial.println();
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
