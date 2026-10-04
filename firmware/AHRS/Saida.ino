//************************************************************************
//    Este programa é usado para transmite atitude da plataforma
//    para a interface de visualização gráfica desenvolvida no Python
//    ou para o algoritmo de plotagem das resposta, desenvolvido no 
//    Matlab.
//************************************************************************

void printdata(void)
{    
  
  
  uint8_t PRINT_DATA = 1; // Variavel de Seleção. 
                      // PRINT_DATA = 0, não envia nada
                      // PRINT_DATA = 1, envia os dados formatados no formato esperado pelo Arduino (controlador de posição do quadrirrotor)

  // Envia dados para o programa de interface gráfica Python    
  if (PRINT_DATA == 1)
   {
      Serial.write("i");
       /*
       float accel_BODY[3]={ToAccelSI(accel_x),ToAccelSI(accel_y),ToAccelSI(accel_z)};
       float accel_NED[3]={0,0,0};
       Multiplica_Matriz_Vetor(Matrix_rotacao,accel_BODY, accel_NED);
       Serial.print(accel_NED[0],3);
       Serial.print(" ");
       Serial.print(accel_NED[1],3);
       Serial.print(" ");
       Serial.print(accel_NED[2],3);
       Serial.print(" ");
       */
       
       //float accel_BODY[3]={ToAccelSI(accel_x),ToAccelSI(accel_y),ToAccelSI(accel_z)};
       //float accel_INERTIAL[3]={0,0,0};
       //Multiplica_Matriz_Vetor(Matrix_rotacao,accel_BODY, accel_INERTIAL);

//       //O eixo z do sistema inercial aponta para cima
//       Serial.print(-accel_INERTIAL[0],3);
//       Serial.print(" ");
//       Serial.print(accel_INERTIAL[1],3);
//       Serial.print(" ");
//       Serial.print(accel_INERTIAL[2]-9.80665,3); //Removing the gravity value from the measured az
//       Serial.print(" ");

         //O eixo z do sistema inercial aponta para cima
       Serial.print(-ToAccelSI(accel_x),3);
       Serial.print(" ");
       Serial.print(ToAccelSI(accel_y),3);
       Serial.print(" ");
       Serial.print(ToAccelSI(accel_z)-9.80665,3); //Removing the gravity value from the measured az
       Serial.print(" ");
       Serial.print(-yaw_output,3); // angulo de yaw, retirado da DCM, corrigido/encontrado usando o magnetometro
       Serial.print(" ");
       Serial.print(-ToDeg(MAG_Heading),3); //leitura do magnetometro, usando pitch e roll da DCM para traduzir de mx,my,mz para o angulo de fato
       
       Serial.write("f");
       Serial.println();
   }

}

