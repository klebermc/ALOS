void Calibration_Compass() {// Função utilizada para calibrar o magnetometro
  unsigned int iter=0;
  int max_mx=0, min_mx=0, max_my=0, min_my=0, max_mz=0, min_mz=0;


  Read_Compass(); // Efetua a leitura dos dados do magnetometro (x,y e z)
  max_mx=magnetom_x; min_mx=magnetom_x;
  max_my=magnetom_y; min_my=magnetom_y;
  max_mz=magnetom_z; min_mz=magnetom_z;
  
  for(iter=0; iter<N_CALIB_DATA; iter++)
  {
    Read_Compass(); // Efetua a leitura dos dados do magnetometro (x,y e z)
    
    Serial.print(iter);       
    Serial.print("=[");
    Serial.print(magnetom_x); 
    Serial.print("] , [");
    Serial.print(magnetom_y); 
    Serial.print("] , [");
    Serial.print(magnetom_z); 
    Serial.print("]");
    Serial.println();
    
    if(magnetom_x>max_mx) max_mx=magnetom_x;
    if(magnetom_x<min_mx) min_mx=magnetom_x;
    if(magnetom_y>max_my) max_my=magnetom_y;
    if(magnetom_y<min_my) min_my=magnetom_y;
    if(magnetom_z>max_mz) max_mz=magnetom_z;
    if(magnetom_z<min_mz) min_mz=magnetom_z;        
    digitalWrite(STATUS_LED,HIGH); // Manda o pino 13 para o nivel alto.
    delay(50); // Gera uma atraso
    digitalWrite(STATUS_LED,LOW); // Manda o pino 13 para o nivel baixo.
    delay(50); // Gera uma atraso
  }

  //Calcula o bias e o ganho de escala das leituras do magnetometro
  //Essas variáveis são globais
  mx_bias=(max_mx+min_mx)/2; // calculo do bias
  my_bias=(max_my+min_my)/2;
  mz_bias=(max_mz+min_mz)/2;
  mx_ganhoescala=(max_mx-min_mx)/2; // calculo do ganho de escala
  my_ganhoescala=(max_my-min_my)/2;
  mz_ganhoescala=(max_mz-min_mz)/2;
  
  Serial.print("-> mx_max=");   Serial.print(max_mx);
  Serial.print(" , mx_min=");   Serial.print(min_mx);
  Serial.print(" , mx_bias=");  Serial.print(mx_bias);
  Serial.print(" , mx_ge=");    Serial.print(mx_ganhoescala);
  Serial.println("<-");
  
  Serial.print("-> my_max=");   Serial.print(max_my);
  Serial.print(" , my_min=");   Serial.print(min_my);
  Serial.print(" , my_bias=");  Serial.print(my_bias);
  Serial.print(" , my_ge=");    Serial.print(my_ganhoescala);
  Serial.println("<-");
  
  Serial.print("-> mz_max=");   Serial.print(max_mz);
  Serial.print(" , mz_min=");   Serial.print(min_mz);
  Serial.print(" , mz_bias=");  Serial.print(mz_bias);
  Serial.print(" , mz_ge=");    Serial.print(mz_ganhoescala);
  Serial.println("<-");
  
}


void Calibration_Accel_Gyro(){ // Função utilizada para calibrar o aceletrometro e o girometro
  unsigned int iter=0;
  long OFFSET_TEMP[6]={0,0,0,0,0,0};
  
  Read_adc_raw(); // Efetua a primeira leitura dos ADC para x,y e z. (girometro)
  Read_Accel(); // Efetua a primeira leitura das aceleração em x,y e z.
  delay(20); // Gera um atraso

  for(iter=0; iter<N_CALIB_DATA; iter++)
  {
        unsigned int iter=0;
        Read_adc_raw(); // Efetua uma nova leitura dos ADC para x,y e z. Desconsidera a primeira leitura (girometro)
        Read_Accel(); // Efetua uma nova leitura de x,y e z dos acelerometros. Desconsidera a primeira leitura
        for(int y=0; y<6; y++)  // Laço usado para armazenar os valores de offset em função da leitura dos sensores
          OFFSET_TEMP[y] += AN[y]; // Atualiza o vetor de offset

        //Para manter a frequencia de leitura de accel e gyro em 50Hz, o delay total tem que ser 20ms
        digitalWrite(STATUS_LED,HIGH); // Manda o pino 13 para o nivel alto.
        delay(10); // Gera uma atraso
        digitalWrite(STATUS_LED,LOW); // Manda o pino 13 para o nivel baixo.
        delay(10); // Gera uma atraso
  }

  //Serial.print(" | ");
  for(int y=0; y<6; y++){
    AN_OFFSET[y] = OFFSET_TEMP[y]/N_CALIB_DATA;
    //Serial.print(AN_OFFSET[y]);
    //Serial.print(" | ");
  }
  //Serial.println();
  
  AN_OFFSET[5]-= GRAVITY*SENSOR_SIGN[5];   // Determina o valor de offset para Z

}
