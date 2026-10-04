void Calibration_Compass() {// Função utilizada para calibrar o magnetometro
  unsigned int iter=0;
  int max_mx=0, min_mx=0, max_my=0, min_my=0, max_mz=0, min_mz=0;

//  Read_Compass(); // Efetua a leitura dos dados do magnetometro (x,y e z)
  imu.readMag();
  magnetom_x = imu.mx;
  magnetom_y = imu.my;
  magnetom_z = imu.mz;
  
  max_mx=magnetom_x; min_mx=magnetom_x;
  max_my=magnetom_y; min_my=magnetom_y;
  max_mz=magnetom_z; min_mz=magnetom_z;
  
  for(iter=0; iter<N_CALIB_DATA; iter++)
  {
    if (imu.magAvailable())
    {
      // Efetua a leitura dos dados do magnetometro (x,y e z)
      imu.readMag();
      magnetom_x = imu.mx;
      magnetom_y = imu.my;
      magnetom_z = imu.mz;
           
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
      digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
      delay(100); // Gera uma atraso
    }
  }

  //Calcula o bias e o ganho de escala das leituras do magnetometro
  //Essas variáveis são globais
  mx_bias=(max_mx+min_mx)/2; // calculo do bias
  my_bias=(max_my+min_my)/2;
  mz_bias=(max_mz+min_mz)/2;
  mx_ge=(max_mx-min_mx)/2; // calculo do ganho de escala
  my_ge=(max_my-min_my)/2;
  mz_ge=(max_mz-min_mz)/2;
  
  Serial.print("-> mx_max=");   Serial.print(max_mx);
  Serial.print(" , mx_min=");   Serial.print(min_mx);
  Serial.print(" , mx_bias=");  Serial.print(mx_bias);
  Serial.print(" , mx_ge=");    Serial.print(mx_ge);
  Serial.println("<-");
  
  Serial.print("-> my_max=");   Serial.print(max_my);
  Serial.print(" , my_min=");   Serial.print(min_my);
  Serial.print(" , my_bias=");  Serial.print(my_bias);
  Serial.print(" , my_ge=");    Serial.print(my_ge);
  Serial.println("<-");
  
  Serial.print("-> mz_max=");   Serial.print(max_mz);
  Serial.print(" , mz_min=");   Serial.print(min_mz);
  Serial.print(" , mz_bias=");  Serial.print(mz_bias);
  Serial.print(" , mz_ge=");    Serial.print(mz_ge);
  Serial.println("<-");
  
}

void Calibration_Accel_Gyro(){ // Função utilizada para calibrar o aceletrometro e o girometro
  unsigned int iter=0;
  float OFFSET_TEMP[6]={0,0,0,0,0,0};

  for(iter=0; iter<N_CALIB_DATA; iter++)
  {
        unsigned int iter=0;
        // Efetua uma nova leitura dos ADC para x,y e z.
        if(imu.gyroAvailable())
        {
        imu.readGyro();
        AN[0]=imu.calcGyro(imu.gx);
        AN[1]=imu.calcGyro(imu.gy);
        AN[2]=imu.calcGyro(imu.gz);
        }

        // Efetua uma nova leitura de x,y e z dos acelerometros.
        if(imu.accelAvailable())
        {
        imu.readAccel();
        AN[3]=imu.calcAccel(imu.ax)*9.8066;
        AN[4]=imu.calcAccel(imu.ay)*9.8066;
        AN[5]=imu.calcAccel(imu.az)*9.8066;
        }
        
        for(int y=0; y<6; y++)  // Laço usado para armazenar os valores de offset em função da leitura dos sensores
        {  
          OFFSET_TEMP[y] += AN[y]; // Atualiza o vetor de offset
//          Serial.print(OFFSET_TEMP[y]);Serial.print(" ");
        }
//        Serial.println();
        //Para manter a frequencia de leitura de accel e gyro em 50Hz, o delay total tem que ser 20ms
        digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
        delay(20);
  }

  for(int y=0; y<6; y++)
  {
    AN_OFFSET[y] = OFFSET_TEMP[y]/N_CALIB_DATA;
//    Serial.print(AN_OFFSET[y]);
//    Serial.print(" ");
  }
//  Serial.println();
  
  AN_OFFSET[5]-= GRAVITY*SENSOR_SIGN[5];   // Determina o valor de offset para Z
}
