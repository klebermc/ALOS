//***********************************************************************
//      Este algoritmo é utilizada para estabelecer a comunicação 
//      entre a IMU LSM9DS1 e o Arduino
//***********************************************************************

void setupIMU()
{
  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  
  imu.settings.accel.bandwidth = 3;        // Anti-aliasing filter at 50Hz, this filter comes before the ADC that is reading the accel values
  imu.settings.accel.highResEnable = true; // habilita o filtro passa baixa
  imu.settings.accel.highResBandwidth = 3; // coloca o filtro para ~3Hz
  
  // Iniciando funções da imu
  if (!imu.begin())
  {
    Serial.println("Falha em comunicar com o LSM9DS1 (IMU).");
     digitalWrite(STATUS_LED, LOW);
    while (1)
    {
      digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
      delay(100);
      digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
      delay(100);
      digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
      delay(100);
      digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
      delay(700);
    };
  }
  delay(2000);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//    Acessa os registradores de armazenamento dos dados amostrados e efetua a leitura
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void Read_Accel() // Função utilizada para ler x,y e z dos acelerometros
{
//  Serial.print( "Read_Accel \t= " );
  if(imu.accelAvailable())
  {
    imu.readAccel();
    ACC[0]=imu.calcAccel(imu.ax)*9.8066;
    ACC[1]=imu.calcAccel(imu.ay)*9.8066;
    ACC[2]=imu.calcAccel(imu.az)*9.8066;
  }  
    accel_b[0] = SENSOR_SIGN[3]*(ACC[0]-AN_OFFSET[3]); // Condiciona os valores recebidos de x,y e z para o acelerometro 
    accel_b[1] = SENSOR_SIGN[4]*(ACC[1]-AN_OFFSET[4]); // em função dos sinais de conversão dos eixos e dos erros de offset 
    accel_b[2] = SENSOR_SIGN[5]*(ACC[2]-AN_OFFSET[5]);

//    Serial.print("ACC = [");
//    Serial.print(ACC[0]);Serial.print(",");
//    Serial.print(ACC[1]);Serial.print(",");
//    Serial.print(ACC[2]);Serial.print("] ");
//    Serial.print("AN_OFFSET = [");
//    Serial.print(AN_OFFSET[3]);Serial.print(",");
//    Serial.print(AN_OFFSET[4]);Serial.print(",");
//    Serial.print(AN_OFFSET[5]);Serial.print("] ");
//    Serial.print("SENSOR_SIGN = [");
//    Serial.print(SENSOR_SIGN[3]);Serial.print(",");
//    Serial.print(SENSOR_SIGN[4]);Serial.print(",");
//    Serial.print(SENSOR_SIGN[5]);Serial.print("] "); 
//    Serial.print("accel = [");
//    Serial.print(accel_b[0]);Serial.print(",");
//    Serial.print(accel_b[1]);Serial.print(",");
//    Serial.print(accel_b[2]);Serial.print("] "); 
//    Serial.println();
    
 }
 //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//         Acessa os registradores de armazenamento dos dados amostrados e efetua a leitura
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void Read_Compass() // Função utilizada para efetuar a leitura dos dados do magnetometro.
{ 
//  Serial.print( "Read_Compass \t= " );
   if (imu.magAvailable())
  {
    imu.readMag();
    magnetom_x = imu.mx;
    magnetom_y = imu.my;
    magnetom_z = imu.mz;

    magnetom_x=((magnetom_x-mx_bias)*100)/mx_ge; // medidas corrigidas do magnetometro usando bias e ganho de escala
    magnetom_y=((magnetom_y-my_bias)*100)/my_ge; // 100 significa que a escala do magnetometro agora vai de -100 a 100
    magnetom_z=((magnetom_z-mz_bias)*100)/mz_ge; 
      
    magnetom_x = SENSOR_SIGN[6]*(magnetom_x);  // Efetua a concatenação dos bytes MSB e LSB recebidos para o eixo x (internal sensor y axis)
    magnetom_y = SENSOR_SIGN[7]*(magnetom_y);  // Efetua a concatenação dos bytes MSB e LSB recebidos para o eixo y (internal sensor x axis)
    magnetom_z = SENSOR_SIGN[8]*(magnetom_z);  // Efetua a concatenação dos bytes MSB e LSB recebidos para o eixo z
  
  //  Serial.print( magnetom_x ); Serial.print(" ");
  //  Serial.print( magnetom_y ); Serial.print(" ");
  //  Serial.print( magnetom_z ); Serial.print("\n");
  }
}


//***********************************************************************
//      Este programa utiliza os ADC 1, 2 e 3 do microcontrolador
//      AVR- ATmega328 com resolução 10 bits para a conversão
//      valores analogicos fornecidos pelo giro LPR530Al e ALH
//***********************************************************************

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//            Recebe e Armazena os valores amostrados para X, Y e Z
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void Read_Gyro(void)   // Após interrupção gerada ao decorrer da execução do programa, são lidos as taxas de velocidades angulares em x,y e z.                        
{
//  Serial.print( "Read_Gyro \t= " );
  if(imu.gyroAvailable())
  {
    imu.readGyro();
    AN[0]=imu.calcGyro(imu.gx);
    AN[1]=imu.calcGyro(imu.gy);
    AN[2]=imu.calcGyro(imu.gz);

   for(short i=0;i<3;i++)
    {
      if (SENSOR_SIGN[i]<0)  // Verifica se a conversão de sinal usada para o eixo de referencia adotado é negativo 
        gyro_b[i] = (AN_OFFSET[i]-AN[i]); // Retorna o valor corrigido. Em outras palvras sem a presença do offset
      else
        gyro_b[i] = (AN[i]-AN_OFFSET[i]); // Retorna o valor corrigido
    }
  }

//  Serial.print("AN = [");
//  Serial.print( AN[0]); Serial.print(" ");
//  Serial.print( AN[1]); Serial.print(" ");
//  Serial.print( AN[2]); Serial.print("] | ");
//  Serial.print( gyro_b[0]); Serial.print(" ");
//  Serial.print( gyro_b[1]); Serial.print(" ");
//  Serial.print( gyro_b[2]); Serial.print("\n");
}

