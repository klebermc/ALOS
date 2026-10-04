void setupIMU()
{
  // Iniciando funções da imu
  if (!imu.begin())
  {
    Serial.println("Falha em comunicar com o LSM9DS1 (IMU).");
    while (1);
  }
  delay(2000);
}

void magnetometerCalibration()
{
  unsigned int iter = 0;
  int max_mx, min_mx, max_my, min_my;

  // Efetua a leitura dos dados do magnetometro (x,y e z)
  imu.readMag();

  max_mx = imu.mx; min_mx = max_mx;
  max_my = imu.my; min_my = max_my;
  delay(250);

  // Laço para calcular bias e ganho de escala do magnetômetro
  for (iter = 0; iter < N_CALIB_DATA; iter++)
  {
    imu.readMag();
    magnetom_x = imu.mx;
    magnetom_y = imu.my;

    if (magnetom_x > max_mx) max_mx = magnetom_x;
    if (magnetom_x < min_mx) min_mx = magnetom_x;
    if (magnetom_y > max_my) max_my = magnetom_y;
    if (magnetom_y < min_my) min_my = magnetom_y;
    delay(100);
    digitalWrite(offBoardLED, !digitalRead(offBoardLED));
  }
    
  // Calcula o bias e o ganho de escala das leituras do magnetômetro
  // cálculo do bias
  mx_bias = (max_mx + min_mx) / 2;
  my_bias = (max_my + min_my) / 2;
  // cálculo do ganho de escala
  mx_ge = (max_mx - min_mx) / 2;
  my_ge = (max_my - min_my) / 2;
  
  delay(10);
}

void readCOMP()
{
  if (imu.magAvailable())
  {
    imu.readMag();
    magnetom_x = imu.mx;
    magnetom_y = imu.my;
    magnetom_x = ((magnetom_x - mx_bias) * 100) / mx_ge; // measurements corrected by bias and scale gain
    magnetom_y = ((magnetom_y - my_bias) * 100) / my_ge; // 100 means the scale goes between -100 and 100
  
    //Compute the heading
    actual_heading = atan2(-magnetom_y, -magnetom_x);
  
    //Magnetic declination
    //Declination at ITA = -22o21'
    declinationAngle = -(22 * PI / 180 + (21 / 60) * PI / 180);
    actual_heading += (declinationAngle);
    actual_heading += offset_value*(PI/180); //This is the read by the magnetometer when the quadrotor is aligned with the inertial X axis, angle betwen the north and test area X axis
  
    // Range correction ( -180 to 180 )
    if (actual_heading > PI) actual_heading -= (2 * PI);
    else if (actual_heading < -PI) actual_heading += (2 * PI);
  
    //Convertion to degrees
    actual_heading *= (-180 / PI); 
  }
}

void findCompassOffsetValue()
{
  
  float temp_offset_value=0;
  int iter;
  for (iter = 0; iter < N_CALIB_DATA; iter++)
  {
    readCOMP(); //read the compass measurment
    temp_offset_value += actual_heading / N_CALIB_DATA; //calculating the mean value
//    Serial.print(temp_offset_value); Serial.print(" "); Serial.println(actual_heading);
    delay(20);
  }
  offset_value=temp_offset_value;
  readCOMP();
//  Serial.println(actual_heading);
}


void readIMU()
{
  if(imu.accelAvailable())
  {
    imu.readAccel();
    accel_I[0]=imu.calcAccel(imu.ax)*9.8066 - OFFSET_ACCEL[0];
    accel_I[1]=-imu.calcAccel(imu.ay)*9.8066 - OFFSET_ACCEL[1];
    accel_I[2]=imu.calcAccel(imu.az)*9.8066 - OFFSET_ACCEL[2];
  }
  readCOMP();
  
//  Serial.print(accel_I[0],4); Serial.print(" ");
//  Serial.print(accel_I[1],4); Serial.print(" ");
//  Serial.print(accel_I[2],4); Serial.print(" ");
//  Serial.print(actual_heading,4); Serial.println(" ");
}

void Calibration_Accel(){ // Função utilizada para calibrar o aceletrometro e o girometro
  unsigned int iter=0;
  
    if(imu.accelAvailable())
  {
    imu.readAccel();
    accel_I[0]=imu.calcAccel(imu.ax)*9.8066;
    accel_I[1]=-imu.calcAccel(imu.ay)*9.8066;
    accel_I[2]=imu.calcAccel(imu.az)*9.8066;
  }
  delay(20); // Gera um atraso

  for(iter=0; iter<N_CALIB_DATA; iter++)
  {
        unsigned int iter=0;
        if(imu.accelAvailable())
        {
        imu.readAccel();
        accel_I[0]=imu.calcAccel(imu.ax)*9.8066;
        accel_I[1]=-imu.calcAccel(imu.ay)*9.8066;
        accel_I[2]=imu.calcAccel(imu.az)*9.8066;
        }
        for(int y=0; y<3; y++)  // Laço usado para armazenar os valores de offset em função da leitura dos sensores
          OFFSET_ACCEL[y] += (accel_I[y]/N_CALIB_DATA); // Atualiza o vetor de offset

        //Para manter a frequencia de leitura de accel e gyro em 50Hz, o delay total tem que ser 20ms
        digitalWrite(offBoardLED,HIGH); // Manda o pino 13 para o nivel alto.
        delay(10); // Gera uma atraso
        digitalWrite(offBoardLED,LOW); // Manda o pino 13 para o nivel baixo.
        delay(10); // Gera uma atraso
//        Serial.print(OFFSET_ACCEL[0]); Serial.print(" ");
//        Serial.print(OFFSET_ACCEL[1]); Serial.print(" ");
//        Serial.print(OFFSET_ACCEL[2]); Serial.println();
  }
}
