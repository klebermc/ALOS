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

void readIMU()
{
  if(imu.accelAvailable())
  {
    imu.readAccel();
    accel_b[0]=imu.calcAccel(imu.ax)*9.8066 - OFFSET_ACCEL[0];
    accel_b[1]=imu.calcAccel(imu.ay)*9.8066 - OFFSET_ACCEL[1];
    accel_b[2]=imu.calcAccel(imu.az)*9.8066 - OFFSET_ACCEL[2];
  }
  
  readCOMP();
  
  //Finds roll, pitch, yaw
  //computeAttitude();

  //read_gyro_to_euler_ang();
  
  //Computes DCM
  //DCMcalculation();

  //Multiply_Rb2I_BodyAccel();

  accel_I[0]=   accel_b[0];
  accel_I[1]=-1*accel_b[1];
  accel_I[2]=   accel_b[2];
  
//  Serial.print(accel_b[0],4); 
//  Serial.print(" ");
//  Serial.print(accel_b[1],4); 
//  Serial.print(" ");
//  Serial.print(accel_b[2],4);
//  Serial.print(" ");
//  Serial.print(accel_I[0],4); 
//  Serial.print(" ");
//  Serial.print(accel_I[1],4); 
//  Serial.print(" ");
//  Serial.print(accel_I[2],4); 
//  Serial.println(" ");
  //Serial.print(actual_heading,4); Serial.println(" ");
}

void findCompassOffsetValue()
{
  
  float temp_offset_value=0;
  int iter;
  for (iter = 0; iter < N_CALIB_DATA; iter++)
  {
    readCOMP(); //read the compass measurment
    temp_offset_value += actual_heading / N_CALIB_DATA; //calculating the mean value
    digitalWrite(offBoardLED, !digitalRead(offBoardLED));
    delay(20);
//    Serial.println(temp_offset_value);
  }
  offset_value=temp_offset_value;
  readCOMP();
//  Serial.println(offset_value);
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

void Calibration_Accel(){ // Função utilizada para calibrar o aceletrometro e o girometro
  unsigned int iter=0;
  for(iter=0; iter<N_CALIB_DATA; iter++)
  {
        countTimeOut=0; //avoiding timeouts
        if(imu.accelAvailable())
        {
        imu.readAccel();
        accel_b[0]=imu.calcAccel(imu.ax)*9.8066;
        accel_b[1]=imu.calcAccel(imu.ay)*9.8066;
        accel_b[2]=imu.calcAccel(imu.az)*9.8066;
        }
        for(int y=0; y<3; y++)  // Laço usado para armazenar os valores de offset em função da leitura dos sensores
          OFFSET_ACCEL[y] += (accel_b[y]/N_CALIB_DATA); // Atualiza o vetor de offset
       
        //Para manter a frequencia de leitura de accel e gyro em 50Hz, o delay total tem que ser 20ms
        digitalWrite(offBoardLED, !digitalRead(offBoardLED));
        delay(20);
//        Serial.print(OFFSET_ACCEL[0]); Serial.print(" ");
//        Serial.print(OFFSET_ACCEL[1]); Serial.print(" ");
//        Serial.print(OFFSET_ACCEL[2]); Serial.println();
  }
  OFFSET_ACCEL[2] = OFFSET_ACCEL[2]-9.8066; //finds the compansation value to reach gravity measurement
}

void computeAttitude()
{
  roll = atan2(accel_b[1], accel_b[2]);
  pitch = atan2(-accel_b[0], sqrt(accel_b[1] * accel_b[1] + accel_b[2] * accel_b[2]));
  
  readCOMP();
  
  Serial.print("Pitch:");  Serial.print(pitch * 180.0 / PI, 2);
  Serial.print(" Roll:");  Serial.print(roll * 180.0 / PI, 2);
  Serial.print(" Heading: "); Serial.println(actual_heading, 2);
}


void Multiply_Rb2I_BodyAccel()
{
  float op[3]; // Declara o vetor auxiliar para somatória
  for(int x=0; x<3; x++) // Laço define o numero da linha da matriz (a), o incremento altera a linha 
  {
    for(int y=0; y<3; y++) // Laço define o numero de linha do vetor (accel_b), o incremento altera a linha
    {
      op[y]=Rb2I[x][y]*accel_b[y]; // O valor da multiplicação dos elementos da linha e coluna e armazenado no vetor op.
                              // op[0]= a[0][0]*accel_b[0] - op[1]=a[0][1]*accel_b[1] - op[2]=a[0][2]*accel_b[2]
    }
    accel_I[x]=op[0]+op[1]+op[2]; // Soma os valores das multiplicações
  }
}

void DCMcalculation()
{
  yaw = actual_heading * PI/ 180.0;

  cr=(float)cos(roll);
  cp=(float)cos(pitch);
  cy=(float)cos(yaw);
  sr=(float)sin(roll);
  sp=(float)sin(pitch);
  sy=(float)sin(yaw);
  
  Rb2I[0][0] = cy*cp;
  Rb2I[0][1] = (cy*sp*sr-sy*cr);
  Rb2I[0][2] = (cy*sp*cr+sy*sr);
  Rb2I[1][0] = sy*cp;
  Rb2I[1][1] = (sy*sp*sr+cy*cr);
  Rb2I[1][2] = (sy*sp*cr-cy*sr);
  Rb2I[2][0] = -sp;
  Rb2I[2][1] = cp*sr;
  Rb2I[2][2] = cp*cr;
}

void Calibration_Gyro(){ // Função utilizada para calibrar o aceletrometro e o girometro
  unsigned int iter=0;
  for(iter=0; iter<N_CALIB_DATA; iter++)
  {
        countTimeOut=0; //avoiding timeouts
        if(imu.gyroAvailable())
        {
        imu.readGyro();
        gyro_b[0]=imu.calcGyro(imu.gx);
        gyro_b[1]=imu.calcGyro(imu.gy);
        gyro_b[2]=imu.calcGyro(imu.gz);
        }
        for(int y=0; y<3; y++)  // Laço usado para armazenar os valores de offset em função da leitura dos sensores
          OFFSET_GYRO[y] += (gyro_b[y]/N_CALIB_DATA); // Atualiza o vetor de offset
       
        //Para manter a frequencia de leitura de accel e gyro em 50Hz, o delay total tem que ser 20ms
        digitalWrite(offBoardLED, !digitalRead(offBoardLED));
        delay(20);
        Serial.print(OFFSET_GYRO[0]); Serial.print(" ");
        Serial.print(OFFSET_GYRO[1]); Serial.print(" ");
        Serial.print(OFFSET_GYRO[2]); Serial.println();
  }
}
