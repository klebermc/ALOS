 void read_gyro_to_euler_ang()
 {
      if ( imu.gyroAvailable() )
  {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu.readGyro();
    gyro_b[0] = imu.calcGyro(imu.gx) - OFFSET_GYRO[0];
    gyro_b[1] = imu.calcGyro(imu.gy) - OFFSET_GYRO[1];
    gyro_b[2] = imu.calcGyro(imu.gz) - OFFSET_GYRO[2];
  }

  R_pqr2deltaEuler[0][0] = 1;
  R_pqr2deltaEuler[0][1] = sin(roll)*tan(pitch);
  R_pqr2deltaEuler[0][2] = cos(roll)*tan(pitch);
  R_pqr2deltaEuler[1][0] = 0;
  R_pqr2deltaEuler[1][1] = cos(roll);
  R_pqr2deltaEuler[1][2] = -sin(roll);
  R_pqr2deltaEuler[2][0] = 0;
  R_pqr2deltaEuler[2][1] = (sin(roll)/cos(pitch));
  R_pqr2deltaEuler[2][2] = (cos(roll)/cos(pitch));
  
  Multiply_Matrix_Vector(R_pqr2deltaEuler, gyro_b , delta_ang_Euler );

  roll  = roll + delta_ang_Euler[0]*0.02;
  pitch  = pitch + delta_ang_Euler[1]*0.02;
  yaw  = yaw + delta_ang_Euler[2]*0.02;

    Serial.print("Pitch:");  Serial.print(pitch * 180.0 / PI, 2);
  Serial.print(" Roll:");  Serial.print(roll * 180.0 / PI, 2);
  Serial.print(" Heading: "); Serial.println(yaw* 180.0 / PI, 2);
 }


void Multiply_Matrix_Vector(float a[3][3], float b[3],float c[3])
{
  float op[3]; // Declara o vetor auxiliar para somatória
  for(int x=0; x<3; x++) // Laço define o numero da linha da matriz (a), o incremento altera a linha 
  {
    for(int y=0; y<3; y++) // Laço define o numero de linha do vetor (b), o incremento altera a linha
    {
      op[y]=a[x][y]*b[y]; // O valor da multiplicação dos elementos da linha e coluna e armazenado no vetor op.
                              // op[0]= a[0][0]*b[0] - op[1]=a[0][1]*b[1] - op[2]=a[0][2]*b[2]
    }
    c[x]=0;  // Zera a posição da matriz de saida
    c[x]=op[0]+op[1]+op[2]; // Soma os valores das multiplicações
  }
}
