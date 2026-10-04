//**********************************************************************************************
//            Esta função é usada para determinar os ângulos de euller (atitude),
//            em função dos valores obtidos da matriz de rotação normalizada.
//**********************************************************************************************

void Euler_angles(void)
{
  pitch = -asin(Matrix_rotacao[2][0]);
  roll = atan2(Matrix_rotacao[2][1],Matrix_rotacao[2][2]);
  yaw = atan2(Matrix_rotacao[1][0],Matrix_rotacao[0][0]);
  
  roll_output = ToDeg(roll); // Converte o valor de roll em radiano para graus.
  pitch_output = ToDeg(pitch); // Converte o valor de pitch em radiano para graus. 
  yaw_output = ToDeg(yaw); // Converte o valor de yaw em radiano para graus.
}
