//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//                Este algoritmo é usada para modelar os eventuais o erro numericos ocasionados pelo girômetro
//                durante a medição, porém, não corrigidos no processo de renormalização da matriz de rotação.
//                A correção deste é efetuado com o auxilio do magnetômetro e acelerômetro. 
//                           
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void Modelagem_correcao(void)
{
  float mag_heading_x;
  float mag_heading_y;
  float MagFator;
  float Accel_magnitude;
  float Accel_weight;
  static float Vetor_temp_P[3];
  static float Vetor_temp_I[3];
  
  Accel_Vetor[0] = accel_x; // Armazena na posição 0 do vetor a medida de aceleração corrigido (sem offset) lido para X
  Accel_Vetor[1] = accel_y; // Armazena na posição 1 do vetor a medida de aceleração corrigido (sem offset) lido para Y
  Accel_Vetor[2] = accel_z; // Armazena na posição 2 do vetor a medida de aceleração corrigido (sem offset) lido para Z
  
 //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 //                                        Modelagem do erro de medida do girômetro
 //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  Accel_magnitude = sqrt(Accel_Vetor[0]*Accel_Vetor[0] + Accel_Vetor[1]*Accel_Vetor[1] + Accel_Vetor[2]*Accel_Vetor[2]); // Calcula a magnitude (modulo) do vetor de aceleração
  Accel_magnitude = Accel_magnitude / GRAVITY; // Obtem o valor de magnitude com a mesma escala utilizada como referencia o eixo z da aceleração
  
  Accel_weight = constrain(1 - 2*abs(1 - Accel_magnitude),0,1);  // Esta função retorna valores entre 0 e 1, valores > 1 a função constrain retorna 1 e
                                                                 // para valores < 0  retorna 0   

  Prod_vetorial(&FCA[0],&Accel_Vetor[0],&Matrix_rotacao[2][0]); // Calcula o vetor de erro a partir do acelerômetro.
                                                                //  Efetua o produto vetorial entre o vetor Z da matriz de rotação normalizada e o vetor que
                                                                // contem as leituras das aceleraçoes efetuadas no eixo Z. 
  
  mag_heading_x = cos(MAG_Heading); // Calcula a componente magnetica em X em função da orientação.
  mag_heading_y = sin(MAG_Heading); // Calcula a componente magnetica em Y em função da orientação.
  
  MagFator=(Matrix_rotacao[0][0]*mag_heading_y) - (Matrix_rotacao[1][0]*mag_heading_x);  // Calcula o indice de erro de orientação.
  Mult_vetor(FCM,&Matrix_rotacao[2][0],MagFator); // Calcula o vetor de erro a partir do magnetômetro. Em função do indice calculado e do vetor Z da matriz de rotação.
  
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  //                Determinação do vetor de ajuste a partir dos vetores de erro de medida  
  //                 do girômetro e do ganhos ajustados para o controlador PI utilizado.
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
  Mult_vetor(&Vetor_ajuste_P[0],&FCA[0],(Kp_ROLLPITCH*Accel_weight)); // Calcula o vetor de ajuste (parte proporcional) em função do ganho Kp definido e do vetor de erro FCA.
  Mult_vetor(&Vetor_temp_I[0],&FCA[0],(Ki_ROLLPITCH*Accel_weight*G_Dt)); // Calcula o vetor de ajuste (parte integral) em função do ganho Ki definido e do vetor de erro FCA.
  Soma_Vetor(Vetor_ajuste_I,Vetor_ajuste_I,Vetor_temp_I);  // Efetua a soma entre o vetor_temp_I atual é o vetor_ajuste_I anterior.  
 
  Mult_vetor(&Vetor_temp_P[0],&FCM[0],Kp_YAW); // Calcula o vetor de ajuste (parte proporcional) em função do ganho Kp definido e do vetor de erro FCM.
  Soma_Vetor(Vetor_ajuste_P,Vetor_ajuste_P,Vetor_temp_P); // Efetua a soma entre o vetor_temp_P atual é o vetor_ajuste_P anterior.  
  
  Mult_vetor(&Vetor_temp_I[0],&FCM[0],(Ki_YAW*G_Dt)); // Calcula o vetor de ajuste (parte integral) em função do ganho Ki definido e do vetor de erro FCM.
  Soma_Vetor(Vetor_ajuste_I,Vetor_ajuste_I,Vetor_temp_I); // Efetua a soma entre o vetor_temp_I atual é o vetor_ajuste_I anterior.  
}
