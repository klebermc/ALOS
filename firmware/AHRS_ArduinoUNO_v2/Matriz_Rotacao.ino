//************************************************************************************************
//            Este algoritmo é utilizado para determinar a matriz de rotação e
//            Corrigir as medidas do girômetro usando o vetor de ajuste obtido
//            a partir do controlador PI.
//**************************************************************************************************

void Matrix_cosseno_diretores(void)
{
  Gyro_Vetor[0]= ToRad(gyro_b[0]); // Converte de graus/segundo para radiano/segundo, obtendo o valor x (roll)
  Gyro_Vetor[1]= ToRad(gyro_b[1]); // Converte de graus/segundo para radiano/segundo, obtendo o valor y (pitch)
  Gyro_Vetor[2]= ToRad(gyro_b[2]); // Converte de graus/segundo para radiano/segundo, obtendo o valor z (yaw)
  
 //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
 // Correções dos erros de medida presente nos valores lidos pelo girômetro para os 3 eixos.  
 //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  Soma_Vetor(&Vetor_temp[0], &Gyro_Vetor[0], &Vetor_ajuste_I[0]);  // Soma ao vetor que contém as medidas feitas pelo girômetro(Gyro_vetor)a parte integral integral 
                                                                   // do vetor de ajuste (Vetor_ajuste_I)e armazena o resultado em (Vetor_temp)
  Soma_Vetor(&Medida_giro_corrigido[0], &Vetor_temp[0], &Vetor_ajuste_P[0]); // soma ao vetor temporario (Vetor_temp) com a parte proporcional do vetor ajuste , obtendo 
                                                                          // o vetor corrigido (Med_giro_corrigido).

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//  Determinação da matriz de rotação.
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 // Construção da matriz beseada na aproximação adotada para a obtenção da matriz de rotação
  Matrix_atualizacao[0][0]=0;
  Matrix_atualizacao[0][1]=-G_Dt*Medida_giro_corrigido[2]; // Multiplica a constante de 20us pela leitura corrigida do girômetro para o eixo Z (-z)
  Matrix_atualizacao[0][2]=G_Dt*Medida_giro_corrigido[1]; // Multiplica a constante de 20us pela leitura corrigida do girômetro para o eixo Y (Y)
  Matrix_atualizacao[1][0]=G_Dt*Medida_giro_corrigido[2]; // Multiplica a constante de 20us pela leitura corrigida do girômetro para o eixo Z (Z)
  Matrix_atualizacao[1][1]=0;
  Matrix_atualizacao[1][2]=-G_Dt*Medida_giro_corrigido[0]; // Multiplica a constante de 20us pela leitura corrigida do girômetro para o eixo X (X)
  Matrix_atualizacao[2][0]=-G_Dt*Medida_giro_corrigido[1]; // Multiplica a constante de 20us pela leitura corrigida do girômetro para o eixo Y (Y)
  Matrix_atualizacao[2][1]=G_Dt*Medida_giro_corrigido[0]; // Multiplica a constante de 20us pela leitura corrigida do girômetro para o eixo X (X)
  Matrix_atualizacao[2][2]=0;
  
  
  Multiplica_Matriz(Matrix_rotacao,Matrix_atualizacao,Matrix_Temporaria); // Multiplica a matriz aproximada (Matrix_atualizacao) pela matriz de rotação atual (matriz tipo identidade)  

  for(int x=0; x<3; x++)  // Laço usado para incremetar a linha das matrizes que serão somadas
  {
    for(int y=0; y<3; y++) // Laço usado para incremetar a coluna das matrizes que serão somadas
    {
      Matrix_rotacao[x][y]+=Matrix_Temporaria[x][y]; // Efetua a somatória da matirz temporaria com o valor atual da matriz de rotação, gerando um nova matriz de rotação.
    } 
  }
}



