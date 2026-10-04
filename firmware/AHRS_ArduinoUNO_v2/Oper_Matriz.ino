//***********************************************************************
//      Este algoritmo é utilizado para efetuar a multiplicação de duas
//      matriz [3][3], resultando um nova matriz [3][3] 
//***********************************************************************
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//     Multiplicação das matrizes de entrada (a) e (b),o resulatado e
//     retornado na matriz de saida (mat)
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void Multiplica_Matriz(float a[3][3], float b[3][3],float mat[3][3])
{
  float op[3]; // Declara o vetor auxiliar para somatória
  for(int x=0; x<3; x++) // Laço define o numero da linha da matriz (a), o incremento altera a linha 
  {
    for(int y=0; y<3; y++) // Laço define o numero de coluna da matriz (b), o incremento altera a coluna 
    {
      for(int w=0; w<3; w++) // Laço usado para efetuar a multiplicação entre os elementos da linha de (a) e coluna de (b)
      {
         op[w]=a[x][w]*b[w][y]; // O valor da multiplicação dos elementos da linha e coluna e armazenado no vetor op.
                              // op[0]= a[0][0]*b[0][0] - op[1]=a[0][1]*b[1][0] - op[2]=a[0][2]*b[2][0]
      } 
     mat[x][y]=0;  // Zera a posição da matriz de saida
     mat[x][y]=op[0]+op[1]+op[2]; // Soma os valores das multiplicações
     }
  }
}


void Multiplica_Matriz_Vetor(float a[3][3], float b[3],float c[3])
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
