//***************************************************************************************************
//        Este algoritmo é utilizado com a função de diminuir os erros numericos presente
//        na matriz de rotação. Aplicando a cada um dos vetores da matriz de rotação a condição
//        de ortogonalidade e normalização (magnitude igual a 1).
//****************************************************************************************************

void Normalizacao(void)
{
  float error=0;
  float temporario[3][3];
  float renorm=0;
  
  error= -Prod_escalar(&Matrix_rotacao[0][0],&Matrix_rotacao[1][0])*.5; // Calcula o erro de ortogonalidade da matriz entre os vetores X e Y. 

  Mult_vetor(&temporario[0][0], &Matrix_rotacao[1][0], error); // Multiplica a constante erro pelo vetor Y da matriz de rotação
  Mult_vetor(&temporario[1][0], &Matrix_rotacao[0][0], error); // Multiplica a constante erro pelo vetor X da matriz de rotação
  
  Soma_Vetor(&temporario[0][0], &temporario[0][0], &Matrix_rotacao[0][0]); // Efetua a soma entre o vetor (temporario [0] [0]) com o vetor X da matriz de rotação
  Soma_Vetor(&temporario[1][0], &temporario[1][0], &Matrix_rotacao[1][0]); // Efetua a soma entre o vetor (temporario [1] [0]) com o vetor Y da matriz de rotação
  
  Prod_vetorial(&temporario[2][0],&temporario[0][0],&temporario[1][0]); // Calcula um vetor que seja ortogonal a Y e X.
  
  renorm= .5 *(3 - Prod_escalar(&temporario[0][0],&temporario[0][0])); // Calcula a constante de normalização para o vetor X da (temporario [0][0]) 
  Mult_vetor(&Matrix_rotacao[0][0], &temporario[0][0], renorm); // Multiplica a constante pela matriz (temporario [0][0]) obtendo o vetor X normalizado.
  
  renorm= .5 *(3 - Prod_escalar(&temporario[1][0],&temporario[1][0])); // Calcula a constante de normalização para o vetor Y da (temporario [1][0])
  Mult_vetor(&Matrix_rotacao[1][0], &temporario[1][0], renorm);  // Multiplica a constante pela matriz (temporario [0][0]) obtendo o vetor Y normalizado.
  
  renorm= .5 *(3 - Prod_escalar(&temporario[2][0],&temporario[2][0])); // Calcula a constante de normalização para o vetor Z da (temporario [2][0])
  Mult_vetor(&Matrix_rotacao[2][0], &temporario[2][0], renorm);  // Multiplica a constante pela matriz (temporario [0][0]) obtendo o vetor Z normalizado.
}

