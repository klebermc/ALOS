//***********************************************************************
//   Este algoritmo é utilizado para efetuar operações com vetores
//   (Soma, multiplicação por uma constante, produto escalar e vetorial) 
//***********************************************************************

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//       Produto escalar ente dois vetores, retorna um numero decimal 
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

float Prod_escalar(float vetor1[3],float vetor2[3])  
{
  float mult_vetor=0;  //  Variavel usada para armazena o resultado
  
  for(int c=0; c<3; c++)
   {
      mult_vetor +=vetor1[c]*vetor2[c]; // Efetua a multiplicação entre os vetores
   }
  return mult_vetor; 
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//          Produto Vetorial entre dois vetores. Os vetores de entrada
//          usados são V1 e V2 e o reultado (novo vetor) e retornado
//          em vetorOut.  
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void Prod_vetorial(float vetorOut[3], float v1[3],float v2[3])
 {
    vetorOut[0]= (v1[1]*v2[2]) - (v1[2]*v2[1]); 
    vetorOut[1]= (v1[2]*v2[0]) - (v1[0]*v2[2]);
    vetorOut[2]= (v1[0]*v2[1]) - (v1[1]*v2[0]);
 }

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//   Multiplicação de um vetor por uma constante. O vetor de entrada (vetorIn)
//   e a constante (dado) são multiplicados retornando uma vetor de saida (vetorOut)
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void Mult_vetor(float vetorOut[3],float vetorIn[3], float dado) 
{
  for(int c=0; c<3; c++)
   {
     vetorOut[c]=vetorIn[c]*dado; 
   }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//   Soma de dois vetores. Os vetores de entrada (vetorIn1) e (vetorIn2) são somados
//   retornando o novo vetor (vetorOut). 
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void Soma_Vetor(float vetorOut[3],float vetorIn1[3], float vetorIn2[3]) 
{
  for(int c=0; c<3; c++)
   {
       vetorOut[c]=vetorIn1[c]+vetorIn2[c]; 
   }
}



