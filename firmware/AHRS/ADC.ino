//***********************************************************************
//      Este programa utiliza os ADC 1, 2 e 3 do microcontrolador
//      AVR- ATmega328 com resolução 10 bits para a conversão
//      valores analogicos fornecidos pelo giro LPR530Al e ALH
//***********************************************************************

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//            Recebe e Armazena os valores amostrados para X, Y e Z
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void Read_adc_raw(void)   // Após interrupção gerada ao decorrer da execução do programa, são lidos as taxas de velocidades angulares em x,y e z.
                           
{                         
  int i;
  uint16_t temp1;    
  uint8_t temp2;    
  
  for (i=0;i<3;i++)  // Laço para seleção do ADC, leituras de X=0, Y=1, Z=2. 
                     // Se mantem no laço ate que os 3 eixos sejam amostrados.
   {
      do // executa o "do" enquanto temp1 e analog_buffer[sensors[i] forem diferentes.
        {
          temp1= analog_buffer[sensors[i]];   // Armazena na variavel a somária do valor amostrado           
          temp2= analog_count[sensors[i]];    // Armazena na variavel o número de vezes que a amostra foi somada
         } 
      while(temp1 != analog_buffer[sensors[i]]);  // Verifica se o valor de temp1 é igual a analog_buffer para sair do laço.  
                                                  // São atualizados com os dados lidos da interrupção. 
      if (temp2>0) 
         AN[i] = (float)temp1/(float)temp2; // Obtem-se o valor amostrado e filtrado, efetuando a média aritmetica 
    }
    
  
  for (int i=0;i<3;i++) // Apos ler os 3 valores para x, y e z, então as variaveis são inicializadas
   {
      do
         {
           analog_buffer[i]=0; // Limpa a variavel de armazenamento dos valores amostrados
           analog_count[i]=0;   // Limpa a variavel de armazenamento da quantidade de somatoria
         } 
      while(analog_buffer[i]!=0);  // Espera ate que a variavel seja zerada para x, y e z
   }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//                    Prepara os dados para a transmissão
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

float read_adc(int select) //Função fornece o valor amostrado e filtrado 
{
  if (SENSOR_SIGN[select]<0)  // Verifica se a conversão de sinal usada para o eixo de referencia adotado é negativo 
    return(AN_OFFSET[select]-AN[select]); // Retorna o valor corrigido. Em outras palvras sem a presença do offset
  else
    return(AN[select]-AN_OFFSET[select]); // Retorna o valor corrigido
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//                            Inicializa ADC
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void Analog_Init(void)  
{
 ADCSRA|=(1<<ADIE)|(1<<ADEN); // Configuração inicial (registrador ADCSRA), flag ADIE = 1 habilita interrupção, flag = 1 habilita ADC.  
 ADCSRA|= (1<<ADSC);  // flag ADSC = 1 inicia leitura dos dados. 
}

void Analog_Reference(uint8_t mode) 
{
  analog_reference = mode; //  Padrão de 3.3v
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//                        Rotina de interrupação
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ISR(ADC_vect) // Rotina de interrupção, chamado quando ocorrer uma interrupção. Iniciada no vetor de interrupção ADC_vect
              // A interrupção ocorre quando a conversão estiver pronta.
{
  volatile uint8_t low, high; 
  
  low = ADCL;   // Ler o byte LSB
  high = ADCH;  // Ler o byte MSB

  if(analog_count[MuxSel]<63) // Inicia o laço de filtragem para os valores de x, y e z  
     { 
        analog_buffer[MuxSel] += (high << 8) | low;   // Acumula os valores do ADC para cada um dos eixos
        analog_count[MuxSel]++; // Armazena a quantidade de valores da somados
     }
  MuxSel++;    // Incrementa a variavel de controle.Quando MuxSel = 0 ler X ,quando 1 ler y e quando 2 ler z.
  MuxSel &= 0x03;   // Limpa a variavel MuxSel, se MuxSel=0x04
  ADMUX = (analog_reference << 6) | MuxSel; // Define a referência interna, neste caso a tensão AVCC do dispositivo, e seleciona o canal ADC
  // start conversion
  ADCSRA|= (1<<ADSC); // Configura o registrador para inicia uma nova leitura
}
