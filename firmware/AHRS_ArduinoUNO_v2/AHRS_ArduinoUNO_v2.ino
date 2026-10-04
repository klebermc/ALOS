//***************************************************************************
//                 Instituto Tecnológico de Aeronáutica (ITA) 
//                  Divisão de Engenharia Eletrônica (IEE) 
//                   Departamento de Eletrônica Aplicada
//                  EA-291 Pilotos Automáticos para VANTS
//                          2º Semestre de 2010.
//                 Autor: Sérgio Ronaldo Barros dos Santos.
//                    V2: Kléber Macedo Cabral
//***************************************************************************
//***************************************************************************
//
// Este programa é usada para obter as leituras para o girômetro, a partir desta 
// leituras é determinada a matriz de cosseno diretores, sendo esta posteriormente
// corrigidas através do processo de renormalização e através de um controlador PI, 
// que corrige os erros de medida do girômetro. Utilizando a matriz de cosseno 
// diretores normalizada é calculado os ângulo de euller referente a atitude.  
//
// Versao 2: 
// Acrescimo da calibração do magnetômetro
// Aumento do número de dados para a calibração do acelerômetro e girometro
//
// Característica:
//
// Medir os 3 graus de libertdade e a orientação da plataforma
//
// Padronização dos sistema de referência do corpo:
//
// Eixo X apontando na direção do FTDI
// Eixo Y apontando para a direta em relação ao eixo X
// Eixo Z apontando para baixo
//
// Hardware Utilizado:
//
// Arduino UNO com ATMega328 e clock 16MHz
// IMU: Sparkfun LSM9DS1, comunicação I2C
//
// Software Utilizado:
//
// Arduino IDE
// Modo de compilação depende do hardware utilizado
// Modo configurado: Arduino UNO w/ATMega328
//
//****************************************************************************

#include <SparkFunLSM9DS1.h>
///////////////////////
// Example I2C Setup //
///////////////////////
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

#define     GRAVITY          9.8066   // Valor de referencia para z, ADXL345 Sensitivity(from datasheet) => 4mg/LSB => 1G = 256 

#define     ToDeg(x)        (x*57.2957795131)  // *180/pi
#define     Gyro_Gain_X      0.92  // Valor de sensibilidade escolhido para x    // Sensibilidade de LPR530 & LY530 Sensitivity (do datasheet) => 0.83 para 3V/25graus
#define     Gyro_Gain_Y      0.92  // Valor de sensibilidade escolhido para y
#define     Gyro_Gain_Z      0.92  // Valor de sensibilidade escolhido para z

#define     ToRad(x)              (x*0.01745329252)      // Usado para converte de graus/radiano -  pi/180 = 0.0174529252
#define     Gyro_Scaled_X(x)       x*ToRad(Gyro_Gain_X)  // Define a multiplicação entre o valor de x do giro, a constante de conversão(degree to rad) e o ganho de sensibilidade x 
#define     Gyro_Scaled_Y(x)       x*ToRad(Gyro_Gain_Y)  // Define a multiplicação entre o valor de y do giro, a constante de conversão(degree to rad) e o ganho de sensibilidade y
#define     Gyro_Scaled_Z(x)       x*ToRad(Gyro_Gain_Z)  // Define a multiplicação entre o valor de z do giro, a constante de conversão(degree to rad) e o ganho de sensibilidade z

#define     ToAccelSI(x)      ((x*9.80665)/GRAVITY) //Converte o valor entre range de leitura do aceleremotro para unidades do SI [m/s^2]

#define     Kp_ROLLPITCH      0.02*10     // Define valor de Kp para correção de pitch e roll 
#define     Ki_ROLLPITCH      0.00002*10   // Define valor de Ki para coreeção de pitch e roll
#define     Kp_YAW            1.2/2  // Define valor de Kp para a correção de pitch e roll
#define     Ki_YAW            0.00002/2  // Define valor de Ki para a correção de pitch e roll

#define     PRINT_MATLAB      0  // Habilita o envio dos dados via serial para o MATLAB  
#define     PRINT_PYTHON      1   // Habilita o envio dos dados via serial para a interface gráfica
#define     STATUS_LED        13  // Define o pino de saida utilizado pelo LED

#define     N_CALIB_DATA      200  //Esse é o numero de dados que são capturados para fazer a calibração dos eixos

#define       numReadings   5          // size of the array for filtering values

LSM9DS1 imu; //IMU object

float Angle_Xinertial2MagNorth=0; //Esse é o valor em graus lido pelo magnetometro quando seu X está alinhado com o X do sistema inercial

int8_t  sensors[3] = {1,2,0};   // Declaração dos canais do módulo ADC utilizados pelo girômetro na seguinte ordem, sensor [0] = ADC 1, sensor [1] = ADC 2 e sensor [2] =ADC 0.

int SENSOR_SIGN[9] = {1,-1,1, 1,-1,1, -1,-1,-1};  // Valores para padronização dos eixos de cada sensor da IMU

float G_Dt=0.02;    // Constante de 20m utilizada para determinar a matriz de rotação
long timer=0;  // Seta variavel para armazenamento do tempo de simulação do algoritmo
long timer_old; // Armazena o tempo de simulação anterior

float AN[6]; // Armazena os valores com os offset para o giro e o acelerômetro em X, Y e Z 
float AN_OFFSET[6]={0,0,0,0,0,0}; // Vetor utilizado para armazenar os valores de offset calculados
float ACC[3];   // Vetor utilizado para armazenar valores lidos pelo acelerômetro com offset

float accel_b[3] = {0,0,0}; // accel vector at the body reference frame
float gyro_b[3] = {0,0,0}; // gyro vector at the body reference frame
float magnetom_x=0, magnetom_y=0, magnetom_z=0;
int   mx_ge = 1, my_ge = 1, mz_ge=1;
int   mx_bias=0, my_bias=0, mz_bias=0;// Bias dos eixos do magnetometro
float MAG_Heading;

float Accel_Vetor[3]= {0,0,0}; //Armazena os valores de aceleração corrigidos (sem offset)
float Gyro_Vetor[3]= {0,0,0}; // Armazena a taxa da velocidade angular em radiano/segundo corrigido (sem offset)

float Medida_giro_corrigido[3]= {0,0,0}; // Armazena o vetor com as medidas do girômetro corrigida pelo vetor de ajuste
float Vetor_ajuste_P[3]= {0,0,0}; // Vetor de ajuste (parte proporcional) utilizado para correção do erro de medida acumulado do girômetro
float Vetor_ajuste_I[3]= {0,0,0}; // Vetor de ajuste (parte integral) utilizado para correção do erro de medida acumulado do girômetro
float Vetor_temp[3]= {0,0,0}; // Vetor utilizado para efetuar as operaçoes 

float roll = 0; // Angulo de rolagem inicial
float pitch = 0; // Angulo de arfagem inicial
float yaw = 0;  // Angulo de heading inicial

float FCA[3]= {0,0,0}; // Vetor de erro obtido a partir do acelerômetro 
float FCM[3]= {0,0,0}; // Vetor de erro obtido a partir do magnetômetro 

unsigned int counter=0; // Variavel de contagem usada para controle da leitura do magnetometro

byte gyro_sat=0;

float Matrix_rotacao[3][3]= {{ 1,0,0 },{ 0,1,0 },{ 0,0,1 }}; // Matriz de rotação assumida inicialmente 
float Matrix_atualizacao[3][3]={{0,1,2},{3,4,5},{6,7,8}}; // Matriz auxiliar usada para determinação da Matriz de rotação
float Matrix_Temporaria[3][3]={{ 0,0,0 },{ 0,0,0 },{ 0,0,0 }}; // Armazenar temporarialmente os dados calculados para obtenção da Matriz de rotação total 
 
volatile uint8_t MuxSel=0; // Configura a variavel de seleção, para contagem do numero de incrementos do filtro do giro
volatile uint8_t analog_reference; // Variavel usada para armazena o modo de referencia do ADC 
volatile uint16_t analog_buffer[8]; // Variavel utilizada para armazenar a somatoria dos valores amostrados para x,y e z
volatile uint8_t analog_count[8]; // Variavel usada para armazena o numero de somatórias feitas para os valores amostrados x,y e z

float roll_output;  // Variavel usada para armazenar o valor de roll convertido de radiano para graus
float pitch_output; // Variavel usada para armazenar o valor de pitch convertido de radiano para graus
float yaw_output;  // Variavel usada para armazenar o valor de yaw convertido de radiano para graus
char buffer_pitch[4]; 
char buffer_roll[4];
char buffer_yaw[4];

byte counter_print_output=0;

// --------------- Variables used to filter data from IMU ----------------------
float history_accelx[numReadings] = {0, 0, 0, 0, 0};  // history of the x accel value, used to filter the value every 100ms
float history_accely[numReadings] = {0, 0, 0, 0, 0};  // history of the y accel value, used to filter the value every 100ms
float history_accelz[numReadings] = {0, 0, 0, 0, 0};  // history of the z accel value, used to filter the value every 100ms
float history_accelwz[numReadings] = {0, 0, 0, 0, 0}; // history of the wz gyro value, used to filter the value every 100ms

byte  posVectAccelx = 0;            // The position of the last stored accel value at the history vector
byte  posVectAccely = 0;            // The position of the last stored accel value at the history vector
byte  posVectAccelz = 0;            // The position of the last stored accel value at the history vector
byte  posVectAccelwz = 0;           // The position of the last stored gyro value at the history vector

float history_Roll[numReadings] = {0, 0, 0, 0, 0};    // history of the heading angle, used to filter the value every 100ms
float history_Pitch[numReadings] = {0, 0, 0, 0, 0};   // history of the heading angle, used to filter the value every 100ms
float history_Yaw[numReadings] = {0, 0, 0, 0, 0}; // history of the heading angle, used to filter the value every 100ms

byte  posVectRoll = 0;    // The position of the last stored roll value at the history vector
byte  posVectPitch = 0;   // The position of the last stored pitch value at the history vector
byte  posVectHD = 0;      // The position of the last stored heading value at the history vector

//filtered values are always in [degrees]
float filtered_Roll = 0;        // The angle of roll after the median filtration
float filtered_Pitch = 0;       // The angle of pitch after the median filtration
float filtered_Yaw = 0;         // The angle of heading after the median filtration
// -----------------------------------------------------------------------------

void setup()
{ 
  Serial.begin(57600); // Habilita e configura a comunicação serial
  pinMode (STATUS_LED,OUTPUT);  // Configura o pino 13 como saida 
  digitalWrite(STATUS_LED,LOW); // Manda o pino 13 para nivel baixo
 
  setupIMU();
  
  Calibration_Accel_Gyro(); //Calibração dos eixos do acelerômetro

  Serial.print("C");
  Calibration_Compass(); // Calibração do magnetômetro
  Serial.print("c");
 
//  //Valores já calibrados para o magnetometro
//  mx_bias=1153; mx_ge=1132;
//  my_bias=641;  my_ge=1013;
//  mz_bias=403;  mz_ge=1079;

  delay(5000);
  findCompassOffsetValue(); // Angulo entre o Eixo X inercial e o sul magnético (próximo ao norte geográfico)

  digitalWrite(STATUS_LED,HIGH); // Manda o pino 13 para o nivel alto.
  timer = millis(); // Retorna o tempo (em milisegundo) deste o instante que o programa começou a rodar.
  delay(20); // Gera um atraso
}

void loop() //Loop principal
{
  if((millis()-timer)>=20)  // Esta rotina (loop principal) é executada a cada 20ms(50Hz).
  {
    timer_old = timer; // Armazena o valor de tempo anterior
    timer=millis(); // Ler o novo valor de tempo de execução do programa 
    
    if (timer>timer_old) // Verifica se o tempo atual lido e maior que o tempo anteriormente
      G_Dt = (timer-timer_old)/1000.0;    // Real time of loop run. We use this on the Matriz_rotações algorithm calculation.
    else
      G_Dt = 0; // Senão G_Dt será igual a zero.
        
    Read_Gyro();   // Ler os dados do giro x, y e z.
    Read_Accel();     // Ler os dados do acelerometro
    
    Read_Compass();    // Efetua a leitura dos dados do magnetometro (x,y e z)
    Compass_Heading();  // chama a função para calcular o ângulo de heading (Azimute) medido em relação ao Norte magnetico       

    Matrix_cosseno_diretores(); // Chama a função para determinar a matriz de rotação
    Normalizacao();     //  Chama a função para determinar a matriz de rotação normalizada
    Modelagem_correcao();  // Chama a função para determina o erro númerico acumulado na matriz de rotação e calculo do vetor de ajuste (correção).
    Euler_angles();   //  Chama a função para calcular os ângulos de Euller (Atitude)
    printdata(); //  Chama a função para transmissão dos dados via serial.
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
//                     Verifica se houve saturação em um dos eixos do girometro
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
   if((abs(Gyro_Vetor[0])>=ToRad(300))||(abs(Gyro_Vetor[1])>=ToRad(300))||(abs(Gyro_Vetor[2])>=ToRad(300))) 
       {
          if (gyro_sat<50) // Incrementa a variavel que indica a saturação
              gyro_sat+=10;
       }
    
    else
        {
           if (gyro_sat>0) // Zera a variavel que indica a saturação
             gyro_sat--;
        }
  
    if (gyro_sat>0)
      digitalWrite(STATUS_LED,LOW); // Caso houve a saturação o LED conectado ao pino 13 é desligado  
    else
      digitalWrite(STATUS_LED,HIGH);  // Senão o LED se mantem acesso
  
  }
  
}
