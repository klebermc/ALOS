//***********************************************************************
//      Este algoritmo é utilizada para estabelecer a comunicação 
//      entre o acelerometro ADXL345 e o magnetometro HMC5843
//      com o microcontrolador AVR-ATmega328, obtendo assim
//      as leituras nos eixos x, y e z para ambos sensores. 
//***********************************************************************

int AccelAddress = 0x53; // Endereço I2C utilizado pelo acelerometro no barramento
int CompassAddress = 0x1E; // Endereço I2C utilizado pelo magnetometro

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//               Função de inicialização geral para a comunicação I2C
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void I2C_Init()  
{
  Wire.begin(); // Inicia o canal de comunicação I2C
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//        Configura os registradores do ACELEROMETRO para leitura dos dados
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void Accel_Init() //  Função utilizada para configurar o acelerometro (Configura os registradores)
{
  
  Wire.beginTransmission(AccelAddress); // Envia endereço I2C para acessar os registradores do acelerometro
  Wire.write(0x2D);  // Envia endereço para acessar o registrado power register do sensor
  Wire.write(0x08);  // Configura o registrador do acelerometro para ser efetuado as leitura nos 3 eixos
  Wire.endTransmission(); // Finaliza a transmissão de configuração
  delay(5); //  Gera um atraso
  //while(true) {digitalWrite(STATUS_LED,HIGH);delay(100);digitalWrite(STATUS_LED,LOW);delay(100);} 
  
  Wire.beginTransmission(AccelAddress);// Envio endereco I2C para acessar os registradores do acelerometro
  Wire.write(0x31);  // Envia o endereço do registrador data_format
  Wire.write(0x08);  // Configura o registrador para o modo full-Res, resolução de saida com range de 4mg/LSB (fator de escala)
  Wire.endTransmission(); // Finaliza e transmite os dados de configuração
  delay(5); // Finaliza a transmissão de configuração
  
  // O programa principal utiliza os dados do acelerometro a cada 50Hz, assim a saida do acelerometro e ajustada para amostra os dados também nesta frequência.
  Wire.beginTransmission(AccelAddress); // Envio endereco I2C para acessar os registradores do acelerometro
  Wire.write(0x2C);  // Envia o endereço do registrador BW_RATE
  Wire.write(0x09);  // Configura o registrador para 50Hz
  Wire.endTransmission(); // Finaliza e transmite os dados de configuração
  delay(5);
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//    Acessa os registradores de armazenamento dos dados amostrados e efetua a leitura
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void Read_Accel() // Função utilizada para ler x,y e z dos acelerometros
{
  int i = 0;
  byte buff[6];
  
  Wire.beginTransmission(AccelAddress); // Envia endereço I2C para acessar os registradores do acelerometro
  Wire.write(0x32);  // Envia o endereços inicial para acessar os regitradores que contem os valores lidos nos 3 eixos do acelerometro.
                    // Cada registrador contem 8 bits.
  Wire.endTransmission(); // Finaliza a transmissão da configuração
  
  Wire.beginTransmission(AccelAddress); // Envia endereço I2C para acessar os registradores do acelerometro
  Wire.requestFrom(AccelAddress, 6);   // Envia o numero de bytes para leitura a partir do endereço 0x32.
                                       // Serão lidos 6 bytes armazenado no registradores 0x32 até 0x37.  
  
  while(Wire.available())  // Verifica se os dados estão prontos para leitura
  { 
    buff[i] = Wire.read();  // Recebe os bytes enviados pelo acelerometro. Enviado primeiramente o LSB e em seguida o MSB.
    i++;
  }
  Wire.endTransmission(); // Finaliza a leitura dos bytes transmitidos. Retornando o ponteiro do acelerometro para o endereço 0x32
  
  if (i==6)  // Verifica se os 6 byte foram recebidos
    {
    ACC[1] = (((int)buff[1]) << 8) | buff[0];  // Efetua a concatenação dos bytes MSB e LSB recebidos para o eixo y (eixo do sensor x)
    ACC[0] = (((int)buff[3]) << 8) | buff[2];  // Efetua a concatenação dos bytes MSB e LSB recebidos para o eixo x (eixo do sensor y)  
    ACC[2] = (((int)buff[5]) << 8) | buff[4];  // Efetua a concatenação dos bytes MSB e LSB recebidos para o eixo z  
    AN[3] = ACC[0];
    AN[4] = ACC[1];
    AN[5] = ACC[2];
    accel_x = SENSOR_SIGN[3]*(ACC[0]-AN_OFFSET[3]); // Condiciona os valores recebidos de x,y e z para o acelerometro 
    accel_y = SENSOR_SIGN[4]*(ACC[1]-AN_OFFSET[4]); // em função dos sinais de conversão dos eixos e dos erros de offset 
    accel_z = SENSOR_SIGN[5]*(ACC[2]-AN_OFFSET[5]);
/*
    Serial.print("ACC = [");
    Serial.print(ACC[0]);Serial.print(",");
    Serial.print(ACC[1]);Serial.print(",");
    Serial.print(ACC[2]);Serial.print("] ");
    Serial.print("AN = [");
    Serial.print(AN[3]);Serial.print(",");
    Serial.print(AN[4]);Serial.print(",");
    Serial.print(AN[5]);Serial.print("] ");
    Serial.print("AN_OFFSET = [");
    Serial.print(AN_OFFSET[3]);Serial.print(",");
    Serial.print(AN_OFFSET[4]);Serial.print(",");
    Serial.print(AN_OFFSET[5]);Serial.print("] ");
    Serial.print("SENSOR_SIGN = [");
    Serial.print(SENSOR_SIGN[3]);Serial.print(",");
    Serial.print(SENSOR_SIGN[4]);Serial.print(",");
    Serial.print(SENSOR_SIGN[5]);Serial.print("] "); 
    Serial.print("accel = [");
    Serial.print(accel_x);Serial.print(",");
    Serial.print(accel_y);Serial.print(",");
    Serial.print(accel_z);Serial.print("] "); 
    Serial.println();
    */
    }
 }
 //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
 //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 //                     Configura os registradores do MAGNETOMETRO para leitura dos dados
 //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void Compass_Init() //  Função utilizada para configurar o magnetometro (Configura os registradores)
{
  Wire.beginTransmission(CompassAddress); // Envia endereço I2C para acessar os registradores do magentometro
  Wire.write(0x02); // Envia endereço para acessar o registrado mode register
  Wire.write(0x00);   // Configura o registrador para o modo de leitura continua (data output rate default to 10Hz)
  Wire.endTransmission(); // Finaliza a transmissão de configuração
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//         Acessa os registradores de armazenamento dos dados amostrados e efetua a leitura
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void Read_Compass() // Função utilizada para efetuar a leitura dos dados do magnetometro.
{
  int i = 0;
  byte buff[6];
 
  Wire.beginTransmission(CompassAddress);  // Envia endereço I2C para acessar os registradores do magentometro
  Wire.write(0x03);  // Envia o endereços inicial para acessar os regitradores que contem os valores lidos nos 3 eixos do magnetometro.     
  Wire.endTransmission(); // Finaliza a transmissão de configuração
  
  Wire.beginTransmission(CompassAddress); // Envia endereço I2C para acessar os registradores do magentometro
  Wire.requestFrom(CompassAddress, 6);    // Envia o numero de bytes para leitura a partir do endereço 0x32.
                                           // Serão lidos os 6 bytes armazenados nos registradores 0x03 até 0x09. 
  while(Wire.available())  // Verifica se os dados estão prontos para leitura
  { 
    buff[i] = Wire.read();  // Recebe os bytes enviados pelo magnetometro. Enviado primeiramente o LSB e em seguida o MSB.
    i++;
  }
  Wire.endTransmission(); // Finaliza o recebimento
  
  if (i==6)  // Verifica se os 6 byte foram recebidos
    {
    magnetom_x = SENSOR_SIGN[6]*((((int)buff[2]) << 8) | buff[3]);  // Efetua a concatenação dos bytes MSB e LSB recebidos para o eixo x (internal sensor y axis)
    magnetom_y = SENSOR_SIGN[7]*((((int)buff[0]) << 8) | buff[1]);  // Efetua a concatenação dos bytes MSB e LSB recebidos para o eixo y (internal sensor x axis)
    magnetom_z = SENSOR_SIGN[8]*((((int)buff[4]) << 8) | buff[5]);  // Efetua a concatenação dos bytes MSB e LSB recebidos para o eixo z
    }
 }

