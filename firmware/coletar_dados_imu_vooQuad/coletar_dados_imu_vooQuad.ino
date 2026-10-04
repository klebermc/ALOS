//***************************************************************************
//                 Instituto Tecnológico de Aeronáutica (ITA)
//                  Divisão de Engenharia Eletrônica (IEE)
//                   Departamento de Eletrônica Aplicada
//                          Projeto de mestrado
//                          1º Semestre de 2017
//                      Autor: Kléber Macedo Cabral
//***************************************************************************
//***************************************************************************
//
// Este programa implementa os controladores de posição do quadrirrotor e o controlador
// do ângulo de heading (orientação).
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
// Arduino atmega2560
//
// Software Utilizado:
//
// Arduino IDE
// Modo de compilação depende do hardware utilizado
// Modo configurado: Arduino Mega 2560
//
//****************************************************************************

#include <SD.h>
#include <SPI.h>

//The TX from AHRS is connected to the port 19, Arduino RX1

#define      MAX_MSG_SIZE   200        // numero máximo de caracteres para a mensagem
//#define                dt   0.020      // time interval between accel readins, step used at euler's integration
//#define   desired_HEADING   0          // desired value of heading, this is the value read by the magnetometer when we place the body X vector overlapping with the inertial X axis
//#define  TIMEOUT_MSG_XBEE   100        // maximum time in milliseconds to wait for a xbee message to be fully received
#define          debugPin1  11         // pin set to debug
#define          debugPin2  12         // pin set to debug
#define         LOG_ON_SD   true       // this flag enables the code to log the position into the SD card

float pitch   = 0; // Position in the inertial frame
float roll   = 0; // Velocity in the inertial frame
float yaw = 0; // Acceleration in the inertial frame
float angles[3] = {0,0,0};

long actual_time_50hz, actual_time_10hz; // Variables used for timing the tasks
int count = 0;  //Used for timing tasks
unsigned int iterator=0;

// Variables used at the xbee communication
char in_char_ahrs;
unsigned int pos_vmsg = 0, size_msg = 0; // Control variables to manipulate the received message
char in_msg_ahrs[MAX_MSG_SIZE]; // Message received from Serial 1
bool new_data_ahrs = false;      // Boolean to indicate when there is a new message

// File Handling variables
#define ledPin 13 // This pins goes HIGH when there is sth wrong with the log
#define pinCS 53  // Chip select for SPI comm
File myFile;      // The file object
boolean SD_ok;    // Indicates the SD comm is OK

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  pinMode(debugPin1, OUTPUT);
  digitalWrite(debugPin1, LOW);
  pinMode(debugPin2, OUTPUT);
  digitalWrite(debugPin2, LOW);

  delay(5000); //When the IMU and Mega start together, give a little time to the IMU to calibrate itself

  Serial.begin(9600);  // USB communication
  Serial1.begin(57600); // AHRS communication
  Serial2.begin(9600);  // XBee communication

  digitalWrite(ledPin, HIGH);    delay(500);
  digitalWrite(ledPin, LOW);    delay(500);
  
  if (LOG_ON_SD) //setup the file for LOG
  {
    Serial.println();
    Serial.println("Start writing on SD");
    init_SD_card();
    sd_open_file("log_data.txt");
    //myFile.println("-------- Log Start ---------");
    if(SD_ok) myFile.println("count pitch roll yaw");
    sd_close_file();

    if(SD_ok) Serial.println("SD OK");
    else      Serial.println("SD NOT OK");
    
    while(!SD_ok); //Holds here if it is derised to do the log but the communication with the SD card is not OK
    //Blinks to indicate that the program will proceed
    digitalWrite(ledPin, HIGH);    delay(500);
    digitalWrite(ledPin, LOW);    delay(500);
  }
  Serial.println("Quad Main Software Start");
  while (!Serial1.available()); // Holds waiting for AHRS message to start the program;
  actual_time_10hz = millis();
  actual_time_50hz = actual_time_10hz;
}

void loop()
{
  receive_data_ahrs(); // Try to read data comming from the AHRS

  if (((millis() - actual_time_50hz) >= 20) && new_data_ahrs) // The data from AHRS should be received ad 50Hz
  {
    actual_time_50hz = millis(); // save the actual time for comparisiont
    //digitalWrite(debugPin1,HIGH); //Debug on osciliscope
    parse_msg_ahrs();   //Retrieve accel and heading information from the image
    new_data_ahrs = false;
    //digitalWrite(debugPin1,LOW); //Debug on osciliscope
    //Serial.print(" "); Serial.print(pitch,4); Serial.print(" "); Serial.print(roll,4); Serial.print(" "); Serial.println(yaw,4);
    angles[0]+=pitch/5;
    angles[1]+=roll/5;
    angles[2]+=yaw/5;
  }

  if ((millis() - actual_time_10hz) >= 100) // Call the position controllers at 10Hz
  {
    actual_time_10hz = millis();
    digitalWrite(debugPin1, HIGH); //Debug on osciliscope

    count++; //Needs to be before this IF
    if (LOG_ON_SD) // Log the calculed position to the SD Card
    {
      //Avoids to keep openning and closing the file all the time
      if (count == 1) sd_open_file("log_data.txt");
      iterator++;
      write_on_file(iterator, angles[0], angles[1], angles[2]);
      if (count == 10) sd_close_file();
    }
    else delay(2); //REMOVE IT, JUST FOR OSCILOSCOPE VISUALIZATION
    Serial.print(iterator); Serial.print(" "); Serial.print(pitch,4); Serial.print(" "); Serial.print(roll,4); Serial.print(" "); Serial.println(yaw,4);
//    Serial.print("--"); Serial.print(" "); Serial.print(angles[0],4); Serial.print(" "); Serial.print(angles[1],4); Serial.print(" "); Serial.println(angles[2],4);
    angles[0]=0;
    angles[1]=0;
    angles[2]=0;
    digitalWrite(debugPin1, LOW); //Debug on osciliscope
  }

  if (count == 10) // at 1Hz => replace the calculated value for the position with a position from the Location System
  {
    digitalWrite(debugPin2, HIGH); //Debug on osciliscope
    count = 0;

    delay(1); // remove it, just for osciloscope visualization
    digitalWrite(debugPin2, LOW); //Debug on osciliscope
  }
}

