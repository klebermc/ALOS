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
#define                dt   0.020      // time interval between accel readins, step used at euler's integration
//#define   desired_HEADING   0          // desired value of heading, this is the value read by the magnetometer when we place the body X vector overlapping with the inertial X axis
#define  TIMEOUT_MSG_XBEE   100        // maximum time in milliseconds to wait for a xbee message to be fully received
#define         debugPin1   11         // pin set to debug
#define         debugPin2   12         // pin set to debug
#define         LOG_ON_SD   false       // this flag enables the code to log the position into the SD card
#define       offBoardLED   31          //the pin port for the offboard green led, placed on quadrotor frame

//// The register value and it's respective ouput pin. These "functions" receive the DESIRED PULSE WIDTH IN MICROSECONDS
//// then it is converted to the respective values for the timer. Inside the timer:
////    OCRnm = 2000 -> duty cycle 5%
////    OCRnm = 4000 -> duty cycle 10%
//#define SET_PWM_PIN2(x)     OCR3B = 2*x
//#define SET_PWM_PIN3(x)     OCR3C = 2*x
//#define SET_PWM_PIN5(x)     OCR3A = 2*x
//#define SET_PWM_PIN6(x)     OCR4A = 2*x
//#define SET_PWM_PIN7(x)     OCR4B = 2*x
//#define SET_PWM_PIN8(x)     OCR4C = 2*x

////Converting the ouput of the controllers to the expected PWM values for the KK
//// 20  - value in degrees of the maximum angle commanded by KK, MEASURED
//// 400 - value in microseconds of difference from the reference PWM signal at the input of KK, 1500 +or- 400 gives the maximum and minimum PWM at the KK input signals
////(((angle*400)/20)+1500)
//#define ANG2uSEC(angle) (int)((angle*20)+1500)// This value was empirically determined by a given AILERON and ELEVETOR input

//#define VEL_ANG2uSEC(vel_ang) (int)(((vel_ang*40)/9)+1500)// This value was empirically determined by a given RUDDER input

////#define MAX_THRUST 33 //value of thrust (FORÇA EM Newtons) exerted by the 4 motors, at full throttle
//#define THRUST2THROTTLE(x) (int)(((float)(1000/33)*x)+1000)  // This function converts from a desired value of Newtons to a PWM pulse size in uSeconds,based on a motor thrust value

#define SATURATION(value,upLimit,lowLimit) value=min(max(value,lowLimit),upLimit) //this function saturates a given value between lower and upper limits

//float distance2ground=0;      //The quadrotor height

float pos_I[3]   = {0, 0, 0}; // Position in the inertial frame
float vel_I[3]   = {0, 0, 0}; // Velocity in the inertial frame
float accel_I[3] = {0, 0, 0}; // Acceleration in the inertial frame

float pos_LS[3]  = {0, 0, 0}; // Position calculated by the location system

float desired_pos_I[3] = {0, 0, 0};  // The waypoint, the desired position for the quadrotor to achieve
//float cumul_error[3]   = {0, 0, 0};  // The cumulative error of the position for the integral component of the positions PIDs
//float previous_pos[3]  = {0, 0, 0};  // The previous position of the quadrotor, used in the derivative component of the PID

float actual_heading = 0;       // Angle of heading
//float previous_HEADING = 0;     // The previous heading angle of the quadrotor. used in the derivative component of the controller
//float cumul_error_HEADING = 0;  // The cumulative error of heading for the integral component of the controller

//float X_out = 0, Y_out = 0, Z_out = 0;  // The commanded values of pitch, roll and throttle, respectively
//float HD_out = 0;                       // The commanded value of heading

//float Kp_X, Ki_X, Kd_X;       // The inertial X position controller gains
//float Kp_Y, Ki_Y, Kd_Y;       // The inertial Y position controller gains
//float Kp_Z, Ki_Z, Kd_Z;       // The inertial Z position controller gains
//float Kp_HD, Ki_HD, Kd_HD;    // The heading controller gains

//float error_X,error_Y,error_Z; //The position error, used at the position controllers
//float dXdt,dYdt,dZdt;          //The derivative of the position value, used at the position controllers

//float error_HEADING;           //Heading error, used at the heading controller
//float dHDdt;                   //Derivative of heading value, used at the heading controller

long actual_time_50hz, actual_time_10hz;  // Variables used for timing the tasks
//long mainLoopTimeOut;  // Variable used to timeout the main loop => if there is more than 1s without any control task, or 5s without data from Location System, stops the quad
//short locationSystemLinkLost=0; // Counting the lost messages from location system
int count = 0;  //Used for timing tasks
unsigned int iterator = 0; //Used for couting the data stored at the SD card

// Variables used at the xbee communication
char in_char_ahrs;
unsigned int pos_vmsg = 0, size_msg = 0; // Control variables to manipulate the received message
char in_msg_ahrs[MAX_MSG_SIZE]; // Message received from Serial 1
bool new_data_ahrs = false;      // Boolean to indicate when there is a new message

// Variables used at the xbee communication
boolean msg_xbeeOK = false;
char in_msg_xbee[MAX_MSG_SIZE];
int length_msg_xbee;
long actual_time_xbee;

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
  pinMode(offBoardLED, OUTPUT);
  digitalWrite(offBoardLED, LOW);

  delay(5000); //When the IMU and Mega start together, give a little time to the IMU to calibrate itself

  Serial.begin(9600);  // USB communication
  Serial1.begin(57600); // AHRS communication
  Serial2.begin(9600);  // XBee communication

  if (LOG_ON_SD) //setup the file for LOG
  {
    init_SD_card();
    sd_open_file("log_data.txt");
    //myFile.println("-------- Log Start ---------");
    myFile.println("count px py pz vx vy vz");
    sd_close_file();

    while (!SD_ok); //Holds here if it is desired to do the log but the communication with the SD card is not OK
    //Blinks to indicate that the program will proceed
    digitalWrite(ledPin, HIGH);    delay(500);
    digitalWrite(ledPin, LOW);    delay(500);
  }

  //while (!Serial1.available());           // Holds waiting for AHRS message to start the program (accel x y and mag);
  //while (!msg_xbeeOK) receive_data_xbee(); // Holds waiting for XBee message to start the program (actual pos and waypoint);
  //Serial.println("Quad Main Software Start");

  digitalWrite(offBoardLED, HIGH);
  //digitalWrite(ledPin, HIGH);

  actual_time_10hz = millis();
  actual_time_50hz = actual_time_10hz;
}

void loop()
{
  receive_data_ahrs(); // Try to read data comming from the AHRS

  if (((millis() - actual_time_50hz) >= 20) && new_data_ahrs) // The data from AHRS should be received ad 50Hz
  {
    actual_time_50hz = millis(); // save the actual time for comparision
    digitalWrite(debugPin1, HIGH); //Debug on osciliscope
    parse_msg_ahrs();   //Retrieve accel and heading information from the image
    euler_integration();  // Integrate the data to obtain the actual position of the quadrotor
    new_data_ahrs = false;
    digitalWrite(debugPin1, LOW); //Debug on osciliscope
  }

  receive_data_xbee();

  if ((millis() - actual_time_10hz) >= 100) // Call the position controllers at 10Hz
  {
    actual_time_10hz = millis();
    digitalWrite(debugPin2, HIGH); //Debug on osciliscope
    digitalWrite(offBoardLED, !digitalRead(offBoardLED));
    
    count++; //Needs to be before this IF
    if (LOG_ON_SD) // Log the calculed position to the SD Card
    {
      //Avoids to keep openning and closing the file all the time
      if (count == 1) sd_open_file("log_data.txt");
      iterator++;
      //Serial.print(iterator);    Serial.print(" ");    Serial.println(pos_I[0],4);
      write_on_file(iterator, pos_I[0], pos_I[1], pos_I[2], vel_I[0], vel_I[1], vel_I[2]);
      if (count == 10) sd_close_file();
    }
    else{
      Serial.print(pos_I[0],4); Serial.print(" ");
      Serial.print(pos_I[1],4); Serial.print(" ");
      Serial.print(pos_I[2],4); Serial.println();
    }
    digitalWrite(debugPin2, LOW); //Debug on osciliscope
  }

  if (count == 10) // at 1Hz => replace the calculated value for the position with a position from the Location System
  {
    //digitalWrite(debugPin2,HIGH); //Debug on osciliscope
    count = 0;
    if (msg_xbeeOK) // If there was a message from the Location System, replace it
    {
      parse_msg_xbee();
      msg_xbeeOK = false;

      //This is where the location system values for position will replace the integrated from IMU
      pos_I[0] = pos_LS[0]; pos_I[1] = pos_LS[1]; pos_I[2] = pos_LS[2];
      vel_I[0] = 0; vel_I[1] = 0; vel_I[2] = 0;

      //Uncomment to verify the correct receivement of the position
      /*Serial.println("Replacing data");
        Serial.print(pos_LS[0]);    Serial.print(" ");
        Serial.print(pos_LS[1]);    Serial.print(" ");
        Serial.print(pos_LS[2]);    Serial.print(" ");
        Serial.print(desired_pos_I[0]);    Serial.print(" ");
        Serial.print(desired_pos_I[1]);    Serial.print(" ");
        Serial.print(desired_pos_I[2]);    Serial.print(" ");
        Serial.println();
      */
    }
    else // If there is no message, put the positio in 0
    {
      //Serial.println("NO data arrived!");
      //pos_I[0] = 0; pos_I[1] = 0; pos_I[2] = 0;
      //vel_I[0] = 0; vel_I[1] = 0; vel_I[2] = 0;
    }
    //delay(1); // remove it, just for osciloscope visualization
    //digitalWrite(debugPin2,LOW); //Debug on osciliscope
  }
}

void euler_integration()
{
  SATURATION(accel_I[0], 5, -5);
  SATURATION(accel_I[1], 5, -5);
  
  vel_I[0] = vel_I[0] + accel_I[0] * dt;
  vel_I[1] = vel_I[1] + accel_I[1] * dt;
  //vel_I[2] = vel_I[2] + accel_I[2] * dt;

  pos_I[0] = pos_I[0] + vel_I[0] * dt;
  pos_I[1] = pos_I[1] + vel_I[1] * dt;
  //pos_I[2] = pos_I[2] + vel_I[2] * dt;

  //Saturation
  SATURATION(pos_I[0], 10, -10);
  SATURATION(pos_I[1], 10, -10);
  //SATURATION(pos_I[2],10,-10);
}

