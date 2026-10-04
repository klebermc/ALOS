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

// ****************** INCLUDES OF EXTERNAL LIBRARIES *************************
#include <SD.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
// ***************************************************************************


// *********************** DEFINITION OF MACROS ******************************

#define      MAX_MSG_SIZE   200        // numero máximo de caracteres para a mensagem
#define                dt   0.020      // time interval between accel readins, step used at euler's integration
#define   desired_HEADING   0          // desired value of heading, this is the value read by the magnetometer when we place the body X vector overlapping with the inertial X axis
#define  TIMEOUT_MSG_XBEE   100        // maximum time in milliseconds to wait for a xbee message to be fully received
#define         debugPin1   11         // pin set to debug
#define         debugPin2   12         // pin set to debug
#define         LOG_ON_SD   false      // this flag enables the code to log the position into the SD card
#define       offBoardLED   31         // the pin port for the offboard green led, placed on quadrotor frame
#define      N_CALIB_DATA   250        // number of data to colect to calibrate IMU


// IMU
#define LSM9DS1_M   0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW
// *************************************************************************


// ***************** DEFINITION OF FUNCTIONS AS MACROS *********************
// The register value and it's respective ouput pin. These "functions" receive the DESIRED PULSE WIDTH IN MICROSECONDS
// then it is converted to the respective values for the timer. Inside the timer:
//    OCRnm = 2000 -> duty cycle 5%
//    OCRnm = 4000 -> duty cycle 10%
#define SET_PWM_PIN2(x)     OCR3B = 2*x
#define SET_PWM_PIN3(x)     OCR3C = 2*x
#define SET_PWM_PIN5(x)     OCR3A = 2*x
#define SET_PWM_PIN6(x)     OCR4A = 2*x
#define SET_PWM_PIN7(x)     OCR4B = 2*x
#define SET_PWM_PIN8(x)     OCR4C = 2*x

//Converting the ouput of the controllers to the expected PWM values for the KK
// 20  - value in degrees of the maximum angle commanded by KK, MEASURED
// 400 - value in microseconds of difference from the reference PWM signal at the input of KK, 1500 +or- 400 gives the maximum and minimum PWM at the KK input signals
//(((angle*400)/20)+1500)
#define ANG2uSEC(angle) (int)((angle*20)+1500)// This value was empirically determined by a given AILERON and ELEVETOR input

#define VEL_ANG2uSEC(vel_ang) (int)(((vel_ang*40)/9)+1500)// This value was empirically determined by a given RUDDER input

//#define MAX_THRUST 33 //value of thrust (FORÇA EM Newtons) exerted by the 4 motors, at full throttle
#define THRUST2THROTTLE(x) (int)(((float)(1000/33)*x)+1000)  // This function converts from a desired value of Newtons to a PWM pulse size in uSeconds,based on a motor thrust value

#define SATURATION(value,upLimit,lowLimit) value=min(max(value,lowLimit),upLimit) //this function saturates a given value between lower and upper limits
// **************************************************************************

// ************************** GLOBAL VARIABLES *****************************

// Rotate the accel read to match the inertial reference system
float Rb2I[3][3]={0,0,0,0,0,0,0,0,0};
float R_pqr2deltaEuler[3][3]={0,0,0,0,0,0,0,0,0};
float pitch=0, roll=0, yaw = 0;
double cr=cos(roll);
double cp=cos(pitch);
double cy=cos(yaw);
double sr=sin(roll);
double sp=sin(pitch);
double sy=sin(yaw);
float gyro_b[3]={0,0,0};
float delta_ang_Euler[3]={0,0,0};


// **** SENSOR DE UNIDADE DE MEDIDAS INERCIAIS ****
LSM9DS1 imu;
float declinationAngle;
int mx_bias = 1095, my_bias = 537, mx_ge = 1423, my_ge = 1487;
float magnetom_x = 0, magnetom_y = 0;
float offset_value=0; //the initial offset value of the magnetometer
float OFFSET_ACCEL[3]={0,0,0};
float OFFSET_GYRO[3]={0,0,0};
//*********************************

float distance2ground=0;      //The quadrotor height

float pos_I[3]   = {0, 0, 0}; // Position in the inertial frame
float vel_I[3]   = {0, 0, 0}; // Velocity in the inertial frame
float accel_I[3] = {0, 0, 0}; // Acceleration in the inertial frame

float accel_b[3] = {0, 0, 0}; // Acceleration in the inertial frame

float pos_LS[3]  = {0, 0, 0}; // Position calculated by the location system

float desired_pos_I[3] = {0, 0, 0};  // The waypoint, the desired position for the quadrotor to achieve
float cumul_error[3]   = {0, 0, 0};  // The cumulative error of the position for the integral component of the positions PIDs
float previous_pos[3]  = {0, 0, 0};  // The previous position of the quadrotor, used in the derivative component of the PID

float previous_accel[3]={0,0,0};

float actual_heading = 0;       // Angle of heading
float previous_HEADING = 0;     // The previous heading angle of the quadrotor. used in the derivative component of the controller
float cumul_error_HEADING = 0;  // The cumulative error of heading for the integral component of the controller

float X_out = 0, Y_out = 0, Z_out = 0;  // The commanded values of pitch, roll and throttle, respectively
float HD_out = 0;                       // The commanded value of heading

float Kp_X, Ki_X, Kd_X;       // The inertial X position controller gains
float Kp_Y, Ki_Y, Kd_Y;       // The inertial Y position controller gains
float Kp_Z, Ki_Z, Kd_Z;       // The inertial Z position controller gains
float Kp_HD, Ki_HD, Kd_HD;    // The heading controller gains

float error_X,error_Y,error_Z; //The position error, used at the position controllers
float dXdt,dYdt,dZdt;          //The derivative of the position value, used at the position controllers

float error_HEADING;           //Heading error, used at the heading controller
float dHDdt;                   //Derivative of heading value, used at the heading controller

long actual_time_50hz, actual_time_10hz;  // Variables used for timing the tasks
int countTimeOut = 0;  // Variable used to timeout the main loop => if there is more than 1s without any control task, or 3s without data from Location System, stops the quad
short locationSystemLinkLost=0; // Counting the lost messages from location system
int count = 0;  //Used for timing tasks
unsigned int iterator=0;

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
//******************************************************************************

//External Interrruptions variables
int pwm_value = 0;
int prev_time = 0;
boolean sinc_pwms_setup=false;
#define externalPwmPin 5
//***********************************


ISR(TIMER4_COMPC_vect){
  // Enters here each 20ms
  countTimeOut++;
  if (countTimeOut>50) //after 1 second without the counter countTimeOut been reseted, it means we have no IMU information, therefore stop the quadrotor
  {
    Serial.println("Timeout IMU comp vect");
    delay(10);
    smoth_landing();
  }
}

void setup() {
   
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  pinMode(debugPin1, OUTPUT);
  digitalWrite(debugPin1,LOW);
  pinMode(debugPin2, OUTPUT);
  digitalWrite(debugPin2,LOW);
  pinMode(offBoardLED, OUTPUT);
  digitalWrite(offBoardLED,LOW);

  setupIMU();
  //magnetometerCalibration();
  
  Serial.begin(9600);  // USB communication
  Serial2.begin(9600);  // XBee communication

  init_PWMs();
  sinc_pwms_setup=true; // after done this, the next time a signal arrives at the external interrupt pin, the pulses from the pwms will be sychronized
  attachInterrupt(externalPwmPin, rising_external_pwm, RISING); //read the external signal from the radio controller
    
  initialize_controllers();  // Set up the gains for the controllers
  
  if(LOG_ON_SD) //setup the file for LOG
  {
    init_SD_card();
    sd_open_file("log_data.txt");
    myFile.println("count px py pz xout yout zout");
    sd_close_file();

    while(!SD_ok); //Holds here if it is derised to do the log but the communication with the SD card is not OK

    //Blinks to indicate that the program will proceed
    digitalWrite(ledPin, HIGH);    delay(500);
    digitalWrite(ledPin, LOW);    delay(500); 
  }
  //while(accel_I[0]==0) readIMU();            // Holds waiting for AHRS message to start the program (accel x y and mag);

  // This function will search the actual heading beeing mesured by the compass 
  // (when the X axis of the quad is aligned with the X inertial axis), 
  // this value will be compensated from the measurement
  findCompassOffsetValue();
  Calibration_Accel();
  Calibration_Gyro();
  
  digitalWrite(offBoardLED,LOW);
  
  while(sinc_pwms_setup); //me sure we are reading the remote control
  
  digitalWrite(offBoardLED,HIGH);
  while(!msg_xbeeOK) 
  {
    receive_data_xbee(); // Holds waiting for XBee message to start the program (actual pos and waypoint);
    delay(10);
  }
  
  arm_kk215(); //Proceding for arming the KK215, allowing flight
    
  actual_time_10hz = millis();
  actual_time_50hz = actual_time_10hz;

  TIMSK4 = (1<<OCIE4C); // activate the interruption for timeouts
}

void loop()
{
  if ((millis() - actual_time_50hz) >= 20) //  50Hz tasks
  {
    actual_time_50hz = millis(); // save the actual time for comparision
    digitalWrite(debugPin1,HIGH); //Debug on osciliscope
    
    readIMU();    

    //Goes inside this if every time there was a valid message from IMU
    if ((previous_accel[0]!=accel_I[0]) || (previous_accel[1]!=accel_I[1]) || (previous_accel[2]!=accel_I[2]))
    {
      // IMU data was OK
      previous_accel[0]=accel_I[0];      previous_accel[1]=accel_I[1];      previous_accel[2]=accel_I[2];
      countTimeOut=0; //reset the timeout every time it has IMU data
    }
    
    euler_integration();  // Integrate the data to obtain the actual position of the quadrotor
    
//    HEADING_controller(); // Calls the heading controller every time there is a IMU reading (50Hz)
    
    Z_controller();  //Height controller
    digitalWrite(debugPin1,LOW); //Debug on osciliscope    
  }
 
  receive_data_xbee(); // Try to read the data from XBee
    
  if ((millis() - actual_time_10hz) >= 100) // Call the position controllers at 10Hz
  {
    actual_time_10hz = millis();
    digitalWrite(debugPin2,HIGH); //Debug on osciliscope
    digitalWrite(offBoardLED, !digitalRead(offBoardLED));
    
//    X_controller();
//    Y_controller();

    //print_position('\n');
    
    count++; //Needs to be before this IF
    if(LOG_ON_SD) // Log the calculed position to the SD Card
    {
      //Avoids to keep openning and closing the file all the time
      if(count==1) sd_open_file("log_data.txt");
      iterator++;
      //Serial.print(iterator);    Serial.print(" ");    Serial.println(pos_I[0],4);
      write_on_file(iterator,pos_I[0],pos_I[1],pos_I[2],X_out,Y_out,Z_out);
      if(count==10) sd_close_file();
    }
    //else delay(2); //REMOVE IT, JUST FOR OSCILOSCOPE VISUALIZATION

    digitalWrite(debugPin2,LOW); //Debug on osciliscope
  }
  
  if (count == 10) // at 1Hz => replace the calculated value for the position with a position from the Location System
  {
    count = 0;
    if (msg_xbeeOK) // If there was a message from the Location System, replace it
    {
      parse_msg_xbee();
      msg_xbeeOK=false;

      //This is where the location system values for position will replace the integrated from IMU
      pos_I[0] = pos_LS[0]; pos_I[1] = pos_LS[1]; //pos_I[2] = pos_LS[2];
      vel_I[0] = 0; vel_I[1] = 0; vel_I[2] = 0;
      
      //print_position(' ');
      //print_desired_position('\n');
      
      if(pos_LS[0]==-1 && pos_LS[1]==-1 && pos_LS[2]==-1)
      { 
        //Something went wrong at the location system and the computer sent a stop message
        countTimeOut=50;
      }
      else
      {
        //The message from computer was OK
       locationSystemLinkLost=0;
       countTimeOut=0;
      }
    }
    else // If there is no message, put the positio in 0
    {
       //Serial.println("NO data arrived!");
      locationSystemLinkLost++;
      if (locationSystemLinkLost>=3)
      {
        countTimeOut=100;
        delay(100);
      }
    }
    //delay(2); // remove it, just for osciloscope visualization
  }
}

void euler_integration()
{
  //Input acceleration 
  SATURATION(accel_I[0], 5, -5);
  SATURATION(accel_I[1], 5, -5);
  
  vel_I[0] = vel_I[0] + accel_I[0] * dt;
  vel_I[1] = vel_I[1] + accel_I[1] * dt;
  //vel_I[2] = vel_I[2] + accel_I[2] * dt;

  pos_I[0] = pos_I[0] + vel_I[0] * dt;
  pos_I[1] = pos_I[1] + vel_I[1] * dt;
  //pos_I[2] = pos_I[2] + vel_I[2] * dt;
  
  //Saturation
  SATURATION(pos_I[0],3,0);
  SATURATION(pos_I[1],3,0);
  SATURATION(pos_I[2],3,0); 
}


void arm_kk215()
{
  countTimeOut=-2000; //Avoiding timeout during kk initialization
  
  SET_PWM_PIN2(1000);//THROTTLE
  SET_PWM_PIN6(1000);//RUDDER

  delay(5000);
  
  SET_PWM_PIN2(1000);//THROTTLE
  SET_PWM_PIN6(1500);//RUDDER

  countTimeOut=0;

}

void smoth_landing()
{
    digitalWrite(ledPin, LOW);
    digitalWrite(debugPin1, LOW);
    digitalWrite(debugPin2, LOW);
    digitalWrite(offBoardLED, LOW);

    idle_state_PWMs();
    //Lets get the quadrotor down
    while (distance2ground>20)
    {
      desired_pos_I[2] = desired_pos_I[2] - 0.0001;
      SATURATION(desired_pos_I[2],1,0);
      Z_controller();
      delay(20);
    }
    idle_state_PWMs();
    while(true);
}

void print_position(char newline)
{
  Serial.print(pos_I[0]);    Serial.print(" ");
  Serial.print(pos_I[1]);    Serial.print(" ");
  Serial.print(pos_I[2]);    Serial.print(newline);
}

void print_desired_position(char newline)
{
  Serial.print(desired_pos_I[0]);    Serial.print(" ");
  Serial.print(desired_pos_I[1]);    Serial.print(" ");
  Serial.print(desired_pos_I[2]);    Serial.print(newline);
}

