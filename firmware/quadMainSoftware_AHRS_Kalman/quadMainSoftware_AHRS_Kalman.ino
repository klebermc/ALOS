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

//The TX from AHRS is connected to the port 19, Arduino RX1

// ****************** INCLUDES OF EXTERNAL LIBRARIES *************************
#include <SD.h>
#include <SPI.h>
#include <MatrixMath.h>
// ***************************************************************************

// *********************** DEFINITION OF MACROS ******************************
#define      MAX_MSG_SIZE   200        // numero máximo de caracteres para a mensagem
#define                dt   0.100      // time interval between accel readins, step used at euler's integration
#define   desired_HEADING   0          // desired value of heading, this is the value read by the magnetometer when we place the body X vector overlapping with the inertial X axis
#define  TIMEOUT_MSG_XBEE   100        // maximum time in milliseconds to wait for a xbee message to be fully received
#define         debugPin1   11         // pin set to debug
#define         debugPin2   12         // pin set to debug
#define         LOG_ON_SD   false       // this flag enables the code to log the position into the SD card
#define       offBoardLED   31         // the pin port for the offboard green led, placed on quadrotor frame
#define       numReadings   5          // size of the array for filtering values
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
#define ANG2uSEC(angle) (int)(((angle*20)/40)+1500)// This value was empirically determined by a given AILERON and ELEVETOR input

#define VEL_ANG2uSEC(vel_ang) (int)(((vel_ang*40)/9)+1500)// This value was empirically determined by a given RUDDER input

//#define MAX_THRUST 33 //value of thrust (FORÇA EM Newtons) exerted by the 4 motors, at full throttle
#define THRUST2THROTTLE(x) (int)(((float)(1000/33)*x)+1000)  // This function converts from a desired value of Newtons to a PWM pulse size in uSeconds,based on a motor thrust value

#define SATURATION(value,upLimit,lowLimit) value=min(max(value,lowLimit),upLimit) //this function saturates a given value between lower and upper limits

#define     rad2deg(x)        (x*57.2957795131)     // * 180/pi = 57.2957795131
#define     deg2rad(x)        (x*0.01745329252)     // * pi/180 =  0.0174529252
// **************************************************************************

// ************************** GLOBAL VARIABLES *****************************

// ----------------------- KALMAN VARIABLES -----------------------
//%All the states that are somehow needed by Kalman Filter

#define     dtK 0.1 //time step form Kalman filter propagation

//%Vector N_INS (standard deviations)
#define noise_ax_INS (0.0475*2)
#define noise_ay_INS (0.0475*2)
#define noise_wz_INS (0.007*2)

//%Vector N_LS (standard deviations)
#define noise_px_LS  (0.1*5)
#define noise_py_LS  (0.1*5)
#define noise_psi_comp  0.01

#define beta 0.001      // parameter to avoid filter divergence

float Bias[3];  //initialization of the bias estimative needed by KF
float G[8][3];  // kalman gain matrix
float P0[8][8]; // initial state error covariance matrix
float P[8][8];  // covariance error matrix
float R[3][3];  // covariance matrix of Location System and Compass N_LS
float Q[3][3];  // process covariance matrix
float X_INS[5] = {0, 0, 0, 0, 0}; //States X_INS(t) = [Vx(t),Vy(t), Px(t), Py(t), ψ(t)]
float U[3];     // input vector for kalman

// X = A_E * X_E + B_E * N_INS
// Y = H * X_E + N_LS
float Ad_E[8][8];
float Bd_E[8][3];
float H[3][8];
float X_E[8];
float Y_E[3];

float Ident[8][8]; // identity matrix

//Temporary matrices to store values within a bigger calculation
float t1[8][8], t2[8][8], t3[8][8], t4[8][8], t5[8][8], t6[8][8];

boolean EKF_convergence_OK=false; //flag that indicates the convergence of the filter
byte prop_step=0, updt_step=0;
// ----------------------------------------------------------

float pos_I[3]   = {0, 0, 0}; // Position in the inertial frame
//float vel_I[3]   = {0, 0, 0}; // Velocity in the inertial frame
//float accel_I[3] = {0, 0, 0}; // Acceleration in the inertial frame

float accel_b[3] = {0, 0, 0}; // Acceleration in body frame
float wz_b = 0;               // gyro measurement in rad/s - body frame

float pos_LS[3]  = {0, 0, 0}; // Position calculated by the location system

float desired_pos_I[3] = {0, 0, 0};  // The waypoint, the desired position for the quadrotor to achieve
float cumul_error[3]   = {0, 0, 0};  // The cumulative error of the position for the integral component of the positions PIDs
float previous_pos[3]  = {0, 0, 0};  // The previous position of the quadrotor, used in the derivative component of the PID
float previous_cumul_error[3]   = {0, 0, 0};  // <USED at antiwindup filter> The previous value of the cumulative error of the position

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
float history_HEADING[numReadings] = {0, 0, 0, 0, 0}; // history of the heading angle, used to filter the value every 100ms

byte  posVectRoll = 0;    // The position of the last stored roll value at the history vector
byte  posVectPitch = 0;   // The position of the last stored pitch value at the history vector
byte  posVectHD = 0;      // The position of the last stored heading value at the history vector

//filtered values are always in [degrees]
float filtered_Roll = 0;        // The angle of roll after the median filtration
float filtered_Pitch = 0;       // The angle of pitch after the median filtration
float filtered_HEADING = 0;     // The angle of heading after the median filtration
// -----------------------------------------------------------------------------

//incoming values are in [degrees]
float actual_roll=0, actual_pitch=0; //measured euler angles at IMU in degrees
float actual_HEADING = 0;       // Angle of heading

float previous_HEADING = 0;     // The previous heading angle of the quadrotor. used in the derivative component of the controller
float cumul_error_HEADING = 0;  // The cumulative error of heading for the integral component of the controller

float X_out = 0, Y_out = 0, Z_out = 0;  // The commanded values of pitch, roll and throttle, respectively
float HD_out = 0;                       // The commanded value of heading

float Kp_X, Ki_X, Kd_X;       // The inertial X position controller gains
float Kp_Y, Ki_Y, Kd_Y;       // The inertial Y position controller gains
float Kp_Z, Ki_Z, Kd_Z;       // The inertial Z position controller gains
float Kp_HD, Ki_HD, Kd_HD;    // The heading controller gains

float error_X, error_Y, error_Z; //The position error, used at the position controllers
float previous_error_X, previous_error_Y, previous_error_Z; //The position error, used at the position controllers
float dXdt, dYdt, dZdt;        //The derivative of the position value, used at the position controllers

float error_HEADING;           //Heading error, used at the heading controller
float dHDdt;                   //Derivative of heading value, used at the heading controller

long actual_time_50hz, actual_time_10hz, actual_time_1hz;  // Variables used for timing the tasks
int countTimeOut = 0;  // Variable used to timeout the main loop => if there is more than 1s without any control task, or 3s without data from Location System, stops the quad
short locationSystemLinkLost = 0; // Counting the lost messages from location system
int count = 0;  //Used for timing tasks

unsigned int iterator = 0;
float distance2ground = 0;    //The quadrotor height
int counter_file = 0;

// -------------- Variables used at the AHRS communication --------------
char in_char_ahrs;
unsigned int pos_vmsg = 0, size_msg = 0; // Control variables to manipulate the received message
char in_msg_ahrs[MAX_MSG_SIZE]; // Message received from Serial 1
bool new_data_ahrs = false;      // Boolean to indicate when there is a new message
//----------------------------------------------------------------

// -------------- Variables used at the xbee communication --------------
boolean msg_xbeeOK = false;
char in_msg_xbee[MAX_MSG_SIZE];
int length_msg_xbee;
long actual_time_xbee;
//----------------------------------------------------------------

// -------------- File Handling variables --------------
#define ledPin 13 // This pins goes HIGH when there is sth wrong with the log
#define pinCS 53  // Chip select for SPI comm
File myFile;      // The file object
boolean SD_ok;    // Indicates the SD comm is OK
boolean do_log=false;
//----------------------------------------------------------------

// -------------- External Interrruptions variables --------------
int pwm_value = 0;
int prev_time = 0;
boolean sinc_pwms_setup = true;
#define externalPwmPin 2
//----------------------------------------------------------------

//******************************************************************************


ISR(TIMER4_COMPC_vect) {
  // Enters here each 20ms
  countTimeOut++;
  if (countTimeOut > 50) //after 1 second without the counter countTimeOut been reseted, it means we have no IMU information, therefore stop the quadrotor
  {
    Serial.println("T1"); //timout 1 segundo sem dado IMU ou 3s  sem dado XBee

    digitalWrite(ledPin, LOW);
    digitalWrite(debugPin1, LOW);
    digitalWrite(debugPin2, LOW);
    digitalWrite(offBoardLED, LOW);

    //    if(LOG_ON_SD) sd_close_file();

    smoth_landing();
  }
}

void setup() {

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  pinMode(debugPin1, OUTPUT);
  digitalWrite(debugPin1, LOW);
  pinMode(debugPin2, OUTPUT);
  digitalWrite(debugPin2, LOW);
  pinMode(offBoardLED, OUTPUT);
  digitalWrite(offBoardLED, LOW);

  //delay(5000); //When the IMU and Mega start together, give a little time to the IMU to calibrate itself

  Serial.begin(57600);  // USB communication
  Serial1.begin(57600); // AHRS communication
  Serial2.begin(57600);  // XBee communication

  EKF_initialization();
  
//  init_PWMs();
//
//  attachInterrupt(externalPwmPin, rising_external_pwm, RISING); //read the external signal from the radio controller
//
//  // *** Verification of the RC remote controller ***
//  digitalWrite(offBoardLED,LOW);
//  while(sinc_pwms_setup);
//  Serial.println("RC Radio Receiver Connected!");
//  // ****************************************
//  
//  // *** Verification of the SD card ***
//  if(LOG_ON_SD) //setup the file for LOG
//  {
//    init_SD_card();
//    sd_open_file("log_data.txt");
//    //myFile.println("count\taccelx\taccely\taccelz\troll\tpitch\theading");
//    myFile.println("count\tposx\tposy\tposz\txout\tyout\tzout\troll\tpitch\theading\thdout");
//    sd_close_file();
//
//    while(!SD_ok) //Holds here if it is desired to do the log but the communication with the SD card is not OK
//      blink_offboard_led(4);// "Morse code" for the user to know the problem was at SD card
//
//    //Blinks to indicate that the program will proceed
//    digitalWrite(ledPin, HIGH);    delay(500);
//    digitalWrite(ledPin, LOW);    delay(500);
//  }
//  // ****************************************
//
//  // *** Verification of the IMU connection ***
//  //The IMU must be connect, and the startup heading value must be inside of 5 degrees range for this procedure to pass
//  verify_IMU_connection(5.0);
//  Serial.println("IMU measurements available!");
//  // ****************************************
//
//  // *** Waits for a message through Xbee to start ***
//  digitalWrite(offBoardLED, HIGH);
//  while (!msg_xbeeOK)
//  {
//    receive_data_xbee(); // Holds waiting for XBee message to start the program (actual pos and waypoint);
//    delay(10);
//  }
//    parse_msg_xbee(); //using the value of the position at the initialization
//    //This is where the location system values for position will replace the integrated from IMU
//    X_INS[2] = pos_LS[0]; X_INS[3] = pos_LS[1]; //pos_I[2] = pos_LS[2];
//    msg_xbeeOK=false;
//  Serial.println("Xbee message received!");
//  // ****************************************
//
//  initialize_controllers();  // Set up the gains for the controllers
//
//  arm_kk215(); //Proceding for arming the KK215, allowing flight
//
//  cumul_error[2]=50; //make the startup of the test a litte bit faster
//
//  TIMSK4 = (1<<OCIE4C); // activate the interruption for timeouts

  actual_time_10hz = millis();
  actual_time_50hz = actual_time_10hz;
  actual_time_1hz  = actual_time_10hz;
}

void loop()
{
  receive_data_ahrs(); // Try to read data comming from the AHRS

  if ((millis() - actual_time_50hz) >= 20)// The data from AHRS should be received ad 50Hz
  {
    actual_time_50hz = millis(); // save the actual time for comparision
    digitalWrite(debugPin1, HIGH); //Debug on osciliscope
    
    if (new_data_ahrs) {
      parse_msg_ahrs();   //Retrieve accel and heading information from the image
      countTimeOut = 0; //reset the timeout every time it has IMU data
      new_data_ahrs = false;
    }
    print_IMU_data('\n');
    //accel x
    history_accelx[posVectAccelx] = accel_b[0];
    posVectAccelx = (posVectAccelx + 1) % numReadings;
    //accel y
    history_accely[posVectAccely] = accel_b[1];
    posVectAccely = (posVectAccely + 1) % numReadings;
    //accel z
    history_accelz[posVectAccelz] = accel_b[2];
    posVectAccelz = (posVectAccelz + 1) % numReadings;
    //roll
    history_Roll[posVectRoll] = actual_roll;
    posVectRoll = (posVectRoll + 1) % numReadings;
    //pitch
    history_Pitch[posVectPitch] = actual_pitch;
    posVectPitch = (posVectPitch + 1) % numReadings;
    //yaw
    history_HEADING[posVectHD] = actual_HEADING;
    posVectHD = (posVectHD + 1) % numReadings;
    //gyro z
    history_accelwz[posVectAccelwz] = wz_b;
    posVectAccelwz = (posVectAccelwz + 1) % numReadings;

//    Z_controller();  //Height controller

//    if((prop_step==0) && (updt_step==0) && do_log)
//    {
//      count++; //Needs to be before this IF
//      if (LOG_ON_SD) // Log the calculed position to the SD Card
//      {
//        //Avoids to keep openning and closing the file all the time
//        if (count == 1) sd_open_file("log_data.txt");
//        iterator++;
//        //      write_on_file(iterator,accel_b[0],accel_b[1],accel_b[2],actual_roll,actual_pitch,filtered_HEADING);
//        write_on_file(iterator, pos_I[0], pos_I[1], pos_I[2], X_out, Y_out, Z_out, filtered_Roll, filtered_Pitch, filtered_HEADING, HD_out);
//        if (count == 10){count=0; sd_close_file();}
//      }
//      do_log=false;
//    }
//    
//    if(prop_step==1)
//    { //Next time this task runs after the 10Hz task, do the second step of the kalman filter propagation
//      prop_step=2;
//      EKF_propagation();  // Kalman Filter Propagation  step 2 ~10ms
//      prop_step=0;
//      //print_P_diag();
//      EKF_CheckConvergence();
//    }
//    else
//    {
//      if(updt_step==1)
//      { //Next time this task runs after the 1Hz task, do the second step of the kalman filter update
//        updt_step=2;
//        EKF_update(); // Kalman Filter Update  step 2 ~10ms
//        updt_step=0;
//      }
//    }
    digitalWrite(debugPin1, LOW); //Debug on osciliscope
  }

  receive_data_xbee(); // Try to read the data from XBee

  if ((millis() - actual_time_10hz) >= 100) // Call the position controllers at 10Hz
  {
    //600us - without sd and EKP prop
    actual_time_10hz = millis();
    digitalWrite(debugPin2, HIGH); //Debug on osciliscope
    digitalWrite(offBoardLED, !digitalRead(offBoardLED));

    // -------  Filtering the IMU data ----------
    accel_b[0] = median(history_accelx);
    accel_b[1] = median(history_accely);
    accel_b[2] = median(history_accelz);

    wz_b = median(history_accelwz);

    //incoming values are in [degrees]
    filtered_Roll     = median(history_Roll); 
    filtered_Pitch    = median(history_Pitch);
    filtered_HEADING  = median(history_HEADING);
       
    SATURATION(filtered_Roll , 10, -10);
    SATURATION(filtered_Pitch, 10, -10);
    SATURATION(filtered_HEADING, 30, -30);
  
//    print_position_I('\n');
//    print_IMU_data('\n');
    // ------- --------------------- -----------

    prop_step=1;
    EKF_propagation();  // Kalman Filter Propagation step 1 ~10ms
    
    pos_I[0]=X_INS[2];
    pos_I[1]=X_INS[3];

//    if (EKF_convergence_OK) // controllers only allowed when EKF is ok
//    {
//      HEADING_controller(); // Calls the heading controller 10Hz with the filtered value of the heading
//      X_controller();
//      Y_controller();
//    }
//    else
//    {
//      digitalWrite(offBoardLED, LOW);
//      idle_state_PWMs();
//    }

    do_log=true;
    digitalWrite(debugPin2, LOW); //Debug on osciliscope
  }

  if (((millis() - actual_time_1hz) >= 1000)|| msg_xbeeOK ) // at 1Hz => replace the calculated value for the position with a position from the Location System
  {
    actual_time_1hz = millis();
//    digitalWrite(debugPin2, HIGH); //Debug on osciliscope

    if (msg_xbeeOK) // If there was a message from the Location System, replace it
    {
      parse_msg_xbee();
      msg_xbeeOK = false;
      
      //print_position_I(' ');
      //print_desired_position('\n');
      
      if (pos_LS[0] == -1 && pos_LS[1] == -1 && pos_LS[2] == -1)
      {
        //Something went wrong at the location system and the computer sent a stop message
        countTimeOut = 50;
        delay(100);
      }
      else
      {
        prop_step=0; // cancel any propagation that may be happening
        
        updt_step=1;
//        EKF_update(); // Kalman Filter Update  step 1 ~10ms
        
        //The message from computer was OK
        locationSystemLinkLost = 0;
        countTimeOut = 0;
      }
    }
    else // If there is no message, start a 3 seconds countdown to stop the quadrotor
    {
      //Serial.println("NO data arrived!");
      locationSystemLinkLost++;
      if (locationSystemLinkLost >= 3)
      {
        countTimeOut = 50;
        delay(100);
      }
    }
//    digitalWrite(debugPin2, LOW); //Debug on osciliscope
  }
}

