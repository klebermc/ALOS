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
#include <Wire.h>
// ***************************************************************************

// *********************** DEFINITION OF MACROS ******************************
#define      MAX_MSG_SIZE   200        // message maximum length
#define                dt   0.100      // time interval between accel readins, step used at euler's integration
#define   desired_HEADING   0          // desired value of heading, this is the value read by the magnetometer when we place the body X vector overlapping with the inertial X axis
#define  TIMEOUT_MSG_XBEE   100        // maximum time in milliseconds to wait for a xbee message to be fully received
#define         debugPin1   11         // pin set to debug
#define         debugPin2   12         // pin set to debug
#define         LOG_ON_SD   false       // this flag enables the code to log the position into the SD card
#define       offBoardLED   31         // the pin port for the offboard green led, placed on quadrotor frame
// *************************************************************************

// ***************** DEFINITION OF FUNCTIONS AS MACROS *********************
// The register value and it's respective ouput pin. These "functions" receive the DESIRED PULSE WIDTH IN MICROSECONDS
// then it is converted to the respective values for the timer. Inside the timer:
//    OCRnm = 2000 -> duty cycle 5%
//    OCRnm = 4000 -> duty cycle 10%
//#define SET_PWM_PIN2(x)     OCR3B = 2*x
//#define SET_PWM_PIN3(x)     OCR3C = 2*x
//#define SET_PWM_PIN5(x)     OCR3A = 2*x
//#define SET_PWM_PIN6(x)     OCR4A = 2*x
//#define SET_PWM_PIN7(x)     OCR4B = 2*x
//#define SET_PWM_PIN8(x)     OCR4C = 2*x

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
#define noise_ax_INS (0.140)
#define noise_ay_INS (0.140)
#define noise_wz_INS (0.07)

//%Vector N_LS (standard deviations)
#define noise_px_LS  (0.1*5)
#define noise_py_LS  (0.1*5)
#define noise_psi_comp  0.01

#define beta 0.0001      // parameter to avoid filter divergence

float Bias[3];  //initialization of the bias estimative needed by KF
float G[8][3];  // kalman gain matrix
float P0[8][8]; // initial state error covariance matrix
float P[8][8];  // covariance error matrix
float R[3][3];  // covariance matrix of Location System and Compass N_LS
float Q[3][3];  // process covariance matrix
float X_INS[5] = {0, 0, 0, 0, 0}; //States X_INS(t) = [Vx(t),Vy(t), Px(t), Py(t), ψ(t)]
float U[3];     // input vector for kalman
float S_inv[3][3]; // S is the measurement prediction covariance, used at the validation gate (auxiliar measurement is ok or not), S_inv is inv(S)
float dM[1] = {0}; //Mahalanobis distance between the predicted measurement of Y_E and the actual measurement! YE = [LSx, LSy, Psi comp]'

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

float LOS, error_psi;
boolean did_update=false;
int x_est_signal=1;

char msg[20];                                 //message to the robot embeeded Arduino 
float desired_velY, desired_velX, desired_wz; //desired velocity to each component of the robot

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
#define ledPin 13   // This pins goes HIGH when there is sth wrong with the log
#define pinCS 53    // Chip select for SPI comm
File myFile;        // The file object
boolean SD_ok;      // Indicates the SD comm is OK
int count = 0;      //Used to know when to close and reopen the file

float variables2save[21];
float variables2save_PREUPDATE[21];
//----------------------------------------------------------------

//******************************************************************************

//int contador=0;


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
    while(true);
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

  Serial.begin(57600);  // USB communication
  Serial1.begin(57600); // AHRS communication
  Serial2.begin(57600);  // XBee communication

//contador=0;

  EKF_initialization();

  init_PWMs();
    
  // *** Verification of the SD card ***
  if(LOG_ON_SD) //setup the file for LOG
  {
    init_SD_card();
    sd_open_file("log_data.txt");
    myFile.println("count\tX_sig\tPx_est\tPy_est\tpsi_est\tbiasax\tbiasay\tbiaswz\tUax\tUay\twz\tLSx\tLSy\tpsi_comp\tLOS\terror_psi\tPz\tdesired_velX\tdesired_velY\tax\tay");
    sd_close_file();

    while(!SD_ok) //Holds here if it is desired to do the log but the communication with the SD card is not OK
      blink_offboard_led(4);// "Morse code" for the user to know the problem was at SD card

    //Blinks to indicate that the program will proceed
    digitalWrite(ledPin, HIGH);    delay(500);
    digitalWrite(ledPin, LOW);    delay(500);
  }
  // ****************************************
    
  // *** Sending a 0,0 vel to robot ***
  initialize_controllers();  // Set up the gains for the controllers
  try_comm_with_embeeded_arduino();
  Serial.println("Communication with Arduino motor board OK!");
  // ****************************************
  
  // *** Verification of the IMU connection ***
  //The IMU must be connect, and the startup heading value must be inside of 5 degrees range for this procedure to pass
  Serial.println("Wainting IMU calibration");
  X_INS[4] = deg2rad(verify_IMU_connection());
  Serial.println("IMU measurements available!");
  // ****************************************

  // *** Waits for a message through Xbee to start ***
  digitalWrite(offBoardLED, HIGH);
  while (!msg_xbeeOK)
  {
    receive_data_xbee(); // Holds waiting for XBee message to start the program (actual pos and waypoint);
    delay(10);
  }
    parse_msg_xbee(); //using the value of the position at the initialization
    //This is where the location system values for position will replace the integrated from IMU
    X_INS[2] = pos_LS[0]; X_INS[3] = pos_LS[1];
    msg_xbeeOK=false;
  Serial.println("Xbee message received!");
  digitalWrite(offBoardLED, LOW);
  // ****************************************
  
  TIMSK4 = (1<<OCIE4C); // activate the interruption for timeouts
  
  actual_time_10hz = millis();
  actual_time_50hz = actual_time_10hz;
  actual_time_1hz  = actual_time_10hz;
}

void loop()
{
  receive_data_ahrs(); // Try to read data comming from the AHRS

  receive_data_xbee(); // Try to read the data from XBee

  if ((millis() - actual_time_10hz) >= 110 || new_data_ahrs) // Call the position controllers at 10Hz
  {
    actual_time_10hz = millis();
    digitalWrite(debugPin1, HIGH); //Debug on osciliscope
    digitalWrite(offBoardLED, !digitalRead(offBoardLED));

    //Simulated failure of SILA and compass
    //if((contador>30)&&(contador<=60))digitalWrite(offBoardLED, LOW);
    //if((contador>100)&&(contador<=130))digitalWrite(offBoardLED, LOW);
    
    // Retrieve data from AHRS
    if (new_data_ahrs) {
      parse_msg_ahrs();   //Retrieve accel and heading information from the image
      countTimeOut = 0; //reset the timeout every time it has IMU data
      new_data_ahrs = false;
    }

    SATURATION(actual_roll , 10, -10);
    SATURATION(actual_pitch, 10, -10);
    SATURATION(actual_HEADING, 180, -180);

    //print_estimated_states('\n');
    //print_position_I('\n');
    //print_IMU_data('\n');

    digitalWrite(debugPin1, LOW); delay(1);
    // ------ Running the EKF ------
    //*** Udate ***  
    if(updt_step==1) //every time there was a new message from LS, runs the Update
    { 
      digitalWrite(debugPin1, HIGH);
      did_update=true;
      variables2save_PREUPDATE[0]=iterator;
      variables2save_PREUPDATE[1]= x_est_signal; //if the X_INS is X- (after prop) or X+ (after update)
      variables2save_PREUPDATE[2]= X_INS[2]; //Px
      variables2save_PREUPDATE[3]= X_INS[3]; //Py
      variables2save_PREUPDATE[4]= rad2deg(X_INS[4]); //Psi estimated
      variables2save_PREUPDATE[5]= Bias[0];
      variables2save_PREUPDATE[6]= Bias[1];
      variables2save_PREUPDATE[7]= Bias[2];
      variables2save_PREUPDATE[8]= U[0]; //ax without bias
      variables2save_PREUPDATE[9]= U[1]; //ay without bias
      variables2save_PREUPDATE[10]= U[2]; //wz without bias [radians]
      variables2save_PREUPDATE[11]= pos_LS[0]; //LSx
      variables2save_PREUPDATE[12]= pos_LS[1]; //LSy
      variables2save_PREUPDATE[13]= actual_HEADING; //Psi comp
      variables2save_PREUPDATE[14]= LOS;   //Phi roll
      variables2save_PREUPDATE[15]= error_psi;  //Theta pitch
      variables2save_PREUPDATE[16]= 0; //Pz
      variables2save_PREUPDATE[17]= desired_velX/1000; //robot's linear speed X body
      variables2save_PREUPDATE[18]= desired_velY/1000; //robot's linear speed Y body
      variables2save_PREUPDATE[19]= accel_b[0]; //Z control, thrust value
      variables2save_PREUPDATE[20]= accel_b[1]; //HD control, desired heading value

       //if((contador>30)&&(contador<=60)){ updt_step=0; digitalWrite(offBoardLED, LOW);}
       //if((contador>100)&&(contador<=130)){ updt_step=0;digitalWrite(offBoardLED, LOW);}
    
      //Next time this task runs after the 1Hz task, do the second step of the kalman filter update
      EKF_update(); // Kalman Filter Update  step 1 ~10ms
      updt_step=2;

      //Simulated failure of SILA and compass
      //if((contador>30)&&(contador<=60)){ updt_step=0; digitalWrite(offBoardLED, LOW);}
      //if((contador>100)&&(contador<=130)){ updt_step=0;digitalWrite(offBoardLED, LOW);}
      
      EKF_update(); // Kalman Filter Update  step 2 ~10ms
      updt_step=0;
      x_est_signal=1;
      digitalWrite(debugPin1, LOW); delay(1);
    }else  did_update=false;
    //*** END Udate ***
    // -----------------------------------    

    digitalWrite(debugPin1, HIGH);
    // ------ Running the Controllers ------
    if (EKF_convergence_OK) // controllers only allowed when EKF is ok
    {
      POS_controller();
    }
    else
    {
      digitalWrite(offBoardLED, LOW);
    }
    // -----------------------------------
    digitalWrite(debugPin1, LOW); delay(1);
    //Serial.print(desired_velX);Serial.print(" "); Serial.println(desired_velY);

    digitalWrite(debugPin1, HIGH);
    variables2save[0]=iterator;
    variables2save[1]=x_est_signal; //if the X_INS is X- (after prop) or X+ (after update)
    variables2save[2]=X_INS[2]; //Px
    variables2save[3]=X_INS[3]; //Py
    variables2save[4]=rad2deg(X_INS[4]); //Psi estimated
    variables2save[5]=Bias[0];
    variables2save[6]=Bias[1];
    variables2save[7]=Bias[2];
    variables2save[8]=U[0]; //ax without bias
    variables2save[9]=U[1]; //ay without bias
    variables2save[10]=U[2]; //wz without bias [radians]
    variables2save[11]=variables2save_PREUPDATE[11]; //LSx
    variables2save[12]=variables2save_PREUPDATE[12]; //LSy
    variables2save[13]=variables2save_PREUPDATE[13]; //Psi comp
    variables2save[14]=LOS;   //Phi roll
    variables2save[15]=error_psi;  //Theta pitch
    variables2save[16]=0; //Pz
    variables2save[17]=desired_velX/1000; //X control
    variables2save[18]=desired_velY/1000; //Y control
    variables2save[19]=accel_b[0]; //Z control, thrust value
    variables2save[20]=accel_b[1]; //HD control, desired heading value
      
    // ------ Running the EKF ------
    //*** Propagation (X[k+1] and P[k+1])***
    prop_step=1;
    EKF_propagation();  // Kalman Filter Propagation step 1 ~10ms
    prop_step=2;
    EKF_propagation();  // Kalman Filter Propagation  step 2 ~10ms
    prop_step=0;
    x_est_signal=-1;
    //*** END Propagation ***
    // -----------------------------------
   
    print_P_diag();
    EKF_CheckConvergence();
    digitalWrite(debugPin1, LOW); delay(1);
   
    // ------ Saving on the SD Card ------
    if (LOG_ON_SD) // Log the calculed position to the SD Card
    {
      digitalWrite(debugPin1, HIGH);
      //Avoids to keep openning and closing the file all the time
      if (count == 0) sd_open_file("log_data.txt");
      iterator++;
      if (did_update)  write_on_file(21,variables2save_PREUPDATE);
      write_on_file(21,variables2save);
      if (count == 9) sd_close_file();
      count = (count+1) % 10;
    }
    // -----------------------------------
    
    digitalWrite(debugPin1, LOW); //Debug on osciliscope
  }

  if (((millis() - actual_time_1hz) >= 1000) || msg_xbeeOK ) // at 1Hz => replace the calculated value for the position with a position from the Location System
  {
    actual_time_1hz = millis();
    digitalWrite(debugPin2, HIGH); //Debug on osciliscope
//    contador++;
    if (msg_xbeeOK) // If there was a message from the Location System, replace it
    {
      parse_msg_xbee();
      msg_xbeeOK = false;
      
      //print_position_I(' ');
      //print_desired_position('\n');
      
      if (pos_LS[0] == -1 && pos_LS[1] == -1 && pos_LS[2] == -1)
      {
        //Something went wrong at the location system and the computer sent a stop message
        
        sprintf(msg,"i%d,%df",(int)0,(int)0);
        //Serial.print("<");Serial.print(msg);Serial.println(">");
        
        byte i=0;  
        Wire.beginTransmission(9); // transmit to device #9  
        while(msg[i]!=0)
        {
          Wire.write(msg[i]);              // sends x 
          i++;
          if (i>=20) break;
        }
        Wire.endTransmission();    // stop transmitting 
        
        countTimeOut = 51;
        delay(100);
      }
      else
      {
        //The message from computer was OK
        updt_step=1;
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
        
         sprintf(msg,"i%d,%df",(int)0,(int)0);
        //Serial.print("<");Serial.print(msg);Serial.println(">");
        byte i=0;  
        Wire.beginTransmission(9); // transmit to device #9  
        while(msg[i]!=0)
        {
          Wire.write(msg[i]);              // sends x 
          i++;
          if (i>=20) break;
        }
        Wire.endTransmission();    // stop transmitting 
        
        countTimeOut = 51;
        delay(100);
      }
    }
    digitalWrite(debugPin2, LOW); //Debug on osciliscope
  }
}

