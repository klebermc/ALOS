#include <Wire.h>
#include <math.h>

#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38
#define X_MIN_PIN           3
#define X_MAX_PIN           2

#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
#define Y_MIN_PIN          14
#define Y_MAX_PIN          15

#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19

#define E_STEP_PIN         26
#define E_DIR_PIN          28
#define E_ENABLE_PIN       24

#define Q_STEP_PIN         36
#define Q_DIR_PIN          34
#define Q_ENABLE_PIN       30

#define SDPOWER            -1
#define SDSS               53
#define LED_PIN            13

#define FAN_PIN            9

#define PS_ON_PIN          12
#define KILL_PIN           -1

#define HEATER_0_PIN       10
#define HEATER_1_PIN       8
#define TEMP_0_PIN          13   // ANALOG NUMBERING
#define TEMP_1_PIN          14   // ANALOG NUMBERING

// Pinos referentes ao eletroima
#define LED_PIN            13
#define HEATER_0_PIN       10
#define step_division       4   //This is the step divisin configuration at the step motor shield
#define SATURATION_VEL     2.0  //2.0 [m/s]


//Tamanho do robo
#define   C   0.26   // [m]
#define   L   0.265  // [m]

//Frente esquerdo
unsigned long time_step_FE;
unsigned long conta_tempo_FE=0 ;

//Frente direito
unsigned long time_step_FD;
unsigned long conta_tempo_FD=0 ;

//Tras esquerdo
unsigned long time_step_TE;
unsigned long conta_tempo_TE=0 ;

//Tras direito
unsigned long time_step_TD;
unsigned long conta_tempo_TD=0 ;

boolean parar_robo=false;

float vel_X=0, vel_Y=0, vel_Wz=0;
float v1=0, v2=0, v3=0, v4=0;

unsigned int pos=0;
typedef void (*GeneralFunction) ();
char data[20], in_char;


// Interrupt
ISR(TIMER3_COMPA_vect)          // timer compare interrupt service routine
{
  conta_tempo_FE += 100;
  conta_tempo_FD += 100;
  conta_tempo_TE += 100;
  conta_tempo_TD += 100;
  
  digitalWrite(LED_PIN, digitalRead(LED_PIN) ^ 1);   // toggle LED pin
  
  if ( conta_tempo_FE >= time_step_FE)
  {
      conta_tempo_FE=0;
      if(v1!=0) movimento_translacional('F','E');
      else parar('F','E');
  }
  if (conta_tempo_FD >= time_step_FD)
  {
    conta_tempo_FD=0;
    if(v2!=0) movimento_translacional('F','D');
    else parar('F','D');
  } 
  if ( conta_tempo_TE >= time_step_TE)
  {
      conta_tempo_TE=0;
      if(v3!=0) movimento_translacional('T','E');
      else parar('T','E');
  }  
  if (conta_tempo_TD >= time_step_TD)
  {
    conta_tempo_TD=0;
    if(v4!=0) movimento_translacional('T','D');
    else parar('T','D');
  }
}

void setup()
{
  Serial.begin(57600);
  
  Wire.begin(9);
  // Attach a function to trigger when something is received.
  Wire.onReceive(receiveEvent);
  
  inicia_motores();
  inicia_timer();
    
  time_step_FE=6000;
  time_step_FD=6000;
  time_step_TE=6000;
  time_step_TD=6000;
}

void loop () {
    if (v1>=0) sentido_frente('F','E');
    else sentido_tras('F','E');
    
    if (v2>=0) sentido_frente('F','D');
    else sentido_tras('F','D');
    
    if (v3>=0) sentido_frente('T','E');
    else sentido_tras('T','E');
    
    if (v4>=0) sentido_frente('T','D');
    else sentido_tras('T','D');

    if(v1>SATURATION_VEL) v1=SATURATION_VEL;
    if(v2>SATURATION_VEL) v2=SATURATION_VEL;
    if(v3>SATURATION_VEL) v3=SATURATION_VEL;
    if(v4>SATURATION_VEL) v4=SATURATION_VEL;
        
    if(v1<-SATURATION_VEL) v1=-SATURATION_VEL;
    if(v2<-SATURATION_VEL) v2=-SATURATION_VEL;
    if(v3<-SATURATION_VEL) v3=-SATURATION_VEL;
    if(v4<-SATURATION_VEL) v4=-SATURATION_VEL;
            
    calcula_time_step(abs(v1),abs(v2),abs(v3),abs(v4));
    //Serial.println(v1);Serial.println(v2);Serial.println(v3); Serial.println(v4);
    
    delay(10);
//    sentido_tras('D');
//    sentido_frente('E');
//    calcula_time_step (0.2,0.01);delay(1000);

//    sentido_frente('D');
//    sentido_tras('E');
//    calcula_time_step (0.05,0.1);delay(1000); 
}

void inicia_timer ()
{
  noInterrupts();           // disable all interrupts
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;
  OCR3A = 1599;//799;            // compare match register 16MHz/1/10kHz
  TCCR3B |= (1 << WGM32);   // CTC mode
  TCCR3B |= (1 << CS30);    // 1 prescaler 
  TIMSK3 |= (1 << OCIE3A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void calcula_time_step(float vel_roda_FE, float vel_roda_FD,float vel_roda_TE,float vel_roda_TD)
{
  time_step_FE = 198.4/(pow(vel_roda_FE, 1.082)*step_division);
  time_step_FD = 198.4/(pow(vel_roda_FD, 1.082)*step_division);
  time_step_TE = 198.4/(pow(vel_roda_TE, 1.082)*step_division);
  time_step_TD = 198.4/(pow(vel_roda_TD, 1.082)*step_division);

//  Serial.print("tsFE=");Serial.print(time_step_FE);Serial.println();
//  Serial.print("tsFD=");Serial.print(time_step_FD);Serial.println();
//  Serial.print("tsTE=");Serial.print(time_step_TE);Serial.println();
//  Serial.print("tsTD=");Serial.print(time_step_TD);Serial.println();
}

