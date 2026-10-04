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
#define step_division       2   //This is the step divisin configuration at the step motor shield
#define SATURATION_VEL     2.0  //2.0 [m/s]

unsigned long time_step_E;
unsigned long conta_tempo_E=0 ;
unsigned long time_step_D;
unsigned long conta_tempo_D=0 ;
boolean parar_robo=false;
float vel_direita=0, vel_esquerda=0;
unsigned int pos=0;
typedef void (*GeneralFunction) ();
char data[20], in_char;


// Interrupt
ISR(TIMER3_COMPA_vect)          // timer compare interrupt service routine
{
  conta_tempo_E += 100;
  conta_tempo_D += 100;
  
  digitalWrite(LED_PIN, digitalRead(LED_PIN) ^ 1);   // toggle LED pin
  
  if ( conta_tempo_E >= time_step_E)
  {
      conta_tempo_E=0;
      if(vel_esquerda!=0) movimento_translacional('E');
      else parar('E');
  }
  
  if (conta_tempo_D >= time_step_D)
  {
    conta_tempo_D=0;
    if(vel_direita!=0) movimento_translacional('D');
    else parar('D');
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
    
  time_step_E=6000;
  time_step_D=6000; 
}

void loop () {
    if (vel_direita>=0) sentido_frente('D');
    else sentido_tras('D');

    if (vel_esquerda>=0) sentido_frente('E');
    else sentido_tras('E');

    if(vel_direita>SATURATION_VEL)  vel_direita=SATURATION_VEL;
    if(vel_esquerda>SATURATION_VEL) vel_esquerda=SATURATION_VEL;
    if(vel_direita<-SATURATION_VEL)  vel_direita=-SATURATION_VEL;
    if(vel_esquerda<-SATURATION_VEL) vel_esquerda=-SATURATION_VEL;
            
    calcula_time_step(abs(vel_direita),abs(vel_esquerda));

    delay(10);
//    sentido_tras('D');
//    sentido_frente('E');
//    calcula_time_step (0.2,0.01);delay(1000);
//    
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

void calcula_time_step(float vel_roda_D,float vel_roda_E)
{
  time_step_D = 198.4/(pow(vel_roda_D, 1.082)*step_division);
  time_step_E = 198.4/(pow(vel_roda_E, 1.082)*step_division);

//  Serial.print("tsD=");Serial.print(time_step_D);Serial.println();
//  Serial.print("tsE=");Serial.print(time_step_E);Serial.println();
}

