#include <VirtualWire.h>
#include "printf.h"

#define emitter_number 1        // The emitter number

#define pin_in_rf_433 2         // 433MHz rf signal input
#define pin_emit_sig_us 3       // Ultrasonic signal
#define pin_enableCircuit 5     // Enables the circuit
#define pin_debug 9             // Debug port for osciloscope 
#define pin_led 13              // Onboard LED for visual debug
#define baud_rf_433 5000        // Bauld rate of 433Mhz communication

/* Global Variables */
short int i;
//int  received_value_RF;
//char received_RF_char[4];
bool state_debug_pin = false;
bool signal_emitted = false;

uint8_t buf[1];
uint8_t buflen = 1;

void debug()
{
  if(state_debug_pin)
    digitalWrite(pin_debug,LOW);
  else
    digitalWrite(pin_debug,HIGH);
  state_debug_pin=!state_debug_pin;
}

/* Ultrasonic signal and timer related functions */
void start_transducer()
{
  /*
  TCNT2 = 0;
  OCR2A = 49; // valor max de tempo, periodo da onda enviada
  OCR2B = 25; // valor do tempo de meia onda, tempo de sinal alto, 50% de duty cycle
  TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(WGM22) | _BV(CS21);
  */
  // /8 de prescaler
  // CTC mode
  // Toggle OC2B on Compare Match
  TCNT2 = 0;
  OCR2A = 24;
  OCR2B = 24; // Half-wave time value, high signal time, 50% duty cycle
  TCCR2A = _BV(COM2B0) | _BV(WGM21);
  TCCR2B = _BV(CS21);
}

void stop_transducer()
{
  //OCR2A=0;
  //OCR2B=0;
  TCNT2 = 0; // The counter register
  TCCR2A = 0;
  TCCR2B = 0; // I think this is all that is needed since setting the CS bits to zero stops the timer.
}

void send_us_signal()
{
    digitalWrite(pin_enableCircuit,LOW);
    delayMicroseconds(25);
    debug();
    start_transducer();
    delayMicroseconds(25*8-5);
    stop_transducer();
    //digitalWrite(pin_emit_sig_us,HIGH);
    digitalWrite(pin_enableCircuit,HIGH);
    debug();
    //delayMicroseconds(25*200);
}

/* i */
void init_rf_433()
{
    //Pin attached to the DATA pin of the RF receiver
    vw_set_rx_pin(pin_in_rf_433);
    //Communication speed (bits per second)
    vw_setup(baud_rf_433);
    //The reception starts
    vw_rx_start();
}

void input_rf_433()
{
  //Dont answer another signal, if I have already attended a radiofrequency signal
  if(!signal_emitted)
  {
      signal_emitted=true;
      send_us_signal();
  }
}


void setup() {
    Serial.begin (9600);
    printf_begin();

    pinMode(pin_emit_sig_us, OUTPUT);
    
    pinMode(pin_led, OUTPUT);

    pinMode(pin_enableCircuit, OUTPUT);
    //Leaves the circuit off
    digitalWrite(pin_enableCircuit, HIGH);

    /*//attach the interruption on RF 433MHz pin
    pinMode(pin_in_rf_433,INPUT);
    attachInterrupt(digitalPinToInterrupt(pin_in_rf_433), input_rf_433, RISING);
    */
    //Starting the 433MHz module
    init_rf_433();

    printf("US EMITTER MODULE INITIALIZED!\n\r");
}

void loop() {
    if (vw_get_message(buf, &buflen))
    {
      /*
       for (i = 0; i < buflen; i++)
       {
          Armazena os caracteres recebidos
          received_RF_char[i] = char(buf[i]);
       }
       received_RF_char[buflen] = '\0';

       //Converte o valor recebido para integer
       received_value_RF = atoi(received_RF_char);
       if (received_value_RF == emitter_number)
       */
        //received_RF_char[0] = char(buf[0]);
    if (buf[0] == emitter_number)
       {
           digitalWrite(pin_led, HIGH);
           //Sends the ultrasonic signal
           input_rf_433();
    
           //Prints the received value at the serial monitor
           //printf("Received: %d - Signal sent!\n\r",buf[0]);
           signal_emitted=false;

           digitalWrite(pin_led,LOW);
           //delay(50);
       }
       else
       {
           //Prints the received value at the serial monitor
           //printf("Received: %d - No Signal!\n\r",buf[0]);
       }
    }
}
