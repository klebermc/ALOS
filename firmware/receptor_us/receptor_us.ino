#include <VirtualWire.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

#define receiver_number 3   // Receiver number

#define pin_in_rf_433 2     // 433MHz rf signal input
#define pin_in_sig_US 3     // Ultrasonic signal input
#define pin_ce_nrf24l01 7   // CE communication pin with NRF24L01
#define pin_csn_nrf24l01 8  // CSN communication pin with the NRF24L01
#define pin_debug 9         // Debug pin for osciloscope monitoring

#define baud_rf_433 5000    // Baudrate of rf communication 433MHz
#define max_time_waiting_nrf_in 2000 //miliseconds to wait for a incoming NRF24l01 message
#define MAX_MSG_SIZE 200    // Maximum number of characters of the output message containing the measured times
#define max_num_reads 10    // This value is the maximum number of readings that will be stored, until a communication through NRF24L01 gets the readings and clears the vectors

/* Global Variables */
uint8_t       distance2emitter_index[max_num_reads]; //this vector storages the emitter number for each distance measurement
unsigned long distance2emitter_times[max_num_reads]; //this vector storages the time measured for the ultrasonic signal, related with the emitter number at the above vector
uint8_t i, j, iterator_d2e, k;

short countInterrupts=0;
unsigned long uSeconds_passed=0;
bool state_debug_pin = false;
bool sig_us_measured = false;
bool timer_initialized = false;
bool done_reading_msg_nrf = false;
int time_waiting_nrf_in = 0;

char dest_receiver_number; //variable to store the destination receiver number
char message2send[MAX_MSG_SIZE];
char tempValueStorage[20];
unsigned char EOM='f';
unsigned char BOM='i';
unsigned char out_char='i';

enum PossibleStates {US_TRANSITION,DISTANCE_INFORMATION};

PossibleStates STATE;

uint8_t buf[1], buflen = 1;

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10
RF24 radio_nrf24l01(pin_ce_nrf24l01,pin_csn_nrf24l01);

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

void debug()
{
  if(state_debug_pin)
    digitalWrite(pin_debug,LOW);
  else
    digitalWrite(pin_debug,HIGH);
  state_debug_pin=!state_debug_pin;
}

/* Timer functions */
void init_timer()
{
    TCCR2A = 0;
    TCCR2B = 0;
    TCNT2  = 0;

    //Sets the timer to overflow at each 1ms
    OCR2A = 249;
    TCCR2A |= (1 << WGM21);   // Set to CTC Mode
    TIMSK2 |= (1 << OCIE2A);    //Set interrupt on compare match
    TCCR2B |= (1 << CS22); // prescaler 64
}

void stop_timer()
{
  TCCR2A = 0;
  TCCR2B = 0; // I think this is all that is needed since setting the CS bits to zero stops the timer.
  TIMSK2 = 0;
  TCNT2 = 0; // Clear the counter register
  countInterrupts = 0;
  timer_initialized=false;
}

/* ultrassonic related functions */
void input_us()
{
  //If I have already attended an ultrasonic signal, I will not aswer it again (prevents the answering after the 15ms, called by loop function)
  if(!sig_us_measured)
  {

    //This function runs when an ultrasonic signal has just arrived.
    uSeconds_passed = countInterrupts*1000 + TCNT2*4;
    //uSeconds_passed = TCNT2;
    //time_values[0] = countInterrupts;
    //time_values[1] = TCNT2;

    //This flag indicate that I have already attended this ultrasonic 
    //signal and ignore the other interruptions generated
    sig_us_measured=true;

    //Stops the time counting
    stop_timer();
  }
}

/* Radio Frequency related functions */
void input_rf_433()
{
  //If I have already triggered the timer, I will not start again.
  if(!timer_initialized)
  {
      timer_initialized=true;
      init_timer();
  }
}

void init_rf_433()
{
    //Pin attached to the DATA pin of the RF receiver
    vw_set_rx_pin(pin_in_rf_433);
    //Communication speed (bits per second)
    vw_setup(baud_rf_433);
    //Starts the receiving through the radio
    vw_rx_start();
}

void init_rf_NRF24L01()
{
    // Setup and configure rf radio
    radio_nrf24l01.begin();

    // optionally, increase the delay between retries & # of retries
    radio_nrf24l01.setRetries(15,15);

    // optionally, reduce the payload size. seems to improve reliability
    //radio.setPayloadSize(8);

    // This simple sketch opens two pipes for these two nodes to communicate
    // back and forth.
    // Open 'our' pipe for writing
    // Open the 'other' pipe for reading, in position #1

    //(we can have up to 5 pipes open for reading)
    radio_nrf24l01.openWritingPipe(pipes[1]);
    radio_nrf24l01.openReadingPipe(1,pipes[0]);

    // Dump the configuration of the rf unit for debugging
    radio_nrf24l01.printDetails();

    radio_nrf24l01.startListening();
}

/* Interruption handler function */
ISR (TIMER2_COMPA_vect)
{
  //Enter this function every 1 millisecond
  countInterrupts+=1;
}

void setup()
{
  Serial.begin(9600);
  printf_begin();

  //interruption on digital pin 2 of Arduino nano for signal US
  pinMode(pin_in_sig_US,INPUT);
  attachInterrupt(digitalPinToInterrupt(pin_in_sig_US), input_us, RISING);

  /*
   * This feature is not been used because the output of the 433MHz radio receiver is presenting high noise, 
   * although, the protocol implemented at the VirtualWires library filters the noise and grab the message correctly
  //attach the interruption on RF 433MHz pin
  pinMode(pin_in_rf_433,INPUT);
  attachInterrupt(digitalPinToInterrupt(pin_in_rf_433), input_rf_433, RISING);
  */
  
  //Starting the 433MHz module
  init_rf_433();

  //Starting the NRF24L01 module
  init_rf_NRF24L01();

  //Puts the state machine to the wait state for RF communication
  //STATE = US_TRANSITION;

  i=0; 
  j=0; 
  iterator_d2e=0;
  k=0;
  
  EOM='f';
  BOM='i';  

  pinMode(pin_debug, OUTPUT);
  digitalWrite(pin_debug,LOW);
  
  printf("US RECEIVER MODULE INITIALIZED!\n\r");
}

void loop()
{
  // Data received through the 433MHz device
  if (vw_get_message(buf, &buflen))
  {
    state_debug_pin = false;
    debug();
    if (buf[0] !=0 )
    {
       //Starts the time count
       input_rf_433();
       sig_us_measured=false;

       delay(15); //Delay time to wait for the ultrasonic signal to arrive
       input_us(); //Here is to ensure that after 15ms (5 meters) the sensor stops waiting for the signal

       distance2emitter_index[iterator_d2e] = (uint8_t)buf[0];
       distance2emitter_times[iterator_d2e] = uSeconds_passed;
       iterator_d2e=(iterator_d2e+1)%max_num_reads;
     }
     stop_timer();

     //Prints the received value on the serial monitor
     printf("->US_TRANSITION!\n\r");
     printf("Received: %d\n\r", buf[0]); // number of the us emitter device
     printf("Tempo transcorrido: %lu us\n\r", uSeconds_passed);

     debug();
  }
  
  // Data received through the NRF24L01 device
  if ( radio_nrf24l01.available() )
  {
    state_debug_pin = false;
    debug();
    printf("->DISTANCE_INFORMATION!\n\r");

    dest_receiver_number = 0; //invalid receiver number

    done_reading_msg_nrf = false; //stopped working after library update
    //while (!done_reading_msg_nrf) //stopped working after library update
    while (radio_nrf24l01.available())
    {
        // Fetch the payload, and see if this was the last one.
        //done_reading_msg_nrf = radio_nrf24l01.read( &dest_receiver_number, sizeof(char) ); //stopped working after library update
        radio_nrf24l01.read( &dest_receiver_number, sizeof(char) );
          
        // Print it
        //printf("Incoming destination number [ %X ]\n\r",dest_receiver_number);
    }
    //delay(10); //Delay just a little bit to let the other unit make the transition to receiver
    
    //Answer only if the message was for me
    if (dest_receiver_number == receiver_number)
    {
        out_char='i';
        // First, stop listening so we can talk
        radio_nrf24l01.stopListening();

        radio_nrf24l01.write( &BOM, sizeof(char) ); //Begin of message
        
        //iterate through all the measurements
        for (j=0;j<iterator_d2e;j++)
        {
          
          //Formating the message to send
          strcpy(message2send,"");
          strcpy(tempValueStorage,"");
          itoa(distance2emitter_index[j],tempValueStorage,10);
          strcat(message2send,tempValueStorage);
          strcat(message2send,"-");
          strcpy(tempValueStorage,"");
          ltoa(distance2emitter_times[j],tempValueStorage,10);
          strcat(message2send,tempValueStorage);
          strcat(message2send,".");

          //Send the message
          for (k=0;k<strlen(message2send);k++)
          {
            out_char=message2send[k];
            radio_nrf24l01.write( &out_char, sizeof(char) ); //Message characters
            printf("[%c]-",out_char); //uncomment for debug
           }
          printf("[%s]-%d\n\r",message2send,strlen(message2send)); //uncomment for debug

          distance2emitter_index[j]=0;
          distance2emitter_times[j]=0;
        }
        
        radio_nrf24l01.write( &EOM, sizeof(char) ); //End of message
        printf("Sent response! Number of meas.=%d\n\r", iterator_d2e);
        iterator_d2e=0;
        
        // Now, resume listening so we catch the next packets.
        radio_nrf24l01.startListening();
    }
    debug();
  }
}//Loop end
