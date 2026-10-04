/*
This code works over a state machine, the states:
US_TRANSITION : send the rf433 signal for the devices
DISTANCE_INFORMATION : send the request for each receiver to the measured distance
WAITING_COMMAND : this state waits for a incoming message from a serial port (USB)
*/


#include <VirtualWire.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

#define pin_out_rf_433 4      // 433MHz rf signal input
#define pin_csn_nrf24l01 8    // CSN communication pin with the NRF24L01
#define pin_ce_nrf24l01 9     // CE communication pin with NRF24L01
#define pin_debug 10          // Debug pin for osciloscope monitoring

#define baud_rf_433 5000      // Baudrate of rf communication 433MHz
#define num_rcv_dev 1         // Number of receiver devices
#define num_emt_dev 8         // Number of emitter devices
#define MAX_MSG_SIZE 200      // Maximum input message size, containing the times read by the US_Receiver_Device

boolean DEBUG=false;            //This flag enables the code to print more information
#define COMM_MATLAB true      //This flag enables the printing with matlab format to the operation mode
#define TIME_MS_DEBUG 3000     //This flag defines the period of the signal to be sent in DEBUG mode


/* Global Variables */
byte msg = 0;
int i=0;
bool state_debug_pin = false;
unsigned long started_waiting_at;
unsigned long uSeconds_passed;
bool timeout_nrf_msg;
uint8_t em_dev=0, rec_dev=0, endFor=0;
char command_in[5];
int contador;
long stateStartTime;
char dest_receiver_number;
uint8_t n_measurements=0;

//Variables used at the NRF24L01 message handling
uint8_t j, pos_vet=0, cont_timeout=0;
char in_msg[MAX_MSG_SIZE];
char in_char='i';

long started_here; //Variable for debug porposes 
unsigned short contTimeOuts;

enum PossibleStates {US_TRANSITION,DISTANCE_INFORMATION,WAITING_COMMAND};
PossibleStates STATE;

/* Radio Object */ 
// Set up nRF24L01 radio on SPI bus plus ce and csn
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

/* Initialization functio for RF 433MHz radio */
void init_rf_433()
{
    //Pin attached to the DATA pin of the RF transmitter
    vw_set_tx_pin(pin_out_rf_433);
    //Communication speed (bits per second)
    vw_setup(baud_rf_433);
}

/* Initialization functio for RF NRF24L01 radio */
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
    radio_nrf24l01.openWritingPipe(pipes[0]);
    radio_nrf24l01.openReadingPipe(1,pipes[1]);

    // Dump the configuration of the rf unit for debugging
    //radio_nrf24l01.printDetails();

    radio_nrf24l01.startListening();
}

void setup()
{
  Serial.begin(57600);
  printf_begin();
  contador =0;

  if(COMM_MATLAB) DEBUG = false; //Just to be sure that the DEBUG flag is off for matlab config
  
  init_rf_433();

  init_rf_NRF24L01();

  started_here=millis();

  STATE = WAITING_COMMAND; //Initial state of the machine
  if(DEBUG) printf("Fixed Module Radio Frequency ready!\n\r");
}

void loop()
{
    /* This is the state machine */
    switch(STATE)
    {
        //Send a bit using the RF433MHz transmitter device for the emitters to start the ultrasonic signal
        case US_TRANSITION:
            debug();
            if (!COMM_MATLAB) stateStartTime = millis();
            
            if(DEBUG) printf("->US_TRANSITION - Send the RF433 signal to modules!\n\r");
            state_debug_pin=false;
            
            //Decide the start and stop values of the device iteration for
            if(em_dev==0)
              endFor = num_emt_dev+1; //Send US from all emitter devices
            else
              endFor=em_dev+1;//Send US from a specific device
            
            //iterator to send the message to us_emitter nodes
            for(i=em_dev;i<endFor;i++)
            {
                if(i==0) i++;
                msg=i;
                //Send the data
                vw_send((uint8_t *)&msg,1);
                //Wait for the data to be sent
                vw_wait_tx();
                //delay(50);
                //delay(10);
                //delay(30);
                delay(35);
                printf("SEND US SIGNAL --> Emitter Device = [%u]\n\r", msg);
            }
            if(COMM_MATLAB)
              STATE = WAITING_COMMAND;
            else
              STATE = DISTANCE_INFORMATION;  

            delay(50);
            if (!COMM_MATLAB) printf("US SIGNL -> %d [ms]\r\n\r\n\r\n", millis() - stateStartTime);
            debug();
        break;

        //Send a bit using the RF433MHz transmitter device for the emitters to start the ultrasonic signal
        case DISTANCE_INFORMATION:
            debug();
            /* ****** Noted here: the time for the this state to occur increase 100ms when the DEBUG flag is on ******* */
            if (!COMM_MATLAB) stateStartTime = millis();
            state_debug_pin=false;
             if(DEBUG) printf("->DISTANCE_INFORMATION - Questioning modules for times!\n\r");
            
            //Decide the start and stop values of the device iteration for
            if(rec_dev==0)
              endFor = num_rcv_dev+1;//Receive the US readings from all receiver devices
            else
              endFor=rec_dev+1; //Receive the US readings from a specific receiver device
            
            //iterator to send the message to the us_receiver nodes
            for(i=rec_dev;i<endFor;i++)
            {
                if(i==0) i++;

                dest_receiver_number = i;
                if(DEBUG) printf("RETRIEVE TIME READ --> Receiver Device = [%u]\n\r",dest_receiver_number); 
                 
                // Wait here until we get a response, or timeout (100ms)
                started_waiting_at = millis()-20;
                timeout_nrf_msg = false;
                contTimeOuts=0;
                while ( !radio_nrf24l01.available() && !timeout_nrf_msg )
                {
                    if (millis() - started_waiting_at >= 20 )
                    {
                      if (DEBUG) printf("-> NRF24L01 Timeout %d!\n\r",contTimeOuts);                      
                      radio_nrf24l01.stopListening(); // First, stop listening so we can talk.
                      radio_nrf24l01.write( &dest_receiver_number, sizeof(char) );  //Send the message. This blocks until complete
                      radio_nrf24l01.startListening(); // Now, start listening
                      started_waiting_at = millis();
                      timeout_nrf_msg = false;

                      if (contTimeOuts>4) timeout_nrf_msg = true;
                      contTimeOuts ++;
                    }
                }
                  
                // Describe the results
                if ( timeout_nrf_msg )
                {
                    printf("FAILED, response timed out NRF24L01!\n\r");
                }
                else
                {
                    // Grab the response
                    pos_vet=0;
                    cont_timeout=0;
                    in_char=0;
                    n_measurements=0;
                    strcpy(in_msg,"");
                    while(in_char!='f' && cont_timeout<250)
                    {
                      if (pos_vet>MAX_MSG_SIZE-2)
                      {
                        pos_vet=0;
                        break;
                      }
                      if(radio_nrf24l01.available())
                      {
                        radio_nrf24l01.read( &in_char, sizeof(char) ); //Grab one character
                      
                        if (in_char!='i' && in_char!='f' && in_char!=0)
                        {
                          in_msg[pos_vet]=in_char;
                          pos_vet=(pos_vet+1)%MAX_MSG_SIZE;
                          if(in_char=='-')n_measurements++;
                        }
                        if(cont_timeout>1)cont_timeout--;
                        //if(DEBUG) printf("[%c] ",in_char);
                      }
                      else
                      {
                        cont_timeout++;
                        //if(DEBUG) printf("\n\r sizeIN(%d) timeout(%d) ",pos_vet,cont_timeout);
                      }
                      delay(1);
                    }
                    in_msg[pos_vet]=0;
                    if (n_measurements!=0) printf("%s\n\r",in_msg);
                    
                    if(DEBUG) printf("%d - Measurements:[%d] - Msg Timeouts:[%d]\n\r",contador,n_measurements,cont_timeout);
                    contador++;
                }
            }

            STATE = WAITING_COMMAND;
            if (!COMM_MATLAB) printf("DIST INF -> %d [ms]\r\n", millis() - stateStartTime);
                       
            debug();
        break;

        case WAITING_COMMAND:
            if (COMM_MATLAB)
            {
              if (Serial.available() >= 3)
              {
                 uint8_t pos=0;
                 do{
                    command_in[pos] = toupper(Serial.read());
                    pos++;
                  } while( command_in[pos-1]!='!' && pos<5 );
                  command_in[pos-1]='\0';
                  char c1=command_in[0];
                  char c2=command_in[1];
                  
                  if(DEBUG) printf("->WAITING_COMMAND - Message at the serial!\n\r");
                  if(DEBUG) printf("[%s] - [%c] [%c]\n\r",command_in,c1,c2);
  
                  if (c1 == 'U')
                  {
                      if(DEBUG) printf("Ultrasonic signal transition!\n\r");
                      if (c2 == 'A') em_dev=0;
                      else           em_dev=c2-'0';
                      STATE = US_TRANSITION;
                  }
                  else if (c1 == 'D')
                  {
                      if(DEBUG) printf("Requesting measured distance!\n\r");
                      if (c2 == 'A') rec_dev=0;
                      else           rec_dev=c2-'0';
                      STATE = DISTANCE_INFORMATION;
                  }
              }
            }
            else
            {
              boolean timeout_teste=false;
              printf("[%d]",millis() - started_here);
              while ( !timeout_teste ){
                      if (millis() - started_here > TIME_MS_DEBUG )
                          timeout_teste = true;
              }
              em_dev=0;
              rec_dev=2;
              STATE = US_TRANSITION;
              
              printf(" - [%d]",millis() - started_here);
              printf("\n\r\n\r");
              started_here = millis();
            }
        break;
    }
}
