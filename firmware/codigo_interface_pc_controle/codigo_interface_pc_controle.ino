/* Code created by
    Kleber Macedo Cabral
    Mestrado - ITA - Brasil
*/
#define DEBUG

#define multiplier (F_CPU/8000000)  //leave this alone
#define CHANNEL_NUMBER 8  //set the number of chanels
#define CHANNEL_DEFAULT_VALUE 1500  //set the default servo value
#define FRAME_LENGTH 22000  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PULSE_LENGTH 400  //set the pulse length
#define COMMAND_LENGTH 50
#define onState 0  //set polarity of the pulses: 1 is positive, 0 is negative
#define ppm_pin_out 10  //set PPM signal output pin on the arduino
#define ppm_pin_in 3  //this must be 2 or 3
#define debug_pin 5

int ppm_out[CHANNEL_NUMBER];
int ppm_in[CHANNEL_NUMBER];
unsigned char inData[COMMAND_LENGTH],inChar;
int index;
boolean auto_controller[4];
unsigned int count_interrupts_t2=0;
boolean estado=false;
boolean ok_pc_values=false;

//leave this alone
static unsigned int pulse;
static unsigned long counter;
static byte channel;
static unsigned long last_micros;
        
enum PossibleStates {READING_PPM,GENERATING_PPM};
PossibleStates state;

/* Timer functions */
void init_timer1_generation()
{
    TCCR1A = 0; // set entire TCCR1 register to 0
    TCCR1B = 0;

    OCR1A = 100;  // compare match register, change this
    TCCR1B |= (1 << WGM12);  // turn on CTC mode
    TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
    TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
}

void init_timer2()
{
    TCCR2A = 0;
    TCCR2B = 0;
    TCNT2  = 0;

    //Coloca o timer para gerar interrupcao a cada 0.5 milisegundo
    OCR2A = 0xFF;
    TCCR2A |= (1 << WGM21);     // Set to CTC Mode
    TIMSK2 |= (1 << OCIE2A);    //Set interrupt on compare match
    TCCR2B |= (1 << CS21);      // prescaler 8 - increment every 0,5us
}

void setup(){

  Serial.begin(57600);
  Serial.println("Comeca");;
  
  //initiallize default ppm values
  for(int i=0; i<CHANNEL_NUMBER; i++)
  {
      ppm_out[i] = CHANNEL_DEFAULT_VALUE;
      ppm_in[i]  = CHANNEL_DEFAULT_VALUE;
  }

  //Output PPM
  pinMode(ppm_pin_out, OUTPUT);
  digitalWrite(ppm_pin_out, !onState);  //set the PPM signal pin to the default state (off)

  //Input PPM
  pinMode(ppm_pin_in, INPUT);
  attachInterrupt(digitalPinToInterrupt(ppm_pin_in), read_ppm, CHANGE);

  pinMode(debug_pin,OUTPUT);
  cli();
  init_timer1_generation();
  init_timer2();
  sei();
}

void loop(){
  //The data is received from USB asynchronously
  if (Serial.available() > 0)
  {
        if(index < COMMAND_LENGTH) // One less than the size of the array
        {
            inChar = Serial.read(); // Read a character
            inData[index] = '\0'; // Null terminate the string
            if (inChar!='i' && inChar!='f')
            {
              inData[index] = inChar; // Store it
              index++; // Increment where to write next
            }
            
            if (inChar=='i') 
              ok_pc_values=false;
            
            if (inChar=='f')
            {
                if (index == 8)
                {
                    ok_pc_values=true;
                }
                index=0;
            }
        }
  }
  //initiallize default ppm values
  for(int i=0; i<4; i++)
  {
      if (inData[i+4]=='a' && ok_pc_values)
          ppm_out[i]=(inData[i])*4 + 1000; //The channel must receive value from pc (auto mode)

      if (inData[i+4]=='m')
          ppm_out[i]=ppm_in[i]; //The channel must receive value from controller (manual mode)
  }
  /*
  delay(500);
  for(int i;i<CHANNEL_NUMBER;i++)
  {
    Serial.print(ppm_in[i]);
    Serial.print("  ");
  }
  Serial.println();
  */
  
}

ISR(TIMER1_COMPA_vect){  //leave this alone
  static boolean state = true;

  TCNT1 = 0;

  if (state) {  //start pulse
    digitalWrite(ppm_pin_out, onState);
    OCR1A = PULSE_LENGTH * 2;
    state = false;
  } else{  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;

    digitalWrite(ppm_pin_out, !onState);
    state = true;

    if(cur_chan_numb >= CHANNEL_NUMBER){
      cur_chan_numb = 0;
      calc_rest = calc_rest + PULSE_LENGTH;
      OCR1A = (FRAME_LENGTH - calc_rest) * 2;
      calc_rest = 0;
    }else{
      OCR1A = (ppm_out[cur_chan_numb] - PULSE_LENGTH) * 2;
      calc_rest = calc_rest + ppm_out[cur_chan_numb];
      cur_chan_numb++;
    }
  }
}

void read_ppm(){

        digitalWrite(debug_pin, estado);
        estado=!estado;

        counter = TCNT2;
        TCNT2 = 0;
        counter += (count_interrupts_t2*256); //Count in microseconds
        counter = counter/multiplier;
        count_interrupts_t2 = 0;

        if(counter < 600){  //must be a pulse if less than 710us
            pulse = counter;
        }else{
          if(counter > 1950){  //sync pulses over 1910us
              channel = 0;
              //#if defined(DEBUG)
                 //Serial.print("PPM Frame Len: ");
                 //Serial.println(micros() - last_micros);
                 //last_micros = micros();
              //#endif
          } else{
              //servo values between 600us and 2460us will end up here
              ppm_in[channel] = (counter + pulse);
              //#if defined(DEBUG)
                  //Serial.print(ppm_in[channel]);
                  //Serial.print("  ");
              //#endif
              channel++;
          }
        }
}

ISR(TIMER2_COMPA_vect)
{
    count_interrupts_t2+=1;
}
