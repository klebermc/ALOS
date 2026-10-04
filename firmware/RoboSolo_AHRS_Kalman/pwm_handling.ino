void init_PWMs()
{
  //Defining the ports as outputs
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  
  noInterrupts();           // disable all interrupts

  // initialize timer3 - PIN2, PIN3, PIN5
  OCR3A = 2000;
  OCR3B = 2000;
  OCR3C = 2000;
  ICR3  = 39999;
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;
  TCCR3A = _BV(COM3A1) | _BV(COM3B1) | _BV(COM3C1) | _BV(WGM31) ;
  TCCR3B = _BV(WGM33) | _BV(WGM32) |_BV(CS31);
  
  // initialize timer4 - PIN6, PIN7, PIN8
  OCR4A = 2000;
  OCR4B = 2000;
  OCR4C = 2000;
  ICR4  = 39999; // MAX VALUE, 50Hz
  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4 = 0;
  TCCR4A = _BV(COM4A1) | _BV(COM4B1) | _BV(COM4C1) | _BV(WGM41) ;
  TCCR4B = _BV(WGM43) | _BV(WGM42) |_BV(CS41);

  interrupts();             // enable all interrupts
}


