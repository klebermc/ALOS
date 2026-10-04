/*
void loop() {
  // empty
  contador++;
  Serial.println();
  Serial.println("Gravando dados no cartao SD...");
  
  sd_open_file("log_data.txt");
  actual_time = millis();
  write_on_file(contador+0.1,contador+0.2,contador+0.3,contador+0.4,contador+0.5,contador+0.6);
  dt = millis() - actual_time;
  sd_close_file();
  
  delay(3000 - dt);
  Serial.println(dt);
}
*/
#define DEGUB_SD false //false to hide the prints

void init_SD_card()
{
  digitalWrite(ledPin, LOW);
  pinMode(pinCS, OUTPUT);
  
   // SD Card Initialization
  if (SD.begin())
  {
    if(DEGUB_SD) Serial.println("SD card is ready to use.");
    digitalWrite(ledPin, LOW);
    SD_ok = true;
  }
  else
  {
    if(DEGUB_SD) Serial.println("SD card initialization failed");
    digitalWrite(ledPin, HIGH);
    SD_ok = false;
  }
}

void sd_open_file(const char * filename)
{
  if(SD_ok)
  {
    // Create/Open file 
    myFile = SD.open(filename, FILE_WRITE);
    if(myFile){}
    else SD_ok = false;
  }
}  

void write_on_file(unsigned int count,float value1,float value2,float value3,float value4,float value5,float value6,float value7,float value8,float value9,float value10)
{
  if(SD_ok)
  {
    // if the file opened okay, write to it:
    if (myFile)
    {
      if(DEGUB_SD) Serial.println("Writing to file...");
      // Write to file

      myFile.print(count);
      myFile.print("\t");
      myFile.print(value1, 4);
      myFile.print("\t");
      myFile.print(value2, 4);
      myFile.print("\t");
      myFile.print(value3, 4);
      myFile.print("\t");
      myFile.print(value4, 4);
      myFile.print("\t");
      myFile.print(value5, 4);
      myFile.print("\t");
      myFile.print(value6, 4);
      myFile.print("\t");
      myFile.print(value7, 4);
      myFile.print("\t");
      myFile.print(value8, 4);
      myFile.print("\t");
      myFile.print(value9, 4);
      myFile.print("\t");
      myFile.println(value10, 4);
            
      if(DEGUB_SD) Serial.println("Done.");
      digitalWrite(ledPin, LOW);
    }
    else // if the file didn't open, print an error:
    {
      if(DEGUB_SD) Serial.println("error file");
      digitalWrite(ledPin, HIGH);
      countTimeOut=50;
      delay(100);
    }
  }
}

void write_on_file(unsigned int count,float value1,float value2,float value3,float value4,float value5,float value6)
{
  if(SD_ok)
  {
    // if the file opened okay, write to it:
    if (myFile)
    {
      if(DEGUB_SD) Serial.println("Writing to file...");
      // Write to file

      myFile.print(count);
      myFile.print("\t");
      myFile.print(value1, 4);
      myFile.print("\t");
      myFile.print(value2, 4);
      myFile.print("\t");
      myFile.print(value3, 4);
      myFile.print("\t");
      myFile.print(value4, 4);
      myFile.print("\t");
      myFile.print(value5, 4);
      myFile.print("\t");
      myFile.println(value6, 4);
            
      if(DEGUB_SD) Serial.println("Done.");
      digitalWrite(ledPin, LOW);
    }
    else // if the file didn't open, print an error:
    {
      if(DEGUB_SD) Serial.println("error file");
      digitalWrite(ledPin, HIGH);
      countTimeOut=50;
      delay(100);
    }
  }
}

void read_from_file()
{
  if(SD_ok)
  {
    // Reading the file
    myFile = SD.open("log_data.txt");
    if (myFile)
    {
      Serial.println("Read:");
      // Reading the whole file
      while (myFile.available()) 
      {
        Serial.write(myFile.read());
        digitalWrite(ledPin, LOW);
      }
      myFile.close();
    }
    else
    {
      Serial.println("error file");
      digitalWrite(ledPin, HIGH);
    }
  }
}
void sd_close_file()
{
  if(SD_ok)
    myFile.close(); // close the file
}

