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
    else {
      if(DEGUB_SD) { Serial.print("SD card failed open file = "); Serial.println(filename); }
      digitalWrite(ledPin, HIGH);
      SD_ok = false;
    }
  }
}  

void write_on_file(unsigned int count,float a,float b,float c)
{
  if(SD_ok)
  {
    // if the file opened okay, write to it:
    if (myFile)
    {
      if(DEGUB_SD) Serial.println("Writing to file...");
      // Write to file

      myFile.print(count);
      myFile.print(" ");
      myFile.print(a, 4);
      myFile.print(" ");
      myFile.print(b, 4);
      myFile.print(" ");
      myFile.println(c, 4);
      
      if(DEGUB_SD) Serial.println("Done.");
      digitalWrite(ledPin, LOW);
    }
    else // if the file didn't open, print an error:
    {
      if(DEGUB_SD) Serial.println("error file");
      digitalWrite(ledPin, HIGH);
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

