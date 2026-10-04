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

void write_on_file(unsigned int count,float pos_x,float pos_y,float pos_z,float vel_x,float vel_y,float vel_z)
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
      myFile.print(pos_x, 4);
      myFile.print(" ");
      myFile.print(pos_y, 4);
      myFile.print(" ");
      myFile.print(pos_z, 4);
      myFile.print(" ");
      myFile.print(vel_x, 4);
      myFile.print(" ");
      myFile.print(vel_y, 4);
      myFile.print(" ");
      myFile.println(vel_z, 4);
      
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

