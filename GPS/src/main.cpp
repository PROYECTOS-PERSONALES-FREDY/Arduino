#include <Arduino.h>
#include <TinyGPS++.h>

#define RXD2 9
#define TXD2 10

HardwareSerial neogps(1);
TinyGPSPlus gps;


char dato=' ';

void setup()
{
 Serial.begin(9600);           
 neogps.begin(115200, SERIAL_8N1, RXD2, TXD2); 
}


void loop() {
    
  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (neogps.available())
    {
      if (gps.encode(neogps.read()))
      {
        newData = true;
      }
    }
  }

  //If newData is true
  if(newData == true)
  {
    newData = false;
    Serial.println(gps.satellites.value());
    //print_speed();
  }
  else
  {
   
    Serial.print("No Data");
   
  }  
  
}

void print_speed()
{
  
       
  if (gps.location.isValid() == 1)
  {
 
    
   
    Serial.print("Lat: ");
    
    Serial.print(gps.location.lat(),6);

    
    Serial.print("Lng: ");
   
    Serial.print(gps.location.lng(),6);

  }
  else
  {
    Serial.print("No Data");
  }  
}