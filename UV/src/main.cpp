#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
 
int REF_3V3 = 36; //3.3V power on the ESP32 board
int UVOUT = 39; //Output from the sensor
 

float averageAnalogRead(float pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0; 
 
  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += pinToRead;
  runningValue /= numberOfReadings;
 
  return(runningValue);
} 
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return ((x - in_min) * (out_max - out_min) )/ ((in_max - in_min) + out_min);
}
void setup()
{
  Serial.begin(9600);
  pinMode(UVOUT, INPUT);
  pinMode(REF_3V3, INPUT);
} 
void loop()
{

  int uvLevel = averageAnalogRead(analogRead(UVOUT))/1000;
  float refLevel = (averageAnalogRead(analogRead(REF_3V3))/1000)-1;
  float outputVoltage = ((3.3/refLevel)* uvLevel);
  
  float uvIntensity = mapfloat(outputVoltage, 0.99, 2.8, 0.0, 15.0); 
  
  Serial.printf("Nivel Uv: %i \n",uvLevel);
  Serial.printf("Intensidad Uv: %.2f mW/cm^2 \n\n",uvIntensity);
  
  delay(1000);
}
