#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>

int REF_3V3 = 36; //3.3V power on the ESP32 board
int UVOUT = 39; //Output from the sensor

 
void setup()
{
  Serial.begin(9600);
  pinMode(UVOUT, INPUT);
  pinMode(REF_3V3, INPUT);
}
 
//Takes an average of readings on a given pin
//Returns the average
int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0; 
 
  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;
 
  return(runningValue);
}
 
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
 
void loop()
{
  int uvLevel = averageAnalogRead(UVOUT);
  int refLevel = averageAnalogRead(REF_3V3);
  
  //Use the 3.3V power pin as a reference to get a very accurate output value from sensor
  float outputVoltage = 3.3 / refLevel * uvLevel;
  
  float uvIntensity = mapfloat(outputVoltage, 0.99, 2.8, 0.0, 15.0); //Convert the voltage to a UV intensity level
  /*
  Serial.print("output: ");
  Serial.print(refLevel);

  Serial.print(" ML8511 output: ");
  Serial.print(uvLevel);

  Serial.print(" / ML8511 voltage: ");
  Serial.print(outputVoltage);
  
  Serial.print(" / UV Intensity (mW/cm^2): ");
  Serial.print(uvIntensity);
   */
  
  Serial.printf("%i,%.2f,%.2f",uvLevel,outputVoltage,uvIntensity);
  Serial.printf("\n");

  delay(200);
}