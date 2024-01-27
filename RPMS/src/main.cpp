#include <Arduino.h>

// VARIABLES SENSOR HALL
int SensorHall = 34;

float sampleTime = 1000;

float getRPM()
{
  float kount = 0;
  boolean kflag = LOW;
  unsigned long currentTime = 0;
  unsigned long startTime = millis();
  unsigned long sampleTime1 = sampleTime-1;
  while (currentTime <= sampleTime1)
  {
    if (digitalRead(SensorHall) == HIGH)
    {
      kflag = HIGH;
    }
    if (digitalRead(SensorHall) == LOW && kflag == HIGH)
    {
      kount++;
      kflag = LOW;
    }
    currentTime = millis() - startTime;
  }
  float kount2rpm = float (60000 / float(sampleTime1)) * kount;
  return kount2rpm;
}
float getVel(float getrpm)
{
  float rpm = getrpm; //método que se crea más adelante para configurar las revoluciones por min
  float w = 2 * 3.1415926535 * (rpm / 60); // Se pasa r.p.m. a rad/s
  float v = w * 0.04; // se halla la Velocidad Lineal w*radio del anemometro en m (r cazo=0.057 y 0.0615 cm hélice [Perimetro = 2*pi*Dext]

  return v;
}
int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0; 
 
  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += pinToRead;
  runningValue /= numberOfReadings;
 
  return(runningValue);
} 

void setup()
{
  pinMode(SensorHall, INPUT); // el Sensor como una entrada
  Serial.begin(9600);
}
void loop()
{
  float Rpm = averageAnalogRead(getRPM());
  float Vel = averageAnalogRead(getVel(Rpm));

  Serial.printf("RPM: %.2f rpm \n",Rpm);
  Serial.printf("Velocidad: %.2f m/s \n\n",Vel);
}