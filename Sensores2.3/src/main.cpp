#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

//Pines de Sensores
int UVOUT = 39; //Output from the sensor
int SensorHall = 34;

int h=1;
float sumaD = 0;
float sampleTime = 1000;
float v = 5.0;

//Velocidad y RPM
float averageAnalogRead(float pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0; 
 
  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += pinToRead;
  runningValue /= numberOfReadings;
 
  return(runningValue);
} 
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
  float kount2rpm = (float (60000 / float(sampleTime1)) * kount);
  return kount2rpm;
}
float getVel(float getrpm)
{
  float rpm = getrpm; //método que se crea más adelante para configurar las revoluciones por min
  float w = 2 * 3.1415926535 * (rpm / 60); // Se pasa r.p.m. a rad/s
  float v = w * 0.04; // se halla la Velocidad Lineal w*radio del anemometro en m (r cazo=0.057 y 0.0615 cm hélice [Perimetro = 2*pi*Dext]

  return v;
}
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  float pendiente = ((out_max - out_min) / (in_max - in_min));
  return (((pendiente*x)-(pendiente*in_max)+ (out_max))/1000);
}

//UV
float UvRead(float Vm)
{
  float sum = 0;
  for(int i=0; i<sampleTime; i++) {
    float v = Vm;
    sum = v + sum;
    delay(2);
  }
  float sensor_value_average = sum / sampleTime;
  float mV = sensor_value_average * v;
  return mV;
}

float index(float read_mV) {
  float voltage = read_mV / 1024.0;
  int uv_index = voltage / 0.1;
  return uv_index;
}


void setup()
{
  pinMode(SensorHall, INPUT);
  pinMode(UVOUT, INPUT);
  Serial.begin(9600);
}

void loop()
{

  int Rpm = getRPM();
  int Vel = averageAnalogRead(getVel(Rpm));
  float uvRead = UvRead(averageAnalogRead(UVOUT));
  float uvLevel =index(uvRead);  
  //Serial.printf("\nRPM: %i rpm, Velocidad: %i m/s, Nivel Uv: %.2f\n",Rpm,Vel,uvLevel);
  Serial.printf("\n %.2f, %.2f, %.2f\n",uvLevel,uvRead,averageAnalogRead(UVOUT));
  //delay(sampleTime); //SAMPLE TIME
}