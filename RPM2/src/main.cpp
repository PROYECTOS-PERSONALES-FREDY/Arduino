#include <Arduino.h>


// VARIABLES SENSOR HALL
const int SensorHall = 15;
int h = 0;
int m = 1;
int aux = 0;
// ESTADOS DEL SENSOR
int estadoSensor = 0;
int estadoAnterior = 0;
// SCL A5 SDA A4
float rpm;
float v, w;

float sampleTime = 1000;

float a = 8.555;

float pr1[5]={0,0,0,0,0};
float pr2[5]={0,0,0,0,0};
float vlt1=0;
float vlt2=0;

// FUNCION RPM
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
float getVel()
{
 
  rpm = vlt2; //método que se crea más adelante para configurar las revoluciones por min
  w = 2 * 3.1415926535 * (rpm / 60); // Se pasa r.p.m. a rad/s
  v = w * 0.04; // se halla la velocidad Lineal w*radio del anemometro en m (r cazo=0.057 y 0.0615 cm hélice [Perimetro = 2*pi*Dext]

  return v;
}
void setup()
{
  pinMode(SensorHall, INPUT); // el Sensor como una entrada
  Serial.begin(9600);
}
void loop()
{
  pr1[h]=getVel();
  pr2[h]=getRPM();

  vlt1=(pr1[0]+pr1[1]+pr1[2]+pr1[3]+pr1[4])/m;
  vlt2=(pr2[0]+pr2[1]+pr2[2]+pr2[3]+pr2[4])/m;

  Serial.printf("%.2f,%.2f,%.2f",vlt1,vlt2,a);
  Serial.printf("\n");

  //delay(sampleTime); //SAMPLE TIME
  h++;
  m++;
  if(h>4)
    {h=0;}
  if(m>4)
    {m=5;}
  if(vlt2==0)
    {m=1;}  
}
