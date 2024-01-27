#include <SoftwareSerial.h>

#include <Wire.h>

// VARIABLES SENSOR HALL
const int SensorHall = A2;
// ESTADOS DEL SENSOR
int estadoSensor = 0;
int estadoAnterior = 0;
// SCL A5 SDA A4
float rpm;
float v, w, V, A;
float sampleTime = 100;
void setup()
{
  
  pinMode(SensorHall, INPUT); // el Sensor como una entrada
  Serial.begin(9600);
}

void loop()
{
  getRPM();
  
  Serial.print(getVel());
  Serial.print("\n");
  Serial.print(getRPM());
  Serial.print("\n");
  
  delay(1000); //SAMPLE TIME
}
// FUNCION RPM
float getRPM()
{
  float kount = 0;
  boolean kflag = LOW;
  unsigned long currentTime = 0;
  unsigned long startTime = millis();
  unsigned long sampleTime = 999;
  while (currentTime <= sampleTime)
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
    /*
    Serial.print(digitalRead(SensorHall));
    Serial.print("\n");
    */
  }
  float kount2rpm = float (60000. / float(sampleTime)) * kount;
  return kount2rpm;
}

// LECTURA DEL SENSOR EFECTO HALL
float getVel()
{
  estadoSensor = digitalRead(SensorHall);// se lee el estado del sensor
  if (estadoSensor != estadoAnterior) // se compara con el valor anterior
  {
    rpm = getRPM(); //método que se crea más adelante para configurar las revoluciones por min
    w = 2 * (rpm / 60) * 3.1415926535; //se Halla la velocidad Angular
    v = w * 0.057; // se halla la velocidad Lineal w*radio del anemometro en m (r cazo=0.057 y 0.0615 cm hélice [Perimetro = 2*pi*Dext]
  }
  estadoAnterior = estadoSensor; // se reinicia el estado del sensor a 0 para que repita la condición
  return v;
}
