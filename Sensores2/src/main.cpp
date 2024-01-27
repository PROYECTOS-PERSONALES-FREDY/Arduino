#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <WiFi.h>
#include <ESPNowW.h>
#include "esp_wifi.h" 

int REF_3V3 = 36; //3.3V power on the ESP32 board
int UVOUT = 39; //Output from the sensor
int SensorHall = 34;

//MAC address MASTER
uint8_t broadcastAddress[] = {0x4C, 0xEB, 0xD6, 0x7C, 0x24, 0xC8};

//ESTRUCTURA DATA
typedef struct struc_message{
float VelAire;
float RPM;
float uvintensity;

} struct_message;

struc_message myData;

//PEER INFO
esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_adrr, esp_now_send_status_t status){
Serial.print("\r\nLast Packet Send Status:\t");
Serial.println(status == ESP_NOW_SEND_SUCCESS ? "DElivery Sucess" : "Delivery Fail");
}

/////////////////////////////////////////////////
// VARIABLES SENSOR HALL

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

float UV=0;
float UVvol=0;

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
void setup()
{
   // put your setup code here, to run once:
  int a= esp_wifi_set_protocol( WIFI_IF_AP, WIFI_PROTOCOL_LR );
  WiFi.mode(WIFI_STA);
   //Iniciar esp_now
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error al iniciar ESP-NOW");
    return;
  }
  //registro callback
  esp_now_register_send_cb(OnDataSent);

  //reguster peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  //add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  //////////////////////////////////
  pinMode(UVOUT, INPUT);
  pinMode(REF_3V3, INPUT);
  pinMode(SensorHall, INPUT); // el Sensor como una entrada
  Serial.begin(9600);
}
void loop()
{

  ///////////////////////////////////////
  int uvLevel = averageAnalogRead(UVOUT);
  int refLevel = averageAnalogRead(REF_3V3);
  
  //Use the 3.3V power pin as a reference to get a very accurate output value from sensor
  float outputVoltage = 3.3 / refLevel * uvLevel;

  float uvintensity = mapfloat(outputVoltage, 0.99, 2.8, 0.0, 15.0); //Convert the voltage to a UV intensity level
  
  pr1[h]=getVel();
  pr2[h]=getRPM();
  
  vlt1=(pr1[0]+pr1[1]+pr1[2]+pr1[3]+pr1[4])/m;
  vlt2=(pr2[0]+pr2[1]+pr2[2]+pr2[3]+pr2[4])/m;

  myData.uvintensity=uvintensity;  
  myData.VelAire=vlt1;
  myData.RPM=vlt2;

  Serial.printf("%.2f,%.2f,%.2f",vlt1,vlt2,uvintensity);
  Serial.printf("\n");
  
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

  if (result == ESP_OK) {
    Serial.println("Sending confirmed");
  }
  else {
    Serial.println("Sending Error");
  }

  delay(sampleTime); //SAMPLE TIME
  h++;
  m++;
  if(h>4)
    {h=0;}
  if(m>4)
    {m=5;}
  if(vlt2==0)
    {m=1;} 
}