#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <WiFi.h>
#include <ESPNowW.h>
#include "esp_wifi.h"

//MAC address MASTER
uint8_t broadcastAddress[] = {0x4C, 0xEB, 0xD6, 0x7C, 0x24, 0xC8};

//ESTRUCTURA DATA
typedef struct struct_mesagge_EstacionMeteorologica{

  float VelAire;
  float RPM;
  float uvintesity;
  float NivelAgua;

}struct_mesagge_EstacionMeteorologica;
struct_mesagge_EstacionMeteorologica EstacionMetereologica;

//PEER INFO
esp_now_peer_info_t peerInfo;

//Pines de Sensores
int REF_3V3 = 36; //3.3V power on the ESP32 board
int UVOUT = 39; //Output from the sensor
int SensorHall = 34;

int h=1;
float sumaD = 0;
float sampleTime = 1000;

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
float uvRead(float x){
  return (x/1000);
}
float Prom(float valor){
  float envioD;
  sumaD+=valor;
  envioD=sumaD/h;
  h++;
  return envioD;
}

void OnDataSent(const uint8_t *mac_adrr, esp_now_send_status_t status){
  Serial.print("\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "DElivery Sucess" : "Delivery Fail");
  Serial.print("----------------------------------------------------");
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

  pinMode(UVOUT, INPUT);
  pinMode(REF_3V3, INPUT);
  pinMode(SensorHall, INPUT);
  Serial.begin(9600);
}

void loop()
{

  int Rpm = getRPM();
  int Vel = averageAnalogRead(getVel(Rpm));
  int uvLevel =(analogRead(UVOUT));// averageAnalogRead(analogRead(UVOUT)-10)/1000;
  float uvIntensity = mapfloat(uvLevel, 0.99*1000, 2.8*1000, 0*1000, 15*1000); 
   
  Serial.printf("\nRPM: %i rpm \n",Rpm);
  Serial.printf("Velocidad: %i m/s \n",Vel);
  Serial.printf("Nivel Uv: %.2f \n",uvRead(uvLevel));
  Serial.printf("Intensidad Uv: %.2f mW/cm^2 \n",uvIntensity);

  //Serial.printf("%i,%i,%i,%.2f \n",Rpm,Vel,uvLevel,uvIntensity);

  EstacionMetereologica.uvintesity=uvIntensity;  
  EstacionMetereologica.VelAire=Vel;
  EstacionMetereologica.RPM=Rpm;

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &EstacionMetereologica, sizeof(EstacionMetereologica));

  if (result == ESP_OK) {
    Serial.printf("Sending confirmed\n");
  }
  else {
    Serial.printf("Sending Error\n");
  }

  //delay(sampleTime); //SAMPLE TIME
}