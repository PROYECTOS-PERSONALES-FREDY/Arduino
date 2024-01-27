#include <Arduino.h>
#include <WiFi.h>
#include <ESPNowW.h>
#include "esp_wifi.h"


#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 12

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

float Celsius;

// Variables para test.
//MAC address MASTER
uint8_t broadcastAddress[] = {0x4C, 0xEB, 0xD6, 0x7C, 0x24, 0xC8};

//ESTRUCTURA DATA
typedef struct struc_message{

float TemperaturaTierra;

} struct_message;

struc_message myData;

//PEER INFO
esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_adrr, esp_now_send_status_t status){
Serial.print("\r\nLast Packet Send Status:\t");
Serial.println(status == ESP_NOW_SEND_SUCCESS ? "DElivery Sucess" : "Delivery Fail");
}
void setup() {
  // put your setup code here, to run once:
  int a= esp_wifi_set_protocol( WIFI_IF_AP, WIFI_PROTOCOL_LR );
  Serial.begin(9600);

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

}

void loop() {
  sensors.requestTemperatures();

  Celsius =sensors.getTempCByIndex(0);

  // put your main code here, to run repeatedly:


  //strcpy (myData.a, "Welcom to the workshop!");
  
  myData.TemperaturaTierra = Celsius ;
  
  //Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

  if (result == ESP_OK) {
    Serial.println("Sending confirmed");
  }
  else {
    Serial.println("Sending Error");
  }
  delay(2000);
}