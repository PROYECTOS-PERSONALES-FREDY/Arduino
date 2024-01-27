#include <Arduino.h>
#include <WiFi.h>
#include <ESPNowW.h>

#include <Wire.h>


//Estructura de datos

typedef struct struct_mesagge {

char a[32];
int b;
float c;
bool d;
String e;

} struct_mesagge;

struct_mesagge myData; 

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x", 
            mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
memcpy(&myData, incomingData, sizeof(myData));
//lcd.setCursor(0,0);
//lcd.printf("Data from:");
//lcd.setCursor(0,1);
//lcd.printf(macStr);
//lcd.scrollDisplayLeft();
//delay(1000);
//lcd.clear();
Serial.print("Data received from: ");
Serial.println(macStr);
Serial.println();
//Serial.print("Character Value: ");
Serial.print("integer: ");
Serial.println(myData.b);
Serial.print("Float: ");
Serial.println(myData.c);
Serial.print("Boolean: ");
Serial.println(myData.d);
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK){
    Serial.println("Error initialiazing esp_now");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // put your main code here, to run repeatedly:
}