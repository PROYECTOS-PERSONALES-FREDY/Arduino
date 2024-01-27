#include <Arduino.h>
#include <WiFi.h>

#include <ESPNowW.h>
#include <esp_wifi.h>
uint8_t current_protocol;
esp_interface_t current_esp_interface;
wifi_interface_t current_wifi_interface;
// Variables para test.
int int_value;
float float_value;
bool bool_value = true;
String palabra;
//MAC address MASTER
uint8_t broadcastAddress[] = {0x4C, 0xEB, 0xD6, 0x7C, 0x24, 0xC8};

//ESTRUCTURA DATA
typedef struct struc_message{
char a[32];
int b;
float c;
bool d;
String e;

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
  Serial.begin(9600);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_protocol(
          ESP_IF_WIFI_STA,
          WIFI_PROTOCOL_LR);


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
  // put your main code here, to run repeatedly:

  int_value = random(1,20);
  float_value = 1.3 * int_value;
  bool_value = !bool_value;
  palabra = "hola mundo";
  //strcpy (myData.a, "Welcom to the workshop!");
  myData.b = int_value;
  myData.c = float_value;
  myData.d = bool_value;
  myData.e = palabra;

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