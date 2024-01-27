//#include <SoftwareSerial.h>

int TRIG = 13; // El puerto TRIG del sensor está conectado al pin 13
int ECHO = 12; // El puerto ECHO del sensor está conectado al pin 12
int LED = 5;   // Led conectado con el pin 5
int duracion = 0;
int distancia = 0;

//SoftwareSerial miBT(10,11);

void setup()
{
  pinMode(TRIG,OUTPUT); // Pin TRIG configurado como salida
  pinMode(ECHO,INPUT); //  Pin TRIG configurado como entrada
  
  pinMode(LED,OUTPUT); // Pin LED configurado como salida
  
  
  //Serial.begin(9600);
  
  //Serial.println("Modulo Bluetooth Conectado");
  
  //miBT.begin(38400);
}

void loop()
{
  
  //if(miBT.available()){      // lee BT y envia a Arduino
    //Serial.write(miBT.read());
  //}
  
  /*if(Serial.available()){    // lee Arduino y envia a BT
    //miBT.write(Serial.read()); 
  }
  */
  
  // Tiempo de envío y recepción
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(1); 
  digitalWrite(TRIG, LOW);
  delayMicroseconds(1); 
  
  duracion = pulseIn (ECHO,HIGH);
      distancia = duracion/58.2;
  
  Serial.println(distancia);
  Serial.println("cm");
  
  
  if (distancia <= 10){ 
    digitalWrite(LED, HIGH);
    Serial.println("Encendido");
    
  }
  else if(distancia > 10){
    digitalWrite(LED,LOW);
    Serial.println("Apagado");
  }
 
}
