#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define RX_pin A1
#define TX_pin A2

TinyGPSPlus gps;
SoftwareSerial ss(RX_pin, TX_pin); // RX, TX

void setup() {
  Serial.begin(9600);      // Монитор порта
  //gpsSerial.begin(9600);   // GPS модуль
  ss.begin(9600);
}

void loop() {
  // Пересылаем данные от GPS в монитор
  /*while (ss.available()) {
    char c = ss.read();
    Serial.write(c);
  }*/
  while (ss.available()>0){
    gps.encode((char)ss.read());
    if (gps.location.isUpdated()){
      Serial.print("latitude ");
      Serial.println(gps.location.lat(), 6);
      Serial.print("Longtitude ");
      Serial.println(gps.location.lng(), 6);
      Serial.println("===============");
    }
  }
}