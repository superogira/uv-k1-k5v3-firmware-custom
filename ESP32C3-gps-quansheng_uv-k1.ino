#include <Arduino.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

#define RX_PIN_GPS 1
#define TX_PIN_GPS 0
#define TX_PIN_RADIO 21  
#define RX_PIN_RADIO 20 
#define LED_PIN 8 

TinyGPSPlus gps;
HardwareSerial GPS_Serial(1);
HardwareSerial Radio_Serial(0);

String currentPacket = "[Wait...]";

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  GPS_Serial.begin(9600, SERIAL_8N1, RX_PIN_GPS, TX_PIN_GPS);
  Radio_Serial.begin(38400, SERIAL_8N1, RX_PIN_RADIO, TX_PIN_RADIO);
}

void loop() {
  while (GPS_Serial.available() > 0) {
    gps.encode(GPS_Serial.read());
  }

  static unsigned long last_update = 0;
  if (millis() - last_update > 1000) {
    last_update = millis();
    
    currentPacket = "[";
    
    if (gps.location.isValid()) {
        currentPacket += String(gps.location.lat(), 5) + ",";
        currentPacket += String(gps.location.lng(), 5) + ",";
        
        // *** เพิ่มตรงนี้: ส่งค่าความสูง (เมตร) ***
        currentPacket += String(gps.altitude.meters(), 1) + ",";
        
        currentPacket += String(gps.satellites.value()) + ",";
        currentPacket += String(gps.speed.kmph(), 1) + ",";
        currentPacket += String(gps.course.deg(), 0) + ",";
        
        char timeBuff[10];
        sprintf(timeBuff, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
        currentPacket += String(timeBuff) + ",";
        
        char dateBuff[16];
        sprintf(dateBuff, "%04d/%02d/%02d", gps.date.year(), gps.date.month(), gps.date.day());
        currentPacket += String(dateBuff);
        
    } else {
        // Demo Packet (เพิ่มค่าความสูงสมมติเข้าไปด้วย เช่น 10.5 เมตร)
        // Format: Lat, Lon, Alt, Sats, Speed, Dir, Time, Date
        currentPacket += "13.59542,100.56175,10.5,0,0.0,0,WaitTime,WaitDate";
    }
    
    currentPacket += "]";
  }

  if (Radio_Serial.available() > 0) {
    int requestedIndex = Radio_Serial.read();
    delay(5);
    if (requestedIndex < currentPacket.length()) {
       Radio_Serial.write(currentPacket.charAt(requestedIndex));
    } else {
       Radio_Serial.write(0); 
    }
    Radio_Serial.flush();
  }
}
