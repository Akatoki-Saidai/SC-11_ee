#include <TinyGPS++.h>
#include <SoftwareSerial.h>
SoftwareSerial gps_serial(2, 3);
TinyGPSPlus gps;
long starttime;
float a, b;
void setup() {
  Serial.begin(115200);
  gps_serial.begin(9600);
}

void loop() {
  int i = 0;
  while(gps_serial.available() >0) {
      char c = gps_serial.read();
      gps.encode(c);
      Serial.println(i);
        i++;
      if (gps.location.isUpdated()) {
        a = gps.location.lat();
        b = gps.location.lng();
        
      }
   }
}
