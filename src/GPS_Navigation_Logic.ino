/* * GPS Telemetry Integration 
 * Module: Beitian BN-220T via UART
 */

#include <SoftwareSerial.h>

// Connect GPS TX to Pin 4, RX to Pin 3
SoftwareSerial gpsSerial(4, 3); 

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
}

void loop() {
  if (gpsSerial.available()) {
    String gpsData = gpsSerial.readStringUntil('\n');
    
    // Show-off: Parsing NMEA Sentences
    if (gpsData.startsWith("$GPRMC")) {
       Serial.println("Vector-X: GPS Signal Locked.");
       Serial.println("Raw Data: " + gpsData);
    }
  }
}
