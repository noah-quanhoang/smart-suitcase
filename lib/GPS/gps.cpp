// gps.cpp
#include <Adafruit_GPS.h>
#include "GPS.h"

void setupGPS() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 17, 18);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  delay(1000);
}

void clearGPS() {
  while (!GPS.newNMEAreceived()) {
    c = GPS.read();
  }
  GPS.parse(GPS.lastNMEA());

  while (!GPS.newNMEAreceived()) {
    c = GPS.read();
  }
  GPS.parse(GPS.lastNMEA());
}

void printGPSData() {
  clearGPS();

  while (!GPS.newNMEAreceived()) {
    c = GPS.read();
  }

  GPS.parse(GPS.lastNMEA());

  Serial.print("Time: ");
  Serial.print(GPS.hour, DEC);
  Serial.print(':');
  Serial.print(GPS.minute, DEC);
  Serial.print(':');
  Serial.print(GPS.seconds, DEC);
  Serial.print('.');
  Serial.println(GPS.milliseconds);

  Serial.print("Date: ");
  Serial.print(GPS.day, DEC);
  Serial.print('/');
  Serial.print(GPS.month, DEC);
  Serial.print("/20");
  Serial.println(GPS.year, DEC);

  Serial.print("Fix: ");
  Serial.print(GPS.fix);
  Serial.print(" quality: ");
  Serial.println(GPS.fixquality);
  Serial.print("Satellites: ");
  Serial.println(GPS.satellites);

  if (GPS.fix) {
    Serial.print("Location: ");
    Serial.print(GPS.latitude, 4);
    Serial.print(GPS.lat);
    Serial.print(", ");
    Serial.print(GPS.longitude, 4);
    Serial.println(GPS.lon);
    Serial.print("Google Maps location: ");
    Serial.print(GPS.latitudeDegrees, 4);
    Serial.print(", ");
    Serial.println(GPS.longitudeDegrees, 4);

    Serial.print("Speed (knots): ");
    Serial.println(GPS.speed);
    Serial.print("Heading: ");
    Serial.println(GPS.angle);
    Serial.print("Altitude: ");
    Serial.println(GPS.altitude);
  }
  Serial.println("-------------------------------------");
}
