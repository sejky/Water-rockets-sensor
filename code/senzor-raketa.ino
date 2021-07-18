#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "Ublox.h"

#include <SoftwareSerial.h>
#define N_FLOATS 4

#define BMP280_ADRESA (0x76)
Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

int pocitadlo=0;
//float maxV=0,maxGPS=0,maxR=0;


//GPS
static const int RXPin = 3, TXPin = 5;
static const uint32_t GPSBaud = 9600;

Ublox M8_Gps;
// Altitude - Latitude - Longitude - N Satellites
float gpsArray[N_FLOATS] = {0, 0, 0, 0};

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

void setup() {
  Serial.begin(9600);
  ss.begin(GPSBaud);

  if (!bmp.begin(BMP280_ADRESA)) 
  {  
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
  Serial.print(F("start"));
}

void loop() 
{
    Serial.print(pocitadlo);Serial.print(F("|"));
    Serial.print(bmp.readTemperature());Serial.print(F("|"));
    Serial.print(bmp.readPressure());Serial.print(F("|"));
    Serial.print(bmp.readAltitude(1015)); Serial.print(F("|"));
    // this should be adjusted to your local forcase
    

//    if(!ss.available())
//    return;

    while(ss.available())
    {
          char c = ss.read();
           if (M8_Gps.encode(c)) 
           {
            gpsArray[0] = M8_Gps.altitude;
            gpsArray[1] = M8_Gps.latitude;
            gpsArray[2] = M8_Gps.longitude; 
            gpsArray[3] = M8_Gps.speed;
          }
    }

      Serial.print(gpsArray[0], 2);Serial.print(F("|"));
      Serial.print(gpsArray[1], 6);Serial.print(F("|"));
      Serial.print(gpsArray[2], 6);Serial.print(F("|"));
      Serial.print(gpsArray[3], 2);Serial.print(F("|"));
//    for(byte i = 0; i < N_FLOATS; i++) {
//      Serial.print(gpsArray[i], 6);Serial.print("|");
//    }

 
    Serial.println();
    
    pocitadlo++;
    delay(500);
}
