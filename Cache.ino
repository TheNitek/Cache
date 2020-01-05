#include <TinyGPS++.h>
#include <NeoPixelBus.h>
#include <Wire.h>
#include <QMC5883LCompass.h>

/**
 * LED Direction
 * Beeping
 * Tapping
 * Wifi Logbook
 */

#define LED_PIN 27
#define LED_COUNT 8

#define RXD2 16
#define TXD2 17
const uint32_t GPSBaud = 9600;

NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> ring(8, 27);
TinyGPSPlus gps;
QMC5883LCompass compass;

#define colorSaturation 4
RgbColor red(colorSaturation, 0, 0);
RgbColor green(0, colorSaturation, 0);
RgbColor blue(0, 0, colorSaturation);
RgbColor white(colorSaturation);
RgbColor black(0);

bool blinker = false;

uint8_t stage = 0;

void setup()
{
  Serial.begin(115200);
  Serial2.begin(GPSBaud, SERIAL_8N1, RXD2, TXD2);

  ring.Begin();
  ring.Show();

  compass.init();
  //compass.setMode(0x01, 0x08, 0x10, 0xC0);
  compass.setCalibration(-1707, 1543, -2415, 772, -2397, 740);
}

void loop()
{
	compass.read();

  switch(stage) {
    case 0: 
      stageTilt();
      break;
    case 1:
      stageGotoPlayground();
      break;
    default:
      Serial.println("Not implemented!");
  }
  
  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}

void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial2.available())
      gps.encode(Serial2.read());
  } while (millis() - start < ms);
}

void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (d.isValid())
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (t.isValid())
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }
}

void stageTilt() {
  
}

void stageGotoPlayground() {
    static const double TARGET_LAT = 48.4168602, TARGET_LON = 9.9433157;

  printDateTime(gps.date, gps.time);

  if(gps.satellites.isValid())
  {
    Serial.printf("%d ", gps.satellites.value());
  }

  ring.ClearTo(black);
  
  if(gps.location.isValid()) {
    Serial.printf("%f, %f, a: %d", gps.location.lat(), gps.location.lng(), gps.location.age());
    Serial.println();

    unsigned long distanceMeters =
      (unsigned long)TinyGPSPlus::distanceBetween(
        gps.location.lat(),
        gps.location.lng(),
        TARGET_LAT, 
        TARGET_LON);
  
    int16_t course =
      TinyGPSPlus::courseTo(
        gps.location.lat(),
        gps.location.lng(),
        TARGET_LAT, 
        TARGET_LON);

  
    int16_t azimuth = compass.getAzimuth();
    
    char myArray[3];
    compass.getDirection(myArray, azimuth);

    Serial.print(" Azimuth: ");
    Serial.print(azimuth);
    Serial.print(" Direction: ");
    Serial.print(myArray[0]);
    Serial.print(myArray[1]);
    Serial.print(myArray[2]);
    
    // Offset between LED #0 and azimuth == 0
    int16_t ledOffset = 90;

    Serial.printf(" d: %lu c: %d ang: %d", distanceMeters, course, (uint16_t)round(azimuth + ledOffset + course + 11.25f));

    uint16_t ledIndex = (uint16_t)round((azimuth + ledOffset + course + 11.25f)/22.5) % 16;
    if(ledIndex % 2 == 0) {
      ring.SetPixelColor(ledIndex/2, green);
    } else {
      ring.SetPixelColor((ledIndex-1)/2, green);
      ring.SetPixelColor((ledIndex+1)/2 % 8, green);
    }
  } else {
    if(blinker) {
      for(uint8_t i = 0; i<ring.PixelCount(); i++) {
        ring.SetPixelColor(i, red);
      }
    }
    blinker = !blinker;
  }
  Serial.println();
  Serial.println();
  
  ring.Show();
}