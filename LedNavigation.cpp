#include "LedNavigation.h"

void LedNavigation::handle() {
  _compass.read();

  _printDateTime(_gps.date, _gps.time);

  if(_gps.satellites.isValid())
  {
    Serial.printf("%d ", _gps.satellites.value());
  }

  _ring.ClearTo(black);
  
  if(_gps.location.isValid()) {
    Serial.printf("%f, %f, a: %d", _gps.location.lat(), _gps.location.lng(), _gps.location.age());
    Serial.println();

    unsigned long distanceMeters =
      (unsigned long)TinyGPSPlus::distanceBetween(
        _gps.location.lat(),
        _gps.location.lng(),
        TARGET_LAT, 
        TARGET_LON);
  
    int16_t course =
      TinyGPSPlus::courseTo(
        _gps.location.lat(),
        _gps.location.lng(),
        TARGET_LAT, 
        TARGET_LON);

  
    int16_t azimuth = _compass.getAzimuth();
    
    char myArray[3];
    _compass.getDirection(myArray, azimuth);

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
    _ring.SetPixelColor(ledIndex, green);
  } else {
    if(_blinker) {
        _ring.ClearTo(red);
    }
    _blinker = !_blinker;
  }
  Serial.println();
  Serial.println();
  
  _ring.Show();

  _smartDelay(500);

  if (millis() > 5000 && _gps.charsProcessed() < 10) {
    Serial.println(F("No GPS data received: check wiring"));
  }
};


void LedNavigation::_smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do 
  {
    while (Serial2.available()) {
      _gps.encode(Serial2.read());
    }
  } while (millis() - start < ms);
};

void LedNavigation::_printDateTime(TinyGPSDate &d, TinyGPSTime &t) {
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
};
