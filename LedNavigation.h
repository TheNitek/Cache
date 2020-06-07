#ifndef LedNavigation_h
#define LedNavigation_h

#include <Arduino.h>
#include <QMC5883LCompass.h>
#include <TinyGPS++.h>
#include <NeoPixelBus.h>
#include "Stage.h"
#include "Colors.h"

class LedNavigation: public Stage {
  public:
    LedNavigation(QMC5883LCompass &compass, TinyGPSPlus &gps, NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> &ring): _compass(compass), _gps(gps), _ring(ring)
    {};
    void handle();
  private:
    const double TARGET_LAT = 48.4168602;
    const double TARGET_LON = 9.9433157;
    bool _blinker = false;
    QMC5883LCompass _compass;
    TinyGPSPlus _gps;
    NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> _ring;
    void _smartDelay(unsigned long ms);
    void _printDateTime(TinyGPSDate &d, TinyGPSTime &t);
};

#endif