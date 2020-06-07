#ifndef Tilt_h
#define Tilt_h

#include <SparkFunLSM6DS3.h>
#include <QMC5883LCompass.h>
#include "Stage.h"
#include "Colors.h"

#define O_X_DOWN 0x01
#define O_X_UP   0x02
#define O_Y_DOWN 0x04
#define O_Y_UP   0x08
#define O_Z_DOWN 0x10
#define O_Z_UP   0x20

class Tilt: public Stage {
  public:
    Tilt(LSM6DS3 &gyro, NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> &ring): _gyro(gyro), _ring(ring) {
    };
    void init();
    void handle();
  private:
    std::vector<uint8_t> _tiltSequence{O_Z_DOWN, O_Y_DOWN, O_Z_DOWN, O_X_UP, O_Y_DOWN, O_Z_DOWN};
    uint8_t _tiltPointer = 0;
    uint8_t _currentOrientation;
    LSM6DS3 _gyro;
    NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> _ring;
};

#endif