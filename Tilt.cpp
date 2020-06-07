#include <Tilt.h>

void Tilt::init() {

  _gyro.writeRegister(LSM6DS3_ACC_GYRO_CTRL3_C, LSM6DS3_ACC_GYRO_SW_RESET_RESET_DEVICE);

  //Over-ride default settings if desired
  _gyro.settings.gyroEnabled = 1;  //Can be 0 or 1
  _gyro.settings.gyroRange = 2000;   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
  _gyro.settings.gyroSampleRate = 104;   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
  _gyro.settings.gyroBandWidth = 200;  //Hz.  Can be: 50, 100, 200, 400;

  _gyro.settings.accelEnabled = 1;
  _gyro.settings.accelRange = 16;      //Max G force readable.  Can be: 2, 4, 8, 16
  _gyro.settings.accelSampleRate = 833;  //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
  _gyro.settings.accelBandWidth = 200;  //Hz.  Can be: 50, 100, 200, 400;

  _gyro.begin();

  // Set orientation threshold
  _gyro.writeRegister(LSM6DS3_ACC_GYRO_TAP_THS_6D, LSM6DS3_ACC_GYRO_SIXD_THS_60_degree);
  // Enable low pass filter 2
  _gyro.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x10);
  _gyro.writeRegister(LSM6DS3_ACC_GYRO_CTRL8_XL, 0x01);
  // Enable interrupt
  _gyro.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, LSM6DS3_ACC_GYRO_INT1_6D_ENABLED);
};

void Tilt::handle() {
  uint8_t orientation;
  _gyro.readRegister(&orientation, LSM6DS3_ACC_GYRO_D6D_SRC);

  if(_currentOrientation == orientation) {
    return;
  }

  _currentOrientation = orientation;

  if(orientation == _tiltSequence[_tiltPointer++]) {
    if(_tiltSequence.size() == _tiltPointer) {
      for(uint8_t i = 0; i < 10; i++) {
        _ring.ClearTo(white);
        _ring.Show();
        sleep(200);
        _ring.ClearTo(green);
        _ring.Show();
        sleep(200);
      }
      return;
    }
    _ring.ClearTo(black);
    _ring.Show();
    sleep(500);
    _ring.ClearTo(green);
    _ring.Show();
    return;
  }

  _tiltPointer = 0;
  _ring.ClearTo(red);
  _ring.Show();

  /*switch(orientation){
    case O_X_DOWN:
      Serial.println("X down");
      break;
    case O_X_UP:
      Serial.println("X up");
      break;
    case O_Y_DOWN:
      Serial.println("Y down");
      break;
    case O_Y_UP:
      Serial.println("Y up");
      break;
    case O_Z_DOWN:
      Serial.println("Z down");
      break;
    case O_Z_UP:
      Serial.println("Z up");
      break;
    default:
      Serial.println("Unknown orientation");
  }


  char orientationString[9];
  itoa(orientation, orientationString, 2);
  Serial.print("\nOrientation:\n");
  Serial.printf("%s", orientationString);*/
};