#include <Arduino.h>
#include <TinyGPS++.h>
#include <NeoPixelBus.h>
#include <Wire.h>
#include <QMC5883LCompass.h>
#include <SparkFunLSM6DS3.h>

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
LSM6DS3 gyro;

#define O_X_DOWN 0x01
#define O_X_UP   0x02
#define O_Y_DOWN 0x04
#define O_Y_UP   0x08
#define O_Z_DOWN 0x10
#define O_Z_UP   0x20

uint8_t currentOrientation;

#define colorSaturation 4
RgbColor red(colorSaturation, 0, 0);
RgbColor green(0, colorSaturation, 0);
RgbColor blue(0, 0, colorSaturation);
RgbColor white(colorSaturation);
RgbColor black(0);

bool blinker = false;

uint8_t stage = 0;

uint8_t tiltSequence[] = {O_Z_DOWN, O_Y_DOWN, O_Z_DOWN, O_X_UP, O_Y_DOWN, O_Z_DOWN, 0x00};
uint8_t tiltPointer = 0;

const double TARGET_LAT = 48.4168602, TARGET_LON = 9.9433157;

void setup()
{
  Serial.begin(115200);
  Serial2.begin(GPSBaud, SERIAL_8N1, RXD2, TXD2);

  ring.Begin();
  ring.Show();

  //Over-ride default settings if desired
  gyro.settings.gyroEnabled = 1;  //Can be 0 or 1
  gyro.settings.gyroRange = 2000;   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
  gyro.settings.gyroSampleRate = 104;   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
  gyro.settings.gyroBandWidth = 200;  //Hz.  Can be: 50, 100, 200, 400;

  gyro.settings.accelEnabled = 1;
  gyro.settings.accelRange = 16;      //Max G force readable.  Can be: 2, 4, 8, 16
  gyro.settings.accelSampleRate = 833;  //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
  gyro.settings.accelBandWidth = 200;  //Hz.  Can be: 50, 100, 200, 400;

  // Set orientation threshold
  gyro.writeRegister(LSM6DS3_ACC_GYRO_TAP_THS_6D, LSM6DS3_ACC_GYRO_SIXD_THS_60_degree);
  // Enable low pass filter 2
  gyro.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x10);
  gyro.writeRegister(LSM6DS3_ACC_GYRO_CTRL8_XL, 0x01);
  // Enable interrupt
  gyro.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, LSM6DS3_ACC_GYRO_INT1_6D_ENABLED);
  
  gyro.begin();

  compass.init();
  //compass.setMode(0x01, 0x08, 0x10, 0xC0);
  compass.setCalibration(-1707, 1543, -2415, 772, -2397, 740);
}

void loop()
{
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
  uint8_t orientation;
  gyro.readRegister(&orientation, LSM6DS3_ACC_GYRO_D6D_SRC);

  if(currentOrientation == orientation) {
    return;
  }

  currentOrientation = orientation;

  if(orientation == tiltSequence[tiltPointer]) {
    tiltPointer++;
    if(tiltSequence[tiltPointer] == 0) {
      for(uint8_t i = 0; i < 10; i++) {
        ring.ClearTo(white);
        ring.Show();
        smartDelay(200);
        ring.ClearTo(green);
        ring.Show();
        smartDelay(200);
      }
      return;
    }
    ring.ClearTo(black);
    ring.Show();
    smartDelay(500);
    ring.ClearTo(green);
    ring.Show();
    return;
  }

  tiltPointer = 0;
  ring.ClearTo(red);
  ring.Show();

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
}

void stageGotoPlayground() {
  compass.read();

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
        ring.ClearTo(red);
    }
    blinker = !blinker;
  }
  Serial.println();
  Serial.println();
  
  ring.Show();
}