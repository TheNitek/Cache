#include <Arduino.h>
#include <TinyGPS++.h>
#include <NeoPixelBus.h>
#include <Wire.h>
#include <QMC5883LCompass.h>
#include <SparkFunLSM6DS3.h>
#include <SPI.h>
#include <MFRC522.h>
#include "Stage.h"
#include "Colors.h"
#include "Tilt.h"
#include "LedNavigation.h"
#include "Rfid.h"
#include "Logbook.h"


/**
 * LED Direction
 * Beeping
 * Tapping
 * Wifi Logbook
 */

#define LED_PIN 27
#define LED_COUNT 16

#define RXD2 16
#define TXD2 17
const uint32_t GPSBaud = 9600;

#define BUZZER_PIN 26

#define RC522_RST_PIN         4
#define RC522_CS_PIN          5

NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> ring(LED_COUNT, LED_PIN);
TinyGPSPlus gps;
QMC5883LCompass compass;
LSM6DS3 gyro;
MFRC522 mfrc522(RC522_CS_PIN, RC522_RST_PIN);


uint8_t currentStage = 0;
std::vector<Stage*> stages{new Tilt(gyro, ring), new LedNavigation(compass, gps, ring), new Rfid(mfrc522), new Logbook()};
Stage* stage = stages[0];


void setup()
{
  Serial.begin(115200);
  Serial2.begin(GPSBaud, SERIAL_8N1, RXD2, TXD2);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, HIGH);

  ring.Begin();
  ring.Show();

  gyro.begin();

  compass.init();
  //compass.setMode(0x01, 0x08, 0x10, 0xC0);
  compass.setCalibration(-1707, 1543, -2415, 772, -2397, 740);

  SPI.begin();
	mfrc522.PCD_Init();
	mfrc522.PCD_DumpVersionToSerial();

  /*int freq = 2000;
  int channel = 0;
  int resolution = 8;
  ledcSetup(channel, freq, resolution);
  ledcAttachPin(BUZZER_PIN, channel);
  ledcWriteTone(channel, freq);
  delay(100);
  ledcDetachPin(BUZZER_PIN);*/

  Serial.println("Setup done");
}

void loop()
{
  stage->handle();
}

void nextStage() {
  stage = stages[currentStage++];
  stage->init();
}
