#include "Rfid.h"

void Rfid::handle() {
  // Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
  if ( ! _mfrc522.PICC_IsNewCardPresent()) {
    return;
  }

  // Select one of the cards
  if ( ! _mfrc522.PICC_ReadCardSerial()) {
    return;
  }

  // Dump debug info about the card; PICC_HaltA() is automatically called
  _mfrc522.PICC_DumpToSerial(&(_mfrc522.uid));
};