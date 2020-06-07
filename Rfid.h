#ifndef Rfid_h
#define Rfid_h

#include "Stage.h"
#include <MFRC522.h>

class Rfid: public Stage {
  public: 
    Rfid(MFRC522 &mfrc522): _mfrc522(mfrc522)
    {};
    void handle();
  private:
    MFRC522 _mfrc522;
};

#endif