#include "EepromUtils.h"

void setup() {
  Serial.begin(9600);

  privateData pvt = {
    "SSID",
    "PWD",
    "API-KEY"
  };

  putPvtDataIntoEEPROM(pvt);

  privateData read;
  read = getPvtDataFromEEPROM();

  Serial.println("Read custom object from EEPROM: ");
  Serial.println(read.ssid);
  Serial.println(read.pwd);
  Serial.println(read.key);
}

void loop() {
}
