#include "EepromUtils.h"

#define READONLY 1

void setup() {
  Serial.begin(9600);

  privateData pvt = {
    "SSID",
    "PWD",
    "API-KEY"
  };

  if(!READONLY) putPvtDataIntoEEPROM(pvt);

  privateData read;
  read = getPvtDataFromEEPROM();

  Serial.println("Read custom object from EEPROM: ");
  Serial.println(read.ssid);
  Serial.println(read.pwd);
  Serial.println(read.channelId);
  Serial.println(read.writeKey);
}

void loop() {
}
