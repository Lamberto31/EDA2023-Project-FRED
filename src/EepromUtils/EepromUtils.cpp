#include "EepromUtils.h"
#include <Arduino.h>
#include <EEPROM.h>

void putPvtDataIntoEEPROM(PrivateData data) {
  EEPROM.put(ADDRESS, data);
}

PrivateData getPvtDataFromEEPROM() {
  PrivateData pvtData;
  EEPROM.get(ADDRESS, pvtData);
  return pvtData;
}