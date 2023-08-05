#include "EepromUtils.h"
#include <Arduino.h>
#include <EEPROM.h>

void putPvtDataIntoEEPROM(privateData data) {
  EEPROM.put(ADDRESS, data);
}

privateData getPvtDataFromEEPROM() {
  privateData pvtData;
  EEPROM.get(ADDRESS, pvtData);
  return pvtData;
}