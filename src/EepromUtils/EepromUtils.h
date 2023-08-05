#ifndef _EEPROMUTILS_H_
#define _EEPROMUTILS_H_

#include <Arduino.h>

#define ADDRESS 0
#define MAX_LEN 32

struct privateData {
  char ssid[MAX_LEN];
  char pwd[MAX_LEN];
  char key[MAX_LEN];
  };

void putPvtDataIntoEEPROM(privateData data);
privateData getPvtDataFromEEPROM();

#endif