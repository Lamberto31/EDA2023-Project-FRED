#ifndef _DATAHELPER_H_
#define _DATAHELPER_H_

#include <Arduino.h>

struct dataToSend {
  unsigned long deltaT;
  double field1;
  double field2;
  };
void insertNewData(dataToSend *dataArray, dataToSend *dataToAdd);

//TODO: creare JSON per POST
void jsonBuilder();

#endif