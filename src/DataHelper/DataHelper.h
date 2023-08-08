#ifndef _DATAHELPER_H_
#define _DATAHELPER_H_

#include <Arduino.h>

struct dataToSend {
  unsigned long deltaT;
  double field1;
  double field2;
  };

void insertNewData(dataToSend *dataArray, unsigned long deltaT, double field1, double field2);

void jsonBuildForSend(dataToSend *dataArray, byte elements, char key[], char json[]);

void swapDouble(double &a, double &b);


#endif