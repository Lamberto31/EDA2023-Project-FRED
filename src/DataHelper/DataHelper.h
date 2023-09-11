#ifndef _DATAHELPER_H_
#define _DATAHELPER_H_

#include <Arduino.h>

#include "../States/States.h"

struct DataToSend {
  unsigned long deltaT;
  double field1;
  double field2;
  double field3;
  double field4;
  double field5;
  };

void jsonBuildForSend(DataToSend *dataArray, unsigned int elements, char key[], char json[]);

void insertNewCircularData(DataToSend *dataArray, unsigned long deltaT, Measures ms, unsigned int elementIndex, byte elementMax);

void swapDouble(double &a, double &b);

void readAndPrintData(DataToSend *dataArray, byte elements);

#endif