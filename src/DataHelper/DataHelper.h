#ifndef _DATAHELPER_H_
#define _DATAHELPER_H_

#include <Arduino.h>

struct dataToSend {
  unsigned long deltaT;
  double field1;
  double field2;
  };

void insertNewData(dataToSend *dataArray, unsigned long deltaT, double field1, double field2);

void jsonBuildForSend(dataToSend *dataArray, unsigned int elements, char key[], char json[]);

void insertNewCircularData(dataToSend *dataArray, unsigned long deltaT, double field1, double field2, unsigned int elementIndex, byte elementMax);

void swapDouble(double &a, double &b);

void readAndPrintData(dataToSend *dataArray, byte elements);

#endif