#include "DataHelper.h"
#include <Arduino.h>

void insertNewData(dataToSend *dataArray, dataToSend *dataToAdd) {
  dataArray->deltaT = dataToAdd->deltaT;
  dataArray->field1 = dataToAdd->field1;
  dataArray->field2 = dataToAdd->field2;
}