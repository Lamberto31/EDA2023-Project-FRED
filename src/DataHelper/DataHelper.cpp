#include "DataHelper.h"
#include <Arduino.h>

void insertNewData(dataToSend *dataArray, unsigned long deltaT, double field1, double field2) {
  dataArray->deltaT = deltaT;
  dataArray->field1 = field1;
  dataArray->field2 = field2;
}

}