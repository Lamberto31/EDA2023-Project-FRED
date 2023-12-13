#include "DataHelper.h"
#include <Arduino.h>

void insertNewCircularData(DataToSend *dataArray, unsigned long deltaT, Measures ms, unsigned int elementIndex, byte elementMax) {
  if (elementIndex < elementMax)
  {
    dataArray->deltaT = deltaT;
    dataArray->field1 = ms.distanceUS;
    dataArray->field2 = ms.distanceOptical;
    dataArray->field4 = ms.rpsOptical;
    dataArray->field5 = ms.velocityUS;
    dataArray->field6 = ms.velocityOptical;
  }
  else {
    double tempField1 = ms.distanceUS;;
    double tempField2 = ms.distanceOptical;
    double tempField4 = ms.rpsOptical;
    double tempField5 = ms.velocityUS;
    double tempField6 = ms.velocityOptical;
    for (byte i = 0; i < elementMax; i++)
    {
      swapDouble(dataArray->field1, tempField1);
      swapDouble(dataArray->field2, tempField2);
      swapDouble(dataArray->field4, tempField4);
      swapDouble(dataArray->field5, tempField5);
      swapDouble(dataArray->field6, tempField6);
      dataArray--;
    }
  }
}

void readAndPrintData(DataToSend *dataArray, byte elements) {
  Serial.println(F("SEND BUFFER ELEMENTS"));
  Serial.print(F("[i]"));
  Serial.print(F("\tdeltaT"));
  Serial.print(F("\tfield1"));
  Serial.print(F("\tfield2"));
  Serial.print(F("\tfield4"));
  Serial.print(F("\tfield5"));
  Serial.println(F("\tfield6"));
  for (byte i = 0; i < elements; i++)
  {
    Serial.print(F("["));
    Serial.print(i);
    Serial.print(F("]"));

    Serial.print(F("\t"));
    Serial.print(dataArray->deltaT);

    Serial.print(F("\t"));
    Serial.print(dataArray->field1);

    Serial.print(F("\t"));
    Serial.print(dataArray->field2);

    Serial.print(F("\t"));
    Serial.print(dataArray->field4);

    Serial.print(F("\t"));
    Serial.print(dataArray->field5);

    Serial.print(F("\t"));
    Serial.print(dataArray->field6);
    //Serial.println();
    dataArray++;
  }
  Serial.println();
}

void swapDouble(double &a, double &b) {
  double c = a;
  a = b;
  b = c;
}