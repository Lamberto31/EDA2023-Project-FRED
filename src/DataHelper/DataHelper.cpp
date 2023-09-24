#include "DataHelper.h"
#include <Arduino.h>

void insertNewCircularData(DataToSend *dataArray, unsigned long deltaT, Measures ms, unsigned int elementIndex, byte elementMax) {
  if (elementIndex < elementMax)
  {
    dataArray->deltaT = deltaT;
    dataArray->field1 = ms.distanceUS;
    dataArray->field2 = ms.distanceOptical;
    dataArray->field3 = ms.distanceUSFiltered;
    dataArray->field4 = ms.rpsOptical;
    dataArray->field5 = ms.velocityUS;
    dataArray->field6 = ms.velocityOptical;
    dataArray->field7 = ms.velocityOpticalFiltered;
  }
  else {
    double tempField1 = ms.distanceUS;;
    double tempField2 = ms.distanceOptical;
    double tempField3 = ms.distanceUSFiltered;
    double tempField4 = ms.rpsOptical;
    double tempField5 = ms.velocityUS;
    double tempField6 = ms.velocityOptical;
    double tempField7 = ms.velocityOpticalFiltered;
    for (byte i = 0; i < elementMax; i++)
    {
      swapDouble(dataArray->field1, tempField1);
      swapDouble(dataArray->field2, tempField2);
      swapDouble(dataArray->field3, tempField3);
      swapDouble(dataArray->field4, tempField4);
      swapDouble(dataArray->field5, tempField5);
      swapDouble(dataArray->field6, tempField6);
      swapDouble(dataArray->field7, tempField7);
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
  Serial.print(F("\tfield3"));
  Serial.print(F("\tfield4"));
  Serial.print(F("\tfield5"));
  Serial.print(F("\tfield6"));
  Serial.println(F("\tfield7"));
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
    Serial.print(dataArray->field3);

    Serial.print(F("\t"));
    Serial.print(dataArray->field4);

    Serial.print(F("\t"));
    Serial.print(dataArray->field5);

    Serial.print(F("\t"));
    Serial.print(dataArray->field6);

    Serial.print(F("\t"));
    Serial.println(dataArray->field7);
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