#include "DataHelper.h"
#include <Arduino.h>

void jsonBuildForSend(DataToSend *dataArray, unsigned int elements, char key[], char json[]) {
  unsigned long deltaT;
  double field1;
  double field2;

  char charDeltaT[2];
  char charTempField[8];

  strcpy(json, "{\"write_api_key\":\"");
  strcat(json, key);
  strcat(json, "\",\"updates\":[");

  for (byte i = 0; i < elements; i++)
  {
    deltaT = dataArray[i].deltaT;
    field1 = dataArray[i].field1;
    field2 = dataArray[i].field2;

    strcat(json,"{\"delta_t\":");
    itoa(deltaT, charDeltaT, 10);
    strcat(json, charDeltaT);

    strcat(json,",\"field1\":");
    dtostrf(field1, 8, 4, charTempField);
    strcat(json, charTempField);

    strcat(json,",\"field2\":");
    dtostrf(field2, 8, 4, charTempField);
    strcat(json, charTempField);
    strcat(json, "}");
    if (i < elements - 1) strcat(json, ",");
  }
  strcat(json, "]}");
}

void insertNewCircularData(DataToSend *dataArray, unsigned long deltaT, double field1, double field2, unsigned int elementIndex, byte elementMax) {
  if (elementIndex < elementMax)
  {
    dataArray->deltaT = deltaT;
    dataArray->field1 = field1;
    dataArray->field2 = field2;
  }
  else {
    double tempField1 = field1;
    double tempField2 = field2;
    for (byte i = 0; i < elementMax; i++)
    {
      swapDouble(dataArray->field1, tempField1);
      swapDouble(dataArray->field2, tempField2);
      dataArray--;
    }
  }
}

void readAndPrintData(DataToSend *dataArray, byte elements) {
  Serial.println(F("SEND BUFFER ELEMENTS"));
  Serial.print(F("[i]"));
  Serial.print(F("\tdeltaT"));
  Serial.print(F("\tfield1"));
  Serial.println(F("\tfield2"));
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
    Serial.println(dataArray->field2);
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