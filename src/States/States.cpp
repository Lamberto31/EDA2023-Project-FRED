#include "States.h"
#include <Arduino.h>

void stateChange(State *st, byte dest) {
  st->current = dest;
  st->just_changed = true;
}

void stateNewCmd(State *st, byte command) {
  st->command = command;
  st->cmd_executed = false;
}
void stateCmdExecuted(State *st) {
  st->cmd_executed = true;
}

void stateNewInput(State *st, byte direction, int input) {
  st->direction = direction;
  st->input = input;
}

void printMeasures(Measures *ms) {
  Serial.println(F("LAST MEASURES"));
  Serial.print(F("Quantity"));
  Serial.print(F("\tUnits"));
  Serial.print(F("\tRaw"));
  Serial.println(F("\tFiltered"));

  Serial.print(F("Distance US\t[cm]\t"));
  Serial.print(ms->distanceUS);
  Serial.print(F("\t"));
  Serial.println(ms->distanceUSFiltered);

  Serial.print(F("Distance OPT\t[cm]\t"));
  Serial.print(ms->distanceOptical);
  Serial.print(F("\t"));
  Serial.println(F("---"));

  Serial.print(F("Rev per second\t[r/s]\t"));
  Serial.print(ms->rpsOptical);
  Serial.print(F("\t"));
  Serial.println(F("---"));

  Serial.print(F("Velocity US\t[cm/s]\t"));
  Serial.print(ms->velocityUS);
  Serial.print(F("\t"));
  Serial.println(F("---"));

  Serial.print(F("Velocity OPT\t[cm/s]\t"));
  Serial.print(ms->velocityOptical);
  Serial.print(F("\t"));
  Serial.println(ms->velocityOpticalFiltered);

  Serial.println();
}