#include "ParamsStates.h"
#include <Arduino.h>

void stateChange(State *st, byte dest) {
  st->current = dest;
}
void stateNewCmd(State *st, byte command) {
  st->command = command;
  st->cmd_executed = false;
}
void stateCmdExecuted(State *st) {
  st->cmd_executed = true;
}
void stateNewDirection(State *st, byte direction) {
  st->direction = direction;
}

void printParams(Params *ms) {
  Serial.println(F("LAST MEASURES"));
  Serial.print(F("Quantity"));
  Serial.print(F("\tUnits"));
  Serial.print(F("\tValue"));

  Serial.print(F("Distance US\t[cm]\t"));
  Serial.println(ms->distanceUS);

  Serial.print(F("Rev per second\t[r/s]\t"));
  Serial.println(ms->rpsOptical);

  Serial.print(F("Velocity OPT\t[cm/s]\t"));
  Serial.println(ms->velocityOptical);

  Serial.println();
}