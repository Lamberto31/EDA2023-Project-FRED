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
  Serial.println(F("\tRaw"));

  Serial.print(F("Distance US\t[cm]\t"));
  Serial.println(ms->distanceUS);

  Serial.print(F("Distance OPT\t[cm]\t"));
  Serial.println(ms->distanceOptical);

  Serial.print(F("Rev per second\t[r/s]\t"));
  Serial.println(ms->rpsOptical);

  Serial.print(F("Velocity US\t[cm/s]\t"));
  Serial.println(ms->velocityUS);

  Serial.print(F("Velocity OPT\t[cm/s]\t"));
  Serial.println(ms->velocityOptical);

  Serial.print(F("Pulses per second\t[pps]\t"));
  Serial.println(ms->ppsOptical);

  Serial.println();
}