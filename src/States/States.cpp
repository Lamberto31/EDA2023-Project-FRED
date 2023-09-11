#include "States.h"
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

void setMeasuredDist(Measures *ms, double measuredDist, double measuredFilteredDist) {
  ms->measuredDist = measuredDist;
  ms->measuredFilteredDist = measuredFilteredDist;
}
void setMeasuredVelocity(Measures *ms, double measuredRps, double measuredVelocity, double measuredFilteredVelocity) {
  ms->measuredRps = measuredRps;
  ms->measuredVelocity = measuredVelocity;
  ms->measuredFilteredVelocity = measuredFilteredVelocity;
}
void printMeasures(Measures *ms) {
  Serial.println(F("LAST MEASURES"));
  Serial.print(F("Quantity"));
  Serial.print(F("\tUnits"));
  Serial.print(F("\tRaw"));
  Serial.println(F("\tFiltered"));

  Serial.print(F("Distance\t[cm]\t"));
  Serial.print(ms->measuredDist);
  Serial.print(F("\t"));
  Serial.println(ms->measuredFilteredDist);

  Serial.print(F("RPS\t[r/s]\t"));
  Serial.print(ms->measuredRps);
  Serial.print(F("\t"));
  Serial.println(F("---"));

  Serial.print(F("Velocity\t[cm/s]\t"));
  Serial.print(ms->measuredVelocity);
  Serial.print(F("\t"));
  Serial.println(ms->measuredFilteredVelocity);

  Serial.println();
}