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
