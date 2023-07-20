#include "states.h"
#include <Arduino.h>

void stateChange(state *st, byte dest) {
  st->current = dest;
}
void stateNewCmd(state *st, byte command) {
  st->command = command;
  st->cmd_executed = false;
}
void stateCmdExecuted(state *st) {
  st->cmd_executed = true;
}
void stateNewDirection(state *st, byte direction) {
  st->direction = direction;
}