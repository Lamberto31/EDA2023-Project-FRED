#include "states.h"
#include <Arduino.h>

void state_change(state *st, byte dest) {
  st->current = dest;
}
void state_new_cmd(state *st, byte command) {
  st->command = command;
  st->cmd_executed = false;
}
void state_cmd_executed(state *st) {
  st->cmd_executed = true;
}