#ifndef _STATES_H_
#define _STATES_H_

#include <Arduino.h>

#define STATE_SETUP 0
#define STATE_FREE 1
#define STATE_SEARCH 2
#define STATE_MEASURE 3

struct state {
  byte current;
  byte command;
  bool cmd_executed;
};

void state_change(state *st, byte dest);
void state_new_cmd(state *st, byte command);
void state_cmd_executed(state *st);

#endif