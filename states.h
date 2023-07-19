#ifndef _STATES_H_
#define _STATES_H_

#include <Arduino.h>

#define STATE_SETUP 0
#define STATE_FREE 1
#define STATE_SEARCH 2
#define STATE_MEASURE 3
#define STATE_READING 4

struct state {
  byte current;
  byte command;
  bool cmd_executed;
};

void stateChange(state *st, byte dest);
void stateNewCmd(state *st, byte command);
void stateCmdExecuted(state *st);

#endif