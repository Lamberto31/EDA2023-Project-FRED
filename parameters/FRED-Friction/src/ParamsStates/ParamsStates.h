#ifndef _PARAMSSTATES_H_
#define _PARAMSSTATES_H_

#include <Arduino.h>

#define STATE_SETUP 0
#define STATE_IDLE 1
#define STATE_INPUT_MAX 2
#define STATE_INPUT_0 3
#define STATE_STOP 4

#define DIRECTION_STOP 0
#define DIRECTION_FORWARD 1
#define DIRECTION_BACKWARD 2
#define DIRECTION_RIGHT 3
#define DIRECTION_LEFT 4

struct State {
  byte current;
  byte command;
  bool cmd_executed;
  byte direction;
};

struct Params {
  double distanceUS;

  double rpsOptical;
  double velocityOptical;

  unsigned long currentTime;

  bool recorded;

  byte state;
};

void stateChange(State *st, byte dest);
void stateNewCmd(State *st, byte command);
void stateCmdExecuted(State *st);
void stateNewDirection(State *st, byte direction);

void printParams(Params *ms);

#endif