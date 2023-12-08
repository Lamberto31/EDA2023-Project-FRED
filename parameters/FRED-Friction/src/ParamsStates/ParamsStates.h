#ifndef _PARAMSSTATES_H_
#define _PARAMSSTATES_H_

#include <Arduino.h>

#define STATE_SETUP 0
#define STATE_FREE 1
#define STATE_SEARCH 2
#define STATE_MEASURE 3
#define STATE_READ 4

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

struct Measures {
  double distanceUS;
  double distanceUSFiltered;
  double velocityUS;

  double distanceOptical;
  double rpsOptical;
  double velocityOptical;
  double velocityOpticalFiltered;

  bool sent;
};

void stateChange(State *st, byte dest);
void stateNewCmd(State *st, byte command);
void stateCmdExecuted(State *st);
void stateNewDirection(State *st, byte direction);

void printMeasures(Measures *ms);

#endif