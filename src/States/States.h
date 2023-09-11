#ifndef _STATES_H_
#define _STATES_H_

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

struct Measurement {
  double measuredDist;
  double measuredFilteredDist;

  double measuredRps;
  double measuredVelocity;
  double measuredFilteredVelocity;
};

void stateChange(State *st, byte dest);
void stateNewCmd(State *st, byte command);
void stateCmdExecuted(State *st);
void stateNewDirection(State *st, byte direction);

void setMeasuredDist(Measurement *ms, double measuredDist, double measuredFilteredDist);
void setMeasuredVelocity(Measurement *ms, double measuredRps, double measuredVelocity, double measuredFilteredVelocity);

#endif