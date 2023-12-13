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
  // State
  byte current;
  bool just_changed;
  // Command
  byte command;
  bool cmd_executed;
  // Input
  byte direction;
  int input;
};

struct Measures {
  // Ultrasonic sensor
  double distanceUS;
  double velocityUS;

  // Optical sensor
  double distanceOptical;
  double rpsOptical;
  double velocityOptical;

  // Used to know if already used
  bool sent;
};

void stateChange(State *st, byte dest);

void stateNewCmd(State *st, byte command);
void stateCmdExecuted(State *st);

void stateNewInput(State *st, byte direction, int input);

void printMeasures(Measures *ms);

#endif