#define IR_RECEIVE_PIN 10     // Defined here because the library requires it
#define NO_LED_FEEDBACK_CODE  // Defined here because the library requires it
#include "TinyIRReceiver.hpp"
#include <Servo.h>

// Custom library for states handling
#include "src/States/States.h"

// Custom library for EEPROM memory handling
#include "src/EepromUtils/EepromUtils.h"

// Custom library for build data to send to remote server
#include "src/DataHelper/DataHelper.h"

// Digital Pins
#define PIN_HC05_TX 0
#define PIN_HC05_RX 1
#define PIN_BLUETOOTH_STATE 2
#define PIN_OPTICAL 3
#define PIN_ULTRASONIC_ECHO 4
#define PIN_ULTRASONIC_TRIG 5
#define PIN_MOTOR_ENB 6
#define PIN_MOTOR_IN4 7
#define PIN_MOTOR_IN3 8
#define PIN_SERVO_HORIZ 9
// #define PIN_IR_RECV 10
#define PIN_MOTOR_ENA 11
#define PIN_MOTOR_IN2 12
#define PIN_MOTOR_IN1 13

// States
State robotState = { STATE_SETUP, 0, true, DIRECTION_STOP };
Measures robotMeasures = {0, 0, 0, 0, 0, 0, 0, true};

// Functionalities active/disabled
#define DEBUG_ACTIVE 0

// PARAMETERS
// Measure
#define PERIOD_MEASURE 100  // [ms] between each measurement. Min value 60, may cause error on ultrasonic measure if lower
#define DECIMALS 4  // [digits] Max value 4, it may cause buffer overflow if greater
// Optical
#define WHEEL_ENCODER_HOLES 20  // Holes in wheel encoder (when counted indicates one round)
#define WHEEL_DIAMETER 65  //[mm] Diameter of wheel
// Movement control
#define STOP_TRESHOLD 0.1  // [cm] Tolerance for diffDist
#define SLOW_TRESHOLD 100  // [cm] Treshold used to go at max speed until reached
#define SLOW_SPEED_MIN 100  // [analog] [0-255] Min value for slow speed
#define SLOW_SPEED_MAX 150  // [analog] [0-255] Max value for slow speed
#define SLOW_FACTOR_MAX 15  // [adim] Max value for slowFactor to prevent too slow speed
#define SLOW_FACTOR_STOP 10  // [adim] Min value for slowFactor to allow stop from checkDistance
// Custom distance [cm]
#define CUSTOM_DIST_MIN 10  // [cm]
#define CUSTOM_DIST_MAX 500  // [cm]
#define CUSTOM_DIST_CHAR 4  // [chars] Max value 4, it may cause buffer overflow if greater
// Bluetooth
#define BLUETOOTH_WAIT_CONNECTION 10000  // [ms] Wait time to receive Bluetooth connection
#define PERIOD_BLUETOOTH 500  // [ms] between each message to Bluetooth. Min value 1000, may cause error response if lower
// TODO_CAPIRE: SERVE O COME MODIFICARE (in particolare il SEND_BUFFER_SIZE che potrebbe diventare PERIOD_BLUETOOTH / PERIOD_MEASURE )
#define PERIOD_SERVER 15000  // [ms] between each message to server. Min value 15000, may cause error response if lower (server allow one message each 15s)
#define PERIOD_MEASURETOSEND 3000  // [ms] between each insertion of data into the structure. Suggested value 3000, it's ok if greater but a lower value may cause high memory consumption
#define SEND_BUFFER_SIZE PERIOD_SERVER / PERIOD_MEASURETOSEND  // [byte] Can be changed to arbitrary value, it's better to don't go over 5 (tested and working) due to memory consumption (see where it's used)
// Servo
#define SERVO_HORIZ_CENTER 100 // [angle] [0-180] Angle considered as center for servo, it depends on the construction
// Feedback Led
#define FEEDBACK_BLINK_READ_RECEIVE 1  // [adim] Number of blinks for feedback led when reading a custom distance
#define FEEDBACK_DURATION_READ_RECEIVE 100  // [ms] Duration of each blink for feedback led when reading a custom distance

// IR
// Button-Command
#define IR_BUTTON_1 0x45
#define IR_BUTTON_2 0x46
#define IR_BUTTON_3 0x47
#define IR_BUTTON_4 0x44
#define IR_BUTTON_5 0x40
#define IR_BUTTON_6 0x43
#define IR_BUTTON_7 0x7
#define IR_BUTTON_8 0x15
#define IR_BUTTON_9 0x9
#define IR_BUTTON_AST 0x16
#define IR_BUTTON_0 0x19
#define IR_BUTTON_HASH 0xD
#define IR_BUTTON_UP 0x18
#define IR_BUTTON_DOWN 0x52
#define IR_BUTTON_RIGHT 0x5A
#define IR_BUTTON_LEFT 0x8
#define IR_BUTTON_OK 0x1C

// Used by TinyIRReceiver library
volatile struct TinyIRReceiverCallbackDataStruct sCallbackData;

// Measure
unsigned long previousMillisMeasure;
unsigned long currentMillisMeasure;

// Optical
volatile int opticalPulses = 0;

// Movement control
double diffDist;
bool firstCheck = true;
byte speedSlowFactor = 0;

// Bluetooth
bool bluetoothConnected = false;
unsigned long previousMillisMeasureToSend;
unsigned long currentMillisMeasureToSend;
// TODO_CAPIRE: USARE PER MANDARE PIU' MISURE VIA BLUETOOTH?
// DataToSend sendBuffer[5];
DataToSend sendBuffer[SEND_BUFFER_SIZE];
unsigned int sendBufferIndex = 0;

// Servomotor
Servo servoH;

// Custom distance [cm]
char customDist[CUSTOM_DIST_CHAR];
byte customDistIdx = 0;
int numericCustomDist = 0;

// Debug macro
#if DEBUG_ACTIVE == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#define debuglnDecimal(x, n) Serial.println(x, n)
#define debugF(x) Serial.print(F(x))
#define debugFln(x) Serial.println(F(x))
#else
#define debug(x)
#define debugln(x)
#define debuglnDecimal(x, n)
#define debugF(x)
#define debugFln(x)
#endif

void setup() {
  Serial.begin(9600);

  // Feedback led
  pinMode(LED_BUILTIN, OUTPUT);

  //Start time counters
  previousMillisMeasure = millis();
  previousMillisMeasureToSend = millis();

  // IR Receiver
  if (!initPCIInterruptForTinyReceiver()) {
    debugF("No interrupt available");
  }

  // Ultrasonic
  pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);
  pinMode(PIN_ULTRASONIC_ECHO, INPUT);

  // Distance
  memset(customDist, '0', sizeof(customDist));

  // Bluetooth
  pinMode(PIN_BLUETOOTH_STATE, INPUT);
  delay(500);
  bluetoothConnected = bluetoothConnection(true);

  // Servomotor
  servoH.attach(PIN_SERVO_HORIZ);
  servoH.write(SERVO_HORIZ_CENTER);

  // Feedback
  servoH.write(SERVO_HORIZ_CENTER - 15);
  delay(1000);
  servoH.write(SERVO_HORIZ_CENTER + 15);
  delay(1000);
  servoH.write(SERVO_HORIZ_CENTER);
  delay(500);
  servoH.detach();
  delay(500);

  // Motors
  pinMode(PIN_MOTOR_ENA, OUTPUT);
  pinMode(PIN_MOTOR_ENB, OUTPUT);
  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  pinMode(PIN_MOTOR_IN3, OUTPUT);
  pinMode(PIN_MOTOR_IN4, OUTPUT);

  // Feedback
  runMotors(DIRECTION_BACKWARD, 255);
  delay(500);
  runMotors(DIRECTION_FORWARD, 255);
  delay(500);
  runMotors(DIRECTION_STOP, 0);

  // Optical
  delay(1000);
  attachInterrupt(digitalPinToInterrupt(PIN_OPTICAL), countPulses, RISING);

  stateChange(&robotState, STATE_FREE);
}

void loop() {
  switch (robotState.current) {
    // Free state handling
    case STATE_FREE: {
      if (robotState.direction == DIRECTION_FORWARD) {
        preventDamage(CUSTOM_DIST_MIN);
      }
      if (!robotState.cmd_executed) {
        switch (robotState.command) {
          case IR_BUTTON_OK: {
            if (DEBUG_ACTIVE) printMeasures(&robotMeasures);
            runMotors(DIRECTION_STOP, 0);
            break;
          }
          case IR_BUTTON_UP: {
            runMotors(DIRECTION_FORWARD, 255);
            break;
          }
          case IR_BUTTON_DOWN: {
            runMotors(DIRECTION_BACKWARD, 100);
            break;
          }
          case IR_BUTTON_RIGHT: {
            runMotors(DIRECTION_RIGHT, 100);
            break;
          }
          case IR_BUTTON_LEFT: {
            runMotors(DIRECTION_LEFT, 100);
            break;
          }
          case IR_BUTTON_HASH: {
            runMotors(DIRECTION_STOP, 0);
            stateChange(&robotState, STATE_MEASURE);
            break;
          }
          case IR_BUTTON_AST: {
            runMotors(DIRECTION_STOP, 0);
            stateChange(&robotState, STATE_READ);
            // Feedback led
            digitalWrite(LED_BUILTIN, HIGH);
            break;
          }
        }
        stateCmdExecuted(&robotState);
      }
      break;
    }
    // Reading state handling
    case STATE_READ: {
      if (!robotState.cmd_executed) {
        switch (robotState.command) {
          case IR_BUTTON_1: {
            ledFeedback(FEEDBACK_BLINK_READ_RECEIVE, FEEDBACK_DURATION_READ_RECEIVE, true);
            readCustomDistance('1');
            break;
          }
          case IR_BUTTON_2: {
            ledFeedback(FEEDBACK_BLINK_READ_RECEIVE, FEEDBACK_DURATION_READ_RECEIVE, true);
            readCustomDistance('2');
            break;
          }
          case IR_BUTTON_3: {
            ledFeedback(FEEDBACK_BLINK_READ_RECEIVE, FEEDBACK_DURATION_READ_RECEIVE, true);
            readCustomDistance('3');
            break;
          }
          case IR_BUTTON_4: {
            ledFeedback(FEEDBACK_BLINK_READ_RECEIVE, FEEDBACK_DURATION_READ_RECEIVE, true);
            readCustomDistance('4');
            break;
          }
          case IR_BUTTON_5: {
            ledFeedback(FEEDBACK_BLINK_READ_RECEIVE, FEEDBACK_DURATION_READ_RECEIVE, true);
            readCustomDistance('5');
            break;
          }
          case IR_BUTTON_6: {
            ledFeedback(FEEDBACK_BLINK_READ_RECEIVE, FEEDBACK_DURATION_READ_RECEIVE, true);
            readCustomDistance('6');
            break;
          }
          case IR_BUTTON_7: {
            ledFeedback(FEEDBACK_BLINK_READ_RECEIVE, FEEDBACK_DURATION_READ_RECEIVE, true);
            readCustomDistance('7');
            break;
          }
          case IR_BUTTON_8: {
            ledFeedback(FEEDBACK_BLINK_READ_RECEIVE, FEEDBACK_DURATION_READ_RECEIVE, true);
            readCustomDistance('8');
            break;
          }
          case IR_BUTTON_9: {
            ledFeedback(FEEDBACK_BLINK_READ_RECEIVE, FEEDBACK_DURATION_READ_RECEIVE, true);
            readCustomDistance('9');
            break;
          }
          case IR_BUTTON_0: {
            ledFeedback(FEEDBACK_BLINK_READ_RECEIVE, FEEDBACK_DURATION_READ_RECEIVE, true);
            readCustomDistance('0');
            break;
          }
          case IR_BUTTON_AST: {
            if (composeNumericDistance()) stateChange(&robotState, STATE_SEARCH); else stateChange(&robotState, STATE_FREE);
            debugF("numericCustomDist = ");
            debugln(numericCustomDist);
            if (bluetoothConnected) bluetoothSendInfo("Custom Distance", numericCustomDist);
            // Feedback led
            digitalWrite(LED_BUILTIN, LOW);
            break;
          }
          case IR_BUTTON_OK: {
            resetCustomDistance();
            stateChange(&robotState, STATE_FREE);
            // Feedback led
            digitalWrite(LED_BUILTIN, LOW);
            break;
          }
          case IR_BUTTON_HASH: {
            resetCustomDistance();
            stateChange(&robotState, STATE_MEASURE);
            // Feedback led
            digitalWrite(LED_BUILTIN, LOW);
            break;
          }
        }
      stateCmdExecuted(&robotState);
      }
      break;
    }
    // Search state handling
    case STATE_SEARCH: {
      checkDistance();
      if (!robotState.cmd_executed) {
        switch (robotState.command) {
          case IR_BUTTON_OK: {
            runMotors(DIRECTION_STOP, 0);
            speedSlowFactor = 0;
            firstCheck = true;
            stateChange(&robotState, STATE_FREE);
            break;
          }
          case IR_BUTTON_HASH: {
            runMotors(DIRECTION_STOP, 0);
            speedSlowFactor = 0;
            firstCheck = true;
            stateChange(&robotState, STATE_MEASURE);
            break;
          }
        }
        stateCmdExecuted(&robotState);
      }
      break;
    }
    // Measure state handling
    // TODO: Capire che fare di questo stato, al momento non fa più nulla
    case STATE_MEASURE: {
      sendBufferIndex = 0;
      memset(sendBuffer, 0, sizeof(sendBuffer));
      if (!robotState.cmd_executed) {
        switch (robotState.command) {
          case IR_BUTTON_OK: {
            stateChange(&robotState, STATE_FREE);
            break;
          }
          case IR_BUTTON_AST: {
            stateChange(&robotState, STATE_READ);
            // Feedback led
            digitalWrite(LED_BUILTIN, HIGH);
            break;
          }
        }
        stateCmdExecuted(&robotState);
      }
      break;
    }
  }
  // Actions performed for each state
  // Measure
  currentMillisMeasure = millis();
  if (currentMillisMeasure - previousMillisMeasure >= PERIOD_MEASURE) {
    measureAll(currentMillisMeasure - previousMillisMeasure);
    previousMillisMeasure = millis();
  }

  // Send measure with Bluetooth
  if (currentMillisMeasureToSend - previousMillisMeasureToSend >= PERIOD_BLUETOOTH) {
    servoH.attach(PIN_SERVO_HORIZ);
    servoH.write(SERVO_HORIZ_CENTER);
    delay(100);
    servoH.detach();

    bluetoothConnection(false);
    if (bluetoothConnected && !robotMeasures.sent) bluetoothSendMeasure();
    previousMillisMeasureToSend = millis();
  }

  // TODO_CAPIRE: CAPIRE SE SERVE
  // Insert new data in sendBuffer
  currentMillisMeasureToSend = millis();
  if (currentMillisMeasureToSend - previousMillisMeasureToSend >= PERIOD_MEASURETOSEND) {
    // insertNewData(&sendBuffer[sendBufferIndex], (PERIOD_MEASURETOSEND/1000)*sendBufferIndex, robotMeasures.distanceUS, robotMeasures.distanceUSFiltered);
    insertNewCircularData(&sendBuffer[min(sendBufferIndex, SEND_BUFFER_SIZE - 1)], (PERIOD_MEASURETOSEND / 1000) * sendBufferIndex, robotMeasures, sendBufferIndex, SEND_BUFFER_SIZE);
    sendBufferIndex++;

    if (DEBUG_ACTIVE) readAndPrintData(&sendBuffer[0], SEND_BUFFER_SIZE);

    previousMillisMeasureToSend = millis();
  }
}

// This is the function, which is called if a complete ir command was received
// It runs in an ISR context with interrupts enabled
void handleReceivedTinyIRData(uint8_t aAddress, uint8_t aCommand, uint8_t aFlags) {
  if (DEBUG_ACTIVE) printTinyReceiverResultMinimal(&Serial, aAddress, aCommand, aFlags);
  // Ignore repeat commands
  if (aFlags != IRDATA_FLAGS_IS_REPEAT) {
    stateNewCmd(&robotState, aCommand);
  }
}

void ledFeedback(byte blinkNumber, unsigned int blinkDuration, bool reverse) {
  for (byte blinkCount = 0; blinkCount < blinkNumber; blinkCount++) {
    if (!reverse) digitalWrite(LED_BUILTIN, HIGH);
    else digitalWrite(LED_BUILTIN, LOW);
    delay(blinkDuration);
    if (!reverse) digitalWrite(LED_BUILTIN, LOW);
    else digitalWrite(LED_BUILTIN, HIGH);
    delay(blinkDuration);
  }
}

void runMotors(byte direction, byte speed) {
  switch (direction) {
    case DIRECTION_STOP: {
      debugFln("Stop");
      digitalWrite(PIN_MOTOR_IN1, LOW);
      digitalWrite(PIN_MOTOR_IN2, LOW);
      digitalWrite(PIN_MOTOR_IN3, LOW);
      digitalWrite(PIN_MOTOR_IN4, LOW);
      analogWrite(PIN_MOTOR_ENA, 0);
      analogWrite(PIN_MOTOR_ENB, 0);
      stateNewDirection(&robotState, DIRECTION_STOP);
      break;
    }
    case DIRECTION_FORWARD: {
      debugFln("Avanti");
      digitalWrite(PIN_MOTOR_IN1, HIGH);
      digitalWrite(PIN_MOTOR_IN2, LOW);
      digitalWrite(PIN_MOTOR_IN3, HIGH);
      digitalWrite(PIN_MOTOR_IN4, LOW);
      analogWrite(PIN_MOTOR_ENA, speed);
      analogWrite(PIN_MOTOR_ENB, speed);
      stateNewDirection(&robotState, DIRECTION_FORWARD);
      break;
    }
    case DIRECTION_BACKWARD: {
      debugFln("Indietro");
      digitalWrite(PIN_MOTOR_IN1, LOW);
      digitalWrite(PIN_MOTOR_IN2, HIGH);
      digitalWrite(PIN_MOTOR_IN3, LOW);
      digitalWrite(PIN_MOTOR_IN4, HIGH);
      analogWrite(PIN_MOTOR_ENA, speed);
      analogWrite(PIN_MOTOR_ENB, speed);
      stateNewDirection(&robotState, DIRECTION_BACKWARD);
      break;
    }
    case DIRECTION_RIGHT: {
      debugFln("Destra");
      digitalWrite(PIN_MOTOR_IN1, HIGH);
      digitalWrite(PIN_MOTOR_IN2, LOW);
      digitalWrite(PIN_MOTOR_IN3, LOW);
      digitalWrite(PIN_MOTOR_IN4, HIGH);
      analogWrite(PIN_MOTOR_ENA, speed);
      analogWrite(PIN_MOTOR_ENB, speed);
      stateNewDirection(&robotState, DIRECTION_RIGHT);
      break;
    }
    case DIRECTION_LEFT: {
      debugFln("Sinistra");
      digitalWrite(PIN_MOTOR_IN1, LOW);
      digitalWrite(PIN_MOTOR_IN2, HIGH);
      digitalWrite(PIN_MOTOR_IN3, HIGH);
      digitalWrite(PIN_MOTOR_IN4, LOW);
      analogWrite(PIN_MOTOR_ENA, speed);
      analogWrite(PIN_MOTOR_ENB, speed);
      stateNewDirection(&robotState, DIRECTION_LEFT);
      break;
    }
  }
}

// MEASURE
void measureAll(unsigned long deltaT) {
  robotMeasures.sent = false;
  double prevDistance = robotMeasures.distanceUS;
  // double prevFilteredDistance = robotMeasures.distanceUSFiltered;
  
  int pulses = opticalPulses;
  opticalPulses = 0;
  int directionSign;
  double travelledRevolution;
  double travelledDistance;

  // Distance from ultrasonic
  robotMeasures.distanceUS = measureDistance();
  //DEBUG_TEMP
  robotMeasures.distanceUSFiltered = int(robotMeasures.distanceUS);

  // Velocity from ultrasonic
  robotMeasures.velocityUS = (robotMeasures.distanceUS - prevDistance) / (deltaT * 0.001);

  // Position from optical
  directionSign = measureDirection();
  travelledRevolution = (pulses / (double)WHEEL_ENCODER_HOLES);
  travelledDistance = PI * (WHEEL_DIAMETER * 0.1) * travelledRevolution * directionSign;
  robotMeasures.distanceOptical = travelledDistance + prevDistance;

  // Velocity from optical
  robotMeasures.rpsOptical = travelledRevolution / (deltaT * 0.001);
  robotMeasures.velocityOptical = travelledDistance / (deltaT * 0.001);
  //DEBUG_TEMP
  robotMeasures.velocityOpticalFiltered = int(robotMeasures.velocityOptical);
}

// DISTANCE
double measureDistance() {
  long tripTime;
  double distance;

  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);

  tripTime = pulseIn(PIN_ULTRASONIC_ECHO, HIGH);
  distance = 0.0343 * tripTime / 2.0;

  return distance;
}

void readCustomDistance(char digit) {
  if (customDistIdx == (CUSTOM_DIST_CHAR - 1)) {
    resetCustomDistance();
    stateChange(&robotState, STATE_FREE);
    // Feedback led
    digitalWrite(LED_BUILTIN, LOW);
  } else {
    customDist[customDistIdx] = digit;
    customDistIdx++;
  }
}

bool composeNumericDistance() {
  // No digits
  if (customDistIdx == 0) {
    numericCustomDist = 0;
    return false;
  }
  // Create numericCustomDist
  char buff[customDistIdx + 1];
  for (byte i = 0; i <= customDistIdx - 1; i++) {
    buff[i] = customDist[i];
  }
  buff[customDistIdx] = '\0';
  numericCustomDist = atoi(buff);
  resetCustomDistance();

  // Check if not in [CUSTOM_DIST_MIN, CUSTOM_DIST_MAX]
  if (numericCustomDist < CUSTOM_DIST_MIN || numericCustomDist > CUSTOM_DIST_MAX) {
    numericCustomDist = 0;
    return false;
  }
  return true;
}

void resetCustomDistance() {
  memset(customDist, '0', sizeof(customDist));
  customDistIdx = 0;
}

void checkDistance() {
  // Measure diffrence between current and custom distance
  diffDist = robotMeasures.distanceUS - numericCustomDist;

  // Move to the custom distance if first check
  if (firstCheck) {
    if (diffDist <= STOP_TRESHOLD + SLOW_TRESHOLD) {
      if (diffDist > STOP_TRESHOLD) {
        // Just slow down
        int speed = map(diffDist, STOP_TRESHOLD, SLOW_TRESHOLD, SLOW_SPEED_MIN, SLOW_SPEED_MAX);
        runMotors(DIRECTION_FORWARD, speed);
      } else {
        // Stop
        runMotors(DIRECTION_STOP, 0);
        firstCheck = false;
      }
    } else {
      runMotors(DIRECTION_FORWARD, 255);
    }
  }

  // Adjust if not first check
  if (!firstCheck) {
    if (abs(diffDist) < STOP_TRESHOLD) {
      runMotors(DIRECTION_STOP, 0);
      if (speedSlowFactor < SLOW_FACTOR_MAX) speedSlowFactor++;
      if (speedSlowFactor >= SLOW_FACTOR_STOP) {
        stateChange(&robotState, STATE_FREE);
        speedSlowFactor = 0;
      }
    }
    if (diffDist > STOP_TRESHOLD && robotState.direction != DIRECTION_FORWARD) {
      runMotors(DIRECTION_FORWARD, SLOW_SPEED_MIN - (speedSlowFactor * 5));
      if (speedSlowFactor < SLOW_FACTOR_MAX) speedSlowFactor++;
    } else if (diffDist < -STOP_TRESHOLD && robotState.direction != DIRECTION_BACKWARD) {
      runMotors(DIRECTION_BACKWARD, SLOW_SPEED_MIN - (speedSlowFactor * 5));
      if (speedSlowFactor < SLOW_FACTOR_MAX) speedSlowFactor++;
    }
  }
}

void preventDamage(int minDistance) {
  // Measure distance and difference from custom
  diffDist = robotMeasures.distanceUS - minDistance;

  // Difference less than treshold
  if (diffDist <= STOP_TRESHOLD + SLOW_TRESHOLD) {
    if (diffDist > STOP_TRESHOLD) {
      // Just slow down
      int speed = map(diffDist, STOP_TRESHOLD, SLOW_TRESHOLD, SLOW_SPEED_MIN, SLOW_SPEED_MAX);
      runMotors(DIRECTION_FORWARD, speed);
    } else {
      // Stop
      runMotors(DIRECTION_STOP, 0);
    }
  }
}

// VELOCITY
int measureDirection() {

  // Take previous direction
  // int directionSign = -1;
  int directionSign = (double(0) < robotMeasures.velocityUS) - (robotMeasures.velocityUS < double(0));


  if (robotState.direction == DIRECTION_FORWARD) {
    directionSign = -1;
  } else if (robotState.direction == DIRECTION_BACKWARD) {
    directionSign = 1;
  } else if (robotState.direction == DIRECTION_RIGHT || robotState.direction == DIRECTION_LEFT) {
    directionSign = 0;
  }
  // if DIRECTION_STOP keep previous direction, so directionSign is already set

  return directionSign;
}

void countPulses() {
  opticalPulses++;
}

// BLUETOOTH
bool bluetoothConnection(bool waitConnection) {
  // If waitConnection true => wait BLUETOOTH_WAIT_CONNECTION ms for connection or skip if OK button pressed
  if (waitConnection) {
    digitalWrite(LED_BUILTIN, HIGH);
    unsigned long previousMillisBluetoothConnected = millis();
    bool skip = false;
    while (millis() - previousMillisBluetoothConnected < BLUETOOTH_WAIT_CONNECTION) {
      if (digitalRead(PIN_BLUETOOTH_STATE) == HIGH || skip) break;
      if (!robotState.cmd_executed) {
        switch (robotState.command) {
          // If OK go ahead without wait for connection
          case IR_BUTTON_OK: {
            skip = true;
            break;
          }
        }
        stateCmdExecuted(&robotState);
      }
    }
    digitalWrite(LED_BUILTIN, LOW);
  }

  // Check if connected
  bluetoothConnected = digitalRead(PIN_BLUETOOTH_STATE) == HIGH;
  return bluetoothConnected;
}

void bluetoothSendMeasure() {
  //BDT: Bluetooth Data Transmission
  Serial.println(F("BDT 1.0 START"));

  Serial.print(F("Distance_US:"));
  Serial.println(robotMeasures.distanceUS, DECIMALS);

  Serial.print(F("Distance_US_Filtered:"));
  Serial.println(robotMeasures.distanceUSFiltered, DECIMALS);

  Serial.print(F("Distance_OPT:"));
  Serial.println(robotMeasures.distanceOptical, DECIMALS);

  Serial.print(F("Rev_per_second:"));
  Serial.println(robotMeasures.rpsOptical, DECIMALS);

  Serial.print(F("Velocity_US:"));
  Serial.println(robotMeasures.velocityUS, DECIMALS);

  Serial.print(F("Velocity_OPT:"));
  Serial.println(robotMeasures.velocityOptical, DECIMALS);

  Serial.print(F("Velocity_OPT_Filtered:"));
  Serial.println(robotMeasures.velocityOpticalFiltered, DECIMALS);

  Serial.println(F("BDT 1.0 END"));

  robotMeasures.sent = true;
}

void bluetoothSendInfo(const char* variable, int value) {
  //BDT: Bluetooth Data Transmission
  Serial.println(F("BDT 1.0 INFO"));

  Serial.print(variable);
  Serial.print(F(": "));
  Serial.println(value);
}