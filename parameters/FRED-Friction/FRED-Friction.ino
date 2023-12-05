// BASIC FRED CONFIGURATION
#define IR_RECEIVE_PIN 10     // Defined here because the library requires it
#define NO_LED_FEEDBACK_CODE  // Defined here because the library requires it
#include "TinyIRReceiver.hpp"
#include <Servo.h>

// Custom library for states handling
#include "src/States/States.h"

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
// Bluetooth
#define BLUETOOTH_WAIT_CONNECTION 10000  // [ms] Wait time to receive Bluetooth connection
#define PERIOD_BLUETOOTH 500  // [ms] between each message to Bluetooth. Min value 1000, may cause error response if lower
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

// Bluetooth
bool bluetoothConnected = false;
unsigned long previousMillisMeasureToSend;
unsigned long currentMillisMeasureToSend;

// Servomotor
Servo servoH;

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

// END BASIC FRED CONFIGURATION

// PARAMETERS FRED CONFIGURATION
PERIOD_SPEED = 2000 // [ms] Time that, if elapsed, ensure a maximum speed
// Max speed timer
unsigned long previousMillisSpeed;
unsigned long currentMillisSpeed;
// Boolean to check if moving or not
bool moving = false;
// Stop speed timer
unsigned long previousMillisStopSpeed;
unsigned long currentMillisStopSpeed;
// Boolean to check if just stopped
bool justStopped = false;
// Measured stop time
unsigned long stopTime = 0;
// END PARAMETERS FRED CONFIGURATION

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
}

void loop() {
  // Handle IR commands
  if (!robotState.cmd_executed) {
    switch (robotState.command) {
      case IR_BUTTON_OK: {
        runMotors(DIRECTION_STOP, 0);
        moving = false;
        break;
      }
      case IR_BUTTON_UP: {
        runMotors(DIRECTION_FORWARD, 255);
        previousMillisSpeed = millis();
        moving = true;
        break;
      }
      case IR_BUTTON_DOWN: {
        runMotors(DIRECTION_BACKWARD, 255);
        previousMillisSpeed = millis();
        moving = true;
        break;
      }
    }
    stateCmdExecuted(&robotState);
  }
  // Check if moving and stop if PERIOD_SPEED elapsed
  currentMillisSpeed = millis();
  if (currentMillisSpeed - previousMillisSpeed >= PERIOD_SPEED && moving) {
    runMotors(DIRECTION_STOP, 0);
    // Begin stop speed timer
    previousMillisStopSpeed = millis();
    moving = false;
    justStopped = true;
  }
  // Measure
  currentMillisMeasure = millis();
  if (currentMillisMeasure - previousMillisMeasure >= PERIOD_MEASURE) {
    measureAll(currentMillisMeasure - previousMillisMeasure);
    previousMillisMeasure = millis();
  }
  // Check if just stopped and measure time until it's effectively stopped
  if (justStopped) {
    if (robotMeasures.velocityOptical < 0.1) {
      currentMillisStopSpeed = millis();
      stopTime = currentMillisStopSpeed - previousMillisStopSpeed;
      justStopped = false;
      bluetoothSendInfo("StopTime", stopTime);
    }
  }
  // Send measure with Bluetooth
  if (currentMillisMeasureToSend - previousMillisMeasureToSend >= PERIOD_BLUETOOTH) {
    bluetoothConnection(false);
    if (bluetoothConnected && !robotMeasures.sent) bluetoothSendMeasure();
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
  
  int pulses = opticalPulses;
  opticalPulses = 0;
  int directionSign;
  double travelledRevolution;
  double travelledDistance;

  // Distance from ultrasonic
  robotMeasures.distanceUS = measureDistance();

  // Velocity from optical
  directionSign = measureDirection();
  travelledRevolution = (pulses / (double)WHEEL_ENCODER_HOLES);
  travelledDistance = PI * (WHEEL_DIAMETER * 0.1) * travelledRevolution * directionSign;

  robotMeasures.rpsOptical = travelledRevolution / (deltaT * 0.001);
  robotMeasures.velocityOptical = travelledDistance / (deltaT * 0.001);
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

  Serial.print(F("Rev_per_second:"));
  Serial.println(robotMeasures.rpsOptical, DECIMALS);

  Serial.print(F("Velocity_OPT:"));
  Serial.println(robotMeasures.velocityOptical, DECIMALS);

  Serial.print(F("Status:"));
  Serial.println(robotState.current);

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