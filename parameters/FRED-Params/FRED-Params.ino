// BASIC FRED CONFIGURATION
#define IR_RECEIVE_PIN 10     // Defined here because the library requires it
#define NO_LED_FEEDBACK_CODE  // Defined here because the library requires it
#define USE_CALLBACK_FOR_TINY_RECEIVER  // Defined here because the library requires it
#include "TinyIRReceiver.hpp"
#include <Servo.h>

// Custom library for states handling
#include "src/ParamsStates/ParamsStates.h"

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
Params robotParams = {0, 0, 0, 0, true, 0};

// Functionalities active/disabled
#define DEBUG_ACTIVE 0
#define AUTOMATIC_TEST_ACTIVE 0

// PARAMETERS
// Measure
#define PERIOD_MEASURE 100  // [ms] between each measurement. Min value 60, may cause error on ultrasonic measure if lower
#define DECIMALS 4  // [digits] Max value 4, it may cause buffer overflow if greater
// Optical
#define WHEEL_ENCODER_HOLES 20  // Holes in wheel encoder (when counted indicates one round)
#define WHEEL_DIAMETER 65  //[mm] Diameter of wheel
// Bluetooth
#define BLUETOOTH_WAIT_CONNECTION 10000  // [ms] Wait time to receive Bluetooth connection
// Servo
#define SERVO_HORIZ_CENTER 100 // [angle] [0-180] Angle considered as center for servo, it depends on the construction

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
#define PERIOD_SPEED 2000 // [ms] Time that, if elapsed, ensure a maximum speed
#define PERIOD_MAX_STOP 1000 // [ms] Time that, if elapsed, ensure robot stop state
#define TESTS_NUMBER 10 // Number of tests to perform in automatic test
#define PERIOD_TEST 2500 // [ms] Time that, if elapsed, ensure a new test
#define SPEED_INPUT 255 // [0-255] Input provided to motors when measuring speed, min value 100 to ensure a minimum speed
// Max speed timer
unsigned long previousMillisSpeed;
// Stop speed timer
unsigned long previousMillisStopSpeed;
unsigned long stopTime = 0;
// Automatic test
byte testCounter = 0;
unsigned long previousMillisAutomaticTest;
byte automaticDirection[] = {IR_BUTTON_UP, IR_BUTTON_DOWN};
// Bluetooth message buffer size (in terms of robotParams)
#define BLUETOOTH_BUFFER_SIZE (PERIOD_SPEED + PERIOD_MAX_STOP) / PERIOD_MEASURE
// Bluetooth message buffer
Params bluetoothBuffer[BLUETOOTH_BUFFER_SIZE];
byte bluetoothBufferIndex = 0;
// Bluetooth message timer
#define PERIOD_BLUETOOTH 50 // [ms] between each message to Bluetooth (while sending buffer). Min value 50, may cause error if lower
// END PARAMETERS FRED CONFIGURATION

void setup() {
  Serial.begin(9600);

  // Feedback led
  pinMode(LED_BUILTIN, OUTPUT);

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

  // Start counters
  previousMillisAutomaticTest = millis();

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

  stateChange(&robotState, STATE_IDLE);
}

void loop() {
  // Automatic command if IDLE, automatic test active, test not finished and test timer elapsed
  if (AUTOMATIC_TEST_ACTIVE && testCounter < TESTS_NUMBER && robotState.current == STATE_IDLE && millis() - previousMillisAutomaticTest >= PERIOD_TEST) {
    stateNewCmd(&robotState, automaticDirection[testCounter % 2]);
    testCounter++;
    
  }
  // Handle IR commands
  if (!robotState.cmd_executed) {
    switch (robotState.command) {
      case IR_BUTTON_OK: {
        runMotors(DIRECTION_STOP, 0);
        // Stop automatic test
        testCounter = TESTS_NUMBER;
        // Reset buffer
        bluetoothBufferIndex = 0;
        bluetoothBuffer[bluetoothBufferIndex] = {0, 0, 0, 0, true, 0};
        stopTime = 0;
        opticalPulses = 0;
        stateChange(&robotState, STATE_IDLE);
        break;
      }
      case IR_BUTTON_UP: {
        if (robotState.current == STATE_IDLE) {
          runMotors(DIRECTION_FORWARD, SPEED_INPUT);
          previousMillisMeasure = millis();
          previousMillisSpeed = millis();
          stateChange(&robotState, STATE_INPUT_MAX);
        }
        break;
      }
      case IR_BUTTON_DOWN: {
        if (robotState.current == STATE_IDLE) {
          runMotors(DIRECTION_BACKWARD, SPEED_INPUT);
          previousMillisMeasure = millis();
          previousMillisSpeed = millis();
          stateChange(&robotState, STATE_INPUT_MAX);
        }
        break;
      }
    }
    stateCmdExecuted(&robotState);
  }
  
  // Measure if not idle
  if (robotState.current != STATE_IDLE) {
    currentMillisMeasure = millis();
    if (currentMillisMeasure - previousMillisMeasure >= PERIOD_MEASURE) {
      measureAll(currentMillisMeasure - previousMillisMeasure);
      robotParams.currentTime = millis() - previousMillisSpeed;
      previousMillisMeasure = millis();
    }
  }

  // Check if accelerating and stop if PERIOD_SPEED elapsed and measure just taken
  if (robotState.current == STATE_INPUT_MAX) {
    if (millis() - previousMillisSpeed >= PERIOD_SPEED && !robotParams.recorded) {
      runMotors(DIRECTION_STOP, 0);
      // Begin stop speed timer
      previousMillisStopSpeed = millis();
      stateChange(&robotState, STATE_INPUT_0);
    }
  }

  // Check if just stopped and measure time until it's effectively stopped
  if (robotState.current == STATE_INPUT_0 && !robotParams.recorded) {
    if (abs(robotParams.velocityOptical) < 0.1) {
      stopTime = robotParams.currentTime + previousMillisSpeed - previousMillisStopSpeed;
      stateChange(&robotState, STATE_STOP);
    }
  }

  // Fill Bluetooth buffer
  if ((robotState.current != STATE_IDLE && (robotState.current == STATE_STOP || !robotParams.recorded))) {
    robotParams.state = robotState.current;
    bluetoothBuffer[bluetoothBufferIndex] = robotParams;
    robotParams.recorded = true;
    bluetoothBufferIndex++;
    }

  // Send Bluetooth buffer if STATE_STOP
  // Note that if bluetoothConnected is false and DEBUG_ACTIVE is false, the vehicle will stay in STATE_STOP
  if (robotState.current == STATE_STOP) {
    // Check if connected
    bluetoothConnection(false);
    if (DEBUG_ACTIVE) bluetoothConnected = true;
    if (bluetoothConnected) bluetoothSendBuffer();
  } 
}

// This is the function, which is called if a complete ir command was received
// It runs in an ISR context with interrupts enabled
void handleReceivedTinyIRData() {
  if (DEBUG_ACTIVE) printTinyReceiverResultMinimal(&Serial);
  // Ignore repeat commands
  if (TinyIRReceiverData.Flags != IRDATA_FLAGS_IS_REPEAT) {
    stateNewCmd(&robotState, TinyIRReceiverData.Command);
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
  robotParams.recorded = false;
  
  int pulses = opticalPulses;
  opticalPulses = 0;
  double travelledRevolution;
  double travelledDistance;

  // Distance from ultrasonic
  robotParams.distanceUS = measureDistance();

  // Velocity from optical
  travelledRevolution = (pulses / (double)WHEEL_ENCODER_HOLES);
  travelledDistance = PI * (WHEEL_DIAMETER * 0.1) * travelledRevolution;

  robotParams.rpsOptical = travelledRevolution / (deltaT * 0.001);
  robotParams.velocityOptical = travelledDistance / (deltaT * 0.001);
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

void bluetoothSendBuffer() {
  unsigned long previousMillisBluetoothSend = 0;
  // Send first message to start communication
  // BDT: Bluetooth Data Transmission
  Serial.println(F("BDT 1.0 PARAMS"));

  // Cycle through buffer and send all messages
  digitalWrite(LED_BUILTIN, HIGH);
  for (byte i = 0; i < bluetoothBufferIndex; i++) {
    bluetoothSendParams(i);
    // Wait PERIOD_BLUETOOTH ms between each message with while and millis
    previousMillisBluetoothSend = millis();
    while (millis() - previousMillisBluetoothSend < PERIOD_BLUETOOTH);
  }

  // Reset buffer and stop time
  bluetoothBufferIndex = 0;
  bluetoothBuffer[bluetoothBufferIndex] = {0, 0, 0, 0, true, 0};
  stopTime = 0;

  stateChange(&robotState, STATE_IDLE);
  previousMillisAutomaticTest = millis();
  digitalWrite(LED_BUILTIN, LOW);
}

void bluetoothSendParams(byte index) {
  // Send status first to know how much message to expect (Status message excluded)
  // INPUT_MAX => 3 messages => Increasing
  // INPUT_0 => 3 messages => Decreasing
  // STOP => 4 messages => Stop => last message
  Serial.print(F("Status:"));
  Serial.println(bluetoothBuffer[index].state);

  Serial.print(F("Current_Time:"));
  Serial.println(bluetoothBuffer[index].currentTime);

  Serial.print(F("Distance_US:"));
  Serial.println(bluetoothBuffer[index].distanceUS, DECIMALS);

  Serial.print(F("Speed_OPT:"));
  Serial.println(bluetoothBuffer[index].velocityOptical, DECIMALS);

  if (bluetoothBuffer[index].state == STATE_STOP) {
    Serial.print(F("Stop_Time:"));
    Serial.println(stopTime);
  }
}