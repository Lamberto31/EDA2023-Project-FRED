#define IR_RECEIVE_PIN 10     // Defined here because the library requires it
#define NO_LED_FEEDBACK_CODE  // Defined here because the library requires it
#define USE_CALLBACK_FOR_TINY_RECEIVER  // Defined here because the library requires it
#include "TinyIRReceiver.hpp"
#include <Servo.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

// Custom library for states handling
#include "src/States/States.h"

// Custom library for estimation and filtering
#include "src/Estimation/Estimation.h"

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
State robotState = { STATE_SETUP, false, 0, true, DIRECTION_STOP, 0};
Measures robotMeasures = {0, 0, 0, 0, 0, 0, true};

// Functionalities active/disabled
#define DEBUG_ACTIVE 0
#define SEND_MEASURE_ACTIVE 1
#define SEND_FILTER_RESULT_ACTIVE 1
#define FIXED_POSITION 0

// PARAMETERS
// Physical
#define MASS 0.731  // [kg] Mass of robot
#define VOLTAGE_PEAK 5.76 // [V] Power supply voltage (Peak)
#define SPEED_MAX 77.8397  // [cm/s] Max speed of robot (measured)
#define TIME_TO_STOP 0.3809  // [s] Time to stop with null input starting from max speed (measured)
#define WHEEL_DIAMETER 6.5 // [cm] Diameter of wheel
#define WHEEL_ENCODER_HOLES 20  // [imp/rev] Impulses per revolution of wheel (when counted indicates one round)
#define DISCRETE_STEP 0.1  // [s] Discrete time step of system. Min value 0.06, may cause error on ultrasonic measure if lower
// Derived
#define FRICTION_COEFFICIENT (5 * (MASS / TIME_TO_STOP)) // [kg/s] Friction coefficient
#define ETA_V (SPEED_MAX * (FRICTION_COEFFICIENT / VOLTAGE_PEAK)) // [N/V] (Newton in centimeter) Constant used to convert voltage to speed
#define KAPPA (ETA_V * VOLTAGE_PEAK / 255) // [N] Constant used for easy computation
// Model
#define STATE_DIM 2  // [adim] Dimension of state vector
#define INPUT_DIM 1  // [adim] Dimension of input vector
#define MEASURE_DIM 2  // [adim] Dimension of measure vector
// Model initial state and covariance matrix
#define STATE_INIT_Xp 202  // [adim] Initial position
#define STATE_INIT_Xv 0  // [adim] Initial velocity
#define STATE_INIT_COV_Xp 66  // [adim] Initial position covariance
#define STATE_INIT_COV_Xv SPEED_MAX/100  // [adim] Initial velocity covariance
// Sensor noises
# define NOISE_ULTRASONIC 0.3  // [cm] Noise of ultrasonic sensor
# define PULSE_OFFSET ((1/2) * (1/WHEEL_ENCODER_HOLES)) // [imp] Offset of optical sensor
# define NOISE_OPTICAL ((1/(2*1.7321)) * (1/WHEEL_ENCODER_HOLES))  // [cm] Noise of optical sensor (standard deviation) TODO: sistemare radice di 3
// Noise
#define NOISE_PROCESS_POSITION_STD 0.03  // [cm] Standard deviation of process noise for position
#define NOISE_PROCESS_VELOCITY_STD 0.01  // [cm/s] Standard deviation of process noise for velocity
#define NOISE_MEASURE_POSITION_STD NOISE_ULTRASONIC  // [cm] Standard deviation of measure noise for position
#define NOISE_MEASURE_VELOCITY_STD NOISE_OPTICAL  // [cm/s] Standard deviation of measure noise for velocity
// Measure
#define PERIOD_MEASURE DISCRETE_STEP * 1000  // [ms] between each measurement. Min value 60, may cause error on ultrasonic measure if lower
#define DECIMALS 4  // [digits] Max value 4, it may cause buffer overflow if greater
// Movement control
#define STOP_TRESHOLD 0.1  // [cm] Tolerance for diffDist
#define SLOW_TRESHOLD 50  // [cm] Treshold used to go at max speed until reached
#define SLOW_SPEED_MIN 50  // [analog] [0-255] Min value for slow speed
#define SLOW_SPEED_MAX 150  // [analog] [0-255] Max value for slow speed
#define HIGH_INPUT 150 // [analog] [0-255] Value for high input
#define LOW_INPUT 100 // [analog] [0-255] Value for low input
#define SPEED_EPSILON 0.01 // [cm/s] Epsilon used to consider speed as zero
#define PERIOD_WAIT_CHECK 1000  // [ms] Wait time to check distance
#define SPEED_SLOW_MAX (KAPPA * LOW_INPUT / FRICTION_COEFFICIENT) // [cm/s] Max speed for slow speed
// Custom distance [cm]
#define CUSTOM_DIST_MIN 10  // [cm]
#define CUSTOM_DIST_MAX 500  // [cm]
#define CUSTOM_DIST_CHAR 4  // [chars] Max value 4, it may cause buffer overflow if greater
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

// Model matrices
BLA::Matrix<STATE_DIM, STATE_DIM> FF;  // State transition matrix (Notation: FF to avoid conflict with F macro)
BLA::Matrix<STATE_DIM, INPUT_DIM> G;  // Input matrix
BLA::Matrix<MEASURE_DIM, STATE_DIM> H;  // Measure matrix
BLA::Matrix<STATE_DIM, STATE_DIM> L;  // Process noise matrix
BLA::Matrix<STATE_DIM, STATE_DIM> Q;  // Process noise covariance matrix
BLA::Matrix<MEASURE_DIM, MEASURE_DIM> R;  // Measure noise covariance matrix

// State, measure and input vectors
BLA::Matrix<STATE_DIM> x_pred;  // State vector prediction
BLA::Matrix<STATE_DIM> x_hat;  // State vector estimate
BLA::Matrix<INPUT_DIM> u;  // Input vector
BLA::Matrix<MEASURE_DIM> z;  // Measure vector

// Kalman filter output
BLA::Matrix<STATE_DIM, STATE_DIM> P_pred;  // State covariance matrix prediction
BLA::Matrix<STATE_DIM, STATE_DIM> P_hat;  // State covariance matrix estimate
BLA::Matrix<STATE_DIM, MEASURE_DIM> W;  // Kalman gain
BLA::Matrix<MEASURE_DIM> innovation;  // Innovation
BLA::Matrix<STATE_DIM, STATE_DIM> S;  // Innovation covariance matrix

// Measure
unsigned long previousMillisMeasure;
unsigned long currentMillisMeasure;

// Optical
volatile int opticalPulses = 0;

// Movement control
double diffDist;
bool stopMode = false;
bool slowMode = false;
byte inputSign = DIRECTION_STOP;
unsigned long previousMillisCheckDistance;
unsigned long currentMillisCheckDistance;

// Bluetooth
bool bluetoothConnected = false;
unsigned long previousMillisMeasureToSend;
unsigned long currentMillisMeasureToSend;

// Servomotor
Servo servoH;

// Custom distance [cm]
char customDist[CUSTOM_DIST_CHAR];
byte customDistIdx = 0;
int numericCustomDist = 0;

// Filtering boolean (Used to avoid sending filtered data when not needed)
bool filtering = false;
bool resetFiltering = true;

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

  // Matrix initialization
  digitalWrite(LED_BUILTIN, HIGH);

  // State transition matrix
  computeMatrixF(DISCRETE_STEP, FRICTION_COEFFICIENT, MASS, &FF);
  // Input matrix
  computeMatrixG(DISCRETE_STEP, FRICTION_COEFFICIENT, MASS, ETA_V, VOLTAGE_PEAK, &G);
  // Measure matrix
  computeMatrixH(WHEEL_ENCODER_HOLES, WHEEL_DIAMETER, &H);
  // Process noise matrix
  computeMatrixL(DISCRETE_STEP, FRICTION_COEFFICIENT, MASS, &L);
  // Process noise covariance matrix
  computeMatrixQ(NOISE_PROCESS_POSITION_STD, NOISE_PROCESS_VELOCITY_STD, &Q);
  // Measure noise covariance matrix
  computeMatrixR(NOISE_MEASURE_POSITION_STD, NOISE_MEASURE_VELOCITY_STD, &R);

  // Matrix correction if fixed position (so that the position is indipedent from the velocity)
  if (FIXED_POSITION) correctMatricesFGL(&FF, &G, &L);

  // Print matrices
  if (DEBUG_ACTIVE) {
    printMatrix(FF, "F", DECIMALS);
    printMatrix(G, "G", DECIMALS);
    printMatrix(H, "H", DECIMALS);
    printMatrix(L, "L", DECIMALS);
    printMatrix(Q, "Q", DECIMALS);
    printMatrix(R, "R", DECIMALS);
  }

  // Send matrices
  bluetoothSendMatrix(FF, "F", DECIMALS);
  bluetoothSendMatrix(G, "G", DECIMALS);
  bluetoothSendMatrix(H, "H", DECIMALS);
  bluetoothSendMatrix(L, "L", DECIMALS);
  bluetoothSendMatrix(Q, "Q", DECIMALS);
  bluetoothSendMatrix(R, "R", DECIMALS);

  digitalWrite(LED_BUILTIN, LOW);

  stateChange(&robotState, STATE_FREE);
}

void loop() {
  switch (robotState.current) {
    // Free state handling
    case STATE_FREE: {
      if (robotState.just_changed) {
        robotState.just_changed = false;
      }
      if (robotState.direction == DIRECTION_FORWARD) {
        if (!FIXED_POSITION) preventDamage(CUSTOM_DIST_MIN);
      }
      if (!robotState.cmd_executed) {
        switch (robotState.command) {
          case IR_BUTTON_OK: {
            if (DEBUG_ACTIVE) printMeasures(&robotMeasures);
            bluetoothSendInfo("Position", robotMeasures.distanceUS);
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
            stateChange(&robotState, STATE_DATA_TRANSMISSION);
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
      if (robotState.just_changed) {
        robotState.just_changed = false;
      }
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
            if (composeNumericDistance()) stateChange(&robotState, STATE_EXPLORE); else stateChange(&robotState, STATE_FREE);
            debugF("numericCustomDist = ");
            debugln(numericCustomDist);
            bluetoothSendInfo("Custom distance", numericCustomDist);
            // Feedback led
            digitalWrite(LED_BUILTIN, LOW);
            break;
          }
          case IR_BUTTON_OK: {
            resetCustomDistance();
            resetFiltering = true;
            stateChange(&robotState, STATE_FREE);
            // Feedback led
            digitalWrite(LED_BUILTIN, LOW);
            break;
          }
          case IR_BUTTON_HASH: {
            resetCustomDistance();
            resetFiltering = true;
            stateChange(&robotState, STATE_DATA_TRANSMISSION);
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
    case STATE_EXPLORE: {
      if (robotState.just_changed) {
        if (resetFiltering) {
          initializeVectorX(STATE_INIT_Xp, STATE_INIT_Xv, &x_hat);
          initializeMatrixP(STATE_INIT_COV_Xp, STATE_INIT_COV_Xv, &P_hat);
          computeVectorU(0, &u);
          computeVectorZ(0, 0, &z);
          previousMillisCheckDistance = millis();
        }
        resetFiltering = true;
        inputSign = DIRECTION_STOP;
        stopMode = false;
        slowMode = false;
        robotState.just_changed = false;
      }
      // Estimate
      // Do only if new measure is available
      if (!robotMeasures.sent) {
        // Fill input and measures vectors
        computeVectorU(robotState.input, &u);
        computeVectorZ(robotMeasures.distanceUS, robotMeasures.ppsOptical, &z);
        // Predictor and corrector
        KalmanPredictor(FF, x_hat, G, u, P_hat, Q, &x_pred, &P_pred);
        KalmanCorrector(P_pred, H, R, z, x_pred, &W, &x_hat, &P_hat, &innovation, &S);
        // Update input
        checkDistance();
        // Send results
        if (SEND_FILTER_RESULT_ACTIVE) {
          bluetoothConnection(false);
          if (bluetoothConnected) bluetoothSendData(true);
          else robotMeasures.sent = true;
        }
      }
      if (!robotState.cmd_executed) {
        switch (robotState.command) {
          case IR_BUTTON_OK: {
            runMotors(DIRECTION_STOP, 0);
            numericCustomDist = 0;
            bluetoothSendInfo("Custom distance", 0);
            stateChange(&robotState, STATE_FREE);
            break;
          }
          case IR_BUTTON_HASH: {
            runMotors(DIRECTION_STOP, 0);
            numericCustomDist = 0;
            bluetoothSendInfo("Custom distance", 0);
            stateChange(&robotState, STATE_DATA_TRANSMISSION);
            break;
          }
          case IR_BUTTON_AST: {
            resetFiltering = false;
            stateChange(&robotState, STATE_READ);
            break;
          }
        }
        stateCmdExecuted(&robotState);
      }
      break;
    }
    // Measure state handling
    case STATE_DATA_TRANSMISSION: {
      if (robotState.just_changed) {
        filtering = waitFilteringChoice();
        if (filtering) {
          initializeVectorX(STATE_INIT_Xp, STATE_INIT_Xv, &x_hat);
          initializeMatrixP(STATE_INIT_COV_Xp, STATE_INIT_COV_Xv, &P_hat);
          computeVectorU(0, &u);
          computeVectorZ(0, 0, &z);
        }
        robotState.just_changed = false;
      }
      if (robotState.direction == DIRECTION_FORWARD) {
        if (!FIXED_POSITION) preventDamage(CUSTOM_DIST_MIN);
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
            stateChange(&robotState, STATE_FREE);
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

      // Filter only if filtering is true
      if (filtering) {
        // Do only if new measure is available
        if (!robotMeasures.sent) {
          // Fill input and measures vectors
          computeVectorU(robotState.input, &u);
          computeVectorZ(robotMeasures.distanceUS, robotMeasures.ppsOptical, &z);
          // Predictor and corrector
          KalmanPredictor(FF, x_hat, G, u, P_hat, Q, &x_pred, &P_pred);
          KalmanCorrector(P_pred, H, R, z, x_pred, &W, &x_hat, &P_hat, &innovation, &S);
          // Set measure as used
          robotMeasures.sent = true;
        }
      }

      // Send data with Bluetooth
      currentMillisMeasureToSend = millis();
      if (SEND_MEASURE_ACTIVE && currentMillisMeasureToSend - previousMillisMeasureToSend >= PERIOD_BLUETOOTH) {
        // Adjust servo
        servoH.attach(PIN_SERVO_HORIZ);
        servoH.write(SERVO_HORIZ_CENTER);
        delay(100);
        servoH.detach();

        // Prepare data to send if not filtering
        if (!filtering) {
          computeVectorU(robotState.input, &u);
          computeVectorZ(robotMeasures.distanceUS, robotMeasures.ppsOptical, &z);
          x_hat(0) = robotMeasures.distanceUS;
          x_hat(1) = robotMeasures.velocityOptical;
        }

        // Send data
        bluetoothConnection(false);
        if (bluetoothConnected) bluetoothSendData(filtering);
        previousMillisMeasureToSend = millis();
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

// MEASURE
void measureAll(unsigned long deltaT) {
  robotMeasures.sent = false;
  double prevDistance = robotMeasures.distanceUS;
  
  int pulses = opticalPulses + PULSE_OFFSET;
  opticalPulses = 0;
  int directionSign;
  double travelledRevolution;
  double travelledDistance;

  // Distance from ultrasonic
  robotMeasures.distanceUS = measureDistance();

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

  // Pulses per second from optical
  robotMeasures.ppsOptical = pulses / (deltaT * 0.001) * directionSign;
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

// CUSTOM DISTANCE
void readCustomDistance(char digit) {
  if (customDistIdx == (CUSTOM_DIST_CHAR - 1)) {
    resetCustomDistance();
    resetFiltering = true;
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
    if (!resetFiltering) return true;
    numericCustomDist = 0;
    resetFiltering = true;
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
    resetFiltering = true;
    return false;
  }
  return true;
}

void resetCustomDistance() {
  memset(customDist, '0', sizeof(customDist));
  customDistIdx = 0;
}

// MOVEMENT CONTROL
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
      stateNewInput(&robotState, DIRECTION_STOP, 0);
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
      stateNewInput(&robotState, DIRECTION_FORWARD, speed*-1);
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
      stateNewInput(&robotState, DIRECTION_BACKWARD, speed);
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
      stateNewInput(&robotState, DIRECTION_RIGHT, 0);
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
      stateNewInput(&robotState, DIRECTION_LEFT, 0);
      break;
    }
  }
}

int checkDistance() {
  double x_position;
  double x_velocity;
  double stopDistance;
  double slowDistance;
  double deltaSpeed;
  double deltaMaxSpeed;
  double deltaDistance;


  if (!stopMode) {
    // Go on only if time to check
    currentMillisCheckDistance = millis();
    if (currentMillisCheckDistance - previousMillisCheckDistance < PERIOD_WAIT_CHECK) {
      return 0;
    }

    // Set state to use
    x_position = x_hat(0);
    x_velocity = x_hat(1);
    //x_position = robotMeasures.distanceUS;
    //x_velocity = robotMeasures.velocityUS;

    // Compute input sign
    if (x_position > numericCustomDist) {
      inputSign = DIRECTION_FORWARD;
    } 
    else if (x_position < numericCustomDist) {
      inputSign = DIRECTION_BACKWARD;
    }

    // First compute all the distances
    stopDistance = abs(x_velocity*MASS/FRICTION_COEFFICIENT);
    if (!slowMode) {
      deltaSpeed = abs(abs(x_velocity) - SPEED_SLOW_MAX);
      if (deltaSpeed < SPEED_EPSILON) {
        deltaMaxSpeed = 0;
      }
      else {
        deltaMaxSpeed = MASS/FRICTION_COEFFICIENT * (SPEED_EPSILON - deltaSpeed + (SPEED_SLOW_MAX * log(SPEED_SLOW_MAX/SPEED_EPSILON)));
      }
      slowDistance = 5*deltaMaxSpeed + stopDistance;
    }
    // Then check if stop or slow
    deltaDistance = abs(x_position - numericCustomDist);
    if (!slowMode && deltaDistance > slowDistance) {
      runMotors(inputSign, HIGH_INPUT);
    }
    else if (!stopMode && deltaDistance > stopDistance) {
      if (!slowMode) {
        bluetoothSendInfo("Slow Position", x_position);
        slowMode = true;
      }
      runMotors(inputSign, LOW_INPUT);
    }
    else {
      if (!stopMode) {
        bluetoothSendInfo("Stop Position", x_position);
        stopMode = true;
    }
    runMotors(DIRECTION_STOP, 0);
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

void bluetoothSendData(bool filtering) {
  //BDT: Bluetooth Data Transmission
  if (filtering) Serial.println(F("BDT 1.0 ESTIMATE"));
  else Serial.println(F("BDT 1.0 DATA"));

  Serial.print(F("Input:"));
  Serial.println(u(0), 0);

  Serial.print(F("Measures:"));
  // Ultrasonic
  Serial.print(z(0), DECIMALS);
  Serial.print(F(","));
  // Optical
  Serial.println(z(1), DECIMALS);

  Serial.print(F("State:"));
  // Position
  Serial.print(x_hat(0), DECIMALS);
  Serial.print(F(","));
  // Velocity
  Serial.println(x_hat(1), DECIMALS);

  if (filtering) {
    Serial.print(F("Covariance:"));
    // Position_covariance
    Serial.print(P_hat(0, 0), DECIMALS);
    Serial.print(F(","));
    // Velocity_covariance
    Serial.println(P_hat(1, 1), DECIMALS);
  }

  Serial.print(F("Status:"));
  Serial.println(robotState.current);

  robotMeasures.sent = true;
}

void bluetoothSendInfo(const char* variable, int value) {
  //BDT: Bluetooth Data Transmission
  Serial.println(F("BDT 1.0 INFO"));

  Serial.print(variable);
  Serial.print(F(":"));
  Serial.println(value);
}

// FILTERING
bool waitFilteringChoice() {
  unsigned long previousMillisFiltering = millis();
  bool skip = false;
  bool filtering = false;

  digitalWrite(LED_BUILTIN, HIGH);
  while (millis() - previousMillisFiltering < BLUETOOTH_WAIT_CONNECTION) {
    if (skip) break;
    if (!robotState.cmd_executed) {
      switch (robotState.command) {
        // If OK go ahead without wait for connection
        case IR_BUTTON_OK: {
          skip = true;
          break;
        }
        case IR_BUTTON_AST: {
          skip = true;
          filtering = true;
          break;
        }
      }
      stateCmdExecuted(&robotState);
    }
  }
  digitalWrite(LED_BUILTIN, LOW);
  return filtering;
}