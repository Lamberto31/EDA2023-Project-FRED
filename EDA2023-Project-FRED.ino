#define IR_RECEIVE_PIN 10     // Defined here because the library requires it
#define NO_LED_FEEDBACK_CODE  // Defined here because the library requires it
#include "TinyIRReceiver.hpp"
#include <Servo.h>
#include "WiFiEsp.h"

// Custom library for states handling
#include "src/States/States.h"

// Custom library for EEPROM memory handling
#include "src/EepromUtils/EepromUtils.h"

// Custom library for build data to send to remote server
#include "src/DataHelper/DataHelper.h"

// Digital Pins
#define PIN_ESP_TX 0
#define PIN_ESP_RX 1
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

// Functionalities active/disabled
#define DEBUG_ACTIVE 1
#define WIFI_ACTIVE 0

// PARAMETERS
// Ultrasonic
#define DECIMALS 4  // [digits] Max value 4, it may cause buffer overflow if greater
#define PERIOD_ULTRASONIC 60  // [ms] between each distance measurement. Min value 60, may cause error on distance measure if lower
// Optical
#define WHEEL_ENCODER_HOLES 20  // Holes in wheel encoder (when counted indicates one round)
#define WHEEL_DIAMETER 65  //[mm] Diameter of wheel
#define PERIOD_VELOCITY 240  //[ms] between each velocity measurement
// Movement control
#define STOP_TRESHOLD 0.1  // [cm] Tolerance for diffDist
#define SLOW_TRESHOLD 50  // [cm] Treshold used to go at max speed until reached
#define SLOW_SPEED_MIN 100  // [analog] [0-255] Max value for low speed
#define SLOW_FACTOR_MAX 15  // [adim] Max value for slowFactor to prevent too slow speed
#define SLOW_FACTOR_STOP 10  // [adim] Min value for slowFactor to allow stop from checkDistance
// Custom distance [cm]
#define CUSTOM_DIST_MIN 10  // [cm]
#define CUSTOM_DIST_MAX 500  // [cm]
#define CUSTOM_DIST_CHAR 4  // [chars] Max value 4, it may cause buffer overflow if greater
// WiFi
#define WIFI_WAIT_DISABLE 5000  // [ms] Initial wait time to receive WiFi active/disable command from IR
#define SERVER "api.thingspeak.com"
#define PORT 80
#define PERIOD_SERVER 15000  // [ms] between each message to server. Min value 15000, may cause error response if lower (server allow one message each 15s)
#define SERVER_HTTP_CORRECT_CODE 202
#define WIFI_CONNECTION_ATTEMPT_MAX 5  // [attempt] Number of WiFi connection attempt before consider a failure and disable WiFi functionalities
#define SERVER_CONNECTION_ATTEMPT_MAX 3  // [attempt] Number of TCP connection attempt to server before consider a failure and send feedback
#define PERIOD_MEASURETOSEND 3000  // [ms] between each insertion of data into the structure. Suggested value 3000, it's ok if greater but a lower value may cause high memory consumption
#define SEND_BUFFER_SIZE PERIOD_SERVER / PERIOD_MEASURETOSEND  // [byte] Can be changed to arbitrary value, it's better to don't go over 5 (tested and working) due to memory consumption (see where it's used)
// WiFi Feedback
#define FEEDBACK_BLINK_WIFI_NO_SHIELD 10
#define FEEDBACK_DURATION_WIFI_NO_SHIELD 250
#define FEEDBACK_BLINK_WIFI_CONNECTING 3
#define FEEDBACK_DURATION_WIFI_CONNECTING 500
#define FEEDBACK_BLINK_WIFI_CONNECTED 1
#define FEEDBACK_DURATION_WIFI_CONNECTED 1000
#define FEEDBACK_BLINK_WIFI_NO_CONNECTION 5
#define FEEDBACK_DURATION_WIFI_NO_CONNECTION 250
//Servo
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

// Ultrasonic
double measuredDist = 0;
double measuredFilteredDist = 0;
unsigned long previousMillisUS;
unsigned long currentMillisUS;

// Optical
volatile int opticalPulses = 0;
double measuredRps = 0;
double measuredVelocity = 0;
double measuredFilteredVelocity = 0;
unsigned long previousMillisVelocity;
unsigned long currentMillisVelocity;

// Movement control
double diffDist;
bool firstCheck = true;
byte speedSlowFactor = 0;

// WiFi
#define RET "\r\n"  //NL & CR characters, used to build HTTP request
int wifiStatus = WL_IDLE_STATUS;
bool wifiActive = WIFI_ACTIVE;
bool connectedToServer = false;
WiFiEspClient client;
unsigned long previousMillisServer;
unsigned long currentMillisServer;
unsigned long previousMillisMeasureToSend;
unsigned long currentMillisMeasureToSend;
// DataToSend sendBuffer[5];
DataToSend sendBuffer[SEND_BUFFER_SIZE];
unsigned int sendBufferIndex = 0;
/*10 is a little extra to avoid problems
  49 is the characters used by the body in general
  104 is the characters used by each DataToSend (with DECIMALS = 4)
    13 for deltaT;
    18 for field1, field2, so 18 * 2 = 36;
    16 for field3;
    19 for field4, field5, so 19 * 2 = 38;
    1 for the comma separator for each object;
    So 13 + 36 + 16 + 38 + 1 = 104
*/
// char jsonToSend[310];
char jsonToSend[10 + 49 + (104 * (SEND_BUFFER_SIZE))];

// Servomotor
Servo servoH;

// Custom distance [cm]
char customDist[CUSTOM_DIST_CHAR] = "000";
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
  if (DEBUG_ACTIVE) Serial.begin(9600);

  // Feedback led
  pinMode(LED_BUILTIN, OUTPUT);

  //Start time counters
  previousMillisUS = millis();
  previousMillisVelocity = millis();
  previousMillisMeasureToSend = millis();
  previousMillisServer = millis();

  // IR Receiver
  if (!initPCIInterruptForTinyReceiver()) {
    debugF("No interrupt available");
  }

  // Ultrasonic
  pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);
  pinMode(PIN_ULTRASONIC_ECHO, INPUT);

  // WiFi
  wifiActive = !waitDisableWifi();
  if (wifiActive) {
    Serial.begin(9600);
    wifiInitializeConnect();
  }

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
            debugF("Distance = ");
            debuglnDecimal(measuredDist, DECIMALS);

            debugF("opticalPulses = ");
            debugln(opticalPulses);

            debugF("wheelRounds = ");
            debuglnDecimal(opticalPulses/WHEEL_ENCODER_HOLES, DECIMALS);

            debugF("measuredRps = ");
            debuglnDecimal(measuredRps, DECIMALS);

            debugF("measuredVelocity = ");
            debuglnDecimal(measuredVelocity, DECIMALS);

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
            readCustomDistance('1');
            break;
          }
          case IR_BUTTON_2: {
            readCustomDistance('2');
            break;
          }
          case IR_BUTTON_3: {
            readCustomDistance('3');
            break;
          }
          case IR_BUTTON_4: {
            readCustomDistance('4');
            break;
          }
          case IR_BUTTON_5: {
            readCustomDistance('5');
            break;
          }
          case IR_BUTTON_6: {
            readCustomDistance('6');
            break;
          }
          case IR_BUTTON_7: {
            readCustomDistance('7');
            break;
          }
          case IR_BUTTON_8: {
            readCustomDistance('8');
            break;
          }
          case IR_BUTTON_9: {
            readCustomDistance('9');
            break;
          }
          case IR_BUTTON_0: {
            readCustomDistance('0');
            break;
          }
          case IR_BUTTON_AST: {
            if (composeNumericDistance()) stateChange(&robotState, STATE_SEARCH); else stateChange(&robotState, STATE_FREE);
              debugF("numericCustomDist = ");
              debugln(numericCustomDist);
            break;
          }
          case IR_BUTTON_OK: {
            resetCustomDistance();
            stateChange(&robotState, STATE_FREE);
            break;
          }
          case IR_BUTTON_HASH: {
            resetCustomDistance();
            stateChange(&robotState, STATE_MEASURE);
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
    case STATE_MEASURE: {
      currentMillisServer = millis();
      if (currentMillisServer - previousMillisServer >= PERIOD_SERVER) {
        jsonBuildForSend(&sendBuffer[0], min(sendBufferIndex, SEND_BUFFER_SIZE), getPvtDataFromEEPROM().writeKey, jsonToSend);
        if (wifiActive) {
          if (!client.connected()) connectedToServer = connectToServer();
          if (connectedToServer) sendBulkDataToServer(getPvtDataFromEEPROM().channelId);
          }
          sendBufferIndex = 0;
          memset(sendBuffer, 0, sizeof(sendBuffer));
          previousMillisServer = millis();
        }
      if (!robotState.cmd_executed) {
        switch (robotState.command) {
          case IR_BUTTON_OK: {
            stateChange(&robotState, STATE_FREE);
            client.stop();
            break;
          }
          case IR_BUTTON_AST: {
            stateChange(&robotState, STATE_READ);
            client.stop();
            break;
          }
        }
        stateCmdExecuted(&robotState);
      }
      break;
    }
  }
  // Actions performed for each state
  // Measure Distance
  currentMillisUS = millis();
  if (currentMillisUS - previousMillisUS >= PERIOD_ULTRASONIC) {
    measuredDist = measureDistance();
    //DEBUG_TEMP
    measuredFilteredDist = int(measuredDist);

    previousMillisUS = millis();
  }

  // Measure Velocity
  currentMillisVelocity = millis();
  if (currentMillisVelocity - previousMillisVelocity >= PERIOD_VELOCITY) {
    measuredVelocity = measureVelocity(currentMillisVelocity - previousMillisVelocity);

    previousMillisVelocity = millis();
    //DEBUG_TEMP
    measuredFilteredVelocity = int(measuredVelocity);
  }

  // Insert new data in sendBuffer
  currentMillisMeasureToSend = millis();
  if (currentMillisMeasureToSend - previousMillisMeasureToSend >= PERIOD_MEASURETOSEND) {
    servoH.attach(PIN_SERVO_HORIZ);
    servoH.write(SERVO_HORIZ_CENTER);
    delay(100);
    servoH.detach();

    // insertNewData(&sendBuffer[sendBufferIndex], (PERIOD_MEASURETOSEND/1000)*sendBufferIndex, measuredDist, measuredFilteredDist);
    insertNewCircularData(&sendBuffer[min(sendBufferIndex, SEND_BUFFER_SIZE - 1)], (PERIOD_MEASURETOSEND / 1000) * sendBufferIndex, measuredDist, measuredFilteredDist, measuredRps, measuredVelocity, measuredFilteredVelocity, sendBufferIndex, SEND_BUFFER_SIZE);
    sendBufferIndex++;

    if (DEBUG_ACTIVE) readAndPrintData(&sendBuffer[0], SEND_BUFFER_SIZE);

    previousMillisMeasureToSend = millis();
  }
}

// This is the function, which is called if a complete command was received
// It runs in an ISR context with interrupts enabled
void handleReceivedTinyIRData(uint8_t aAddress, uint8_t aCommand, uint8_t aFlags) {
  if (DEBUG_ACTIVE) printTinyReceiverResultMinimal(&Serial, aAddress, aCommand, aFlags);
  if (!aFlags == IRDATA_FLAGS_IS_REPEAT) {
    stateNewCmd(&robotState, aCommand);
  }
}

void countPulses() {
  opticalPulses++;
}

void ledFeedback(byte blinkNumber, unsigned int blinkDuration) {
  for (byte blinkCount = 0; blinkCount < blinkNumber; blinkCount++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(blinkDuration);
    digitalWrite(LED_BUILTIN, LOW);
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
  if (customDistIdx == 3) {
    resetCustomDistance();
    stateChange(&robotState, STATE_FREE);
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
  customDist[0] = '0';
  customDist[1] = '0';
  customDist[2] = '0';
  customDistIdx = 0;
}

void checkDistance() {
  // Measure distance and difference from custom
  diffDist = measuredDist - numericCustomDist;

  // Move to the custom distance if first check
  if (firstCheck) {
    if (diffDist < STOP_TRESHOLD + SLOW_TRESHOLD) {
      if (diffDist >= STOP_TRESHOLD) {
        // Just slow down
        int speed = map(diffDist, 0, numericCustomDist + SLOW_TRESHOLD, SLOW_SPEED_MIN, 255);
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
  diffDist = measuredDist - minDistance;

  // Difference less than treshold
  if (diffDist < STOP_TRESHOLD + SLOW_TRESHOLD) {
    if (diffDist >= STOP_TRESHOLD) {
      // Just slow down
      int speed = map(diffDist, 0, minDistance + SLOW_TRESHOLD, 0, 255);
      runMotors(DIRECTION_FORWARD, speed);
    } else {
      // Stop
      runMotors(DIRECTION_STOP, 0);
    }
  }
}

// VELOCITY
double measureVelocity(unsigned long deltaT) {

  int pulses = opticalPulses;
  opticalPulses = 0;
  double velocity;

  measuredRps = pulses / (WHEEL_ENCODER_HOLES * (deltaT * 0.001));
  velocity = PI * (WHEEL_DIAMETER * 0.1) * measuredRps;

  if (robotState.direction == DIRECTION_BACKWARD) {
    velocity = -1 * velocity;
  } else if (robotState.direction == DIRECTION_RIGHT || robotState.direction == DIRECTION_LEFT) {
    velocity = 0;
  }

  return velocity;
}

// WIFI
void wifiInitializeConnect() {
  PrivateData pvt = getPvtDataFromEEPROM();

  // ESP module initialization
  WiFi.init(&Serial);

  // Check if module is connected
  if (WiFi.status() == WL_NO_SHIELD) {
    ledFeedback(FEEDBACK_BLINK_WIFI_NO_SHIELD, FEEDBACK_DURATION_WIFI_NO_SHIELD);
    wifiActive = 0;
    debugFln("WiFi shield not present and WiFi disabled");
    return;
  }

  // Connect to WiFi network
  byte wifiConnectionAttemptCount = 0;
  while (wifiStatus != WL_CONNECTED) {
    wifiConnectionAttemptCount++;
    if (wifiConnectionAttemptCount > WIFI_CONNECTION_ATTEMPT_MAX) {
      ledFeedback(FEEDBACK_BLINK_WIFI_NO_CONNECTION, FEEDBACK_DURATION_WIFI_NO_CONNECTION);
      wifiActive = 0;
      debugFln("WiFi connection failed and WiFi disabled");
      return;
    }
    ledFeedback(FEEDBACK_BLINK_WIFI_CONNECTING, FEEDBACK_DURATION_WIFI_CONNECTING);
    debugF("Attempting to connect to WPA SSID: ");
    debugln(pvt.ssid);
    // Connect to WPA/WPA2 network
    wifiStatus = WiFi.begin(pvt.ssid, pvt.pwd);
  }
  // Connected
  ledFeedback(FEEDBACK_BLINK_WIFI_CONNECTED, FEEDBACK_DURATION_WIFI_CONNECTED);
  debugFln("You're connected to the network");
  if (DEBUG_ACTIVE) printWifiStatus();
}

void printWifiStatus() {
  // Print the SSID of the network you're attached to
  debugF("SSID: ");
  debugln(WiFi.SSID());

  // Print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  debugF("IP Address: ");
  debugln(ip);

  // Print the received signal strength
  long rssi = WiFi.RSSI();
  debugF("Signal strength (RSSI):");
  debug(rssi);
  debugFln(" dBm");
}

bool connectToServer() {
  byte serverConnectionAttemptCount = 0;
  while (!client.connected()) {
    serverConnectionAttemptCount++;
    if (serverConnectionAttemptCount > SERVER_CONNECTION_ATTEMPT_MAX) {
      ledFeedback(FEEDBACK_BLINK_WIFI_NO_CONNECTION, FEEDBACK_DURATION_WIFI_NO_CONNECTION);
      debugFln("Connection failed");
      return false;
    }
    ledFeedback(FEEDBACK_BLINK_WIFI_CONNECTING, FEEDBACK_DURATION_WIFI_CONNECTING);
    debugFln("Starting connection to server...");
    client.connect(SERVER, PORT);
    delay(100);
  }
  // Connected
  ledFeedback(FEEDBACK_BLINK_WIFI_CONNECTED, FEEDBACK_DURATION_WIFI_CONNECTED);
  debugFln("Connected to server");
  return true;
}

void sendBulkDataToServer(char channelId[]) {
  char c;  //Store received char from server
  byte httpCodeLen = 3;
  int httpCode;

  String dataLength = String(strlen(jsonToSend));

  client.print("POST /channels/" + String(channelId) + "/bulk_update.json HTTP/1.1" + RET + "Host: " + SERVER + RET + /*"Connection: close" + RET */ +"Content-Type: application/json" + RET + "Content-Length: " + dataLength + RET + RET + jsonToSend);

  delay(250);  //Wait to receive the response
  debugFln("");
  httpCode = getHttpResponseCode(httpCodeLen);
  debugF("Response code: ");
  debugln(httpCode);

  //Feedback
  if (httpCode == SERVER_HTTP_CORRECT_CODE) {
    ledFeedback(FEEDBACK_BLINK_WIFI_CONNECTED, FEEDBACK_DURATION_WIFI_CONNECTED);
  } else {
    ledFeedback(FEEDBACK_BLINK_WIFI_NO_CONNECTION, FEEDBACK_DURATION_WIFI_NO_CONNECTION);
  }

  Serial.println();
}

int getHttpResponseCode(byte responseCodeLen) {
  char c;
  char toFind[] = "HTTP/1.1 ";
  byte toFindLen = sizeof(toFind) / sizeof(toFind[0]) - 1;
  byte currentIndex = 0;
  char responseCode[responseCodeLen];
  int responseCodeInt;

  while (client.available()) {
    c = client.read();
    if (c == toFind[currentIndex]) {
      currentIndex++;
    } else {
      currentIndex = 0;
    }
    if (currentIndex == toFindLen) {
      break;
    }
  }
  // questo o salvare risposta meglio
  for (byte i = 0; i < responseCodeLen; i++) {
    c = client.read();
    responseCode[i] = c;
  }
  responseCodeInt = atoi(&responseCode[0]);
  return responseCodeInt;
}

bool waitDisableWifi() {
  bool wifiDisabled = !WIFI_ACTIVE;
  unsigned long previousMillisWifiDisable = millis();
  unsigned long currentMillisWifiDisable;

  digitalWrite(LED_BUILTIN, HIGH);

  while (millis() - previousMillisWifiDisable < WIFI_WAIT_DISABLE) {
    if (!robotState.cmd_executed) {
      switch (robotState.command) {
        case IR_BUTTON_OK: {
          wifiDisabled = false;
          break;
        }
        case IR_BUTTON_HASH: {
          wifiDisabled = true;
          break;
        }
      }
      stateCmdExecuted(&robotState);
      break;
    }
  }

  digitalWrite(LED_BUILTIN, LOW);
  return wifiDisabled;
}