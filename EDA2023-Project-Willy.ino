#define IR_RECEIVE_PIN 10     // Defined here because the library requires it
#define NO_LED_FEEDBACK_CODE  // Defined here because the library requires it
#include "TinyIRReceiver.hpp"
#include <Servo.h>
#include "WiFiEsp.h"

// Custom library for states handling
#include "src/states/states.h"

// Custom library for EEPROM memory handling
#include "src/EepromUtils/EepromUtils.h"

// Custom library for build data to send to remote server
#include "src/DataHelper/DataHelper.h"

// Digital Pins
#define PIN_ESP_TX 0
#define PIN_ESP_RX 1
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
state robot_state = { STATE_SETUP, 0, true, DIRECTION_STOP };

// Functionalities active/disabled
#define DEBUG_ACTIVE 1
#define WIFI_ACTIVE 1

// PARAMETERS
// Ultrasonic
#define DECIMALS 4      // Max value 4, it may cause buffer overflow if increased
#define STOP_TRESHOLD 0.1
#define STOP_SECURE_TRESHOLD 10
#define SLOW_FACTOR_MAX 15
#define SLOW_FACTOR_STOP 10
#define PERIOD_ULTRASONIC 60
#define PERIOD_MEASURETOSEND 3000
// Custom distance [cm]
#define CUSTOM_DIST_MIN 10
#define CUSTOM_DIST_MAX 500
#define CUSTOM_DIST_CHAR 4    // Max value 4, it may cause buffer overflow if increased
// WiFi
#define SERVER "api.thingspeak.com"
#define PORT 80
#define PERIOD_SERVER 15000
#define WIFI_CONNECTION_ATTEMPT_MAX 5
#define SEND_BUFFER_SIZE PERIOD_SERVER/PERIOD_MEASURETOSEND   //Can be changed to arbitrary value, it's better to don't go over 5 (tested and working) due to memory consumption (see where it's used)
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
#define SERVO_HORIZ_CENTER 120

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
double diffDist;
byte speedSlowFactor = 0;
double measuredFilteredDist = 0;
unsigned long previousMillisUS;
unsigned long currentMillisUS;
unsigned long previousMillisMeasureToSend;
unsigned long currentMillisMeasureToSend;
// dataToSend sendBuffer[5];
dataToSend sendBuffer[SEND_BUFFER_SIZE];
unsigned int sendBufferIndex = 0;
/*10 is a little extra to avoid problems
  50 is the characters used by the body in general
  51 is the caracters used by each dataToSend (with DECIMALS = 4)
*/
// char jsonToSend[310];
char jsonToSend[10 + 50 + (51*(SEND_BUFFER_SIZE))];

// WiFi
#define RET "\r\n"  //NL & CR characters
int wifiStatus = WL_IDLE_STATUS;
bool wifiActive = WIFI_ACTIVE;
WiFiEspClient client;
unsigned long previousMillisServer;
unsigned long currentMillisServer;

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
#define debuglnDecimal(x,n) Serial.println(x,n)
#define debugF(x) Serial.print(F(x))
#define debugFln(x) Serial.println(F(x))
#else
#define debug(x)
#define debugln(x)
#define debuglnDecimal(x,n)
#define debugF(x)
#define debugFln(x)
#endif

void setup() {
  if (DEBUG_ACTIVE) Serial.begin(9600);

  // Feedback led
  pinMode(LED_BUILTIN, OUTPUT);

  //Start time counters
  previousMillisUS = millis();
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

  // Motors
  pinMode(PIN_MOTOR_ENA, OUTPUT);
  pinMode(PIN_MOTOR_ENB, OUTPUT);
  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  pinMode(PIN_MOTOR_IN3, OUTPUT);
  pinMode(PIN_MOTOR_IN4, OUTPUT);

  // Feedback
  runMotors(DIRECTION_BACKWARD, 255);
  delay(1000);
  runMotors(DIRECTION_FORWARD, 255);
  delay(1000);
  runMotors(DIRECTION_STOP, 0);

  stateChange(&robot_state, STATE_FREE);
}

void loop() {
  switch (robot_state.current) {
    // Free state handling
    case STATE_FREE: {
      if (robot_state.direction == DIRECTION_FORWARD) {
        currentMillisUS = millis();
        if (currentMillisUS - previousMillisUS >= PERIOD_ULTRASONIC) {
          preventDamage(CUSTOM_DIST_MIN);
          previousMillisUS = millis();
        }
      }
      if (!robot_state.cmd_executed) {
        switch (robot_state.command) {
          case IR_BUTTON_OK: {
            debugF("Distance = ");
            debuglnDecimal(measureDistance(), DECIMALS);
            runMotors(DIRECTION_STOP, 0);
            break;
          }
          case IR_BUTTON_UP: {
            runMotors(DIRECTION_FORWARD, 255);
            break;
          }
          case IR_BUTTON_DOWN: {
            runMotors(DIRECTION_BACKWARD, 255);
            break;
          }
          case IR_BUTTON_RIGHT: {
            runMotors(DIRECTION_RIGHT, 255);
            break;
          }
          case IR_BUTTON_LEFT: {
            runMotors(DIRECTION_LEFT, 255);
            break;
          }
          case IR_BUTTON_HASH: {
            runMotors(DIRECTION_STOP, 0);
            stateChange(&robot_state, STATE_MEASURE);
            break;
          }
          case IR_BUTTON_AST: {
            runMotors(DIRECTION_STOP, 0);
            stateChange(&robot_state, STATE_READ);
            break;
          }
        }
        stateCmdExecuted(&robot_state);
      }
      break;
    }
    // Reading state handling
    case STATE_READ: {
      if (!robot_state.cmd_executed) {
        switch (robot_state.command) {
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
            if (composeNumericDistance()) stateChange(&robot_state, STATE_SEARCH); else stateChange(&robot_state, STATE_FREE);
              debugF("numericCustomDist = ");
              debugln(numericCustomDist);
            break;
          }
          case IR_BUTTON_OK: {
            resetCustomDistance();
            stateChange(&robot_state, STATE_FREE);
            break;
          }
          case IR_BUTTON_HASH: {
            resetCustomDistance();
            stateChange(&robot_state, STATE_MEASURE);
            break;
          }
        }
      stateCmdExecuted(&robot_state);
      }
      break;
    }
    // Search state handling
    case STATE_SEARCH: {
      currentMillisUS = millis();
      if (currentMillisUS - previousMillisUS >= PERIOD_ULTRASONIC) {
        checkDistance();
        previousMillisUS = millis();
      }
      if (!robot_state.cmd_executed) {
        switch (robot_state.command) {
          case IR_BUTTON_OK: {
            runMotors(DIRECTION_STOP, 0);
            speedSlowFactor = 0;
            stateChange(&robot_state, STATE_FREE);
            break;
          }
          case IR_BUTTON_HASH: {
            runMotors(DIRECTION_STOP, 0);
            speedSlowFactor = 0;
            stateChange(&robot_state, STATE_MEASURE);
            break;
          }
        }
        stateCmdExecuted(&robot_state);
      }
      break;
    }
    // Measure state handling
    case STATE_MEASURE: {
      currentMillisServer = millis();
      if (currentMillisServer - previousMillisServer >= PERIOD_SERVER) {
        jsonBuildForSend(&sendBuffer[0], min(sendBufferIndex, SEND_BUFFER_SIZE), getPvtDataFromEEPROM().writeKey, jsonToSend);
        if (wifiActive) {
          if (!client.connected()) connectToServer();
          // sendDataToServer();
          sendBulkDataToServer(getPvtDataFromEEPROM().channelId);
          }
          sendBufferIndex = 0;
          memset(sendBuffer, 0, sizeof(sendBuffer));
          previousMillisServer = millis();
        }
      if (!robot_state.cmd_executed) {
        switch (robot_state.command) {
          case IR_BUTTON_OK: {
            stateChange(&robot_state, STATE_FREE);
            client.stop();
            break;
          }
          case IR_BUTTON_AST: {
            stateChange(&robot_state, STATE_READ);
            client.stop();
            break;
          }
        }
        stateCmdExecuted(&robot_state);
      }
      break;
    }
  }
  currentMillisMeasureToSend = millis();
  if (currentMillisMeasureToSend - previousMillisMeasureToSend >= PERIOD_MEASURETOSEND) {
    measuredDist = measureDistance();
    //DEBUG_TEMP
    measuredFilteredDist = int(measuredDist);

    // insertNewData(&sendBuffer[sendBufferIndex], (PERIOD_MEASURETOSEND/1000)*sendBufferIndex, measuredDist, measuredFilteredDist);
    insertNewCircularData(&sendBuffer[min(sendBufferIndex, SEND_BUFFER_SIZE - 1)], (PERIOD_MEASURETOSEND/1000)*sendBufferIndex, measuredDist, measuredFilteredDist, sendBufferIndex, SEND_BUFFER_SIZE);
    sendBufferIndex++;

    if (DEBUG_ACTIVE) readAndPrintData(&sendBuffer[0], SEND_BUFFER_SIZE);

    previousMillisMeasureToSend = millis();
  }
  servoH.write(SERVO_HORIZ_CENTER);
}

// This is the function, which is called if a complete command was received
// It runs in an ISR context with interrupts enabled
void handleReceivedTinyIRData(uint8_t aAddress, uint8_t aCommand, uint8_t aFlags) {
  if (DEBUG_ACTIVE) printTinyReceiverResultMinimal(&Serial, aAddress, aCommand, aFlags);
  if (!aFlags == IRDATA_FLAGS_IS_REPEAT) {
    stateNewCmd(&robot_state, aCommand);
  }
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
      stateNewDirection(&robot_state, DIRECTION_STOP);
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
      stateNewDirection(&robot_state, DIRECTION_FORWARD);
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
      stateNewDirection(&robot_state, DIRECTION_BACKWARD);
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
      stateNewDirection(&robot_state, DIRECTION_RIGHT);
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
      stateNewDirection(&robot_state, DIRECTION_LEFT);
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
    stateChange(&robot_state, STATE_FREE);
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
  measuredDist = measureDistance();
  diffDist = measuredDist - numericCustomDist;

  // Difference less than treshold
  if (abs(diffDist) < STOP_TRESHOLD) {
    runMotors(DIRECTION_STOP, 0);
    if (speedSlowFactor < SLOW_FACTOR_MAX) speedSlowFactor++;
    if (speedSlowFactor >= SLOW_FACTOR_STOP) {
      stateChange(&robot_state, STATE_FREE);
      speedSlowFactor = 0;
    }
  }
  // Difference greater than treshold
  else if (diffDist > STOP_TRESHOLD && robot_state.direction != DIRECTION_FORWARD) {
    runMotors(DIRECTION_FORWARD, 255 - (speedSlowFactor * 10));
    if (speedSlowFactor < SLOW_FACTOR_MAX) speedSlowFactor++;
  }
  else if (diffDist < -STOP_TRESHOLD && robot_state.direction != DIRECTION_BACKWARD) {
    runMotors(DIRECTION_BACKWARD, 255 - (speedSlowFactor * 10));
    if (speedSlowFactor < SLOW_FACTOR_MAX) speedSlowFactor++;
  }
}

void preventDamage(int minDistance) {
  // Measure distance and difference from custom
  measuredDist = measureDistance();
  diffDist = measuredDist - minDistance - STOP_SECURE_TRESHOLD;

  // Difference less than treshold
  if (diffDist < STOP_TRESHOLD) {
    runMotors(DIRECTION_STOP, 0);
  }
}

// WIFI
void wifiInitializeConnect() {
  privateData pvt = getPvtDataFromEEPROM();

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
  ledFeedback(FEEDBACK_BLINK_WIFI_CONNECTING, FEEDBACK_DURATION_WIFI_CONNECTING);
  debugFln("Starting connection to server...");
  client.connect(SERVER, PORT);
  //TODO: Capire questo feedback che sembra scorretto (sembra non entrare mai anche se si connette)
  delay(100);
  if (client.connected()) {
    ledFeedback(FEEDBACK_BLINK_WIFI_CONNECTED, FEEDBACK_DURATION_WIFI_CONNECTED);
    debugFln("Connected to server");
    return true;
  } else {
    ledFeedback(FEEDBACK_BLINK_WIFI_NO_CONNECTION, FEEDBACK_DURATION_WIFI_NO_CONNECTION);
    debugFln("Connection failed");
    return false;
  }
  // return connected;
}

void sendDataToServer() {
  privateData pvt = getPvtDataFromEEPROM();
  //Feedback
  digitalWrite(LED_BUILTIN, HIGH);
  
  servoH.detach();

  client.print("GET /update?api_key=" + String(pvt.writeKey) + "&field1=" + String(measuredDist, DECIMALS) + "&field2=" + String(measuredFilteredDist, DECIMALS) + " HTTP/1.1" + RET + "Accept: */*" + RET + "Host: "+ SERVER + RET + RET);

  // if there are incoming bytes available
  // from the server, read them and print them
  while (client.available()) {
    char c = client.read();
    // Serial.write(c);
  }
  // Serial.println();

  servoH.attach(PIN_SERVO_HORIZ);

  //Feedback
  digitalWrite(LED_BUILTIN, LOW);
}

void sendBulkDataToServer(char channelId[]) {
  char c;   //Store received char from server
  byte httpCodeLen = 3;
  char httpCode [httpCodeLen];
  int httpCodeInt;

  //Feedback
  digitalWrite(LED_BUILTIN, HIGH);
  
  servoH.detach();

  String dataLength = String(strlen(jsonToSend));

  client.print("POST /channels/"+ String(channelId) + "/bulk_update.json HTTP/1.1" + RET + "Host: " + SERVER + RET + /*"Connection: close" + RET */+ "Content-Type: application/json" + RET + "Content-Length: " + dataLength + RET + RET + jsonToSend);

  delay(250); //Wait to receive the response
  //COMMENTO_TEMP: vecchia funzione che legge fino al caporigo ma che non funziona bene
  // while (client.available() && c != '\n') {
  //   c = client.read();
  //   Serial.print(c);
  // }
  
  //COMMENTO_TEMP: funzione che legge finchè non trova 202, sembra funzionare ma non va bene per leggere eventuali altre risposte
  // while (client.available()) {
  //   c = client.read();
  //   if (c == '2')
  //   {
  //     httpCode[0] = c;
  //     c = client.read();
  //     if (c == '0') {
  //       httpCode[1] = c;
  //       c = client.read();
  //       if (c == '2') 
  //       {
  //         httpCode[2] = c;
  //         debugF("Response code: ");
  //         debugln(httpCode);
  //         break;
  //       }
  //     }
  //   }
  // }
  debugFln("");
  gethResponseCode(httpCode, httpCodeLen);
  debugF("Response code: ");
  debugln(httpCode);
  httpCodeInt = atoi(&httpCode[0]);
  debugF("Response code: ");
  debugln(httpCodeInt);
  
  Serial.println();

  servoH.attach(PIN_SERVO_HORIZ);

  //Feedback
  digitalWrite(LED_BUILTIN, LOW);
}

void gethResponseCode(char *responseCode, byte responseCodeLen) {
  char c;
  char toFind[] = "HTTP/1.1 ";
  byte toFindLen = sizeof(toFind)/sizeof(toFind[0]) - 1;
  byte currentIndex = 0;

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
}