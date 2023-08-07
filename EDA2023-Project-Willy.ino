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
#define CUSTOM_DIST_CHAR 4
// WiFi
#define SERVER "api.thingspeak.com"
#define PORT 80
#define PERIOD_SERVER 15000
#define WIFI_CONNECTION_ATTEMPT_MAX 5
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
//TODO: Capire come allocare dinamicamente, è dato da PERIOD_SERVER/PERIOD_MEASURETOSEND
dataToSend sendBuffer[5];
byte sendBufferIndex = 0;
//TODO: Anche questo dipende indirettamente, approssimando dovrebbe essere 50 + 51 per ogni elemento (quindi dipende dal calcolo precedente)
// Dovrebbe dipendere anche da DECIMALS (con 4 ogni elemento + 51, con 3 ogni elemento + 50 e così via)
// char jsonToSend[310] = "{\"write_api_key\":\"";
char jsonToSend[310];

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
#else
#define debug(x)
#define debugln(x)
#define debuglnDecimal(x,n)
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
    debugln("No interrupt available");
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
            debug("Distance = ");
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
              debug("numericCustomDist = ");
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
      currentMillisMeasureToSend = millis();
      if (currentMillisMeasureToSend - previousMillisMeasureToSend >= PERIOD_MEASURETOSEND) {
        measuredDist = measureDistance();
        //DEBUG_TEMP
        measuredFilteredDist = int(measuredDist);

        insertNewData(&sendBuffer[sendBufferIndex], 3*sendBufferIndex, measuredDist, measuredFilteredDist);

        debug("sendBuffer[sendBufferIndex].deltaT = ");
        debugln(sendBuffer[sendBufferIndex].deltaT);
        debug("sendBuffer[sendBufferIndex].field1 = ");
        debuglnDecimal(sendBuffer[sendBufferIndex].field1, DECIMALS);
        debug("sendBuffer[sendBufferIndex].field2 = ");
        debuglnDecimal(sendBuffer[sendBufferIndex].field2, 0);
        sendBufferIndex++;
        previousMillisMeasureToSend = millis();

      }
      currentMillisServer = millis();
      if (currentMillisServer - previousMillisServer >= PERIOD_SERVER) {
        jsonBuildForSend(&sendBuffer[0], sendBufferIndex, getPvtDataFromEEPROM().key, jsonToSend);
        if (wifiActive) {
          if (!client.connected()) connectToServer();
          // sendDataToServer();
          sendBulkDataToServer();
          }
          sendBufferIndex = 0;
          previousMillisServer = millis();
        }
      if (!robot_state.cmd_executed) {
        switch (robot_state.command) {
          case IR_BUTTON_OK: {
            sendBufferIndex = 0;
            stateChange(&robot_state, STATE_FREE);
            break;
          }
          case IR_BUTTON_AST: {
            sendBufferIndex = 0;
            stateChange(&robot_state, STATE_READ);
            break;
          }
        }
        stateCmdExecuted(&robot_state);
      }
      break;
    }
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
      debugln("Stop");
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
      debugln("Avanti");
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
      debugln("Indietro");
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
      debugln("Destra");
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
      debugln("Sinistra");
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
    debugln("WiFi shield not present and WiFi disabled");
    return;
  }

  // Connect to WiFi network
  byte wifiConnectionAttemptCount = 0;
  while (wifiStatus != WL_CONNECTED) {
    wifiConnectionAttemptCount++;
    if (wifiConnectionAttemptCount > WIFI_CONNECTION_ATTEMPT_MAX) {
      // TODO: Capire se questo feedback è corretto o viene eseguito sempre
      ledFeedback(FEEDBACK_BLINK_WIFI_NO_CONNECTION, FEEDBACK_DURATION_WIFI_NO_CONNECTION);
      wifiActive = 0;
      debugln("WiFi connection failed and WiFi disabled");
      return;
    }
    ledFeedback(FEEDBACK_BLINK_WIFI_CONNECTING, FEEDBACK_DURATION_WIFI_CONNECTING);
    debug("Attempting to connect to WPA SSID: ");
    debugln(pvt.ssid);
    // Connect to WPA/WPA2 network
    wifiStatus = WiFi.begin(pvt.ssid, pvt.pwd);
  }
  // you're connected now
  ledFeedback(FEEDBACK_BLINK_WIFI_CONNECTED, FEEDBACK_DURATION_WIFI_CONNECTED);
  debugln("You're connected to the network");
  if (DEBUG_ACTIVE) printWifiStatus();
  //TODO: Capire se connettersi da subito o solo quando serve
  // connectToServer();
}

void printWifiStatus() {
  // print the SSID of the network you're attached to
  debug("SSID: ");
  debugln(WiFi.SSID());

  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  debug("IP Address: ");
  debugln(ip);

  // print the received signal strength
  long rssi = WiFi.RSSI();
  debug("Signal strength (RSSI):");
  debug(rssi);
  debugln(" dBm");
}

bool connectToServer() {
  bool connected;
  ledFeedback(FEEDBACK_BLINK_WIFI_CONNECTING, FEEDBACK_DURATION_WIFI_CONNECTING);
  debugln("Starting connection to server...");
  // if you get a connection, report back via led feedback
  connected = client.connect(SERVER, PORT);
  if (connected) {
    ledFeedback(FEEDBACK_BLINK_WIFI_CONNECTED, FEEDBACK_DURATION_WIFI_CONNECTED);
    debugln("Connected to server");
  } else {
    ledFeedback(FEEDBACK_BLINK_WIFI_NO_CONNECTION, FEEDBACK_DURATION_WIFI_NO_CONNECTION);
  }
  return connected;
}

void sendDataToServer() {
  privateData pvt = getPvtDataFromEEPROM();
  //Feedback
  digitalWrite(LED_BUILTIN, HIGH);
  
  servoH.detach();

  client.print("GET /update?api_key=" + String(pvt.key) + "&field1=" + String(measuredDist, DECIMALS) + "&field2=" + String(measuredFilteredDist, DECIMALS) + " HTTP/1.1" + RET + "Accept: */*" + RET + "Host: "+ SERVER + RET + RET);

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

void sendBulkDataToServer() {
  //Feedback
  digitalWrite(LED_BUILTIN, HIGH);
  
  servoH.detach();

  String DataLength = String(strlen(jsonToSend));
  debug("json: ");
  debugln(jsonToSend);
  debug("DataLenght =");
  debugln(DataLength);

  client.print("POST /channels/2219976/bulk_update.json HTTP/1.1" + String(RET) + "Host: " + SERVER + RET + /*"Connection: close" + RET */+ "Content-Type: application/json" + RET + "Content-Length: " + DataLength + RET + RET + jsonToSend);
  
  // if there are incoming bytes available
  // from the server, read them and print them
  // while (client.available()) {
  //   char c = client.read();
  // }
  delay(250); //Wait to receive the response
  client.parseFloat();
  String resp = String(client.parseInt());
  Serial.println("Response code:"+ resp);

  servoH.attach(PIN_SERVO_HORIZ);

  //Feedback
  digitalWrite(LED_BUILTIN, LOW);
}