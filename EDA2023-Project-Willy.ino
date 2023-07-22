#define IR_RECEIVE_PIN 10  // Defined here because the library requires it
#include "TinyIRReceiver.hpp"
#include <Servo.h>
#include "WiFiEsp.h"
// #include "SoftwareSerial.h"

// Custom library for states handling
#include "states.h"

// Digital Pins
#define PIN_ESP_TX 2
#define PIN_ESP_RX 3
#define PIN_ULTRASONIC_ECHO 4
#define PIN_ULTRASONIC_TRIG 5
#define PIN_SERVO_HORIZ 6
#define PIN_MOTOR_IN4 7
#define PIN_MOTOR_IN3 8
#define PIN_MOTOR_ENB 9
// #define PIN_IR_RECV 10
#define PIN_MOTOR_ENA 11
#define PIN_MOTOR_IN2 12
#define PIN_MOTOR_IN1 13

// States
state robot_state = { STATE_SETUP, 0, true, DIRECTION_STOP };

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
#define DECIMALS 4
#define STOP_TRESHOLD 0.1
#define SLOW_FACTOR_MAX 15
#define SLOW_FACTOR_STOP 10
double measuredDist = 0;
double diffDist;
byte speedSlowFactor = 0;
double measuredFilteredDist = 0;
#define PERIOD_ULTRASONIC 60
unsigned long previousMillisUS;
unsigned long currentMillisUS;

// WiFi
// #define WIFI_SSID "Fastweb - Preite - Ospiti"
// #define WIFI_PWD "Grp3mTYLFaf1NhJo"
#define WIFI_SSID "Fastweb-Mauro"
#define WIFI_PWD "31EYPGxyASL!G?"
#define SERVER "api.thingspeak.com"
#define PORT 80
#define API_KEY "WHH69YD9VAM7NLG5"
#define RET "\r\n"  //NL & CR characters
// SoftwareSerial WifiSerial(PIN_ESP_TX, PIN_ESP_RX);
int wifiStatus = WL_IDLE_STATUS;
WiFiEspClient client;
#define PERIOD_SERVER 15000
unsigned long previousMillisServer;
unsigned long currentMillisServer;
#define FEEDBACK_BLINK_WIFI_NO_SHIELD 50
#define FEEDBACK_DURATION_WIFI_NO_SHIELD 500
#define FEEDBACK_BLINK_WIFI_CONNECTING 3
#define FEEDBACK_BLINK_WIFI_CONNECTING 500
#define FEEDBACK_BLINK_WIFI_CONNECTED 1
#define FEEDBACK_DURATION_WIFI_CONNECTED 3000 
#define FEEDBACK_BLINK_WIFI_NO_CONNECTION 5
#define FEEDBACK_DURATION_WIFI_NO_CONNECTION 500

// Servomotor
#define SERVO_HORIZ_CENTER 90
Servo servoH;

// Custom distance [cm]
#define CUSTOM_DIST_MIN 5
#define CUSTOM_DIST_MAX 500
char customDist[4] = "000";
byte customDistIdx = 0;
int numericCustomDist = 0;


void setup() {
  // DEBUG serial communication
  Serial.begin(9600);
  // Feedback led
  pinMode(LED_BUILTIN, OUTPUT);

  //Start time counters
  previousMillisUS = millis();
  previousMillisServer = millis();

  // IR Receiver
  if (!initPCIInterruptForTinyReceiver()) {
    // DEBUG
    Serial.println("No interrupt available");
  }

  // Ultrasonic
  pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);
  pinMode(PIN_ULTRASONIC_ECHO, INPUT);

  // WiFi
  wifiInitializeConnect();
  //TODO: Capire se collegarsi ora o ogni volta che si entra nello stato MEASURE
  // connectToServer();

  // Servomotor
  servoH.attach(PIN_SERVO_HORIZ);
  // Feedback
  servoH.write(SERVO_HORIZ_CENTER - 45);
  delay(1000);
  servoH.write(SERVO_HORIZ_CENTER + 45);
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
  runMotors(DIRECTION_BACKWARD, 200);
  delay(1000);
  runMotors(DIRECTION_FORWARD, 200);
  delay(1000);
  runMotors(DIRECTION_STOP, 0);

  stateChange(&robot_state, STATE_FREE);
}

void loop() {

  switch (robot_state.current) {
    // Free state handling
    case STATE_FREE: {
      if (!robot_state.cmd_executed) {
        switch (robot_state.command) {
          case IR_BUTTON_OK: {
            // DEBUG
            Serial.print("Distance = ");
            Serial.println(measureDistance(), DECIMALS);
            runMotors(DIRECTION_STOP, 0);
            break;
          }
          case IR_BUTTON_UP: {
            runMotors(DIRECTION_FORWARD, 200);
            break;
          }
          case IR_BUTTON_DOWN: {
            runMotors(DIRECTION_BACKWARD, 200);
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
            stateChange(&robot_state, STATE_MEASURE);
            break;
          }
          case IR_BUTTON_AST: {
            runMotors(DIRECTION_STOP, 0);
            stateChange(&robot_state, STATE_READ);
            break;
          }
          default: {
            Serial.println("NO");
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
            // DEBUG
            Serial.print("numericCustomDist = ");
            Serial.println(numericCustomDist);
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
          default: {
            Serial.println("NO");
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
        //TODO: Decidere se misure e filtraggio le fa lo stesso sempre o solo se può inviare al server
        measuredDist = measureDistance();
        //DEBUG
        measuredFilteredDist = int(measuredDist);
        //sendDataToServer();
        //DEBUG
        Serial.print("measuredDist = ");
        Serial.println(measuredDist, DECIMALS);
        Serial.print("measuredFilteredDist = ");
        Serial.println(measuredFilteredDist, 0);
        previousMillisServer = millis();
      }
      if (!robot_state.cmd_executed) {
        switch (robot_state.command) {
          case IR_BUTTON_OK: {
            stateChange(&robot_state, STATE_FREE);
            break;
          }
          case IR_BUTTON_AST: {
            stateChange(&robot_state, STATE_READ);
            break;
          }
          default: {
            Serial.println("NO");
          }
        }
        stateCmdExecuted(&robot_state);
      }
      break;
    }
  }
  servoH.write(SERVO_HORIZ_CENTER);
  // sendToServer();
}

// This is the function, which is called if a complete command was received
// It runs in an ISR context with interrupts enabled
void handleReceivedTinyIRData(uint8_t aAddress, uint8_t aCommand, uint8_t aFlags) {
  // DEBUG
  printTinyReceiverResultMinimal(&Serial, aAddress, aCommand, aFlags);
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

void wifiInitializeConnect() {
  // WifiSerial.begin(9600);

  // // ESP module initialization
  // WiFi.init(&WifiSerial);

  // // Check if module is connected
  // if (WiFi.status() == WL_NO_SHIELD) {
  //   ledFeedback(FEEDBACK_BLINK_WIFI_NO_SHIELD, FEEDBACK_DURATION_WIFI_NO_SHIELD);
  //   // DEBUG
  //   Serial.println("WiFi shield not present");
  //   // don't continue
  //   while (true);
  // }

  // // Connect to WiFi network
  // while (wifiStatus != WL_CONNECTED) {
  //   ledFeedback(FEEDBACK_BLINK_WIFI_CONNECTING, FEEDBACK_DURATION_WIFI_CONNECTING);
  //   // DEBUG
  //   Serial.print("Attempting to connect to WPA SSID: ");
  //   Serial.println(WIFI_SSID);
  //   // Connect to WPA/WPA2 network
  //   wifiStatus = WiFi.begin(WIFI_SSID, WIFI_PWD);
  //   if(wifiStatus != WL_CONNECTED) ledFeedback(FEEDBACK_BLINK_WIFI_NO_CONNECTION, FEEDBACK_BLINK_WIFI_NO_CONNECTION);
  // }

  // // you're connected now, so print out the data
  ledFeedback(FEEDBACK_BLINK_WIFI_CONNECTED, FEEDBACK_DURATION_WIFI_CONNECTED);
  // // DEBUG
  // Serial.println("You're connected to the network");
  // printWifiStatus();
}

void printWifiStatus() {
  // // print the SSID of the network you're attached to
  // Serial.print("SSID: ");
  // Serial.println(WiFi.SSID());

  // // print your WiFi shield's IP address
  // IPAddress ip = WiFi.localIP();
  // Serial.print("IP Address: ");
  // Serial.println(ip);

  // // print the received signal strength
  // long rssi = WiFi.RSSI();
  // Serial.print("Signal strength (RSSI):");
  // Serial.print(rssi);
  // Serial.println(" dBm");
}
// TODO: rendere la funzione bool in modo tale da poter gestire il caso in cui non ci si riesce a connettere
void connectToServer() {
  ledFeedback(FEEDBACK_BLINK_WIFI_CONNECTING, FEEDBACK_DURATION_WIFI_CONNECTING);
  Serial.println("Starting connection to server...");
  // if you get a connection, report back via serial
  if (client.connect(SERVER, PORT)) {
    ledFeedback(FEEDBACK_BLINK_WIFI_CONNECTED, FEEDBACK_DURATION_WIFI_CONNECTED);
    Serial.println("Connected to server");
  } else {
    ledFeedback(FEEDBACK_BLINK_WIFI_NO_CONNECTION, FEEDBACK_DURATION_WIFI_NO_CONNECTION);
  }
}

void sendToServer() {
  // double distance = measureDistance();
  // String content = "{\"distance\": " + String(distance) + "}";
  // String content_length = String(content.length());
  // Serial.println(String(distance, 4));

  // servoH.detach();

  // // client.print("POST /t/3110/post/ HTTP/1.1" + ret + "Content-Type: application/json" + ret + "Accept: */*" + ret + "Host: ptsv3.com" + ret + "Content-Length: " + content_length + ret + ret + content);
  // // TODO: capire come gestire api_key (se fare dichiarazione o no)
  // client.print("GET /update?api_key=WHH69YD9VAM7NLG5&field1=" + String(distance, 4) + " HTTP/1.1" + RET + "Accept: */*" + RET + "Host: " + SERVER + RET + RET);

  // Serial.println("Sent!");
  // // if there are incoming bytes available
  // // from the server, read them and print them
  // while (client.available()) {
  //   char c = client.read();
  //   Serial.write(c);
  // }
  // Serial.println();

  // servoH.attach(PIN_SERVO_HORIZ);
}

// TODO: in base a come si assembla potrebbero cambiare le funzioni, soprattutto destra e sinistra
void runMotors(byte direction, byte speed) {
  switch (direction) {
    case DIRECTION_STOP: {
      // DEBUG
      Serial.println("Stop");
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
      // DEBUG
      Serial.println("Avanti");
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
      // DEBUG
      Serial.println("Indietro");
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
      // DEBUG
      Serial.println("Destra");
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
      // DEBUG
      Serial.println("Sinistra");
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
    runMotors(DIRECTION_FORWARD, 200 - (speedSlowFactor * 10));
    if (speedSlowFactor < SLOW_FACTOR_MAX) speedSlowFactor++;
  }
  else if (diffDist < -STOP_TRESHOLD && robot_state.direction != DIRECTION_BACKWARD) {
    runMotors(DIRECTION_BACKWARD, 200 - (speedSlowFactor * 10));
    if (speedSlowFactor < SLOW_FACTOR_MAX) speedSlowFactor++;
  }
}

void sendDataToServer() {
  //Feedback
  digitalWrite(LED_BUILTIN, HIGH);
  
  servoH.detach();

  // TODO: capire come gestire api_key (se fare dichiarazione o no)
  client.print("GET /update?api_key=" + String(API_KEY) + "&field1=" + String(measuredDist, DECIMALS) + "&field2=" + String(measuredFilteredDist, DECIMALS) + " HTTP/1.1" + RET + "Accept: */*" + RET + "Host: "+ SERVER + RET + RET);

  Serial.println("Sent!");
  // if there are incoming bytes available
  // from the server, read them and print them
  while (client.available()) {
    char c = client.read();
    Serial.write(c);
  }
  Serial.println();

  servoH.attach(PIN_SERVO_HORIZ);

  //Feedback
  digitalWrite(LED_BUILTIN, LOW);
}