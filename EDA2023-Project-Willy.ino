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
state robot_state = {STATE_SETUP, 0, true};

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

// WiFi
// #define WIFI_SSID "Fastweb - Preite - Ospiti"
// #define WIFI_PWD "Grp3mTYLFaf1NhJo"
#define WIFI_SSID "Fastweb-Mauro"
#define WIFI_PWD "31EYPGxyASL!G?"
#define SERVER "api.thingspeak.com"
#define PORT 80
#define RET "\r\n"  //NL & CR characters
// SoftwareSerial WifiSerial(PIN_ESP_TX, PIN_ESP_RX);
int wifiStatus = WL_IDLE_STATUS;
WiFiEspClient client;

// Servomotor
#define SERVO_HORIZ_CENTER 90
Servo servoH;

// Motors
#define DIRECTION_STOP 0
#define DIRECTION_FORWARD 1
#define DIRECTION_BACKWARD 2
#define DIRECTION_RIGHT 3
#define DIRECTION_LEFT 4

// Custom distance in cm
char customDist [4] = "000";
byte customDistIdx = 0;
int numericCustomDist = 0;

void setup() {
  // Debug serial communication
  Serial.begin(9600);

  // IR Receiver
  if (!initPCIInterruptForTinyReceiver()) {
    Serial.println("No interrupt available");
  }

  // Ultrasonic
  pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);
  pinMode(PIN_ULTRASONIC_ECHO, INPUT);

  // WiFi
  // wifiInitializeConnect();
  // connectToServer();

  // Servomotor
  servoH.attach(PIN_SERVO_HORIZ);
  servoH.write(SERVO_HORIZ_CENTER);

  // Motors
  pinMode(PIN_MOTOR_ENA, OUTPUT);
  pinMode(PIN_MOTOR_ENB, OUTPUT);

  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  pinMode(PIN_MOTOR_IN3, OUTPUT);
  pinMode(PIN_MOTOR_IN4, OUTPUT);

  stateChange(&robot_state, STATE_FREE);
}

void loop() {

  if(!robot_state.cmd_executed) {
    switch(robot_state.current) {
      // Free state handling
      case STATE_FREE: {
        switch(robot_state.command) {
          case IR_BUTTON_OK: {
            runMotors(DIRECTION_STOP, 0);
            stateCmdExecuted(&robot_state);
            break;
          }
          case IR_BUTTON_UP: {
            runMotors(DIRECTION_FORWARD, 200);
            stateCmdExecuted(&robot_state);
            break;
          }
          case IR_BUTTON_DOWN: {
            runMotors(DIRECTION_BACKWARD, 200);
            stateCmdExecuted(&robot_state);
            break;
          }
          case IR_BUTTON_RIGHT: {
            runMotors(DIRECTION_RIGHT, 100);
            stateCmdExecuted(&robot_state);
            break;
          }
          case IR_BUTTON_LEFT: {
            runMotors(DIRECTION_LEFT, 100);
            stateCmdExecuted(&robot_state);
            break;
          }
          case IR_BUTTON_HASH: {
            stateChange(&robot_state, STATE_MEASURE);
            stateCmdExecuted(&robot_state);
            break;
          }
          case IR_BUTTON_AST: {
            stateChange(&robot_state, STATE_READING);
            stateCmdExecuted(&robot_state);
            break;
          }
          default: {
            stateCmdExecuted(&robot_state);
            Serial.println("NO");
          }
        }
        break;
      }
            Serial.println("NO");
          }
        }
        break;
      }
      // Search state handling
      case STATE_SEARCH: {
        //TODO SEARCH
        break;
      }
      // Measure state handling
      case STATE_MEASURE: {
        //TODO MEASURE
        break;
      }
    }
  }

  // if (IrReceiver.decode()) {
  //   IrReceiver.resume();
  //   switch (IrReceiver.decodedIRData.command) {
  //     case IR_BUTTON_OK: {
  //       Serial.println("OK");
  //       Serial.println(measureDistance(), DECIMALS);
  //       runMotors(DIRECTION_STOP, 0);
  //       break;
  //     }
  //     case IR_BUTTON_UP: {
  //       Serial.println("UP");
  //       runMotors(DIRECTION_FORWARD, 200);
  //       break;
  //     }
  //     case IR_BUTTON_DOWN: {
  //       Serial.println("DOWN");
  //       runMotors(DIRECTION_BACKWARD, 100);
  //       break;
  //     }
  //     case IR_BUTTON_RIGHT: {
  //       Serial.println("RIGHT");
  //       runMotors(DIRECTION_RIGHT,100);
  //       break;
  //     }
  //     case IR_BUTTON_LEFT: {
  //       Serial.println("LEFT");
  //       runMotors(DIRECTION_LEFT,100);
  //       break;
  //     }
  //     default: {
  //       Serial.println("NO");
  //     }
  //   }
  // }
  servoH.write(SERVO_HORIZ_CENTER);
  // sendToServer();
}

// This is the function, which is called if a complete command was received
// It runs in an ISR context with interrupts enabled
void handleReceivedTinyIRData(uint8_t aAddress, uint8_t aCommand, uint8_t aFlags) {
  printTinyReceiverResultMinimal(&Serial, aAddress, aCommand, aFlags);
  if (!aFlags == IRDATA_FLAGS_IS_REPEAT) {
  stateNewCmd(&robot_state, aCommand);
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

// void wifiInitializeConnect() {
//   WifiSerial.begin(9600);

//   // ESP module initialization
//   WiFi.init(&WifiSerial);

//   // Check if module is connected
//   //TODO: Capire come gestire questa situazione (avvisare tramite feedback)
//   if (WiFi.status() == WL_NO_SHIELD) {
//     Serial.println("WiFi shield not present");
//     // don't continue
//     while (true);
//   }

//   // Connect to WiFi network
//   // TODO: Capire anche questa situazione
//   while (wifiStatus != WL_CONNECTED) {
//     Serial.print("Attempting to connect to WPA SSID: ");
//     Serial.println(WIFI_SSID);
//     // Connect to WPA/WPA2 network
//     wifiStatus = WiFi.begin(WIFI_SSID, WIFI_PWD);
//   }

//   // you're connected now, so print out the data
//   Serial.println("You're connected to the network");
//   printWifiStatus();
// }

// void printWifiStatus() {
//   // print the SSID of the network you're attached to
//   Serial.print("SSID: ");
//   Serial.println(WiFi.SSID());

//   // print your WiFi shield's IP address
//   IPAddress ip = WiFi.localIP();
//   Serial.print("IP Address: ");
//   Serial.println(ip);

//   // print the received signal strength
//   long rssi = WiFi.RSSI();
//   Serial.print("Signal strength (RSSI):");
//   Serial.print(rssi);
//   Serial.println(" dBm");
// }

void connectToServer() {
  Serial.println("Starting connection to server...");
  // if you get a connection, report back via serial
  if (client.connect(SERVER, PORT)) {
    Serial.println("Connected to server");
  }
}

void sendToServer() {
  double distance = measureDistance();
  String content = "{\"distance\": " + String(distance) + "}";
  String content_length = String(content.length());
  Serial.println(String(distance, 4));

  servoH.detach();

  // client.print("POST /t/3110/post/ HTTP/1.1" + ret + "Content-Type: application/json" + ret + "Accept: */*" + ret + "Host: ptsv3.com" + ret + "Content-Length: " + content_length + ret + ret + content);
  // TODO: capire come gestire api_key (se fare dichiarazione o no) e sistemare nomi dei campi (field1)
  client.print("GET /update?api_key=WHH69YD9VAM7NLG5&field1=" + String(distance, 4) + " HTTP/1.1" + RET + "Accept: */*" + RET + "Host: api.thingspeak.com" + RET + RET);

  Serial.println("Sent!");
  // if there are incoming bytes available
  // from the server, read them and print them
  while (client.available()) {
    char c = client.read();
    Serial.write(c);
  }
  Serial.println();

  servoH.attach(PIN_SERVO_HORIZ);
}

void runMotors(byte direction, byte speed) {
  Serial.println(direction);
  Serial.println(speed);
  switch (direction) {
    case DIRECTION_STOP:
      {
        Serial.println("Stop");
        digitalWrite(PIN_MOTOR_IN1, LOW);
        digitalWrite(PIN_MOTOR_IN2, LOW);
        digitalWrite(PIN_MOTOR_IN3, LOW);
        digitalWrite(PIN_MOTOR_IN4, LOW);
        analogWrite(PIN_MOTOR_ENA, 0);
        analogWrite(PIN_MOTOR_ENB, 0);
        break;
      }
    case DIRECTION_FORWARD:
      {
        Serial.println("Avanti");
        digitalWrite(PIN_MOTOR_IN1, HIGH);
        digitalWrite(PIN_MOTOR_IN2, LOW);
        digitalWrite(PIN_MOTOR_IN3, HIGH);
        digitalWrite(PIN_MOTOR_IN4, LOW);
        analogWrite(PIN_MOTOR_ENA, speed);
        analogWrite(PIN_MOTOR_ENB, speed);
        break;
      }
    case DIRECTION_BACKWARD:
      {
        Serial.println("Indietro");
        digitalWrite(PIN_MOTOR_IN1, LOW);
        digitalWrite(PIN_MOTOR_IN2, HIGH);
        digitalWrite(PIN_MOTOR_IN3, LOW);
        digitalWrite(PIN_MOTOR_IN4, HIGH);
        analogWrite(PIN_MOTOR_ENA, speed);
        analogWrite(PIN_MOTOR_ENB, speed);
        break;
      }
    case DIRECTION_RIGHT:
      {
        Serial.println("Destra");
        break;
      }
    case DIRECTION_LEFT:
      {
        Serial.println("Sinistra");
        break;
      }
  }
}

void readCustomDistance(char digit) {
  if (customDistIdx == 3) {
    resetCustomDistance();
    stateChange(&robot_state, STATE_FREE);
  }
  else {
  customDist[customDistIdx] = digit;
  customDistIdx++;
  }
}

void resetCustomDistance() {
    customDist[0] = '0';
    customDist[1] = '0';
    customDist[2] = '0';
    customDistIdx = 0;
}
