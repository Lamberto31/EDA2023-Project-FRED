#include "IRremote.hpp"
#include <Servo.h>
#include "WiFiEsp.h"
#include "SoftwareSerial.h"

// Digital Pins
#define PIN_ESP_TX 2
#define PIN_ESP_RX 3
#define PIN_ULTRASONIC_ECHO 4
#define PIN_ULTRASONIC_TRIG 5
#define PIN_SERVO_HORIZ 6
#define PIN_MOTOR_IN4 7
#define PIN_MOTOR_IN3 8
#define PIN_MOTOR_ENB 9
#define PIN_IR_RECV 10
#define PIN_MOTOR_ENA 11
#define PIN_MOTOR_IN2 12
#define PIN_MOTOR_IN1 13

// States
#define STATE_SETUP 0
#define STATE_FREE 1
#define STATE_SEARCH 2
#define STATE_MEASURE 3
byte state = STATE_SETUP;

// IR Button-Command
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

// Ultrasonic
#define DECIMALS 4

// Servomotor
#define SERVO_HORIZ_CENTER 90
Servo servoH;

// WiFi
#define WIFI_SSID "Fastweb - Preite - Ospiti"
#define WIFI_PWD "Grp3mTYLFaf1NhJo"
#define SERVER "api.thingspeak.com"
#define RET "\r\n"    //NL & CR characters
SoftwareSerial WifiSerial(PIN_ESP_TX, PIN_ESP_RX);
int wifiStatus = WL_IDLE_STATUS;
WiFiEspClient client;

void setup() {
  // Debug serial communication
  Serial.begin(9600);

  // IR Receiver
  IrReceiver.begin(PIN_IR_RECV, 0);

  // Ultrasonic
  pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);
  pinMode(PIN_ULTRASONIC_ECHO, INPUT);

  // Servomotor
  servoH.attach(PIN_SERVO_HORIZ);
  servoH.write(SERVO_HORIZ_CENTER);

  // WiFi
  //TODO: Capire perchè quando si esegue questa funzione il servo si muove a occhio
  wifiInitializeConnect();
}

void loop() {

  if (IrReceiver.decode()) {
    IrReceiver.resume();
    switch (IrReceiver.decodedIRData.command) {
      case IR_BUTTON_OK: {
          Serial.println("OK");
          Serial.println(measureDistance(), DECIMALS);
          break;
        }
      case IR_BUTTON_UP: {
          Serial.println("UP");
          break;
        }
      case IR_BUTTON_DOWN: {
          Serial.println("DOWN");
          break;
        }
      case IR_BUTTON_RIGHT: {
          Serial.println("RIGHT");
          break;
        }
      case IR_BUTTON_LEFT: {
          Serial.println("LEFT");
          break;
        }
      default: {
          Serial.println("NO");
        }
    }
  }
  servoH.write(SERVO_HORIZ_CENTER);
  delay(1000);
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
  WifiSerial.begin(9600);

  // ESP module initialization
  WiFi.init(&WifiSerial);
  
  // Check if module is connected
  //TODO: Capire come gestire questa situazione (avvisare tramite feedback)
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue
    while (true);
  }

  // Connect to WiFi network
  // TODO: Capire anche questa situazione
  while ( wifiStatus != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(WIFI_SSID);
    // Connect to WPA/WPA2 network
    wifiStatus = WiFi.begin(WIFI_SSID, WIFI_PWD);
  }

  // you're connected now, so print out the data
  Serial.println("You're connected to the network");
  printWifiStatus();
}

void printWifiStatus(){
  // print the SSID of the network you're attached to
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
