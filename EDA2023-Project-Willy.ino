#include "IRremote.hpp"

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
void setup() {
  Serial.begin(9600);

  // IR Receiver
  IrReceiver.begin(PIN_IR_RECV, 0);

  // Ultrasonic
  pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);
  pinMode(PIN_ULTRASONIC_ECHO, INPUT);

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
}

double measureDistance(){
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
