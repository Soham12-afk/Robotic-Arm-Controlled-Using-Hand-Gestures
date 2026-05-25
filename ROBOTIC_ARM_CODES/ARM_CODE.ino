/*
  ROBOTIC ARM - ESP32 WiFi UDP RECEIVER
  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  Coordinated with Python controller:
    Python SEND_DELAY  = 350ms
    ESP32 DEBOUNCE     = 250ms
    → every Python send gets through, no doubles

  WIRING:
    GPIO21 (SDA) --> PCA9685 SDA
    GPIO22 (SCL) --> PCA9685 SCL
    dirPin  = GPIO 4
    stepPin = GPIO 5
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include "HCPCA9685.h"

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// DEBOUNCE — must be < Python SEND_DELAY (350ms)
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
char          lastCmd       = 0;
unsigned long lastCmdTime   = 0;
const int     CMD_DEBOUNCE_MS = 250;   // 250 < 350 → every Python send executes once

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// WiFi CONFIG
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
const char* ssid     = " "; // WIFI/HOTSPOT NAME
const char* password = " "; // WIFI/HOTSPOT PASSWORD
const int   UDP_PORT = 4210;

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// PCA9685
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#define I2CAdd 0x40
HCPCA9685 HCPCA9685(I2CAdd);
WiFiUDP udp;
char packetBuffer[8];

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// MOTOR CONFIG
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// Parking positions (home)
const int PARK_L = 60,  PARK_R = 60;
const int PARK_1 = 70,  PARK_2 = 47;
const int PARK_3 = 63,  PARK_4 = 63;

// Step size per command — tune these for desired speed
int inc_L = 5,  inc_R = 5;
int inc_1 = 5,  inc_2 = 5;
int inc_3 = 5,  inc_4 = 30;

// Current positions (mutable)
int pos_L = PARK_L, pos_R = PARK_R;
int pos_1 = PARK_1, pos_2 = PARK_2;
int pos_3 = PARK_3, pos_4 = PARK_4;

// Limits
const int MIN_L = 10, MAX_L = 180;
const int MIN_R = 10, MAX_R = 180;
const int MIN_1 = 10, MAX_1 = 400;
const int MIN_2 = 10, MAX_2 = 380;
const int MIN_3 = 10, MAX_3 = 380;
const int MIN_4 = 0,  MAX_4 = 500;

// Settle time after servo write (ms)
// Must be long enough for servo to reach position before next cmd
const int SERVO_SETTLE  = 60;
const int WRIST2_SETTLE = 30;

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// STEPPER
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
const int dirPin  = 4;
const int stepPin = 5;
const int STEPS_PER_CMD = 10;       // steps per command packet
const int STEP_DELAY_US = 4000;     // microseconds per half-step

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// SETUP
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
void setup() {
  Serial.begin(115200);

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin,  OUTPUT);

  HCPCA9685.Init(SERVO_MODE);
  HCPCA9685.Sleep(false);

  Serial.print("Connecting WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());    // ← paste this into Python ESP32_IP

  udp.begin(UDP_PORT);
  Serial.print("UDP port: ");
  Serial.println(UDP_PORT);
  Serial.print("Debounce: ");
  Serial.print(CMD_DEBOUNCE_MS);
  Serial.println("ms");
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// LOOP - FIXED
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len > 0) {
      packetBuffer[len] = '\0';
      char state = packetBuffer[0];
      unsigned long now = millis();

      // Accept: new command always | same command only after debounce window
      if (state != lastCmd || (now - lastCmdTime) >= CMD_DEBOUNCE_MS) {
        lastCmd     = state;
        lastCmdTime = now;
        Serial.print("CMD: ");
        Serial.println(state);
        handleCommand(state);
      }
      // else: duplicate within debounce window → silently drop
    }
  }
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// COMMAND ROUTER
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
void handleCommand(char state) {
  switch (state) {
    case 'S': baseRotateLeft();        break;
    case 'O': baseRotateRight();       break;
    case 'c': shoulderServoForward();  break;
    case 'C': shoulderServoBackward(); break;
    case 'p': elbowServoForward();     break;
    case 'P': elbowServoBackward();    break;
    case 'G': wristServo1Backward();   break;
    case 'U': wristServo1Forward();    break;
    case 'R': wristServoCW();          break;
    case 'L': wristServoCCW();         break;
    case 'F': gripperServoBackward();  break;
    case 'f': gripperServoForward();   break;
  }
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// SERVO FUNCTIONS
// Pattern: update pos → clamp → write → settle
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

void gripperServoForward() {
  if (pos_4 > MIN_4) {
    pos_4 = max(pos_4 - inc_4, MIN_4);
    HCPCA9685.Servo(5, pos_4);
    delay(SERVO_SETTLE);
  }
}

void gripperServoBackward() {
  if (pos_4 < MAX_4) {
    pos_4 = min(pos_4 + inc_4, MAX_4);
    HCPCA9685.Servo(5, pos_4);
    delay(SERVO_SETTLE);
  }
}

void wristServoCW() {
  if (pos_3 > MIN_3) {
    pos_3 = max(pos_3 - inc_3, MIN_3);
    HCPCA9685.Servo(4, pos_3);
    delay(WRIST2_SETTLE);
  }
}

void wristServoCCW() {
  if (pos_3 < MAX_3) {
    pos_3 = min(pos_3 + inc_3, MAX_3);
    HCPCA9685.Servo(4, pos_3);
    delay(WRIST2_SETTLE);
  }
}

void wristServo1Forward() {
  if (pos_2 < MAX_2) {
    pos_2 = min(pos_2 + inc_2, MAX_2);
    HCPCA9685.Servo(3, pos_2);
    delay(SERVO_SETTLE);
  }
}

void wristServo1Backward() {
  if (pos_2 > MIN_2) {
    pos_2 = max(pos_2 - inc_2, MIN_2);
    HCPCA9685.Servo(3, pos_2);
    delay(SERVO_SETTLE);
  }
}

void elbowServoForward() {
  if (pos_L < MAX_L) {
    pos_L = min(pos_L + inc_L, MAX_L);
    pos_R = MAX_L - pos_L;               // mirror joint
    HCPCA9685.Servo(0, pos_L);
    HCPCA9685.Servo(1, pos_R);
    delay(SERVO_SETTLE);
  }
}

void elbowServoBackward() {
  if (pos_L > MIN_L) {
    pos_L = max(pos_L - inc_L, MIN_L);
    pos_R = MAX_L - pos_L;               // mirror joint
    HCPCA9685.Servo(0, pos_L);
    HCPCA9685.Servo(1, pos_R);
    delay(SERVO_SETTLE);
  }
}

void shoulderServoForward() {
  if (pos_1 < MAX_1) {
    pos_1 = min(pos_1 + inc_1, MAX_1);
    HCPCA9685.Servo(2, pos_1);
    delay(SERVO_SETTLE);
  }
}

void shoulderServoBackward() {
  if (pos_1 > MIN_1) {
    pos_1 = max(pos_1 - inc_1, MIN_1);
    HCPCA9685.Servo(2, pos_1);
    delay(SERVO_SETTLE);
  }
}

// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
// STEPPER
// ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
void baseRotateLeft() {
  digitalWrite(dirPin, LOW);
  for (int i = 0; i < STEPS_PER_CMD; i++) {
    digitalWrite(stepPin, HIGH); delayMicroseconds(STEP_DELAY_US);
    digitalWrite(stepPin, LOW);  delayMicroseconds(STEP_DELAY_US);
  }
}

void baseRotateRight() {
  digitalWrite(dirPin, HIGH);
  for (int i = 0; i < STEPS_PER_CMD; i++) {
    digitalWrite(stepPin, HIGH); delayMicroseconds(STEP_DELAY_US);
    digitalWrite(stepPin, LOW);  delayMicroseconds(STEP_DELAY_US);
  }
}
