#include <SPI.h>
#include <RF24.h>
#include <Servo.h>
#include <EEPROM.h>

// ---------------- NRF24 PINS ----------------
#define PIN_CE   9
#define PIN_CSN 10

// ---------------- SERVO ----------------
#define SERVO_PIN 8

// ---------------- MOTOR DRIVER (L293D) ----------------
#define MOTOR_FWD 5
#define MOTOR_REV 6

// ---------------- LED OUTPUTS ----------------
#define SW1_OUT 3
#define SW2_A   4
#define SW2_B   7

// ---------------- CALIBRATION SWITCH ----------------
#define KILL_SWITCH 2

// ---------------- RADIO SETUP ----------------
RF24 radio(PIN_CE, PIN_CSN);
const byte address[6] = "00001";

struct Packet {
  uint8_t steer;
  uint8_t throttle;
  uint8_t sw1;
  uint8_t sw2;
  uint16_t heartbeat;
};

unsigned long lastHB = 0;
const unsigned long TIMEOUT = 1000;

Servo steering;

bool sw1State = false;
bool sw2State = false;
unsigned long sw2Timer = 0;

bool firstPacket = true;

// EEPROM-stored zero points
uint8_t zeroSteer = 127;
uint8_t zeroThrottle = 127;

void setup() {
  Serial.begin(9600);
  delay(500);

  pinMode(SW1_OUT, OUTPUT);
  pinMode(SW2_A, OUTPUT);
  pinMode(SW2_B, OUTPUT);

  digitalWrite(SW1_OUT, LOW);
  digitalWrite(SW2_A, LOW);
  digitalWrite(SW2_B, LOW);

  pinMode(KILL_SWITCH, INPUT_PULLUP);

  steering.attach(SERVO_PIN);

  zeroSteer = EEPROM.read(0);
  zeroThrottle = EEPROM.read(1);

  if (zeroSteer == 0xFF) zeroSteer = 127;
  if (zeroThrottle == 0xFF) zeroThrottle = 127;

  Serial.print("Loaded zeroSteer: ");
  Serial.println(zeroSteer);
  Serial.print("Loaded zeroThrottle: ");
  Serial.println(zeroThrottle);

  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(0, address);
  radio.startListening();

  Serial.println("NRF24 Receiver Ready");
}

void loop() {
  Packet pkt;

  // ---------------- RADIO PACKET HANDLING ----------------
  if (radio.available()) {
    radio.read(&pkt, sizeof(pkt));

    // Corrupted/duplicate packet protection
    static uint16_t lastGoodHB = 0;

    if (pkt.heartbeat == 0 || pkt.heartbeat == lastGoodHB) {
      // Ignore bad packet but DO NOT return
    } else {
      lastGoodHB = pkt.heartbeat;
      lastHB = millis();

      // First packet resets outputs
      if (firstPacket) {
        digitalWrite(SW1_OUT, LOW);
        digitalWrite(SW2_A, LOW);
        digitalWrite(SW2_B, LOW);
        sw1State = false;
        sw2State = false;
        firstPacket = false;
      }

      // ---------------- CALIBRATION MODE ----------------
      bool calibrate = (digitalRead(KILL_SWITCH) == LOW);

      if (calibrate) {
        zeroSteer = pkt.steer;
        zeroThrottle = pkt.throttle;

        EEPROM.write(0, zeroSteer);
        EEPROM.write(1, zeroThrottle);

        Serial.print("Calibrated zeroSteer = ");
        Serial.println(zeroSteer);
        Serial.print("Calibrated zeroThrottle = ");
        Serial.println(zeroThrottle);

        for (int i = 0; i < 5; i++) {
          digitalWrite(SW2_A, HIGH);
          digitalWrite(SW2_B, LOW);
          delay(150);

          digitalWrite(SW2_A, LOW);
          digitalWrite(SW2_B, HIGH);
          delay(150);
        }

        digitalWrite(SW2_A, LOW);
        digitalWrite(SW2_B, LOW);

        steering.writeMicroseconds(1400);
        analogWrite(MOTOR_FWD, 0);
        analogWrite(MOTOR_REV, 0);
      }

      // ---------------- STEERING ----------------
      int steer = pkt.steer;
      int steerAdj = (int)steer - (int)zeroSteer;

      if (steerAdj > -2 && steerAdj < 2) steerAdj = 0;

      int pulseWidth = map(steerAdj, -127, 127, 1250, 1550);
      pulseWidth = constrain(pulseWidth, 1250, 1550);

      static int lastPulse = 1400;
      if (abs(pulseWidth - lastPulse) > 3) {
        steering.writeMicroseconds(pulseWidth);
        lastPulse = pulseWidth;
      }

      // ---------------- THROTTLE ----------------
      int throttle = (int)pkt.throttle - (int)zeroThrottle;

      if (throttle > 0) {
        int pwm = throttle * 2;
        pwm = constrain(pwm, 0, 255);
        analogWrite(MOTOR_FWD, pwm);
        analogWrite(MOTOR_REV, 0);
      } else {
        int pwm = (-throttle) * 2;
        pwm = constrain(pwm, 0, 255);
        analogWrite(MOTOR_FWD, 0);
        analogWrite(MOTOR_REV, pwm);
      }

      // ---------------- SW1 (toggle) ----------------
      if (pkt.sw1 == 1) {
        sw1State = !sw1State;
        digitalWrite(SW1_OUT, sw1State);
      }

      // ---------------- SW2 (alternating strobe) ----------------
      if (pkt.sw2 == 1) {
        sw2State = !sw2State;
        sw2Timer = millis();
      }

      if (sw2State) {
        if (millis() - sw2Timer > 500) {
          sw2Timer = millis();
          bool newState = !digitalRead(SW2_A);
          digitalWrite(SW2_A, newState);
          digitalWrite(SW2_B, !newState);
        }
      } else {
        digitalWrite(SW2_A, LOW);
        digitalWrite(SW2_B, LOW);
      }

      // ---------------- DEBUG ----------------
      Serial.print("SteerRaw=");
      Serial.print(steer);
      Serial.print(" SteerAdj=");
      Serial.print(steerAdj);
      Serial.print(" Pulse=");
      Serial.print(pulseWidth);
      Serial.print(" ThrAdj=");
      Serial.print(throttle);
      Serial.print(" SW1=");
      Serial.print(sw1State);
      Serial.print(" SW2=");
      Serial.print(sw2State);
      Serial.print(" HB=");
      Serial.println(pkt.heartbeat);
    }
  }

  // ---------------- FAILSAFE ----------------
  if (millis() - lastHB > TIMEOUT) {
    analogWrite(MOTOR_FWD, 0);
    analogWrite(MOTOR_REV, 0);

    steering.writeMicroseconds(1400);

    digitalWrite(SW2_A, LOW);
    digitalWrite(SW2_B, LOW);

    static unsigned long flashTimer = 0;
    if (millis() - flashTimer > 1000) {
      flashTimer = millis();
      digitalWrite(SW1_OUT, !digitalRead(SW1_OUT));
    }

    Serial.println("FAILSAFE: No signal");
  }
}
