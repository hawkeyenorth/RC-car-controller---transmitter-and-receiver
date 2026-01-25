#include <SPI.h>
#include <RF24.h>
#include <Servo.h>
#include <EEPROM.h>

// ---------------- NRF24 PINS ----------------
#define PIN_CE   9
#define PIN_CSN 10

// ---------------- SERVO ----------------
#define SERVO_PIN 8   // Servo library works on any digital pin

// ---------------- MOTOR DRIVER (L293D) ----------------
#define MOTOR_FWD 5   // PWM forward
#define MOTOR_REV 6   // PWM reverse

// ---------------- LED OUTPUTS ----------------
#define SW1_OUT 3     // Headlight toggle LED
#define SW2_A   4     // Strobe LED A
#define SW2_B   7     // Strobe LED B

// ---------------- CALIBRATION SWITCH ----------------
#define KILL_SWITCH 2   // Calibration switch (to GND, INPUT_PULLUP)

// ---------------- RADIO SETUP ----------------
RF24 radio(PIN_CE, PIN_CSN);
const byte address[6] = "00001";

struct Packet {
  uint8_t steer;
  uint8_t throttle;
  uint8_t sw1;       // 1 = toggle headlights
  uint8_t sw2;       // 1 = toggle strobes
  uint16_t heartbeat;
};

unsigned long lastHB = 0;
const unsigned long TIMEOUT = 1000;  // 1 second failsafe

Servo steering;

bool sw1State = false;   // headlight ON/OFF
bool sw2State = false;   // strobe mode ON/OFF
unsigned long sw2Timer = 0;

bool firstPacket = true; // ensures lights default OFF

// EEPROM-stored zero points
uint8_t zeroSteer = 127;     // default midpoints
uint8_t zeroThrottle = 127;

void setup() {
  Serial.begin(9600);
  delay(500);

  // LED outputs
  pinMode(SW1_OUT, OUTPUT);
  pinMode(SW2_A, OUTPUT);
  pinMode(SW2_B, OUTPUT);

  // Force ALL lights OFF at startup
  digitalWrite(SW1_OUT, LOW);
  digitalWrite(SW2_A, LOW);
  digitalWrite(SW2_B, LOW);

  // Calibration switch
  pinMode(KILL_SWITCH, INPUT_PULLUP);

  // Servo
  steering.attach(SERVO_PIN);

  // Load zero points from EEPROM
  zeroSteer = EEPROM.read(0);
  zeroThrottle = EEPROM.read(1);

  if (zeroSteer == 0xFF) zeroSteer = 127;
  if (zeroThrottle == 0xFF) zeroThrottle = 127;

  Serial.print("Loaded zeroSteer: ");
  Serial.println(zeroSteer);
  Serial.print("Loaded zeroThrottle: ");
  Serial.println(zeroThrottle);

  // Radio
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(0, address);
  radio.startListening();

  Serial.println("NRF24 Receiver Ready");
}

void loop() {
  Packet pkt;

  // ---------------- NORMAL OPERATION ----------------
  if (radio.available()) {
    radio.read(&pkt, sizeof(pkt));
    lastHB = millis();

    // On FIRST packet, force all lights OFF
    if (firstPacket) {
      digitalWrite(SW1_OUT, LOW);
      digitalWrite(SW2_A, LOW);
      digitalWrite(SW2_B, LOW);
      sw1State = false;
      sw2State = false;
      firstPacket = false;
    }

    // ----- Calibration Switch (store zero points) -----
    bool calibrate = (digitalRead(KILL_SWITCH) == LOW);

    if (calibrate) {
      // Store current transmitter values as new zero points
      zeroSteer = pkt.steer;
      zeroThrottle = pkt.throttle;

      EEPROM.write(0, zeroSteer);
      EEPROM.write(1, zeroThrottle);

      Serial.print("Calibrated zeroSteer = ");
      Serial.println(zeroSteer);
      Serial.print("Calibrated zeroThrottle = ");
      Serial.println(zeroThrottle);

      // ----- Calibration Complete: Flash strobes 5 times -----
      for (int i = 0; i < 5; i++) {
        digitalWrite(SW2_A, HIGH);
        digitalWrite(SW2_B, LOW);
        delay(150);

        digitalWrite(SW2_A, LOW);
        digitalWrite(SW2_B, HIGH);
        delay(150);
      }

      // Turn strobes off after sequence
      digitalWrite(SW2_A, LOW);
      digitalWrite(SW2_B, LOW);

      // Output neutral based on new zero points
      steering.writeMicroseconds(1400);
      analogWrite(MOTOR_FWD, 0);
      analogWrite(MOTOR_REV, 0);

      return;   // Skip normal processing while switch is held
    }

    // ----- Steering using pulse width (1250–1550 µs, center 1400) -----
    int steer = pkt.steer;
    int steerAdj = (int)steer - (int)zeroSteer;  // centered around zero

    // Convert steerAdj (-127..127) into pulse width
    int pulseWidth = map(steerAdj, -127, 127, 1250, 1550);

    // Constrain for safety
    if (pulseWidth < 1250) pulseWidth = 1250;
    if (pulseWidth > 1550) pulseWidth = 1550;

    steering.writeMicroseconds(pulseWidth);

    // ----- Throttle (NO deadband, centered on calibrated zero) -----
    int throttle = (int)pkt.throttle - (int)zeroThrottle;

    if (throttle > 0) {
      analogWrite(MOTOR_FWD, throttle * 2);
      analogWrite(MOTOR_REV, 0);
    } else {
      analogWrite(MOTOR_FWD, 0);
      analogWrite(MOTOR_REV, -throttle * 2);
    }

    // ----- SW1: Headlight toggle -----
    if (pkt.sw1 == 1) {
      sw1State = !sw1State;
      digitalWrite(SW1_OUT, sw1State);
    }

    // ----- SW2: Strobe toggle -----
    if (pkt.sw2 == 1) {
      sw2State = !sw2State;
      sw2Timer = millis();
    }

    // ----- Strobe Alternation -----
    if (sw2State) {
      if (millis() - sw2Timer > 500) {
        sw2Timer = millis();

        // Toggle LED A
        bool newState = !digitalRead(SW2_A);
        digitalWrite(SW2_A, newState);

        // LED B is always opposite
        digitalWrite(SW2_B, !newState);
      }
    } else {
      digitalWrite(SW2_A, LOW);
      digitalWrite(SW2_B, LOW);
    }

    // ----- Debug -----
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

    return;  // Prevents failsafe from running during valid packets
  }

  // ---------------- FAILSAFE MODE ----------------
  if (millis() - lastHB > TIMEOUT) {
    // Stop motor
    analogWrite(MOTOR_FWD, 0);
    analogWrite(MOTOR_REV, 0);

    // Center steering (absolute center)
    steering.writeMicroseconds(1400);

    // Turn off strobes
    digitalWrite(SW2_A, LOW);
    digitalWrite(SW2_B, LOW);

    // Flash headlights at 1-second interval
    static unsigned long flashTimer = 0;
    if (millis() - flashTimer > 1000) {
      flashTimer = millis();
      digitalWrite(SW1_OUT, !digitalRead(SW1_OUT));
    }

    Serial.println("FAILSAFE: No signal");
  }
}