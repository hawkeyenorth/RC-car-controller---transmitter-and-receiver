#include <SPI.h>
#include <RF24.h>

#define PIN_CE   9
#define PIN_CSN 10

#define STEER_PIN A7
#define THROTTLE_PIN A0
#define SW1_PIN 7
#define SW2_PIN 8

RF24 radio(PIN_CE, PIN_CSN);
const byte address[6] = "00001";

struct Packet {
  uint8_t steer;
  uint8_t throttle;
  uint8_t sw1;
  uint8_t sw2;
  uint16_t heartbeat;
};

uint16_t hb = 0;

// Debounce state
unsigned long lastDebounceSW1 = 0;
unsigned long lastDebounceSW2 = 0;
const unsigned long debounceDelay = 50;

bool lastReadingSW1 = HIGH;
bool stableSW1 = HIGH;

bool lastReadingSW2 = HIGH;
bool stableSW2 = HIGH;

void setup() {
  Serial.begin(9600);
  delay(500);

  pinMode(SW1_PIN, INPUT_PULLUP);
  pinMode(SW2_PIN, INPUT_PULLUP);

  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(address);
  radio.stopListening();

  Serial.println("NRF24 Transmitter Ready");
}

void loop() {
  Packet pkt;

  // ----- Joysticks -----
  pkt.steer = map(analogRead(STEER_PIN), 0, 1023, 0, 255);
  pkt.throttle = map(analogRead(THROTTLE_PIN), 0, 1023, 0, 255);

  // ----- Read raw switches -----
  bool readingSW1 = digitalRead(SW1_PIN);
  bool readingSW2 = digitalRead(SW2_PIN);

  pkt.sw1 = 0;
  pkt.sw2 = 0;

  // ----- Debounce SW1 -----
  if (readingSW1 != lastReadingSW1) {
    lastDebounceSW1 = millis();
  }

  if ((millis() - lastDebounceSW1) > debounceDelay) {
    if (readingSW1 != stableSW1) {
      stableSW1 = readingSW1;

      if (stableSW1 == LOW) {
        pkt.sw1 = 1;  // toggle event
      }
    }
  }

  lastReadingSW1 = readingSW1;

  // ----- Debounce SW2 -----
  if (readingSW2 != lastReadingSW2) {
    lastDebounceSW2 = millis();
  }

  if ((millis() - lastDebounceSW2) > debounceDelay) {
    if (readingSW2 != stableSW2) {
      stableSW2 = readingSW2;

      if (stableSW2 == LOW) {
        pkt.sw2 = 1;  // toggle event
      }
    }
  }

  lastReadingSW2 = readingSW2;

  // ----- Heartbeat -----
  pkt.heartbeat = hb++;

  // ----- Send -----
  bool ok = radio.write(&pkt, sizeof(pkt));

  // ----- Debug -----
  Serial.print("Steer=");
  Serial.print(pkt.steer);
  Serial.print(" Thr=");
  Serial.print(pkt.throttle);
  Serial.print(" SW1_evt=");
  Serial.print(pkt.sw1);
  Serial.print(" SW2_evt=");
  Serial.print(pkt.sw2);
  Serial.print(" HB=");
  Serial.print(pkt.heartbeat);
  Serial.println(ok ? " SENT" : " FAIL");

  delay(50);
}