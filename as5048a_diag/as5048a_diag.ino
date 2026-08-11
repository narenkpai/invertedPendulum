/*
 * as5048a_diag.ino — hardware probe for a dead/garbled AS5048A SPI bus.
 * Symptom this diagnoses: angle reads stuck at 0 or 16383 (MISO at a rail).
 *
 * Wiring under test: CS -> D10, SCK -> D13, MOSI -> D11, MISO -> D12.
 * Bit-bangs SPI mode 1 slowly (no SPI library), so it can also control the
 * MISO pin's pull-up to detect a floating (undriven) line.
 *
 * Open Serial Monitor @ 115200. Repeats every 2 s — wiggle wires while it
 * runs and watch for the verdict changing.
 */

const uint8_t PIN_CS   = 10;
const uint8_t PIN_SCK  = 13;
const uint8_t PIN_MOSI = 11;
const uint8_t PIN_MISO = 12;

// One 16-bit frame, SPI mode 1 (clock idles low, sample on falling edge)
uint16_t frame(uint16_t out, bool misoPullup) {
  pinMode(PIN_MISO, misoPullup ? INPUT_PULLUP : INPUT);
  uint16_t in = 0;
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(5);
  for (int8_t i = 15; i >= 0; i--) {
    digitalWrite(PIN_SCK, HIGH);
    digitalWrite(PIN_MOSI, (out >> i) & 1);
    delayMicroseconds(5);
    digitalWrite(PIN_SCK, LOW);
    delayMicroseconds(2);
    in = (in << 1) | digitalRead(PIN_MISO);
    delayMicroseconds(3);
  }
  digitalWrite(PIN_CS, HIGH);
  delayMicroseconds(5);
  return in;
}

void printHexBin(uint16_t v) {
  Serial.print(F("0x"));
  if (v < 0x1000) Serial.print('0');
  if (v < 0x100)  Serial.print('0');
  if (v < 0x10)   Serial.print('0');
  Serial.print(v, HEX);
}

void setup() {
  pinMode(PIN_CS, OUTPUT);   digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_SCK, OUTPUT);  digitalWrite(PIN_SCK, LOW);
  pinMode(PIN_MOSI, OUTPUT);
  pinMode(PIN_MISO, INPUT);
  Serial.begin(115200);
  Serial.println(F("\n== AS5048A bus diagnostic =="));
}

void loop() {
  // 0xFFFF = valid 'read angle' command (parity works out). Send it twice:
  // the second frame carries the response to the first.
  frame(0xFFFF, true);
  uint16_t withPullup = frame(0xFFFF, true);
  frame(0xFFFF, false);
  uint16_t noPullup   = frame(0xFFFF, false);

  Serial.print(F("response with pull-up: "));
  printHexBin(withPullup);
  Serial.print(F("   without: "));
  printHexBin(noPullup);
  Serial.println();

  bool floating = (withPullup == 0xFFFF && noPullup == 0x0000) ||
                  (withPullup == 0xFFFF && noPullup == 0xFFFF) ||
                  (withPullup == 0x0000 && noPullup == 0x0000);

  if (floating) {
    Serial.println(F("VERDICT: MISO is NOT being driven by the sensor."));
    Serial.println(F("  -> check in this order: sensor power (measure VCC at"));
    Serial.println(F("     the sensor board), GND to Uno, CS wire on D10,"));
    Serial.println(F("     MISO wire on D12. One of these is disconnected,"));
    Serial.println(F("     or the sensor is dead."));
  } else {
    uint16_t angle = withPullup & 0x3FFF;
    Serial.print(F("VERDICT: sensor IS driving the bus. angle="));
    Serial.print(angle);
    if (withPullup & 0x4000) {
      Serial.print(F("  (error flag set — magnet/field problem, check gap)"));
    }
    Serial.println();
    Serial.println(F("  If angle looks sane here but the main sketch reads 0/16383,"));
    Serial.println(F("  suspect SCK/MOSI wires (D13/D11) or noise pickup."));
  }
  Serial.println();
  delay(2000);
}
