#include <SPI.h>

static const uint8_t CS_PIN = 10;
static const uint16_t ANGLE_REG = 0x3FFF;

// ---------- parity ----------
static inline uint8_t evenParity16(uint16_t x) {
  x ^= x >> 8;
  x ^= x >> 4;
  x ^= x >> 2;
  x ^= x >> 1;
  return x & 1;
}

// Build AS5048A read command
// bit 15 = parity
// bit 14 = 1 for read
// bits 13:0 = register
static inline uint16_t makeReadCommand(uint16_t reg) {
  uint16_t cmd = 0x4000 | (reg & 0x3FFF);
  if (evenParity16(cmd)) {
    cmd |= 0x8000;
  }
  return cmd;
}

// Single 16-bit SPI frame
static inline uint16_t spiFrame(uint16_t data) {
  digitalWrite(CS_PIN, LOW);
  uint16_t resp = SPI.transfer16(data);
  digitalWrite(CS_PIN, HIGH);
  return resp;
}

// Read one register from AS5048A
// First frame sends command
// Second frame gets response
static inline uint16_t readAS5048A(uint16_t reg) {
  spiFrame(makeReadCommand(reg));
  return spiFrame(0x0000);
}

// Return raw 14-bit angle: 0..16383
static inline uint16_t readRawAngle() {
  return readAS5048A(ANGLE_REG) & 0x3FFF;
}

// Return degrees: 0..360
static inline float rawToDegrees(uint16_t raw) {
  return raw * (360.0f / 16384.0f);
}

void setup() {
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  SPI.begin();

  // AS5048A uses SPI mode 1
  // 8 MHz is the max on Uno if signal quality is good
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE1));

  Serial.begin(2000000);

  // Prime pipeline once
  readRawAngle();
}

void loop() {
  uint16_t raw = readRawAngle();

  // Fastest output:
  // raw integer only
  Serial.println(raw);

  // If you want degrees instead, use this instead:
  // float deg = rawToDegrees(raw);
  // Serial.println(deg, 3);
}