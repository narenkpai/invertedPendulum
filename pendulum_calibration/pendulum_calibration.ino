/*
 * pendulum_calibration.ino — one-time hardware calibration for the
 * inverted-pendulum gantry. Results are stored in EEPROM; the balance
 * (LQR) sketch reads them back, so this only needs to run once.
 *
 * What it measures and stores:
 *   1. Pendulum "hanging down" angle (AS5048A raw counts, circular mean)
 *   2. Motor direction sign: which DIR level increases MT6835 counts (+x)
 *   3. Scale: MT6835 encoder counts per commanded step
 *   4. Pendulum angle sign: which raw-count direction = leaning toward +x
 *   5. Total gantry travel + soft-stop margin, found by driving into both
 *      hard stops using encoder-stall detection (no limit switches)
 *
 * Wiring:
 *   TB6600:  PUL+ -> D9, DIR+ -> D2, PUL-/DIR- -> GND, ENA unconnected
 *   AS5048A: CS -> D10, hardware SPI D11/D12/D13   (pendulum pivot)
 *   MT6835:  CSN -> A0, SCK -> A1, MOSI -> A2, MISO -> A3  (motor shaft)
 *
 * Procedure (Serial Monitor @ 115200):
 *   - Place the cart roughly mid-travel
 *   - Let the pendulum hang completely still
 *   - Send 'g' and don't touch anything until it reports DONE
 *   - 'm' at any time: live monitor of angle/position using stored cal
 */

#include <SPI.h>
#include <EEPROM.h>

// ---------------- pins ----------------
const uint8_t PIN_DIR   = 2;    // TB6600 DIR+
const uint8_t PIN_PUL   = 9;    // TB6600 PUL+
const uint8_t PIN_AS_CS = 10;   // AS5048A chip select (hardware SPI)

const uint8_t PIN_MT_CS   = A0; // MT6835 bit-banged SPI
const uint8_t PIN_MT_SCK  = A1;
const uint8_t PIN_MT_MOSI = A2;
const uint8_t PIN_MT_MISO = A3;

// ---------------- constants ----------------
const int32_t  MT_CPR  = 2097152L;  // MT6835: 2^21 counts / motor rev
const uint16_t AS_CPR  = 16384;     // AS5048A: 2^14 counts / rev
const uint16_t AS_HALF = 8192;

const uint16_t HOME_STEP_US   = 1500;    // homing / slow-move step period
const uint16_t JOG_STEP_US    = 1200;    // snappier jog for sign detection
const int32_t  HOMING_MAX_STEPS = 200000L;
const int16_t  DEFLECT_THRESH = 80;      // ~1.8 deg, auto sign detection
const int16_t  MANUAL_THRESH  = 150;     // ~3.3 deg, manual fallback

// ---------------- calibration data (EEPROM) ----------------
struct CalData {
  uint32_t magic;          // 'PEND'
  uint16_t version;
  uint16_t thetaDownRaw;   // AS5048A raw with pendulum hanging
  int8_t   dirHighIsPos;   // 1 if DIR=HIGH moves cart in +x (MT counts up)
  int8_t   pendSign;       // theta = pendSign * wrap14(raw - upRaw), + = lean +x
  float    countsPerStep;  // MT6835 counts per commanded step
  int32_t  travelCounts;   // wall-to-wall travel, MT counts
  int32_t  marginCounts;   // soft-stop margin from each wall, MT counts
  uint16_t checksum;
};
const uint32_t CAL_MAGIC   = 0x50454E44UL; // 'PEND'
const uint16_t CAL_VERSION = 1;

CalData cal;
bool haveCal = false;

uint16_t calcChecksum(const CalData &c) {
  const uint8_t *p = (const uint8_t *)&c;
  uint16_t sum = 0;
  for (size_t i = 0; i < sizeof(CalData) - sizeof(uint16_t); i++) sum += p[i];
  return sum;
}

// ---------------- angle wrap helpers ----------------
int16_t wrap14(int32_t d) {            // -> [-8192, 8191]
  d %= AS_CPR;
  if (d < -AS_HALF) d += AS_CPR;
  if (d >= AS_HALF) d -= AS_CPR;
  return (int16_t)d;
}

// ---------------- AS5048A (pendulum), hardware SPI ----------------
uint8_t evenParity16(uint16_t x) {
  x ^= x >> 8; x ^= x >> 4; x ^= x >> 2; x ^= x >> 1;
  return x & 1;
}

uint16_t asFrame(uint16_t data) {
  // 1 MHz: 8 MHz over jumper wires causes single-bit read corruption
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  digitalWrite(PIN_AS_CS, LOW);
  uint16_t resp = SPI.transfer16(data);
  digitalWrite(PIN_AS_CS, HIGH);
  SPI.endTransaction();
  return resp;
}

uint16_t makeReadCommand(uint16_t reg) {
  uint16_t cmd = 0x4000 | (reg & 0x3FFF);
  if (evenParity16(cmd)) cmd |= 0x8000;
  return cmd;
}

uint32_t asBadFrames = 0;              // corrupted frames seen (for diagnostics)

// Validated register read: checks the response parity bit and error flag,
// retries on corruption instead of passing garbage to the calibration.
uint16_t asReadReg(uint16_t reg) {
  uint16_t resp = 0;
  for (uint8_t attempt = 0; attempt < 4; attempt++) {
    asFrame(makeReadCommand(reg));
    resp = asFrame(makeReadCommand(0x0000));         // NOP fetches the answer
    bool parityOk  = (evenParity16(resp) == 0);
    bool errorFlag = resp & 0x4000;
    if (parityOk && !errorFlag) return resp & 0x3FFF;
    asBadFrames++;
    if (errorFlag) {                                 // clear error flag register
      asFrame(makeReadCommand(0x0001));
      asFrame(makeReadCommand(0x0000));
    }
  }
  return resp & 0x3FFF;                // all retries failed; best effort
}

uint16_t asReadRaw() {                 // 0..16383
  return asReadReg(0x3FFF);            // angle register
}

// Median of 5 (wrap-aware) — kills any residual single-sample glitches
uint16_t asReadFiltered() {
  uint16_t ref = asReadRaw();
  int16_t d[5];
  d[0] = 0;
  for (uint8_t i = 1; i < 5; i++) d[i] = wrap14((int32_t)asReadRaw() - ref);
  for (uint8_t i = 1; i < 5; i++) {    // insertion sort
    int16_t v = d[i];
    int8_t j = i - 1;
    while (j >= 0 && d[j] > v) { d[j + 1] = d[j]; j--; }
    d[j + 1] = v;
  }
  return (uint16_t)(((int32_t)ref + d[2] + AS_CPR) % AS_CPR);
}

// Magnet placement diagnostics (register 0x3FFD: AGC + compensation flags)
void asPrintDiagnostics() {
  uint16_t d   = asReadReg(0x3FFD);
  uint16_t mag = asReadReg(0x3FFE);
  Serial.print(F("AS5048A diag: AGC="));
  Serial.print(d & 0xFF);              // 0 = strong field ... 255 = too weak
  Serial.print(F("/255, magnitude="));
  Serial.print(mag);
  if (d & 0x0800) Serial.print(F("  WARNING: magnet too WEAK/far"));
  if (d & 0x0400) Serial.print(F("  WARNING: magnet too STRONG/close"));
  Serial.println();
}

// ---------------- MT6835 (motor shaft), bit-banged SPI mode 3 ----------------
uint8_t mtTransfer(uint8_t out) {
  uint8_t in = 0;
  for (int8_t i = 7; i >= 0; i--) {
    digitalWrite(PIN_MT_SCK, LOW);
    digitalWrite(PIN_MT_MOSI, (out >> i) & 1);
    digitalWrite(PIN_MT_SCK, HIGH);
    in = (in << 1) | digitalRead(PIN_MT_MISO);
  }
  return in;
}

uint32_t mtReadRaw() {                 // 0..2^21-1
  digitalWrite(PIN_MT_CS, LOW);
  mtTransfer(0xA0);                    // burst angle read, reg 0x003
  mtTransfer(0x03);
  uint8_t b2 = mtTransfer(0);
  uint8_t b3 = mtTransfer(0);
  uint8_t b4 = mtTransfer(0);
  mtTransfer(0);                       // CRC, ignored
  digitalWrite(PIN_MT_CS, HIGH);
  return ((uint32_t)b2 << 13) | ((uint32_t)b3 << 5) | (b4 >> 3);
}

// multi-turn accumulation (valid within one power cycle)
int32_t  mtPos = 0;
uint32_t mtLastRaw = 0;

int32_t mtUpdate() {
  uint32_t raw = mtReadRaw();
  int32_t delta = (int32_t)raw - (int32_t)mtLastRaw;
  if      (delta >  MT_CPR / 2) delta -= MT_CPR;
  else if (delta < -MT_CPR / 2) delta += MT_CPR;
  mtPos += delta;
  mtLastRaw = raw;
  return mtPos;
}

// ---------------- stepper ----------------
void setDirLevel(uint8_t level) {
  digitalWrite(PIN_DIR, level);
  delayMicroseconds(20);               // DIR setup time before first pulse
}

void setDirX(bool positiveX) {         // +x = direction of increasing MT counts
  setDirLevel((positiveX == (cal.dirHighIsPos == 1)) ? HIGH : LOW);
}

void stepPulse(uint16_t periodUs) {
  digitalWrite(PIN_PUL, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_PUL, LOW);
  delayMicroseconds(periodUs - 10);
}

// ---------------- misc ----------------
void fail(const __FlashStringHelper *msg) {
  Serial.print(F("\nCALIBRATION FAILED: "));
  Serial.println(msg);
  Serial.println(F("Fix the issue and reset the board to retry."));
  while (1) {}
}

float rawToDeg(int32_t counts14) { return counts14 * (360.0f / AS_CPR); }

// ================= calibration phases =================

// Phase 1: pendulum hanging angle (circular mean of 500 samples over ~1 s)
void calPendulumDown() {
  Serial.println(F("[1/5] Measuring pendulum rest (hanging) angle..."));
  asBadFrames = 0;
  uint16_t ref = asReadFiltered();
  int32_t acc = 0;
  int16_t maxDev = 0;
  for (uint16_t i = 0; i < 500; i++) {
    int16_t d = wrap14((int32_t)asReadFiltered() - ref);
    acc += d;
    if (abs(d) > maxDev) maxDev = abs(d);
    delay(2);
  }
  Serial.print(F("      peak deviation during sampling: "));
  Serial.print(maxDev);
  Serial.print(F(" counts, corrupted SPI frames retried: "));
  Serial.println(asBadFrames);
  if (maxDev > 80)                      // ~1.8 deg of wobble during sampling
    fail(F("pendulum was moving during rest measurement — let it settle and retry"));
  cal.thetaDownRaw = (uint16_t)((ref + acc / 500 + AS_CPR) % AS_CPR);
  Serial.print(F("      down angle raw = "));
  Serial.println(cal.thetaDownRaw);
}

// Phase 2+3: motor direction sign and counts-per-step scale
void calScaleAndDir() {
  Serial.println(F("[2/5] Jogging to find motor direction and encoder scale..."));
  const int16_t N = 800;
  int32_t start = mtUpdate();
  setDirLevel(HIGH);
  for (int16_t i = 0; i < N; i++) {
    stepPulse(HOME_STEP_US);
    if ((i & 7) == 7) mtUpdate();
  }
  int32_t deltaOut = mtUpdate() - start;
  setDirLevel(LOW);
  for (int16_t i = 0; i < N; i++) {
    stepPulse(HOME_STEP_US);
    if ((i & 7) == 7) mtUpdate();
  }
  int32_t deltaBack = mtUpdate() - start - deltaOut;

  cal.dirHighIsPos = (deltaOut > 0) ? 1 : 0;
  float measured = (fabs((float)deltaOut) + fabs((float)deltaBack)) / (2.0f * N);
  Serial.print(F("      DIR=HIGH moves +x: "));
  Serial.println(cal.dirHighIsPos ? F("yes") : F("no"));
  Serial.print(F("      measured counts/step = "));
  Serial.println(measured);

  if (measured < 100.0f || measured > 30000.0f)
    fail(F("counts/step implausible — check MT6835 wiring or that the motor actually moved"));

  // Snap to the nearest standard TB6600 microstep setting: belt/coupling
  // compliance makes the raw measurement a few percent off, but the true
  // ratio is exactly 2^21 / (steps per rev).
  const uint16_t sprTable[] = {200, 400, 800, 1600, 3200, 6400};
  float bestErr = 1e9f;
  uint16_t bestSpr = 0;
  for (uint8_t i = 0; i < 6; i++) {
    float cps = (float)MT_CPR / sprTable[i];
    float err = fabs(cps - measured) / cps;
    if (err < bestErr) { bestErr = err; bestSpr = sprTable[i]; }
  }
  if (bestErr < 0.08f) {
    cal.countsPerStep = (float)MT_CPR / bestSpr;
    Serial.print(F("      snapped to "));
    Serial.print(bestSpr);
    Serial.print(F(" steps/rev -> counts/step = "));
    Serial.println(cal.countsPerStep);
  } else {
    cal.countsPerStep = measured;
    Serial.println(F("      WARNING: not near a standard microstep setting, using measured value"));
  }
}

bool waitForStill(uint32_t timeoutMs) {
  uint32_t t0 = millis();
  uint16_t good = 0;
  while (millis() - t0 < timeoutMs) {
    int16_t d = wrap14((int32_t)asReadFiltered() - cal.thetaDownRaw);
    good = (abs(d) < 30) ? good + 1 : 0;
    if (good >= 100) return true;      // still for ~1 s
    delay(10);
  }
  return false;
}

// Phase 4: pendulum sign. Jog the cart in +x; the hanging pendulum lags,
// and that lag has the SAME raw-count sign as the upright pendulum
// leaning toward +x (both are the same rotation direction of the shaft).
void calPendSign() {
  Serial.println(F("[3/5] Waiting for pendulum to settle..."));
  if (!waitForStill(20000)) fail(F("pendulum would not settle"));

  Serial.println(F("[4/5] Jogging +x to detect pendulum sign..."));
  int32_t start = mtUpdate();
  int8_t sign = 0;

  setDirX(true);
  for (int16_t i = 0; i < 250 && sign == 0; i++) {
    stepPulse(JOG_STEP_US);
    if ((i & 7) == 7) mtUpdate();
    int16_t d = wrap14((int32_t)asReadFiltered() - cal.thetaDownRaw);
    if (abs(d) >= DEFLECT_THRESH) sign = (d > 0) ? 1 : -1;
  }
  // keep watching briefly after the jog
  uint32_t t0 = millis();
  while (sign == 0 && millis() - t0 < 400) {
    int16_t d = wrap14((int32_t)asReadFiltered() - cal.thetaDownRaw);
    if (abs(d) >= DEFLECT_THRESH) sign = (d > 0) ? 1 : -1;
  }

  if (sign != 0) {
    cal.pendSign = sign;               // lag(-x) sign == upright lean(+x) sign
  } else {
    // Manual fallback. NOTE: a hanging bob tilted toward +x is the OPPOSITE
    // rotation of an upright pendulum leaning toward +x, hence the minus.
    Serial.println(F("      Auto-detect failed."));
    Serial.println(F("      Tilt the pendulum bob a few degrees in the SAME"));
    Serial.println(F("      direction the cart just moved, and hold it."));
    uint32_t tm = millis();
    while (sign == 0) {
      if (millis() - tm > 30000) fail(F("no manual deflection detected"));
      int16_t d = wrap14((int32_t)asReadFiltered() - cal.thetaDownRaw);
      if (abs(d) >= MANUAL_THRESH) sign = (d > 0) ? -1 : 1;
      delay(5);
    }
    cal.pendSign = sign;
    Serial.println(F("      Got it — release the pendulum."));
  }
  Serial.print(F("      pendSign = "));
  Serial.println(cal.pendSign);

  // jog back to where we started
  int32_t back = (int32_t)(fabs((float)(mtUpdate() - start)) / cal.countsPerStep);
  setDirX(false);
  for (int32_t i = 0; i < back; i++) {
    stepPulse(JOG_STEP_US);
    if ((i & 7) == 7) mtUpdate();
  }
  mtUpdate();
}

// Drive in one direction until the encoder stops following commanded steps
// (= pressed against a hard stop). Returns MT position at the wall.
int32_t driveUntilStall(bool positiveX) {
  setDirX(positiveX);
  const uint8_t WINDOW = 64;
  uint8_t bad = 0;
  int32_t windowStart = mtUpdate();
  for (int32_t s = 1; s <= HOMING_MAX_STEPS; s++) {
    stepPulse(HOME_STEP_US);
    if ((s & 7) == 0) mtUpdate();
    if (s % WINDOW == 0) {
      int32_t p = mtUpdate();
      int32_t moved = positiveX ? (p - windowStart) : (windowStart - p);
      windowStart = p;
      if (moved < (int32_t)(WINDOW * cal.countsPerStep * 0.3f)) {
        if (++bad >= 3) return p;
      } else {
        bad = 0;
      }
    }
  }
  fail(F("homing timed out before finding a hard stop"));
  return 0;
}

void moveSteps(bool positiveX, int32_t n, uint16_t periodUs) {
  setDirX(positiveX);
  for (int32_t i = 0; i < n; i++) {
    stepPulse(periodUs);
    if ((i & 7) == 7) mtUpdate();
  }
  mtUpdate();
}

void moveToCounts(int32_t target) {
  while (true) {
    int32_t err = target - mtUpdate();
    if ((float)abs(err) < 2.0f * cal.countsPerStep) return;
    int32_t n = (int32_t)(abs(err) / cal.countsPerStep);
    if (n > 64) n = 64;
    moveSteps(err > 0, n, HOME_STEP_US);
  }
}

// Phase 5: find both walls, compute travel + margin, park at center
void calTravel() {
  Serial.println(F("[5/5] Homing: driving to -x wall..."));
  int32_t negWall = driveUntilStall(false);
  moveSteps(true, 200, HOME_STEP_US);            // back off half a motor rev

  Serial.println(F("      Driving to +x wall..."));
  int32_t posWall = driveUntilStall(true);
  moveSteps(false, 200, HOME_STEP_US);

  cal.travelCounts = posWall - negWall;
  if (cal.travelCounts < MT_CPR)                 // less than one motor rev of travel?
    fail(F("measured travel is implausibly small"));

  int32_t marginA = cal.travelCounts / 12;       // ~8% each side
  int32_t marginB = (int32_t)(150.0f * cal.countsPerStep);
  cal.marginCounts = (marginA > marginB) ? marginA : marginB;

  Serial.println(F("      Parking at center..."));
  moveToCounts(negWall + cal.travelCounts / 2);
}

// ================= EEPROM =================
void saveCal() {
  cal.magic = CAL_MAGIC;
  cal.version = CAL_VERSION;
  cal.checksum = calcChecksum(cal);
  EEPROM.put(0, cal);
  CalData check;
  EEPROM.get(0, check);
  if (calcChecksum(check) != check.checksum || check.magic != CAL_MAGIC)
    fail(F("EEPROM verify failed"));
  haveCal = true;
}

bool loadCal() {
  EEPROM.get(0, cal);
  return cal.magic == CAL_MAGIC && cal.version == CAL_VERSION &&
         calcChecksum(cal) == cal.checksum;
}

void printCal() {
  Serial.println(F("---- stored calibration ----"));
  Serial.print(F("pendulum down raw:  ")); Serial.println(cal.thetaDownRaw);
  Serial.print(F("DIR=HIGH is +x:     ")); Serial.println(cal.dirHighIsPos ? F("yes") : F("no"));
  Serial.print(F("pendulum sign:      ")); Serial.println(cal.pendSign);
  Serial.print(F("counts per step:    ")); Serial.println(cal.countsPerStep);
  Serial.print(F("travel: "));
  Serial.print(cal.travelCounts);
  Serial.print(F(" counts = "));
  Serial.print((float)cal.travelCounts / MT_CPR, 2);
  Serial.print(F(" motor revs = "));
  Serial.print((int32_t)(cal.travelCounts / cal.countsPerStep));
  Serial.println(F(" steps"));
  Serial.print(F("soft-stop margin:   "));
  Serial.print(cal.marginCounts);
  Serial.print(F(" counts ("));
  Serial.print((int32_t)(cal.marginCounts / cal.countsPerStep));
  Serial.println(F(" steps) from each wall"));
  Serial.println(F("----------------------------"));
}

// ================= live monitor =================
// theta: 0 = upright, positive = leaning toward +x
void monitor() {
  Serial.println(F("Live monitor (reset board to exit):"));
  Serial.println(F("theta_deg (0=upright, +=lean toward +x) | cart counts from power-on 0"));
  uint16_t upRaw = (cal.thetaDownRaw + AS_HALF) % AS_CPR;
  while (true) {
    int16_t thetaCounts = cal.pendSign * wrap14((int32_t)asReadFiltered() - upRaw);
    Serial.print(F("theta: "));
    Serial.print(rawToDeg(thetaCounts), 2);
    Serial.print(F("  pos: "));
    Serial.print(mtUpdate());
    Serial.print(F("  badFrames: "));
    Serial.println(asBadFrames);
    delay(100);
  }
}

// ================= main =================
void runCalibration() {
  calPendulumDown();
  calScaleAndDir();
  calPendSign();
  calTravel();
  saveCal();
  Serial.println(F("\nDONE — calibration saved to EEPROM."));
  printCal();
  Serial.println(F("Sanity checks in the live monitor below:"));
  Serial.println(F(" - hanging still, theta should read ~ +/-180"));
  Serial.println(F(" - hold it upright: theta ~ 0"));
  Serial.println(F(" - lean it toward +x (the homing '+x wall'): theta > 0"));
  monitor();
}

void setup() {
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_PUL, OUTPUT);
  pinMode(PIN_AS_CS, OUTPUT);  digitalWrite(PIN_AS_CS, HIGH);
  pinMode(PIN_MT_CS, OUTPUT);  digitalWrite(PIN_MT_CS, HIGH);
  pinMode(PIN_MT_SCK, OUTPUT); digitalWrite(PIN_MT_SCK, HIGH); // mode 3 idle
  pinMode(PIN_MT_MOSI, OUTPUT);
  pinMode(PIN_MT_MISO, INPUT);
  SPI.begin();
  Serial.begin(115200);

  mtLastRaw = mtReadRaw();
  mtPos = 0;
  asReadRaw();                          // prime AS5048A read pipeline

  Serial.println(F("\n== Inverted pendulum calibration =="));
  asPrintDiagnostics();
  if (loadCal()) {
    haveCal = true;
    Serial.println(F("Existing calibration found:"));
    printCal();
  } else {
    Serial.println(F("No valid calibration in EEPROM."));
  }
  Serial.println(F("Before starting: cart roughly mid-travel, pendulum"));
  Serial.println(F("hanging COMPLETELY STILL. The cart will jog, then drive"));
  Serial.println(F("slowly into both ends of travel."));
  Serial.println(F("Send 'g' to calibrate, 'm' for live monitor."));
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == 'g') {
    runCalibration();
  } else if (c == 'm') {
    if (haveCal) monitor();
    else Serial.println(F("No calibration stored yet — send 'g' first."));
  }
}
