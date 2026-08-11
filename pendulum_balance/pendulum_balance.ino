/*
 * pendulum_balance.ino — LQR balancing controller for the inverted
 * pendulum gantry. Requires calibration stored in EEPROM by
 * pendulum_calibration.ino (run that once first).
 *
 * !!! SET THIS BEFORE FIRST RUN: PENDULUM_LEFF below — effective pendulum
 * length in meters. Point mass on a light rod: pivot-to-mass distance.
 * Uniform rod of total length L: (2/3)*L.
 *
 * Wiring:
 *   TB6600:  PUL+ -> D9 (Timer1 OC1A), DIR+ -> D2, PUL-/DIR- -> GND
 *   AS5048A: CS -> D10, hardware SPI D11/D12/D13   (pendulum pivot)
 *   MT6835:  CSN -> A0, SCK -> A1, MOSI -> A2, MISO -> A3  (motor shaft)
 *
 * Operation (Serial Monitor @ 115200):
 *   1. Boot -> sanity info. Send 'g' to home (cart drives gently to the
 *      -x wall, then parks at center; x = 0 is center from then on).
 *   2. IDLE: motor off. Hold the pendulum upright and still (within ~2 deg
 *      for 0.5 s) -> controller engages by itself. Let go.
 *   3. BALANCE: LQR holds it upright and centered. Disengages back to IDLE
 *      if the angle exceeds ~20 deg, the cart reaches a soft stop, a motor
 *      stall is detected, or any serial character is received (e-stop).
 *
 * Architecture:
 *   - Timer1 generates step pulses in hardware (OC1A toggle, CTC mode);
 *     an ISR counts steps, so commanded cart position is exact.
 *   - x, x_dot come from commanded steps/velocity (exact for a stepper),
 *     theta, theta_dot from the AS5048A (parity-validated reads).
 *   - MT6835 is a stall watchdog: commanded steps vs measured motor
 *     rotation diverging by > 1 motor rev -> disengage.
 *   - Control law u (cart accel) = K1*x + K2*v + K3*theta + K4*theta_dot,
 *     gains from LQR (tools/compute_lqr_gains.py), interpolated by length.
 */

#include <SPI.h>
#include <EEPROM.h>

// ================= USER PARAMETERS =================
#define PENDULUM_LEFF 0.20f   // meters — SET ME (see header comment)

const float M_PER_REV  = 0.040f;  // GT2 belt, 20T pulley, 2 mm pitch
const float V_MAX      = 0.45f;   // m/s. Firmware caps at 0.6 (12 kHz steps);
                                  // motor torque dies near ~700 RPM = 0.45 anyway
const float A_MAX      = 10.0f;   // m/s^2. Raise ONLY while no stall reports —
                                  // a stalled motor gives ZERO correction
const float DT         = 0.005f;  // control period (s) -> 200 Hz

// ---- controller: cascade PID (intuitive to hand-tune) ----
// Inner loop: angle error -> cart accel. Outer loop: cart offset -> small
// lean request, which is what brings the cart back to center. Structurally
// this is the same law as the LQR below (u = Kp_th*th + Kd_th*thd
// + Kp_th*Kp_x*x + Kp_th*Kd_x*v); these defaults match the LQR gains.
// Set USE_PID 0 to switch back to the LQR gain table.
#define USE_PID 1
// TUNING MATH — inner loop response: omega = sqrt((KP_TH - 9.8)/l_eff),
// damping ratio zeta = KD_TH / (2 * l_eff * omega). Aim for zeta = 0.7..1.2.
// KP_TH=70, l=0.2 -> omega=17, critical KD_TH ~ 7. KD_TH far above that makes
// the response SLUGGISH (overdamped) — "too slow to catch the fall".
float KP_TH = 80.0f;   // too low (<~40): flops over. too high: fast shaking
float KD_TH = 9.0f;    // keep near 2*l_eff*sqrt((KP_TH-9.8)/l_eff)
float KP_X  = 0.44f;   // rad of lean per m of offset. drifts to a wall: raise
float KD_X  = 0.20f;   // rad per m/s. cart sloshes side to side: raise
const float TH_REF_MAX = 0.08f;  // rad (~4.6 deg) cap on the outer-loop lean request

const float ENGAGE_TH  = 0.06f;   // rad (~3.4 deg) upright window to engage
const float ENGAGE_THD = 0.8f;    // rad/s stillness to engage
const float ABORT_TH   = 0.45f;   // rad (~20 deg) give up beyond this

const float TRIM_STEP  = 0.00436f; // rad (0.25 deg) per +/- keypress
const float TRIM_MAX   = 0.30f;    // rad (~17 deg) trim authority
const float TRIM_ADAPT = 0.02f;    // auto-trim: rad of trim per meter of drift

// LQR gain table from tools/compute_lqr_gains.py (Q=diag(60,15,400,40), R=1)
const float GAIN_LEN[] = {0.10f, 0.15f, 0.20f, 0.30f, 0.40f};
const float GAIN_K[][4] = {
  {7.75f, 9.10f, 52.79f, 8.02f},
  {7.75f, 9.23f, 54.31f, 8.89f},
  {7.75f, 9.36f, 55.82f, 9.77f},
  {7.75f, 9.61f, 58.83f, 11.56f},
  {7.75f, 9.85f, 61.80f, 13.39f}
};
const uint8_t GAIN_N = sizeof(GAIN_LEN) / sizeof(GAIN_LEN[0]);

// ================= pins =================
const uint8_t PIN_DIR   = 2;
const uint8_t PIN_PUL   = 9;    // must stay on D9: Timer1 OC1A
const uint8_t PIN_AS_CS = 10;

const uint8_t PIN_MT_CS   = A0;
const uint8_t PIN_MT_SCK  = A1;
const uint8_t PIN_MT_MOSI = A2;
const uint8_t PIN_MT_MISO = A3;

// ================= constants =================
const int32_t  MT_CPR  = 2097152L;
const uint16_t AS_CPR  = 16384;
const uint16_t AS_HALF = 8192;
const float    RAD_PER_AS = 6.2831853f / AS_CPR;
const uint16_t HOME_STEP_US = 1500;
const int32_t  HOMING_MAX_STEPS = 200000L;

// ================= calibration (must match pendulum_calibration.ino) ====
struct CalData {
  uint32_t magic;
  uint16_t version;
  uint16_t thetaDownRaw;
  int8_t   dirHighIsPos;
  int8_t   pendSign;
  float    countsPerStep;
  int32_t  travelCounts;
  int32_t  marginCounts;
  uint16_t checksum;
};
const uint32_t CAL_MAGIC   = 0x50454E44UL;
const uint16_t CAL_VERSION = 1;
CalData cal;

// Upright trim: correction added on top of the hanging+180deg reference.
// Needed because magnet eccentricity bends the angle scale, so the point
// opposite "down" is not exactly true vertical. Stored separately in EEPROM
// so the calibration sketch's data block is untouched.
// (Declared up here: the Arduino IDE inserts auto-generated function
// prototypes before the first function definition, so any type used in a
// function signature must already exist by then.)
struct TrimData { uint32_t magic; float trimRad; uint16_t checksum; };
const uint32_t TRIM_MAGIC = 0x5452494DUL;  // 'TRIM'
const int      TRIM_ADDR  = 64;            // past CalData at addr 0
float trimRad = 0.0f;
float lastTh  = 0.0f;                      // last trimmed angle (for 't' capture)

uint16_t calcChecksum(const CalData &c) {
  const uint8_t *p = (const uint8_t *)&c;
  uint16_t sum = 0;
  for (size_t i = 0; i < sizeof(CalData) - sizeof(uint16_t); i++) sum += p[i];
  return sum;
}

uint16_t trimChecksum(const TrimData &t) {
  const uint8_t *p = (const uint8_t *)&t;
  uint16_t sum = 0;
  for (size_t i = 0; i < sizeof(TrimData) - sizeof(uint16_t); i++) sum += p[i];
  return sum;
}

void printTrim() {
  Serial.print(F("trim = "));
  Serial.print(trimRad * 57.2958f, 2);
  Serial.println(F(" deg (send 's' to save)"));
}

void saveTrim() {
  TrimData t = {TRIM_MAGIC, trimRad, 0};
  t.checksum = trimChecksum(t);
  EEPROM.put(TRIM_ADDR, t);
  Serial.println(F("trim saved to EEPROM"));
}

void loadTrim() {
  TrimData t;
  EEPROM.get(TRIM_ADDR, t);
  if (t.magic == TRIM_MAGIC && trimChecksum(t) == t.checksum) trimRad = t.trimRad;
}

// derived after loading cal / homing
uint16_t upRaw;            // AS5048A raw at upright
float    mPerStep;         // meters of cart travel per step
float    xLimit;           // soft stop, meters from center (symmetric)
int32_t  mtCenter;         // MT cumulative position at center
float    K1, K2, K3, K4;   // interpolated LQR gains

// ================= angle helpers =================
int16_t wrap14(int32_t d) {
  d %= AS_CPR;
  if (d < -AS_HALF) d += AS_CPR;
  if (d >= AS_HALF) d -= AS_CPR;
  return (int16_t)d;
}

// ================= AS5048A (pendulum) =================
uint8_t evenParity16(uint16_t x) {
  x ^= x >> 8; x ^= x >> 4; x ^= x >> 2; x ^= x >> 1;
  return x & 1;
}

uint16_t asFrame(uint16_t data) {
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

uint16_t asReadReg(uint16_t reg) {
  uint16_t resp = 0;
  for (uint8_t attempt = 0; attempt < 4; attempt++) {
    asFrame(makeReadCommand(reg));
    resp = asFrame(makeReadCommand(0x0000));
    if (evenParity16(resp) == 0 && !(resp & 0x4000)) return resp & 0x3FFF;
    if (resp & 0x4000) { asFrame(makeReadCommand(0x0001)); asFrame(makeReadCommand(0x0000)); }
  }
  return resp & 0x3FFF;
}

// pendulum angle in radians: 0 = upright, positive = leaning toward +x
// median of 3 validated reads keeps single-sample noise out of the derivative
float readTheta() {
  int16_t a = wrap14((int32_t)asReadReg(0x3FFF) - upRaw);
  int16_t b = wrap14((int32_t)asReadReg(0x3FFF) - upRaw);
  int16_t c = wrap14((int32_t)asReadReg(0x3FFF) - upRaw);
  int16_t m = (a > b) ? ((b > c) ? b : (a > c ? c : a))
                      : ((a > c) ? a : (b > c ? c : b));
  return cal.pendSign * m * RAD_PER_AS;
}

// ================= MT6835 (motor watchdog) =================
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

uint32_t mtReadRaw() {
  digitalWrite(PIN_MT_CS, LOW);
  mtTransfer(0xA0);
  mtTransfer(0x03);
  uint8_t b2 = mtTransfer(0);
  uint8_t b3 = mtTransfer(0);
  uint8_t b4 = mtTransfer(0);
  mtTransfer(0);
  digitalWrite(PIN_MT_CS, HIGH);
  return ((uint32_t)b2 << 13) | ((uint32_t)b3 << 5) | (b4 >> 3);
}

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

// ================= stepper: Timer1 hardware step generation =============
volatile int32_t stepCount = 0;   // commanded steps, + = +x, 0 = center
volatile int8_t  stepDir   = 1;
volatile uint8_t pulPhase  = 0;
bool stepRunning = false;

ISR(TIMER1_COMPA_vect) {
  pulPhase ^= 1;
  if (pulPhase) stepCount += stepDir;   // rising edge just went out
}

void setDirX(bool positiveX) {
  digitalWrite(PIN_DIR, (positiveX == (cal.dirHighIsPos == 1)) ? HIGH : LOW);
  delayMicroseconds(20);
}

void stepperStop() {
  TIMSK1 = 0;
  TCCR1A = 0;                 // disconnect OC1A; pin follows PORTB (low)
  TCCR1B = 0;
  PORTB &= ~_BV(PB1);
  pulPhase = 0;
  stepRunning = false;
}

void stepperSetRate(float stepsPerSec, bool positiveX) {
  if (stepsPerSec < 20.0f) { stepperStop(); return; }
  if (stepsPerSec > 12000.0f) stepsPerSec = 12000.0f;

  int8_t wantDir = positiveX ? 1 : -1;
  if (!stepRunning || wantDir != stepDir) {
    stepperStop();
    setDirX(positiveX);
    stepDir = wantDir;
    // force the OC1A internal latch low so the first toggle is a rising edge
    TCCR1A = _BV(COM1A1);     // "clear on match" mode
    TCCR1C = _BV(FOC1A);
  }
  uint16_t top = (uint16_t)(1000000.0f / stepsPerSec) - 1;  // /8 prescaler
  noInterrupts();
  OCR1A = top;
  if (TCNT1 > top) TCNT1 = 0;
  TCCR1A = _BV(COM1A0);                    // toggle OC1A on match
  TCCR1B = _BV(WGM12) | _BV(CS11);         // CTC, clk/8 (2 MHz)
  TIMSK1 = _BV(OCIE1A);
  interrupts();
  stepRunning = true;
}

int32_t stepCountAtomic() {
  noInterrupts();
  int32_t c = stepCount;
  interrupts();
  return c;
}

// ================= blocking moves (homing only, timer off) ==============
void stepPulse(uint16_t periodUs) {
  digitalWrite(PIN_PUL, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_PUL, LOW);
  delayMicroseconds(periodUs - 10);
}

void fail(const __FlashStringHelper *msg) {
  stepperStop();
  Serial.print(F("\nFATAL: "));
  Serial.println(msg);
  while (1) {}
}

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
  fail(F("homing timed out"));
  return 0;
}

void moveToCounts(int32_t target) {
  while (true) {
    int32_t err = target - mtUpdate();
    if ((float)labs(err) < 2.0f * cal.countsPerStep) return;
    setDirX(err > 0);
    int32_t n = (int32_t)(labs(err) / cal.countsPerStep);
    if (n > 64) n = 64;
    if (n < 1) n = 1;
    for (int32_t i = 0; i < n; i++) {
      stepPulse(HOME_STEP_US);
      if ((i & 7) == 7) mtUpdate();
    }
  }
}

void home() {
  Serial.println(F("Homing: driving to -x wall..."));
  int32_t wall = driveUntilStall(false);
  setDirX(true);
  for (int16_t i = 0; i < 200; i++) { stepPulse(HOME_STEP_US); if ((i & 7) == 7) mtUpdate(); }
  Serial.println(F("Moving to center..."));
  moveToCounts(wall + cal.travelCounts / 2);
  mtCenter = mtUpdate();
  noInterrupts(); stepCount = 0; interrupts();
  Serial.println(F("Homed. x = 0 at center."));
}

// ================= controller =================
enum RunState { UNHOMED, IDLE, BALANCE };
RunState runState = UNHOMED;
bool paused = false;       // 'k': motor off + auto-engage inhibited, 'l': resume

float vCmd = 0.0f;         // commanded cart velocity (m/s)
float thPrev = 0.0f;
float thdFilt = 0.0f;
uint16_t engageTicks = 0;
uint32_t nextTickUs = 0;
uint16_t tickNum = 0;

void interpolateGains() {
  float l = PENDULUM_LEFF;
  if (l <= GAIN_LEN[0]) l = GAIN_LEN[0];
  if (l >= GAIN_LEN[GAIN_N - 1]) l = GAIN_LEN[GAIN_N - 1];
  uint8_t i = 0;
  while (i < GAIN_N - 2 && GAIN_LEN[i + 1] < l) i++;
  float t = (l - GAIN_LEN[i]) / (GAIN_LEN[i + 1] - GAIN_LEN[i]);
  K1 = GAIN_K[i][0] + t * (GAIN_K[i + 1][0] - GAIN_K[i][0]);
  K2 = GAIN_K[i][1] + t * (GAIN_K[i + 1][1] - GAIN_K[i][1]);
  K3 = GAIN_K[i][2] + t * (GAIN_K[i + 1][2] - GAIN_K[i][2]);
  K4 = GAIN_K[i][3] + t * (GAIN_K[i + 1][3] - GAIN_K[i][3]);
}

void disengage(const __FlashStringHelper *why) {
  stepperStop();
  // Re-sync commanded position from the motor encoder: if steps were lost
  // in a stall, stepCount is wrong and the soft stops would be too.
  int32_t truePos = (int32_t)((mtUpdate() - mtCenter) / cal.countsPerStep);
  noInterrupts();
  stepCount = truePos;
  interrupts();
  vCmd = 0.0f;
  engageTicks = 0;
  runState = IDLE;
  Serial.print(F("DISENGAGED: "));
  Serial.println(why);
  Serial.println(F("Hold pendulum upright & still to re-engage."));
}

void controlTick() {
  float thMeas = readTheta();
  float dth = thMeas - thPrev;
  // shortest path across the +/-180deg wrap: swinging through the bottom is
  // a small motion, not a 360deg jump
  if (dth >  3.14159f) dth -= 6.2831853f;
  if (dth < -3.14159f) dth += 6.2831853f;
  // Reject physically impossible jumps (corrupted SPI frame): >0.3 rad in
  // 5 ms would be 60 rad/s. Escape hatch after 3 consecutive rejects so a
  // genuine step change can never latch the angle forever.
  static uint8_t rejects = 0;
  if (fabs(dth) > 0.3f && rejects < 3) {
    rejects++;
    thMeas = thPrev;
    dth = 0.0f;
  } else {
    rejects = 0;
  }
  float thdRaw = dth / DT;                 // derivative on raw measurement,
  thPrev = thMeas;                         // immune to trim adjustments
  thdFilt += 0.25f * (thdRaw - thdFilt);  // filter lag directly erodes KD_TH's
                                          // stabilizing phase — keep alpha >= 0.2
  float th = thMeas - trimRad;
  lastTh = th;

  float x = stepCountAtomic() * mPerStep;
  mtUpdate();                       // keep multi-turn tracking alive

  if (runState == IDLE) {
    bool thOk  = fabs(th) < ENGAGE_TH;
    bool thdOk = fabs(thdFilt) < ENGAGE_THD;
    // fabs(thMeas) check: never engage unless the pendulum is genuinely near
    // the up reference — a bad trim value must not fake "upright" while it hangs
    bool upright = thOk && thdOk && fabs(x) < xLimit && fabs(thMeas) < ABORT_TH &&
                   !paused;
    engageTicks = upright ? engageTicks + 1 : 0;
    if ((tickNum & 0x7F) == 0) {          // show why it isn't engaging
      Serial.print(paused ? F("PAUSED  th ") : F("idle  th "));
      Serial.print(th * 57.2958f, 1);
      Serial.print(thOk ? F(" ok") : F(" NEED <3.4"));
      Serial.print(F("  raw ")); Serial.print(thMeas * 57.2958f, 1);
      if (fabs(thMeas) >= ABORT_TH)
        Serial.print(F(" NOT UP (trim wrong? press r)"));
      Serial.print(F("  thd ")); Serial.print(thdFilt, 2);
      Serial.print(thdOk ? F(" ok") : F(" NEED <0.8"));
      Serial.print(F("  hold ")); Serial.print(engageTicks * DT, 2);
      Serial.println(F("/0.30s"));
    }
    if (engageTicks >= (uint16_t)(0.3f / DT)) {
      vCmd = 0.0f;
      thdFilt = 0.0f;
      runState = BALANCE;
      Serial.println(F("ENGAGED — let go gently."));
    }
    return;
  }

  // ---- BALANCE ----
  if (fabs(th) > ABORT_TH)          { disengage(F("pendulum fell over"));  return; }
  if (fabs(x)  > xLimit + 0.010f)   { disengage(F("ran past soft stop"));  return; }

  // stall watchdog: commanded steps vs measured motor rotation
  if ((tickNum & 0x3F) == 0) {
    int32_t predicted = mtCenter + (int32_t)(stepCountAtomic() * cal.countsPerStep);
    if (labs(mtPos - predicted) > MT_CPR) {
      Serial.print(F("step/encoder mismatch: "));
      Serial.print((mtPos - predicted) / (float)MT_CPR, 2);
      Serial.println(F(" motor revs"));
      disengage(F("motor stall detected"));
      return;
    }
  }

#if USE_PID
  float thRef = -(KP_X * x + KD_X * vCmd);        // lean request to re-center
  if (thRef >  TH_REF_MAX) thRef =  TH_REF_MAX;
  if (thRef < -TH_REF_MAX) thRef = -TH_REF_MAX;
  float u = KP_TH * (th - thRef) + KD_TH * thdFilt;        // cart accel
#else
  float u = K1 * x + K2 * vCmd + K3 * th + K4 * thdFilt;   // cart accel
#endif
  if (u >  A_MAX) u =  A_MAX;
  if (u < -A_MAX) u = -A_MAX;

  vCmd += u * DT;
  if (vCmd >  V_MAX) vCmd =  V_MAX;
  if (vCmd < -V_MAX) vCmd = -V_MAX;

  // Auto-trim: persistent drift velocity means the upright reference is off
  // (the cart must keep chasing a mis-calibrated lean). Slowly shift the
  // reference until steady-state drift disappears.
  trimRad -= TRIM_ADAPT * vCmd * DT;
  if (trimRad >  TRIM_MAX) trimRad =  TRIM_MAX;
  if (trimRad < -TRIM_MAX) trimRad = -TRIM_MAX;
  if (x >=  xLimit && vCmd > 0) vCmd = 0;   // soft stops
  if (x <= -xLimit && vCmd < 0) vCmd = 0;

  stepperSetRate(fabs(vCmd) / mPerStep, vCmd > 0);

  if ((tickNum & 0x7F) == 0) {              // status ~1.6 Hz
    Serial.print(F("th "));  Serial.print(th * 57.2958f, 1);
    Serial.print(F("  x ")); Serial.print(x * 1000.0f, 0);
    Serial.print(F("mm  v ")); Serial.print(vCmd, 2);
    Serial.print(F("  trim ")); Serial.println(trimRad * 57.2958f, 2);
  }
}

// ================= setup / loop =================
void setup() {
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_PUL, OUTPUT);   digitalWrite(PIN_PUL, LOW);
  pinMode(PIN_AS_CS, OUTPUT); digitalWrite(PIN_AS_CS, HIGH);
  pinMode(PIN_MT_CS, OUTPUT); digitalWrite(PIN_MT_CS, HIGH);
  pinMode(PIN_MT_SCK, OUTPUT); digitalWrite(PIN_MT_SCK, HIGH);
  pinMode(PIN_MT_MOSI, OUTPUT);
  pinMode(PIN_MT_MISO, INPUT);
  SPI.begin();
  Serial.begin(115200);

  EEPROM.get(0, cal);
  if (cal.magic != CAL_MAGIC || cal.version != CAL_VERSION ||
      calcChecksum(cal) != cal.checksum)
    fail(F("no valid calibration — run pendulum_calibration first"));

  upRaw = (cal.thetaDownRaw + AS_HALF) % AS_CPR;
  long stepsPerRev = lround((float)MT_CPR / cal.countsPerStep);
  mPerStep = M_PER_REV / stepsPerRev;
  xLimit = (cal.travelCounts / 2.0f - cal.marginCounts) / MT_CPR * M_PER_REV;
  interpolateGains();

  loadTrim();
  mtLastRaw = mtReadRaw();
  mtPos = 0;
  asReadReg(0x3FFF);

  Serial.println(F("\n== Inverted pendulum LQR balance =="));
  Serial.print(F("steps/rev: "));   Serial.print(stepsPerRev);
  Serial.print(F("  soft stop: +/-")); Serial.print(xLimit * 1000.0f, 0);
  Serial.print(F("mm  l_eff: "));   Serial.print(PENDULUM_LEFF, 3);
  Serial.println(F("m"));
#if USE_PID
  Serial.print(F("PID gains: KP_TH ")); Serial.print(KP_TH, 1);
  Serial.print(F("  KD_TH ")); Serial.print(KD_TH, 1);
  Serial.print(F("  KP_X ")); Serial.print(KP_X, 2);
  Serial.print(F("  KD_X ")); Serial.println(KD_X, 2);
#else
  Serial.print(F("gains K = ["));
  Serial.print(K1, 2); Serial.print(F(", ")); Serial.print(K2, 2);
  Serial.print(F(", ")); Serial.print(K3, 2); Serial.print(F(", "));
  Serial.print(K4, 2); Serial.println(F("]"));
#endif
  printTrim();
  Serial.println(F("Keys: g=home/start  t=capture upright (IDLE, pendulum held"));
  Serial.println(F("POINTING UP)  +/-=nudge trim 0.25deg  r=reset trim  s=save"));
  Serial.println(F("trim  c=center cart (IDLE)  k=pause motor  l=unpause"));
  Serial.println(F("other=e-stop"));
  Serial.println(F("Send 'g' to home and start."));
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '+' || c == '-') {
      trimRad += (c == '+') ? TRIM_STEP : -TRIM_STEP;
      if (trimRad >  TRIM_MAX) trimRad =  TRIM_MAX;
      if (trimRad < -TRIM_MAX) trimRad = -TRIM_MAX;
      printTrim();
    } else if (c == 's') {
      saveTrim();
    } else if (c == 't' && runState == IDLE) {
      float cand = trimRad + lastTh;     // equals the raw measured angle
      if (fabs(cand) > TRIM_MAX) {
        Serial.print(F("capture REJECTED: raw angle "));
        Serial.print(cand * 57.2958f, 1);
        Serial.println(F(" deg from up reference."));
        Serial.println(F("The pendulum must point UP (not hang down) for 't'."));
      } else {
        trimRad = cand;
        Serial.print(F("upright captured: "));
        printTrim();
      }
    } else if (c == 'r') {
      trimRad = 0.0f;
      Serial.print(F("trim reset. "));
      printTrim();
    } else if (c == 'k') {
      if (runState == BALANCE) disengage(F("paused with 'k'"));
      paused = true;
      Serial.println(F("PAUSED — motor off, auto-engage inhibited. 'l' resumes."));
    } else if (c == 'l') {
      paused = false;
      engageTicks = 0;
      Serial.println(F("resumed — auto-engage active."));
    } else if (c == 'c' && runState == IDLE) {
      Serial.println(F("Centering cart..."));
      moveToCounts(mtCenter);
      noInterrupts(); stepCount = 0; interrupts();
      thPrev = readTheta();          // the move jostles the pendulum
      thdFilt = 0.0f;
      nextTickUs = micros();         // don't "catch up" ticks missed while moving
      Serial.println(F("Centered (x = 0)."));
    } else if (runState == BALANCE) {
      disengage(F("serial e-stop"));
      return;
    } else if (runState == UNHOMED && c == 'g') {
      home();
      thPrev = readTheta();
      runState = IDLE;
      nextTickUs = micros();
      Serial.println(F("IDLE — hold pendulum upright & still to engage."));
    }
  }
  if (runState == UNHOMED) return;

  if ((int32_t)(micros() - nextTickUs) >= 0) {
    nextTickUs += (uint32_t)(DT * 1e6f);
    controlTick();
    tickNum++;
  }
}
