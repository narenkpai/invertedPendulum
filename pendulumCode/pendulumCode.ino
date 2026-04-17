#include <SPI.h>
#include <math.h>
#include <FastAccelStepper.h>

// ======================================================
// PINS
// ======================================================

static const uint8_t DIR_PIN  = 2;
static const uint8_t STEP_PIN = 9;

static const uint8_t ECHO_PIN = 6;
static const uint8_t TRIG_PIN = 7;

static const uint8_t  CS_PIN    = 10;
static const uint16_t ANGLE_REG = 0x3FFF;

// Optional TB6600 enable pin for true motor disable
static const bool USE_ENABLE_PIN = false;   // set true if ENA is wired
static const uint8_t ENABLE_PIN = 8;        // example pin
static const bool ENABLE_ACTIVE_LOW = false; // TB6600 usually enables when low on ENA-

// ======================================================
// STEPPER
// ======================================================

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// ======================================================
// SYSTEM SETTINGS
// ======================================================

static const int32_t ENCODER_MAX   = 16384;
static const int32_t UPRIGHT_COUNT = 3110;

// Same style as old code
static const int32_t ENGAGE_WINDOW_COUNTS  = 1400;
static const int32_t STOP_WINDOW_COUNTS    = 2600;
static const int32_t ANGLE_DEADBAND_COUNTS = 8;
static const float   CMD_DEADBAND_SPEED    = 80.0f;

// Ultrasonic allowed range
static const float CART_POS_MIN_CM    = 6.0f;
static const float CART_POS_MAX_CM    = 35.0f;
static const float CART_POS_CENTER_CM = 0.5f * (CART_POS_MIN_CM + CART_POS_MAX_CM);

// timing
static const uint32_t CONTROL_PERIOD_US    = 2000;
static const uint32_t ULTRASONIC_PERIOD_US = 120000;

// limits
static const float MAX_STEP_RATE = 50000.0f;
static const float MAX_ACCEL     = 100000.0f;

// gains
static float Kp = 0.5f;
static float Ki = 0.0f;
static float Kd = 0.0f;

static float Kx = 0.0f;
static float Kv = 0.0f;

static float OUTPUT_SCALE = 2.0f;

// integral clamp
static const float I_LIMIT = 800.0f;

// filtering
static const float DERIV_FILTER_ALPHA     = 0.25f;
static const float ULTRA_VEL_FILTER_ALPHA = 0.20f;

// keep ultrasonic implementation same as your working test
static const unsigned long TIMEOUT_US = 30000UL;
static const int SAMPLES = 5;

// motion threshold
static const float MIN_RUN_SPEED = 50.0f;

// debug
static const uint32_t PRINT_PERIOD_MS = 100;

// ======================================================
// STATE
// ======================================================

uint32_t lastControlUs = 0;
uint32_t lastUltraUs   = 0;
uint32_t lastPrintMs   = 0;

float filteredDerivative = 0.0f;
float prevError = 0.0f;
float integralTerm = 0.0f;

float cartPosCm = CART_POS_CENTER_CM;
float prevCartPosCm = CART_POS_CENTER_CM;
float cartVelCmPerSec = 0.0f;
bool cartPosValid = false;
bool cartPosInsideLimits = true;

float commandedStepRate = 0.0f;
bool controlEngaged = false;

// ======================================================
// AS5048A
// ======================================================

static inline uint8_t evenParity16(uint16_t x) {
  x ^= x >> 8;
  x ^= x >> 4;
  x ^= x >> 2;
  x ^= x >> 1;
  return x & 1;
}

static inline uint16_t makeReadCommand(uint16_t reg) {
  uint16_t cmd = 0x4000 | (reg & 0x3FFF);
  if (evenParity16(cmd)) cmd |= 0x8000;
  return cmd;
}

static inline uint16_t spiFrame(uint16_t data) {
  digitalWrite(CS_PIN, LOW);
  uint16_t resp = SPI.transfer16(data);
  digitalWrite(CS_PIN, HIGH);
  return resp;
}

static inline uint16_t readRawAngle() {
  spiFrame(makeReadCommand(ANGLE_REG));
  return spiFrame(0x0000) & 0x3FFF;
}

// ======================================================
// ULTRASONIC
// kept same as your working test
// ======================================================

float readDistanceCmOnce() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, TIMEOUT_US);

  if (duration == 0) {
    return -1.0f;
  }

  return duration * 0.01715f;
}

float readDistanceCmFiltered() {
  float sum = 0.0f;
  int good = 0;

  for (int i = 0; i < SAMPLES; i++) {
    float d = readDistanceCmOnce();
    if (d > 0.0f) {
      sum += d;
      good++;
    }
    delay(10);
  }

  if (good == 0) return -1.0f;
  return sum / good;
}

// ======================================================
// HELPERS
// ======================================================

int32_t wrappedErrorCounts(int32_t target, int32_t measured) {
  int32_t err = target - measured;
  while (err > 8192) err -= 16384;
  while (err < -8192) err += 16384;
  return err;
}

float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

void disableMotorNow() {
  if (!stepper) return;
  stepper->forceStop();
  if (USE_ENABLE_PIN) {
    stepper->disableOutputs();
  }
}

void enableMotorIfNeeded() {
  if (!stepper) return;
  if (USE_ENABLE_PIN) {
    stepper->enableOutputs();
  }
}

// ======================================================
// SETUP
// ======================================================

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  SPI.begin();
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));

  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);

  if (stepper == NULL) {
    Serial.println("stepper connect failed");
    while (1) {}
  }

  stepper->setDirectionPin(DIR_PIN);

  if (USE_ENABLE_PIN) {
    stepper->setEnablePin(ENABLE_PIN, ENABLE_ACTIVE_LOW);
    stepper->setAutoEnable(false);
  } else {
    stepper->setAutoEnable(false);
  }

  stepper->setAcceleration(MAX_ACCEL);

  float firstD = readDistanceCmFiltered();
  if (firstD > 0.0f) {
    cartPosCm = firstD;
    prevCartPosCm = cartPosCm;
    cartVelCmPerSec = 0.0f;
    cartPosValid = true;
    cartPosInsideLimits = (cartPosCm >= CART_POS_MIN_CM && cartPosCm <= CART_POS_MAX_CM);
  }

  disableMotorNow();

  lastControlUs = micros();
  lastUltraUs   = micros();
  lastPrintMs   = millis();

  Serial.println("FastAccelStepper pendulum control");
}

// ======================================================
// LOOP
// ======================================================

void loop() {
  uint32_t nowUs = micros();

  updateUltrasonic(nowUs);
  runControl(nowUs);
  applyMotor();
  runDebug();
}

// ======================================================
// ULTRASONIC UPDATE
// ======================================================

void updateUltrasonic(uint32_t nowUs) {
  if ((uint32_t)(nowUs - lastUltraUs) < ULTRASONIC_PERIOD_US) return;

  float dt = (nowUs - lastUltraUs) * 1e-6f;
  lastUltraUs = nowUs;

  float d = readDistanceCmFiltered();
  if (d < 0.0f) return;

  // keep actual value, do not clamp
  if (!cartPosValid) {
    cartPosCm = d;
    prevCartPosCm = d;
    cartVelCmPerSec = 0.0f;
    cartPosValid = true;
    cartPosInsideLimits = (cartPosCm >= CART_POS_MIN_CM && cartPosCm <= CART_POS_MAX_CM);
    return;
  }

  float rawVel = (d - prevCartPosCm) / dt;
  prevCartPosCm = d;

  cartPosCm = d;
  cartVelCmPerSec =
      (1.0f - ULTRA_VEL_FILTER_ALPHA) * cartVelCmPerSec +
      ULTRA_VEL_FILTER_ALPHA * rawVel;

  cartPosInsideLimits = (cartPosCm >= CART_POS_MIN_CM && cartPosCm <= CART_POS_MAX_CM);
}

// ======================================================
// CONTROL
// ======================================================

void runControl(uint32_t nowUs) {
  if ((uint32_t)(nowUs - lastControlUs) < CONTROL_PERIOD_US) return;

  float dt = (nowUs - lastControlUs) * 1e-6f;
  lastControlUs = nowUs;

  uint16_t raw = readRawAngle();
  float error = wrappedErrorCounts(UPRIGHT_COUNT, raw);
  int32_t absErr = abs((int32_t)error);

  // Only engage when close enough to upright
  if (!controlEngaged) {
    if (absErr < ENGAGE_WINDOW_COUNTS) {
      controlEngaged = true;
      integralTerm = 0.0f;
      prevError = error;
      filteredDerivative = 0.0f;
    } else {
      commandedStepRate = 0.0f;
      return;
    }
  }

  // Drop out if too far from upright
  if (absErr > STOP_WINDOW_COUNTS) {
    controlEngaged = false;
    commandedStepRate = 0.0f;
    integralTerm = 0.0f;
    prevError = error;
    filteredDerivative = 0.0f;
    return;
  }

  float derivative = (error - prevError) / dt;
  filteredDerivative =
      (1.0f - DERIV_FILTER_ALPHA) * filteredDerivative +
      DERIV_FILTER_ALPHA * derivative;
  prevError = error;

  integralTerm += error * dt;
  integralTerm = clampf(integralTerm, -I_LIMIT, I_LIMIT);

  float cartErr = 0.0f;
  float cartVel = 0.0f;

  if (cartPosValid) {
    cartErr = cartPosCm - CART_POS_CENTER_CM;
    cartVel = cartVelCmPerSec;
  }

  float u =
      (Kp * error) +
      (Ki * integralTerm) +
      (Kd * filteredDerivative) +
      (Kx * cartErr) -
      (Kv * cartVel);

  float rateCmd = clampf(OUTPUT_SCALE * u, -MAX_STEP_RATE, MAX_STEP_RATE);

  // Small deadband near upright
  if (absErr < ANGLE_DEADBAND_COUNTS && fabs(rateCmd) < CMD_DEADBAND_SPEED) {
    rateCmd = 0.0f;
  }

  // If ultrasonic says cart is outside allowed range,
  // do not command the motor at all
  if (cartPosValid && !cartPosInsideLimits) {
    rateCmd = 0.0f;
  }

  commandedStepRate = rateCmd;
}

// ======================================================
// APPLY MOTOR COMMAND
// ======================================================

void applyMotor() {
  if (!stepper) return;

  float speed = commandedStepRate;

  // fully stop and disable when not active
  if (!controlEngaged || fabs(speed) < MIN_RUN_SPEED) {
    disableMotorNow();
    return;
  }

  // also stop and disable if ultrasonic says outside allowed range
  if (cartPosValid && !cartPosInsideLimits) {
    disableMotorNow();
    return;
  }

  enableMotorIfNeeded();

  stepper->setAcceleration(MAX_ACCEL);
  stepper->setSpeedInHz((uint32_t)fabs(speed));

  if (speed > 0.0f) {
    stepper->runForward();
  } else {
    stepper->runBackward();
  }
}

// ======================================================
// DEBUG
// ======================================================

void runDebug() {
  if (millis() - lastPrintMs < PRINT_PERIOD_MS) return;
  lastPrintMs = millis();

  uint16_t raw = readRawAngle();
  int32_t err = wrappedErrorCounts(UPRIGHT_COUNT, raw);

  Serial.print("err=");
  Serial.print(err);
  Serial.print(" engaged=");
  Serial.print(controlEngaged ? 1 : 0);
  Serial.print(" posCm=");
  Serial.print(cartPosCm, 2);
  Serial.print(" inRange=");
  Serial.print(cartPosInsideLimits ? 1 : 0);
  Serial.print(" velCm=");
  Serial.print(cartVelCmPerSec, 2);
  Serial.print(" cmd=");
  Serial.print(commandedStepRate, 1);
  Serial.print(" valid=");
  Serial.println(cartPosValid ? 1 : 0);
}