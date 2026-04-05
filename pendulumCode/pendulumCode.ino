#include <SPI.h>
#include <math.h>

// ======================================================
// USER SETTINGS
// ======================================================

// TB6600 pins
static const uint8_t DIR_PIN  = 2;
static const uint8_t STEP_PIN = 3;

// AS5048A SPI
static const uint8_t  CS_PIN    = 10;
static const uint16_t ANGLE_REG = 0x3FFF;

// Encoder setup
static const int32_t ENCODER_MAX   = 16384;
static const int32_t UPRIGHT_COUNT = 3000;   // your upright reference

// Control timing
static const uint32_t CONTROL_PERIOD_US = 2000;   // 2 ms = 500 Hz

// Safety
static const int32_t ENGAGE_WINDOW_COUNTS = 1400;
static const int32_t STOP_WINDOW_COUNTS   = 2600;
static const uint32_t STEP_PULSE_US       = 8;

// Stepper speed limits
static const float MAX_STEP_RATE = 5000.0f;
static const float MIN_STEP_RATE = 200.0f;

// PID gains for pendulum angle
static float Kp = 20.0f;
static float Ki = 0.5f;
static float Kd = 0.05f;

// Cart state feedback gains
static float Kx = 0.010f;   // cart position gain, in steps
static float Kv = 0.002f;   // cart velocity gain, in steps/sec

// Output scaling
static float OUTPUT_SCALE = 10.0f;

// Integral clamp
static const float I_LIMIT = 800.0f;

// Direction invert
static const bool INVERT_DIR = false;

// Cart travel limits in estimated step counts
static const long CART_POS_MIN_STEPS = -2000;
static const long CART_POS_MAX_STEPS =  2000;

// Optional margin for stronger centering as cart nears end
static const long CART_LIMIT_MARGIN_STEPS = 250;
static const float EDGE_PUSH_GAIN = 0.050f;

// Filtering
// 0.0 = no update, 1.0 = no filtering
static const float ANGLE_FILTER_ALPHA    = 0.25f;
static const float DERIV_FILTER_ALPHA    = 0.20f;
static const float CART_VEL_FILTER_ALPHA = 0.20f;

// ======================================================
// AS5048A helpers
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
  if (evenParity16(cmd)) {
    cmd |= 0x8000;
  }
  return cmd;
}

static inline uint16_t spiFrame(uint16_t data) {
  digitalWrite(CS_PIN, LOW);
  uint16_t resp = SPI.transfer16(data);
  digitalWrite(CS_PIN, HIGH);
  return resp;
}

static inline uint16_t readAS5048A(uint16_t reg) {
  spiFrame(makeReadCommand(reg));
  return spiFrame(0x0000);
}

static inline uint16_t readRawAngle() {
  return readAS5048A(ANGLE_REG) & 0x3FFF;
}

// ======================================================
// Control helpers
// ======================================================

static inline int32_t wrappedErrorCounts(int32_t target, int32_t measured) {
  int32_t err = target - measured;
  while (err >  (ENCODER_MAX / 2)) err -= ENCODER_MAX;
  while (err < -(ENCODER_MAX / 2)) err += ENCODER_MAX;
  return err;
}

static inline float wrapCountsFloat(float x) {
  while (x >  (ENCODER_MAX / 2)) x -= ENCODER_MAX;
  while (x < -(ENCODER_MAX / 2)) x += ENCODER_MAX;
  return x;
}

static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline int8_t motionSignFromRate(float u) {
  bool dir = (u >= 0.0f);
  if (INVERT_DIR) dir = !dir;
  return dir ? 1 : -1;
}

static inline void setDirectionFromSign(float u) {
  bool dir = (u >= 0.0f);
  if (INVERT_DIR) dir = !dir;
  digitalWrite(DIR_PIN, dir ? HIGH : LOW);
}

static inline float lowPassWrappedCounts(float prevFiltered, float newRaw, float alpha) {
  float diff = wrapCountsFloat(newRaw - prevFiltered);
  return prevFiltered + alpha * diff;
}

// ======================================================
// Step generation state
// ======================================================

volatile float commandedStepRate = 0.0f;
uint32_t lastStepToggleUs = 0;
bool stepPinState = LOW;
float currentAbsRate = 0.0f;
uint32_t halfPeriodUs = 0;

// ======================================================
// Cart position estimate from step count
// ======================================================

volatile long cartStepCount = 0;
float cartPosSteps = 0.0f;
float prevCartPosSteps = 0.0f;
float cartVelStepsPerSec = 0.0f;

// ======================================================
// PID / filter state
// ======================================================

uint32_t lastControlUs = 0;
float integralTerm = 0.0f;
float prevError = 0.0f;
float filteredAngle = 0.0f;
float filteredDerivative = 0.0f;
bool controlEngaged = false;
bool filterInitialized = false;

// ======================================================
// Debug timing
// ======================================================

uint32_t lastPrintMs = 0;

// ======================================================
// Setup
// ======================================================

void setup() {
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(STEP_PIN, LOW);

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  SPI.begin();
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));

  Serial.begin(115200);

  uint16_t firstRaw = readRawAngle();
  filteredAngle = (float)firstRaw;
  filterInitialized = true;

  lastControlUs = micros();
  lastStepToggleUs = micros();

  Serial.println("Pendulum balance with cart state feedback");
  Serial.println("Keep pendulum near upright before control engages");
}

// ======================================================
// Main loop
// ======================================================

void loop() {
  uint32_t nowUs = micros();

  runControl(nowUs);
  runStepper(nowUs);
  runDebug();
}

// ======================================================
// Control loop
// ======================================================

void runControl(uint32_t nowUs) {
  if ((uint32_t)(nowUs - lastControlUs) < CONTROL_PERIOD_US) return;

  float dt = (nowUs - lastControlUs) * 1e-6f;
  lastControlUs = nowUs;

  // ----------------------------
  // Pendulum angle read + filter
  // ----------------------------
  uint16_t raw = readRawAngle();
  float rawAngle = (float)raw;

  if (!filterInitialized) {
    filteredAngle = rawAngle;
    filterInitialized = true;
  } else {
    filteredAngle = lowPassWrappedCounts(filteredAngle, rawAngle, ANGLE_FILTER_ALPHA);
  }

  float error = (float)wrappedErrorCounts(UPRIGHT_COUNT, (int32_t)(filteredAngle + 0.5f));
  int32_t absErr = abs((int32_t)error);

  // ----------------------------
  // Cart estimated position/velocity
  // ----------------------------
  noInterrupts();
  long stepsSnapshot = cartStepCount;
  interrupts();

  cartPosSteps = (float)stepsSnapshot;

  float rawCartVel = (cartPosSteps - prevCartPosSteps) / dt;
  prevCartPosSteps = cartPosSteps;

  cartVelStepsPerSec =
      (1.0f - CART_VEL_FILTER_ALPHA) * cartVelStepsPerSec +
      CART_VEL_FILTER_ALPHA * rawCartVel;

  // ----------------------------
  // Engage window
  // ----------------------------
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

  // ----------------------------
  // Safety dropout
  // ----------------------------
  if (absErr > STOP_WINDOW_COUNTS) {
    controlEngaged = false;
    commandedStepRate = 0.0f;
    integralTerm = 0.0f;
    prevError = error;
    filteredDerivative = 0.0f;
    return;
  }

  // ----------------------------
  // Pendulum derivative
  // ----------------------------
  float rawDerivative = (error - prevError) / dt;
  filteredDerivative =
      (1.0f - DERIV_FILTER_ALPHA) * filteredDerivative +
      DERIV_FILTER_ALPHA * rawDerivative;
  prevError = error;

  // ----------------------------
  // Integral
  // ----------------------------
  integralTerm += error * dt;
  integralTerm = clampf(integralTerm, -I_LIMIT, I_LIMIT);

  // ----------------------------
  // Cart edge pushback
  // Adds extra centering near the ends
  // ----------------------------
  float edgePush = 0.0f;

  if (cartPosSteps > (CART_POS_MAX_STEPS - CART_LIMIT_MARGIN_STEPS)) {
    edgePush = EDGE_PUSH_GAIN * (cartPosSteps - (CART_POS_MAX_STEPS - CART_LIMIT_MARGIN_STEPS));
  } else if (cartPosSteps < (CART_POS_MIN_STEPS + CART_LIMIT_MARGIN_STEPS)) {
    edgePush = EDGE_PUSH_GAIN * (cartPosSteps - (CART_POS_MIN_STEPS + CART_LIMIT_MARGIN_STEPS));
  }

  // ----------------------------
  // Full control law
  // ----------------------------
  float u =
      (Kp * error) +
      (Ki * integralTerm) +
      (Kd * filteredDerivative) -
      (Kx * cartPosSteps) -
      (Kv * cartVelStepsPerSec) -
      edgePush;

  float rateCmd = OUTPUT_SCALE * u;

  // ----------------------------
  // Hard cart travel threshold
  // Do not command farther outward once at the edge
  // ----------------------------
  if (cartPosSteps >= CART_POS_MAX_STEPS && rateCmd > 0.0f) {
    rateCmd = 0.0f;
  }
  if (cartPosSteps <= CART_POS_MIN_STEPS && rateCmd < 0.0f) {
    rateCmd = 0.0f;
  }

  rateCmd = clampf(rateCmd, -MAX_STEP_RATE, MAX_STEP_RATE);

  // small deadband near perfect balance
  if (absErr < 8 && fabs(rateCmd) < 80.0f) {
    rateCmd = 0.0f;
  }

  commandedStepRate = rateCmd;
}

// ======================================================
// Step pulse engine
// ======================================================

void runStepper(uint32_t nowUs) {
  float rate = commandedStepRate;
  float absRate = fabs(rate);

  if (absRate <= MIN_STEP_RATE) {
    digitalWrite(STEP_PIN, LOW);
    stepPinState = LOW;
    return;
  }

  setDirectionFromSign(rate);

  currentAbsRate = absRate;
  halfPeriodUs = (uint32_t)(500000.0f / currentAbsRate);

  if (halfPeriodUs < STEP_PULSE_US) {
    halfPeriodUs = STEP_PULSE_US;
  }

  if ((uint32_t)(nowUs - lastStepToggleUs) >= halfPeriodUs) {
    lastStepToggleUs = nowUs;

    bool prevStepPinState = stepPinState;
    stepPinState = !stepPinState;
    digitalWrite(STEP_PIN, stepPinState ? HIGH : LOW);

    // count one commanded motor step on each rising edge
    if (!prevStepPinState && stepPinState) {
      cartStepCount += motionSignFromRate(rate);
    }
  }
}

// ======================================================
// Debug output
// ======================================================

void runDebug() {
  if (millis() - lastPrintMs < 50) return;
  lastPrintMs = millis();

  uint16_t raw = readRawAngle();
  int32_t errRaw = wrappedErrorCounts(UPRIGHT_COUNT, raw);
  int32_t errFilt = wrappedErrorCounts(UPRIGHT_COUNT, (int32_t)(filteredAngle + 0.5f));

  noInterrupts();
  long stepCountSnapshot = cartStepCount;
  interrupts();

  Serial.print("raw=");
  Serial.print(raw);
  Serial.print(" filt=");
  Serial.print(filteredAngle, 1);
  Serial.print(" errRaw=");
  Serial.print(errRaw);
  Serial.print(" errFilt=");
  Serial.print(errFilt);
  Serial.print(" dFilt=");
  Serial.print(filteredDerivative, 1);
  Serial.print(" pos=");
  Serial.print(stepCountSnapshot);
  Serial.print(" vel=");
  Serial.print(cartVelStepsPerSec, 1);
  Serial.print(" rate=");
  Serial.print(commandedStepRate, 1);
  Serial.print(" engaged=");
  Serial.println(controlEngaged ? 1 : 0);
}