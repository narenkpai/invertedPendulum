// Ultrasonic cart position test
// Echo = 6
// Trig = 7

const int ECHO_PIN = 6;
const int TRIG_PIN = 7;

// change if needed
const unsigned long TIMEOUT_US = 30000UL;   // about 5 m max
const int SAMPLES = 5;

float readDistanceCmOnce() {
  // clean trigger
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);

  // 10 us trigger pulse
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, TIMEOUT_US);

  if (duration == 0) {
    return -1.0f;   // timeout
  }

  // speed of sound: distance cm = duration * 0.0343 / 2
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

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(TRIG_PIN, LOW);

  Serial.begin(115200);
  delay(500);

  Serial.println("Ultrasonic cart position test");
  Serial.println("Move cart to each endpoint and record distance in cm");
}

void loop() {
  float d = readDistanceCmFiltered();

  if (d < 0.0f) {
    Serial.println("No echo");
  } else {
    Serial.print("Distance_cm: ");
    Serial.println(d, 2);
  }

  delay(100);
}