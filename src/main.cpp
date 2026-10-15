#include <Arduino.h>
#include <M5Unified.h>

// Scotch yoke bench test: AtomS3 + akita11 HbridgeS_UNIT + N20 motor with encoder.
//   Button click : run / stop
//   Button hold  : next duty preset
//   Serial       : "g" run, "s" stop, "f" forward, "r" reverse, "0".."255" duty
//                  "p HI LO" duty HI for the first half turn and LO for the second, "p" off
//                  "c" record a timestamp at every encoder edge, "d" dump the record as CSV
//                  "b 1" brake during PWM off-time, "b 0" coast (default)

namespace {
constexpr uint8_t kMotorForwardPin = 2;  // Grove yellow: MF
constexpr uint8_t kMotorReversePin = 1;  // Grove white: MR
constexpr uint8_t kEncoderAPin = 5;
constexpr uint8_t kEncoderBPin = 6;

// AtomS3 uses LEDC channel 7 for the display backlight.
constexpr uint8_t kMotorForwardPwmChannel = 0;
constexpr uint8_t kMotorReversePwmChannel = 1;
constexpr uint8_t kPwmResolutionBits = 8;
constexpr uint32_t kPwmFrequencyHz = 20000;
constexpr uint32_t kDirectionDeadTimeUs = 100;

// Ramp the duty instead of stepping it, so the yoke is not jerked at start/stop.
constexpr uint8_t kRampStep = 3;
constexpr uint32_t kRampIntervalMs = 10;  // 0 -> 255 in about 0.9 s

// The loaded yoke does not start below ~70% duty, but keeps turning at lower duty
// once moving. Kick at full duty for a moment, then ramp down to the set duty.
constexpr uint8_t kKickDuty = 255;
constexpr uint32_t kKickMs = 150;

// Stop when the encoder does not move for this many rate windows while driven.
constexpr uint8_t kStallWindows = 2;

constexpr uint8_t kDutyPresets[] = {64, 96, 128, 192, 255};
constexpr uint32_t kHoldMs = 800;

// Counts per output-shaft revolution (x4 quadrature). 0 = unknown, show counts/s only.
// Measured 2026-10-15: 10 crank turns by hand = 2713 counts.
constexpr int32_t kEncoderCountsPerRev = 271;
// Second run gave 2710; the profile uses the mean so the half-turn boundary drifts less.
constexpr float kEncoderCountsPerRevExact = 271.15F;

// One entry per encoder edge. 12000 edges = about 2.2 s at full speed, 8 s at 5 Hz.
constexpr size_t kCaptureCapacity = 12000;
constexpr size_t kDutyEventCapacity = 2000;

constexpr uint32_t kRateIntervalMs = 500;
constexpr uint32_t kDisplayIntervalMs = 200;

enum class Direction : uint8_t {
  Forward,
  Reverse,
};

portMUX_TYPE g_encoderMux = portMUX_INITIALIZER_UNLOCKED;
volatile int32_t g_encoderCount = 0;
volatile uint8_t g_previousEncoderState = 0;

bool g_running = false;
Direction g_direction = Direction::Forward;
Direction g_requestedDirection = Direction::Forward;
uint8_t g_presetIndex = 0;
uint8_t g_setDuty = kDutyPresets[0];
uint8_t g_appliedDuty = 0;

int32_t g_countsPerSecond = 0;
int32_t g_previousRateCount = 0;
uint32_t g_lastRateAtMs = 0;
uint32_t g_lastRampAtMs = 0;
uint32_t g_kickUntilMs = 0;
uint8_t g_stallWindows = 0;
uint32_t g_lastDisplayAtMs = 0;

// Coast PWM (off-time = both LOW) never applies braking torque, so a low-duty half turn
// barely slows the crank. Brake PWM (off-time = both HIGH) shorts the motor instead.
bool g_brakePwm = false;

bool g_profileEnabled = false;
uint8_t g_profileHighDuty = 0;
uint8_t g_profileLowDuty = 0;

struct EdgeSample {
  uint32_t tUs;
  int32_t count;
};
struct DutyEvent {
  uint32_t tUs;
  uint8_t duty;
};
EdgeSample g_edges[kCaptureCapacity];
DutyEvent g_dutyEvents[kDutyEventCapacity];
volatile bool g_capturing = false;
volatile size_t g_edgeCount = 0;
size_t g_dutyEventCount = 0;
uint32_t g_captureStartUs = 0;

const char* directionName(Direction direction) {
  return direction == Direction::Forward ? "FWD" : "REV";
}

void recordDuty(uint8_t duty) {
  if (g_capturing && g_dutyEventCount < kDutyEventCapacity) {
    g_dutyEvents[g_dutyEventCount++] = {micros(), duty};
  }
}

void releaseMotor() {
  recordDuty(0);
  ledcWrite(kMotorForwardPwmChannel, 0);
  ledcWrite(kMotorReversePwmChannel, 0);
}

void writeMotor(uint8_t duty) {
  recordDuty(duty);
  if (g_brakePwm) {
    // Hold the driving input HIGH and PWM the other one inverted: LOW drives, HIGH brakes.
    // ledcWrite(255) is full on with 8-bit resolution, so the held input never drops.
    const uint8_t brakeDuty = static_cast<uint8_t>(255U - duty);
    if (g_direction == Direction::Forward) {
      ledcWrite(kMotorForwardPwmChannel, 255);
      ledcWrite(kMotorReversePwmChannel, brakeDuty);
    } else {
      ledcWrite(kMotorReversePwmChannel, 255);
      ledcWrite(kMotorForwardPwmChannel, brakeDuty);
    }
    return;
  }
  if (g_direction == Direction::Forward) {
    ledcWrite(kMotorReversePwmChannel, 0);
    ledcWrite(kMotorForwardPwmChannel, duty);
  } else {
    ledcWrite(kMotorForwardPwmChannel, 0);
    ledcWrite(kMotorReversePwmChannel, duty);
  }
}

// Move the applied duty toward the target. A direction change ramps down to 0 first.
void updateRamp(uint32_t nowMs) {
  if (nowMs - g_lastRampAtMs < kRampIntervalMs) {
    return;
  }
  g_lastRampAtMs = nowMs;

  const bool reversing = g_requestedDirection != g_direction;
  const uint8_t target = (g_running && !reversing) ? g_setDuty : 0;

  if (g_appliedDuty == 0 && reversing) {
    // Release both bridge inputs before enabling the opposite direction.
    releaseMotor();
    delayMicroseconds(kDirectionDeadTimeUs);
    g_direction = g_requestedDirection;
    Serial.printf("[MOTOR] direction=%s\n", directionName(g_direction));
    return;
  }

  if (static_cast<int32_t>(nowMs - g_kickUntilMs) < 0) {
    return;  // Hold the kick duty.
  }

  if (g_running && g_profileEnabled && !reversing && g_appliedDuty > 0) {
    return;  // updateProfile() owns the duty; ramping would smear the switching.
  }

  if (g_appliedDuty == 0 && target > 0) {
    g_appliedDuty = kKickDuty;
    g_kickUntilMs = nowMs + kKickMs;
    g_stallWindows = 0;
    writeMotor(g_appliedDuty);
    return;
  }

  if (g_appliedDuty == target) {
    return;
  }
  if (g_appliedDuty < target) {
    g_appliedDuty = static_cast<uint8_t>(min<int>(g_appliedDuty + kRampStep, target));
  } else {
    g_appliedDuty = static_cast<uint8_t>(max<int>(g_appliedDuty - kRampStep, target));
  }

  if (g_appliedDuty == 0) {
    releaseMotor();
  } else {
    writeMotor(g_appliedDuty);
  }
}

int32_t encoderCountSnapshot();

// Switch the duty by crank angle. Angle 0 is where the count was last reset (boot or "z").
void updateProfile(uint32_t nowMs) {
  if (!g_running || !g_profileEnabled || g_requestedDirection != g_direction ||
      g_appliedDuty == 0 || static_cast<int32_t>(nowMs - g_kickUntilMs) < 0) {
    return;
  }

  float turns = static_cast<float>(encoderCountSnapshot()) / kEncoderCountsPerRevExact;
  turns -= floorf(turns);
  const uint8_t duty = turns < 0.5F ? g_profileHighDuty : g_profileLowDuty;
  if (duty != g_appliedDuty) {
    g_appliedDuty = duty;
    writeMotor(duty);
  }
}

void setRunning(bool running) {
  g_running = running;
  Serial.printf("[MOTOR] %s duty=%u/255 dir=%s\n", running ? "RUN" : "STOP", g_setDuty,
                directionName(g_requestedDirection));
}

void setDuty(uint8_t duty) {
  g_setDuty = duty;
  Serial.printf("[MOTOR] duty=%u/255 (%u%%)\n", duty, (duty * 100U) / 255U);
}

int32_t encoderCountSnapshot() {
  portENTER_CRITICAL(&g_encoderMux);
  const int32_t count = g_encoderCount;
  portEXIT_CRITICAL(&g_encoderMux);
  return count;
}

void IRAM_ATTR updateEncoder() {
  static constexpr int8_t kQuadratureTransitions[16] = {
      0, -1, 1, 0,
      1, 0, 0, -1,
      -1, 0, 0, 1,
      0, 1, -1, 0,
  };

  const uint8_t currentState =
      (static_cast<uint8_t>(digitalRead(kEncoderAPin)) << 1U) |
      static_cast<uint8_t>(digitalRead(kEncoderBPin));
  const uint8_t transition =
      static_cast<uint8_t>((g_previousEncoderState << 2U) | currentState);

  const int8_t step = kQuadratureTransitions[transition];

  portENTER_CRITICAL_ISR(&g_encoderMux);
  g_encoderCount += step;
  g_previousEncoderState = currentState;
  if (step != 0 && g_capturing) {
    if (g_edgeCount < kCaptureCapacity) {
      g_edges[g_edgeCount++] = {micros(), g_encoderCount};
    } else {
      g_capturing = false;
    }
  }
  portEXIT_CRITICAL_ISR(&g_encoderMux);
}

void startCapture() {
  portENTER_CRITICAL(&g_encoderMux);
  g_edgeCount = 0;
  g_dutyEventCount = 0;
  g_captureStartUs = micros();
  g_capturing = true;
  portEXIT_CRITICAL(&g_encoderMux);
  recordDuty(g_appliedDuty);
  Serial.println("[CAPTURE] start");
}

void dumpCapture() {
  portENTER_CRITICAL(&g_encoderMux);
  g_capturing = false;
  const size_t edgeCount = g_edgeCount;
  portEXIT_CRITICAL(&g_encoderMux);

  Serial.printf("[DUMP] edges=%u duty_events=%u counts_per_rev=%.2f\n",
                static_cast<unsigned>(edgeCount), static_cast<unsigned>(g_dutyEventCount),
                kEncoderCountsPerRevExact);
  Serial.println("kind,t_us,value");
  for (size_t i = 0; i < g_dutyEventCount; ++i) {
    Serial.printf("duty,%lu,%u\n",
                  static_cast<unsigned long>(g_dutyEvents[i].tUs - g_captureStartUs),
                  g_dutyEvents[i].duty);
  }
  for (size_t i = 0; i < edgeCount; ++i) {
    Serial.printf("edge,%lu,%ld\n", static_cast<unsigned long>(g_edges[i].tUs - g_captureStartUs),
                  static_cast<long>(g_edges[i].count));
  }
  Serial.println("[DUMP END]");
}

void updateEncoderRate(uint32_t nowMs) {
  const uint32_t elapsedMs = nowMs - g_lastRateAtMs;
  if (elapsedMs < kRateIntervalMs) {
    return;
  }

  const int32_t count = encoderCountSnapshot();
  const int32_t delta = count - g_previousRateCount;
  g_countsPerSecond =
      static_cast<int32_t>((static_cast<int64_t>(delta) * 1000) / elapsedMs);
  g_previousRateCount = count;
  g_lastRateAtMs = nowMs;

  // A stalled DC motor draws its locked-rotor current, so cut the drive.
  if (g_running && g_appliedDuty > 0 && delta == 0) {
    if (++g_stallWindows >= kStallWindows) {
      g_running = false;
      Serial.println("[STALL] encoder did not move, motor stopped");
    }
  } else {
    g_stallWindows = 0;
  }

  // CSV-like line so the log can be pasted into a spreadsheet later.
  Serial.printf("[LOG] t_ms=%lu duty=%u count=%ld rate=%ld", static_cast<unsigned long>(nowMs),
                g_appliedDuty, static_cast<long>(count), static_cast<long>(g_countsPerSecond));
  if (kEncoderCountsPerRev > 0) {
    Serial.printf(" freq_hz=%.2f", static_cast<float>(g_countsPerSecond) / static_cast<float>(kEncoderCountsPerRev));
  }
  Serial.println();
}

void handleSerial() {
  if (!Serial.available()) {
    return;
  }
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.isEmpty()) {
    return;
  }

  const char command = line.charAt(0);
  if (command == 'g') {
    setRunning(true);
  } else if (command == 's') {
    setRunning(false);
  } else if (command == 'f') {
    g_requestedDirection = Direction::Forward;
  } else if (command == 'r') {
    g_requestedDirection = Direction::Reverse;
  } else if (command == 'z') {
    portENTER_CRITICAL(&g_encoderMux);
    g_encoderCount = 0;
    portEXIT_CRITICAL(&g_encoderMux);
    g_previousRateCount = 0;
    Serial.println("[ENCODER] count reset");
  } else if (command == 'p') {
    int high = 0;
    int low = 0;
    if (sscanf(line.c_str(), "p %d %d", &high, &low) == 2) {
      g_profileHighDuty = static_cast<uint8_t>(constrain(high, 0, 255));
      g_profileLowDuty = static_cast<uint8_t>(constrain(low, 0, 255));
      g_profileEnabled = true;
      Serial.printf("[PROFILE] on high=%u low=%u\n", g_profileHighDuty, g_profileLowDuty);
    } else {
      g_profileEnabled = false;
      Serial.println("[PROFILE] off");
    }
  } else if (command == 'b') {
    g_brakePwm = line.endsWith("1");
    if (g_appliedDuty > 0) {
      writeMotor(g_appliedDuty);
    }
    Serial.printf("[PWM] %s\n", g_brakePwm ? "brake" : "coast");
  } else if (command == 'c') {
    startCapture();
  } else if (command == 'd') {
    dumpCapture();
  } else if (isDigit(command)) {
    setDuty(static_cast<uint8_t>(constrain(line.toInt(), 0, 255)));
  } else {
    Serial.println("[HELP] g=run s=stop f/r=direction z=reset count 0..255=duty "
                   "p HI LO=half-turn profile b 0/1=coast/brake PWM c=capture d=dump");
  }
}

void drawStatus() {
  const int32_t count = encoderCountSnapshot();

  M5.Display.startWrite();
  M5.Display.fillScreen(BLACK);
  M5.Display.setCursor(4, 4);
  M5.Display.setTextColor(WHITE, BLACK);
  M5.Display.setTextSize(1);
  M5.Display.println("SCOTCH YOKE TEST");
  M5.Display.println();
  M5.Display.printf("State: %s %s\n", g_running ? "RUN" : "STOP",
                    directionName(g_requestedDirection));
  M5.Display.printf("Set:   %u%%\n", (g_setDuty * 100U) / 255U);
  M5.Display.printf("Out:   %u%%\n", (g_appliedDuty * 100U) / 255U);
  M5.Display.printf("Count: %ld\n", static_cast<long>(count));
  M5.Display.printf("Rate:  %ld/s\n", static_cast<long>(g_countsPerSecond));
  if (kEncoderCountsPerRev > 0) {
    M5.Display.printf("Freq:  %.2f Hz\n",
                      static_cast<float>(g_countsPerSecond) / static_cast<float>(kEncoderCountsPerRev));
  }
  M5.Display.println();
  M5.Display.println("Click: run/stop");
  M5.Display.println("Hold:  next duty");
  M5.Display.endWrite();
}
}  // namespace

void setup() {
  // Put the bridge in its safe, open state before initializing peripherals.
  pinMode(kMotorForwardPin, OUTPUT);
  pinMode(kMotorReversePin, OUTPUT);
  digitalWrite(kMotorForwardPin, LOW);
  digitalWrite(kMotorReversePin, LOW);

  auto config = M5.config();
  M5.begin(config);
  M5.BtnA.setHoldThresh(kHoldMs);

  Serial.begin(115200);
  Serial.setTimeout(20);
  delay(200);

  ledcSetup(kMotorForwardPwmChannel, kPwmFrequencyHz, kPwmResolutionBits);
  ledcSetup(kMotorReversePwmChannel, kPwmFrequencyHz, kPwmResolutionBits);
  ledcAttachPin(kMotorForwardPin, kMotorForwardPwmChannel);
  ledcAttachPin(kMotorReversePin, kMotorReversePwmChannel);
  releaseMotor();

  pinMode(kEncoderAPin, INPUT_PULLUP);
  pinMode(kEncoderBPin, INPUT_PULLUP);
  g_previousEncoderState =
      (static_cast<uint8_t>(digitalRead(kEncoderAPin)) << 1U) |
      static_cast<uint8_t>(digitalRead(kEncoderBPin));
  attachInterrupt(digitalPinToInterrupt(kEncoderAPin), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kEncoderBPin), updateEncoder, CHANGE);

  M5.Display.setRotation(0);
  M5.Display.setTextWrap(false);
  drawStatus();

  const uint32_t nowMs = millis();
  g_lastRateAtMs = nowMs;
  g_lastRampAtMs = nowMs;
  g_lastDisplayAtMs = nowMs;

  Serial.println("[READY] Scotch yoke test (AtomS3 + HbridgeS_UNIT)");
  Serial.println("[PINS] MF=G2 MR=G1 EncoderA=G5 EncoderB=G6");
  Serial.println("[HELP] click=run/stop hold=next duty | serial: g s f r z 0..255");
}

void loop() {
  M5.update();

  if (M5.BtnA.wasHold()) {
    g_presetIndex = static_cast<uint8_t>(
        (g_presetIndex + 1U) % (sizeof(kDutyPresets) / sizeof(kDutyPresets[0])));
    setDuty(kDutyPresets[g_presetIndex]);
  } else if (M5.BtnA.wasClicked()) {
    setRunning(!g_running);
  }

  handleSerial();

  const uint32_t nowMs = millis();
  updateRamp(nowMs);
  updateProfile(nowMs);
  updateEncoderRate(nowMs);

  // Redrawing takes milliseconds, which would delay the profile switching.
  if (g_capturing || g_profileEnabled && g_running) {
    return;
  }
  if (nowMs - g_lastDisplayAtMs >= kDisplayIntervalMs) {
    g_lastDisplayAtMs = nowMs;
    drawStatus();
  }

  delay(2);
}
