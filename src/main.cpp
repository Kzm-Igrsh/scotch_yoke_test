#include <Arduino.h>
#include <M5Unified.h>

// Scotch yoke bench test: AtomS3 + akita11 HbridgeS_UNIT + N20 motor with encoder.
//   Button click : run / stop
//   Button hold  : next duty preset
//   Serial       : "g" run, "s" stop, "f" forward, "r" reverse, "0".."255" duty

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

const char* directionName(Direction direction) {
  return direction == Direction::Forward ? "FWD" : "REV";
}

void releaseMotor() {
  ledcWrite(kMotorForwardPwmChannel, 0);
  ledcWrite(kMotorReversePwmChannel, 0);
}

void writeMotor(uint8_t duty) {
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

  portENTER_CRITICAL_ISR(&g_encoderMux);
  g_encoderCount += kQuadratureTransitions[transition];
  g_previousEncoderState = currentState;
  portEXIT_CRITICAL_ISR(&g_encoderMux);
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
  } else if (isDigit(command)) {
    setDuty(static_cast<uint8_t>(constrain(line.toInt(), 0, 255)));
  } else {
    Serial.println("[HELP] g=run s=stop f/r=direction z=reset count 0..255=duty");
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
  updateEncoderRate(nowMs);

  if (nowMs - g_lastDisplayAtMs >= kDisplayIntervalMs) {
    g_lastDisplayAtMs = nowMs;
    drawStatus();
  }

  delay(2);
}
