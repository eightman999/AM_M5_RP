/*
  Arduino Mega + L298N MotorOnly
  High-torque anti-stall 500Hz PWM version for no ENA/ENB jumper removal.

  Fixes:
    - previous kick could repeat too often and feel jerky
    - low PWM command stalled under load
    - target changes are smoothed
    - kick is short and has cooldown

  v2 (paired with M5 v16):
    - applyKickToOutput() no longer overrides direction on a sign-change kick
      while actual is still ramping through zero. Previously a sign change
      (even a tiny one, e.g. +4 -> -4 from the M5-side slew) instantly forced
      actual's output to full KICK_CMD in the NEW direction regardless of
      actual's current (old-direction) magnitude, defeating approachCmd's
      "always pass through zero" reversal safety. Now the kick only applies
      once actual's own sign already matches kickSign.

  Wiring:
    L298N IN1 -> AM D22 / PA0
    L298N IN2 -> AM D23 / PA1
    L298N IN3 -> AM D24 / PA2
    L298N IN4 -> AM D25 / PA3

    AM D18 TX1 -> level shifter/divider -> M5 GPIO36 RX
    AM D19 RX1 <- M5 GPIO26 TX
    GND common
*/

#include <Arduino.h>

// D22-D25 are PORTA bit0-bit3 on Arduino Mega.
const uint8_t MOTOR_MASK = 0x0F;

const bool INVERT_LEFT_MOTOR  = false;
const bool INVERT_RIGHT_MOTOR = false;
const bool SWAP_LEFT_RIGHT    = false;

// 500Hz PWM, close to common Arduino PWM frequency.
const uint16_t PWM_PERIOD_US = 2000;

// Smooth ramp.
const unsigned long RAMP_PERIOD_MS = 2;
const int RAMP_UP_STEP = 20;
const int RAMP_DOWN_STEP = 22;
const int REVERSE_ZERO_STEP = 36;

// L298N + loaded robot needs enough duty to overcome static friction.
const int MIN_EFFECTIVE_CMD = 132;

// Short kick only when starting from rest, not on every target update.
const int KICK_CMD = 225;
const unsigned long KICK_MS = 150;
const unsigned long KICK_COOLDOWN_MS = 180;

const unsigned long COMMAND_TIMEOUT_MS = 720;
const unsigned long TELEMETRY_PERIOD_MS = 35;
const unsigned long BRAKE_PULSE_MS = 30;

const bool DEBUG_USB = false;

int targetLeft = 0;
int targetRight = 0;
int actualLeft = 0;
int actualRight = 0;

unsigned long kickLeftUntilMs = 0;
unsigned long kickRightUntilMs = 0;
unsigned long lastKickLeftMs = 0;
unsigned long lastKickRightMs = 0;
int kickLeftSign = 0;
int kickRightSign = 0;

unsigned long lastCommandMs = 0;
unsigned long lastTelemetryMs = 0;
unsigned long lastRampMs = 0;
unsigned long brakeUntilMs = 0;

char m5Line[80];
uint8_t m5LineLen = 0;

char usbLine[80];
uint8_t usbLineLen = 0;

int clampRaw(int v) {
  if (v > 255) return 255;
  if (v < -255) return -255;
  return v;
}

int signOf(int v) {
  if (v > 0) return 1;
  if (v < 0) return -1;
  return 0;
}

int applyMinimumEffective(int v) {
  v = clampRaw(v);
  if (v == 0) return 0;

  int s = signOf(v);
  int mag = abs(v);

  if (mag < MIN_EFFECTIVE_CMD) {
    mag = MIN_EFFECTIVE_CMD;
  }

  return s * mag;
}

void maybeStartKick(int oldTarget, int newTarget, int actual,
                    unsigned long &kickUntil, unsigned long &lastKick, int &kickSign) {
  if (newTarget == 0) return;

  unsigned long now = millis();
  int oldS = signOf(oldTarget);
  int newS = signOf(newTarget);

  bool startingFromRest = oldS == 0 && abs(actual) < MIN_EFFECTIVE_CMD;
  bool signChanged = oldS != 0 && newS != 0 && oldS != newS;

  if ((startingFromRest || signChanged) && (now - lastKick > KICK_COOLDOWN_MS)) {
    kickUntil = now + KICK_MS;
    lastKick = now;
    kickSign = newS;
  }
}

int approachCmd(int cur, int tgt) {
  cur = clampRaw(cur);
  tgt = applyMinimumEffective(tgt);

  if (cur == tgt) return cur;

  // No instant reverse. Always pass through zero.
  if ((cur > 0 && tgt < 0) || (cur < 0 && tgt > 0)) {
    if (cur > 0) return max(0, cur - REVERSE_ZERO_STEP);
    if (cur < 0) return min(0, cur + REVERSE_ZERO_STEP);
  }

  int step = (abs(tgt) > abs(cur)) ? RAMP_UP_STEP : RAMP_DOWN_STEP;

  if (cur < tgt) {
    cur += step;
    if (cur > tgt) cur = tgt;
  } else {
    cur -= step;
    if (cur < tgt) cur = tgt;
  }

  return clampRaw(cur);
}

int applyKickToOutput(int actual, unsigned long kickUntil, int kickSign) {
  if (actual == 0) return 0;

  // v2: キックはactualの符号が既にkickSignと一致している間だけ効かせる。
  // 符号反転の直後はactualがまだ旧符号のまま0へ向けてランプ中(approachCmdの
  // REVERSE_ZERO_STEPによる「必ず0を経由する」処理の途中)であり、そこでkickSignを
  // 無条件適用すると瞬間フルパワーの逆転になってしまう(approachCmdの安全策を無効化する)。
  // actualが実際に新符号側へ入ってから初めてキックを乗せることで、
  // 0経由の滑らかな反転を保ったまま「発進/転換直後の粘り抜け」だけ助ける。
  int actualSign = (actual > 0) ? 1 : -1;
  if (millis() < kickUntil && kickSign != 0 && actualSign == kickSign) {
    int mag = max(abs(actual), KICK_CMD);
    return kickSign * constrain(mag, 0, 255);
  }

  return actual;
}

void writeMotorBits(uint8_t bits) {
  PORTA = (PORTA & ~MOTOR_MASK) | (bits & MOTOR_MASK);
}

void applyMotorOutputs() {
  int left = applyKickToOutput(actualLeft, kickLeftUntilMs, kickLeftSign);
  int right = applyKickToOutput(actualRight, kickRightUntilMs, kickRightSign);

  if (SWAP_LEFT_RIGHT) {
    int t = left;
    left = right;
    right = t;
  }

  if (INVERT_LEFT_MOTOR) left = -left;
  if (INVERT_RIGHT_MOTOR) right = -right;

  left = clampRaw(left);
  right = clampRaw(right);

  bool brakeNow = millis() < brakeUntilMs;
  if (brakeNow) {
    writeMotorBits(MOTOR_MASK);
    return;
  }

  uint16_t phase = micros() % PWM_PERIOD_US;
  uint16_t leftOn  = (uint32_t)abs(left)  * PWM_PERIOD_US / 255;
  uint16_t rightOn = (uint32_t)abs(right) * PWM_PERIOD_US / 255;

  bool onL = (abs(left) > 0) && (phase < leftOn);
  bool onR = (abs(right) > 0) && (phase < rightOn);

  uint8_t out = 0;

  if (onL) {
    if (left > 0) out |= _BV(0);
    else          out |= _BV(1);
  }

  if (onR) {
    if (right > 0) out |= _BV(2);
    else           out |= _BV(3);
  }

  writeMotorBits(out);
}

void updateRamp() {
  unsigned long now = millis();
  if (now - lastRampMs < RAMP_PERIOD_MS) return;
  lastRampMs = now;

  actualLeft = approachCmd(actualLeft, targetLeft);
  actualRight = approachCmd(actualRight, targetRight);
}

void setTarget(int l, int r) {
  l = clampRaw(l);
  r = clampRaw(r);

  maybeStartKick(targetLeft, l, actualLeft, kickLeftUntilMs, lastKickLeftMs, kickLeftSign);
  maybeStartKick(targetRight, r, actualRight, kickRightUntilMs, lastKickRightMs, kickRightSign);

  targetLeft = l;
  targetRight = r;
  lastCommandMs = millis();
}

void softStop() {
  targetLeft = 0;
  targetRight = 0;
  lastCommandMs = millis();
}

void brakeStop() {
  targetLeft = 0;
  targetRight = 0;
  actualLeft = 0;
  actualRight = 0;
  kickLeftUntilMs = 0;
  kickRightUntilMs = 0;
  brakeUntilMs = millis() + BRAKE_PULSE_MS;
  lastCommandMs = millis();
}

void sendTelemetry() {
  Serial1.print("A,");
  Serial1.print(millis());
  Serial1.print(",0,0,-1,-1,");
  Serial1.print(actualLeft);
  Serial1.print(',');
  Serial1.println(actualRight);

  if (DEBUG_USB) {
    Serial.print("[AM] tgt=");
    Serial.print(targetLeft);
    Serial.print(',');
    Serial.print(targetRight);
    Serial.print(" act=");
    Serial.print(actualLeft);
    Serial.print(',');
    Serial.println(actualRight);
  }
}

void handleCommand(char *line, const char *src) {
  while (*line == ' ' || *line == '\t') line++;
  if (*line == '\0') return;

  if (line[0] == 'S' || line[0] == 's') {
    softStop();
    return;
  }

  if (line[0] == 'B' || line[0] == 'b') {
    brakeStop();
    return;
  }

  if (line[0] == 'Z' || line[0] == 'z') {
    lastCommandMs = millis();
    return;
  }

  if ((line[0] == 'M' || line[0] == 'm') && line[1] == ',') {
    int l = 0;
    int r = 0;
    if (sscanf(line + 2, "%d,%d", &l, &r) == 2) {
      setTarget(l, r);

      if (DEBUG_USB) {
        Serial.print("[AM] ");
        Serial.print(src);
        Serial.print(" M ");
        Serial.print(targetLeft);
        Serial.print(',');
        Serial.println(targetRight);
      }
    }
  }
}

void readSerialLine(Stream &s, char *buf, uint8_t &len, uint8_t maxLen, const char *src) {
  while (s.available()) {
    char c = (char)s.read();

    if (c == '\r') continue;

    if (c == '\n') {
      buf[len] = '\0';
      if (len > 0) handleCommand(buf, src);
      len = 0;
      continue;
    }

    if (len < maxLen - 1) {
      buf[len++] = c;
    } else {
      len = 0;
    }
  }
}

void setup() {
  DDRA |= MOTOR_MASK;
  writeMotorBits(0);

  Serial.begin(115200);
  Serial1.begin(115200);

  brakeStop();
  lastCommandMs = millis();

  if (DEBUG_USB) {
    Serial.println("AM Smooth AntiStall 500Hz PWM boot");
  }
}

void loop() {
  readSerialLine(Serial1, m5Line, m5LineLen, sizeof(m5Line), "M5");

  if (DEBUG_USB) {
    readSerialLine(Serial, usbLine, usbLineLen, sizeof(usbLine), "USB");
  }

  if (millis() - lastCommandMs > COMMAND_TIMEOUT_MS) {
    softStop();
  }

  updateRamp();
  applyMotorOutputs();

  if (millis() - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    sendTelemetry();
    lastTelemetryMs = millis();
  }
}
