#include <Arduino.h>
#include <EEPROM.h>
#include <ctype.h>

// ===================== BLUETOOTH =====================
// Arduino Mega: Serial1 -> RX1=19, TX1=18
// Typical HC-05 default baud = 9600
HardwareSerial &BT = Serial1;
const unsigned long USB_BAUD = 115200;
const unsigned long BT_BAUD  = 9600;

// ===================== PINS =====================
const uint8_t L_IN1 = 22;
const uint8_t L_IN2 = 23;
const uint8_t R_IN3 = 24;
const uint8_t R_IN4 = 25;
const uint8_t EN_LEFT  = 5;
const uint8_t EN_RIGHT = 6;

const uint8_t SENS_PINS[5] = {30, 31, 32, 33, 34};   // left -> right
const uint8_t LED_PIN = 13;

const uint8_t SENSOR_LINE_LOW = LOW;   // active-low sensors

// ===================== SPEEDS =====================
const int BASE_SPEED            = 67;
const int TURN_SPEED            = 110;   // normal left/right turn speed
const int UTURN_SPEED           = 117;   // separate speed only for U-turn
const int SEARCH_SPEED          = 72;
const int PROBE_SPEED           = 52;
const int DEAD_END_PROBE_SPEED  = 58;
const int START_SPEED           = 58;
const int ALIGN_SPEED           = 70;
const int POST_TURN_SPEED       = 65;

// ===================== TIMING =====================
const unsigned long START_BOX_CONFIRM_MS  = 300;
const unsigned long START_EXIT_TIMEOUT_MS = 1200;

const unsigned long NODE_LOCKOUT_MS       = 105;
const unsigned long END_CONFIRM_MS        = 260;
const unsigned long PROBE_ENTRY_MS        = 55;
const unsigned long PROBE_STRAIGHT_MS     = 90;

const unsigned long DEAD_END_PROBE_MS     = 150;

const unsigned long LOST_LINE_TIMEOUT_MS  = 180;

const unsigned long DEAD_END_GRACE_AFTER_NODE_MS = 480;
const unsigned long TURN_TIMEOUT_MS       = 1000;
const unsigned long UTURN_TIMEOUT_MS      = 1500;
const unsigned long ALIGN_TIMEOUT_MS      = 3000;
const unsigned long ALIGN_STABLE_MS       = 40;

const unsigned long POST_TURN_FORWARD_MS  = 60;
const unsigned long POST_TURN_SETTLE_MS   = 20;

const uint8_t PROBE_BURST_N        = 12;
const uint8_t MAX_EXIT_CANDIDATES  = 32;
const unsigned long RECORD_RESOLVE_MS = 220;

// ===================== STORAGE =====================
const int MAX_NODES = 200;

// rawPath: L/S/R at junctions; B = dead end / U-turn
char rawPath[MAX_NODES + 1];
char optimalPath[MAX_NODES + 1];

// bit0 = left exit exists
// bit1 = straight exit exists
// bit2 = right exit exists
uint8_t exitsRecorded[MAX_NODES];

// time between stored nodes
unsigned int segmentTimeMs[MAX_NODES];

int rawPathLen = 0;
int optimalPathLen = 0;
int solveStepIndex = 0;

bool pendingTrainingRecord = false;
unsigned int pendingSegMs = 0;
char pendingTurn = 0;
uint8_t pendingPrimaryMask = 0;
unsigned long pendingRecordStartMs = 0;
bool recordWitnessLeft = false;
bool recordWitnessRight = false;

uint8_t candMask[MAX_EXIT_CANDIDATES];
uint8_t candVotes[MAX_EXIT_CANDIDATES];
uint8_t candCount = 0;

// ===================== EEPROM LAYOUT =====================
const uint32_t EEPROM_MAGIC   = 0x4D415A45UL; // "MAZE"
const uint16_t EEPROM_VERSION = 1;

struct EepromMazeData
{
  uint32_t magic;
  uint16_t version;
  uint16_t rawLen;
  uint16_t optLen;
  char raw[MAX_NODES + 1];
  char opt[MAX_NODES + 1];
  uint16_t checksum;
};

// ===================== STATE =====================
enum RobotState
{
  WAIT_COMMAND,
  ARM_TRAINING,
  TRAINING,
  READY_TO_SOLVE,
  ARM_SOLVING,
  SOLVING
};

RobotState state = WAIT_COMMAND;

struct SensorState
{
  uint8_t v[5];
  uint8_t onCount;
  bool allBlack;
  bool noLine;
};

SensorState sensors;

unsigned long segmentStartMs    = 0;
unsigned long lastNodeHandledMs = 0;
unsigned long lastLineSeenMs    = 0;
unsigned long startBoxSeenMs    = 0;

int lastSteer = 0;   // -1 left, 0 center, +1 right

// ===================== COMMAND BUFFERS =====================
const uint8_t CMD_BUF_SIZE = 32;
char btCmdBuf[CMD_BUF_SIZE];
char usbCmdBuf[CMD_BUF_SIZE];
uint8_t btCmdIdx = 0;
uint8_t usbCmdIdx = 0;

// ===================== STAGE TEXT =====================
char currentStage[40] = "IDLE";

// ===================== HELPERS =====================
void logLine(const String &msg)
{
  Serial.println(msg);
  BT.println(msg);
}

void setStage(const char *stageText)
{
  if (strcmp(currentStage, stageText) != 0) {
    strncpy(currentStage, stageText, sizeof(currentStage) - 1);
    currentStage[sizeof(currentStage) - 1] = '\0';
    logLine(String("STAGE=") + currentStage);
  }
}

const char* stateName(RobotState s)
{
  switch (s) {
    case WAIT_COMMAND:   return "WAIT_COMMAND";
    case ARM_TRAINING:   return "ARM_TRAINING";
    case TRAINING:       return "TRAINING";
    case READY_TO_SOLVE: return "READY_TO_SOLVE";
    case ARM_SOLVING:    return "ARM_SOLVING";
    case SOLVING:        return "SOLVING";
    default:             return "UNKNOWN";
  }
}

String exitsMaskToString(uint8_t exitsMask)
{
  String s = "";
  if (exitsMask & 0x01) s += 'L';
  if (exitsMask & 0x02) s += 'S';
  if (exitsMask & 0x04) s += 'R';
  if (exitsMask == 0)   s = "DEAD";
  return s;
}

bool validTurnChar(char c)
{
  return (c == 'L' || c == 'S' || c == 'R' || c == 'B');
}

void clearRamPaths()
{
  rawPathLen = 0;
  optimalPathLen = 0;
  solveStepIndex = 0;

  rawPath[0] = '\0';
  optimalPath[0] = '\0';

  memset(exitsRecorded, 0, sizeof(exitsRecorded));
  memset(segmentTimeMs, 0, sizeof(segmentTimeMs));

  segmentStartMs = millis();

  pendingTrainingRecord = false;
  candCount = 0;
  pendingSegMs = 0;
  pendingTurn = 0;
  pendingPrimaryMask = 0;
  recordWitnessLeft = false;
  recordWitnessRight = false;
}

char selectTurn(bool foundLeft, bool foundStraight, bool foundRight);

char turnFromMaskLHR(uint8_t mask)
{
  bool L = (mask & 0x01) != 0;
  bool S = (mask & 0x02) != 0;
  bool R = (mask & 0x04) != 0;
  return selectTurn(L, S, R);
}

void addExitCandidateVote(uint8_t mask)
{
  for (uint8_t i = 0; i < candCount; i++) {
    if (candMask[i] == mask) {
      if (candVotes[i] < 255) {
        candVotes[i]++;
      }
      return;
    }
  }
  if (candCount >= MAX_EXIT_CANDIDATES) {
    return;
  }
  candMask[candCount] = mask;
  candVotes[candCount] = 1;
  candCount++;
}

void buildExitCandidatesFromBursts(const bool *burstL,
                                   const bool *burstR,
                                   const bool *burstS,
                                   uint8_t n)
{
  candCount = 0;
  for (uint8_t ki = 0; ki < n; ki++) {
    for (uint8_t mj = 0; mj < n; mj++) {
      uint8_t m = 0;
      if (burstL[ki]) {
        m |= 0x01;
      }
      if (burstS[mj]) {
        m |= 0x02;
      }
      if (burstR[ki]) {
        m |= 0x04;
      }
      addExitCandidateVote(m);
    }
  }
}

void startPendingTrainingRecord(uint8_t primaryMask, char turn, unsigned int segMs)
{
  pendingTrainingRecord = true;
  pendingPrimaryMask = primaryMask;
  pendingTurn = turn;
  pendingSegMs = segMs;
  recordWitnessLeft = false;
  recordWitnessRight = false;
  pendingRecordStartMs = millis();
}

void trainingRecordAccumulateWitness()
{
  if (sensors.v[0] || sensors.v[1]) {
    recordWitnessLeft = true;
  }
  if (sensors.v[3] || sensors.v[4]) {
    recordWitnessRight = true;
  }
}

void finalizePendingTrainingRecord(bool force)
{
  if (!pendingTrainingRecord) {
    return;
  }
  if (!force && (millis() - pendingRecordStartMs < RECORD_RESOLVE_MS)) {
    return;
  }

  uint8_t bestMask = pendingPrimaryMask;
  int bestScore = -1;
  uint8_t bestVotes = 0;

  for (uint8_t i = 0; i < candCount; i++) {
    uint8_t m = candMask[i];
    if (turnFromMaskLHR(m) != pendingTurn) {
      continue;
    }

    int score = (int)candVotes[i] * 3;
    bool L = (m & 0x01) != 0;
    bool S = (m & 0x02) != 0;
    bool R = (m & 0x04) != 0;
    if (L && recordWitnessLeft) {
      score += 2;
    }
    if (R && recordWitnessRight) {
      score += 2;
    }
    if (S && (recordWitnessLeft || recordWitnessRight)) {
      score += 1;
    }

    if (score > bestScore ||
        (score == bestScore && candVotes[i] > bestVotes)) {
      bestScore = score;
      bestVotes = candVotes[i];
      bestMask = m;
    }
  }

  String msg = String("RECORD_RESOLVED mask=") +
               exitsMaskToString(bestMask) +
               " turn=" + String(pendingTurn) +
               " candN=" + String(candCount) +
               " primary=" + exitsMaskToString(pendingPrimaryMask);
  logLine(msg);

  recordNode(bestMask, pendingTurn, pendingSegMs);

  pendingTrainingRecord = false;
  candCount = 0;
}

void reportStatus()
{
  logLine(String("STATE=") + stateName(state));
  logLine(String("STAGE=") + currentStage);
  logLine(String("RAW_LEN=") + rawPathLen);
  logLine(String("OPT_LEN=") + optimalPathLen);
  logLine(String("SOLVE_INDEX=") + solveStepIndex);

  if (rawPathLen > 0) {
    logLine(String("RAW_PATH=") + rawPath);
  }
  if (optimalPathLen > 0) {
    logLine(String("OPTIMAL_PATH=") + optimalPath);
  }
}

void printTrainingSummary()
{
  logLine("=== TRAINING SUMMARY ===");
  logLine(String("RAW_PATH=") + rawPath);
  logLine(String("OPTIMAL_PATH=") + optimalPath);

  for (int i = 0; i < rawPathLen; i++) {
    String line = String(i) +
                  ": exits=" + exitsMaskToString(exitsRecorded[i]) +
                  ", turn=" + String(rawPath[i]) +
                  ", segMs=" + String(segmentTimeMs[i]);
    logLine(line);
  }
}

// ===================== EEPROM =====================
uint16_t calcEepromChecksum(const EepromMazeData &d)
{
  uint32_t sum = 0;
  sum += (uint16_t)(d.magic & 0xFFFF);
  sum += (uint16_t)((d.magic >> 16) & 0xFFFF);
  sum += d.version;
  sum += d.rawLen;
  sum += d.optLen;

  for (int i = 0; i < MAX_NODES + 1; i++) {
    sum += (uint8_t)d.raw[i];
    sum += (uint8_t)d.opt[i];
  }

  return (uint16_t)(sum & 0xFFFF);
}

bool savePathsToEEPROM()
{
  EepromMazeData data;
  memset(&data, 0, sizeof(data));

  data.magic   = EEPROM_MAGIC;
  data.version = EEPROM_VERSION;
  data.rawLen  = rawPathLen;
  data.optLen  = optimalPathLen;

  strncpy(data.raw, rawPath, MAX_NODES);
  strncpy(data.opt, optimalPath, MAX_NODES);

  data.raw[MAX_NODES] = '\0';
  data.opt[MAX_NODES] = '\0';
  data.checksum = calcEepromChecksum(data);

  EEPROM.put(0, data);

  EepromMazeData verify;
  EEPROM.get(0, verify);

  return (verify.magic == EEPROM_MAGIC &&
          verify.version == EEPROM_VERSION &&
          verify.checksum == calcEepromChecksum(verify));
}

bool loadPathsFromEEPROM()
{
  EepromMazeData data;
  EEPROM.get(0, data);

  if (data.magic != EEPROM_MAGIC) return false;
  if (data.version != EEPROM_VERSION) return false;
  if (data.rawLen > MAX_NODES) return false;
  if (data.optLen > MAX_NODES) return false;
  if (data.checksum != calcEepromChecksum(data)) return false;

  for (int i = 0; i < data.rawLen; i++) {
    if (!validTurnChar(data.raw[i])) return false;
  }
  for (int i = 0; i < data.optLen; i++) {
    if (!validTurnChar(data.opt[i])) return false;
  }

  clearRamPaths();

  rawPathLen = data.rawLen;
  optimalPathLen = data.optLen;

  memcpy(rawPath, data.raw, MAX_NODES + 1);
  memcpy(optimalPath, data.opt, MAX_NODES + 1);

  rawPath[rawPathLen] = '\0';
  optimalPath[optimalPathLen] = '\0';

  solveStepIndex = 0;
  return true;
}

void clearEEPROMPaths()
{
  EepromMazeData data;
  memset(&data, 0, sizeof(data));
  EEPROM.put(0, data);
}

// ===================== MOTOR HELPERS =====================
void setLeft(int pwm)
{
  if (pwm > 0) {
    digitalWrite(L_IN1, HIGH);
    digitalWrite(L_IN2, LOW);
    analogWrite(EN_LEFT, pwm);
  } else if (pwm < 0) {
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, HIGH);
    analogWrite(EN_LEFT, -pwm);
  } else {
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, LOW);
    analogWrite(EN_LEFT, 0);
  }
}

void setRight(int pwm)
{
  if (pwm > 0) {
    digitalWrite(R_IN3, LOW);
    digitalWrite(R_IN4, HIGH);
    analogWrite(EN_RIGHT, pwm);
  } else if (pwm < 0) {
    digitalWrite(R_IN3, HIGH);
    digitalWrite(R_IN4, LOW);
    analogWrite(EN_RIGHT, -pwm);
  } else {
    digitalWrite(R_IN3, LOW);
    digitalWrite(R_IN4, LOW);
    analogWrite(EN_RIGHT, 0);
  }
}

void drive(int leftPwm, int rightPwm)
{
  setLeft(leftPwm);
  setRight(rightPwm);
}

void stopMotors()          { drive(0, 0); }
void driveForward(int pwm) { drive(pwm, pwm); }
void spinLeft(int pwm)     { drive(-pwm, pwm); }
void spinRight(int pwm)    { drive(pwm, -pwm); }

// ===================== SENSOR HELPERS =====================
uint8_t sensorOnLine(uint8_t pin)
{
  return (digitalRead(pin) == SENSOR_LINE_LOW) ? 1 : 0;
}

void readSensors()
{
  sensors.onCount = 0;
  for (int i = 0; i < 5; i++) {
    sensors.v[i] = sensorOnLine(SENS_PINS[i]);
    sensors.onCount += sensors.v[i];
  }

  sensors.allBlack = (sensors.onCount == 5);
  sensors.noLine   = (sensors.onCount == 0);
}

bool isPerfectCenterPattern()
{
  return sensors.v[2] &&
         !sensors.v[0] &&
         !sensors.v[1] &&
         !sensors.v[3] &&
         !sensors.v[4];
}

bool nodeCandidateNow()
{
  if (sensors.allBlack) return true;

  bool outerWithBody = (sensors.v[0] || sensors.v[4]) &&
                       (sensors.v[1] || sensors.v[2] || sensors.v[3]);

  bool wideCenter = sensors.v[1] && sensors.v[2] && sensors.v[3];

  return outerWithBody || wideCenter;
}

// ===================== PATH OPTIMIZATION =====================
char selectTurn(bool foundLeft, bool foundStraight, bool foundRight)
{
  if (foundLeft)     return 'L';
  if (foundStraight) return 'S';
  if (foundRight)    return 'R';
  return 'B';
}

int turnAngle(char t)
{
  switch (t) {
    case 'L': return 270;
    case 'S': return 0;
    case 'R': return 90;
    case 'B': return 180;
    default:  return 0;
  }
}

char angleToTurn(int angle)
{
  angle %= 360;
  if (angle < 0) angle += 360;

  switch (angle) {
    case 0:   return 'S';
    case 90:  return 'R';
    case 180: return 'B';
    case 270: return 'L';
    default:  return 'S';
  }
}

void buildOptimalPath()
{
  optimalPathLen = 0;
  optimalPath[0] = '\0';

  for (int i = 0; i < rawPathLen; i++) {
    if (optimalPathLen >= MAX_NODES) break;

    optimalPath[optimalPathLen++] = rawPath[i];
    optimalPath[optimalPathLen] = '\0';

    while (optimalPathLen >= 3 && optimalPath[optimalPathLen - 2] == 'B') {
      int totalAngle =
        turnAngle(optimalPath[optimalPathLen - 3]) +
        turnAngle(optimalPath[optimalPathLen - 2]) +
        turnAngle(optimalPath[optimalPathLen - 1]);

      char reduced = angleToTurn(totalAngle);

      optimalPathLen -= 3;
      optimalPath[optimalPathLen++] = reduced;
      optimalPath[optimalPathLen] = '\0';
    }
  }
}

void recordNode(uint8_t exitsMask, char turnTaken, unsigned int segMs)
{
  if (rawPathLen >= MAX_NODES) return;

  rawPath[rawPathLen] = turnTaken;
  exitsRecorded[rawPathLen] = exitsMask;
  segmentTimeMs[rawPathLen] = segMs;
  rawPathLen++;
  rawPath[rawPathLen] = '\0';

  segmentStartMs = millis();

  String msg = String("NODE ") +
               String(rawPathLen - 1) +
               " exits=" + exitsMaskToString(exitsMask) +
               " turn=" + String(turnTaken) +
               " segMs=" + String(segMs);

  logLine(msg);
}

bool turnExists(char requested, bool foundLeft, bool foundStraight, bool foundRight)
{
  switch (requested) {
    case 'L': return foundLeft;
    case 'S': return foundStraight;
    case 'R': return foundRight;
    case 'B': return true;
    default:  return false;
  }
}

// ===================== ALIGNMENT =====================
void alignToCenterStrict(unsigned long timeoutMs = ALIGN_TIMEOUT_MS)
{
  unsigned long t0 = millis();
  unsigned long stableStart = 0;

  while (millis() - t0 < timeoutMs) {
    readSensors();

    if (isPerfectCenterPattern()) {
      if (stableStart == 0) stableStart = millis();
      stopMotors();

      if (millis() - stableStart >= ALIGN_STABLE_MS) {
        return;
      }
      continue;
    }

    stableStart = 0;

    if ((sensors.v[0] || sensors.v[1]) && !sensors.v[3] && !sensors.v[4]) {
      lastSteer = -1;
      spinLeft(ALIGN_SPEED);
    }
    else if ((sensors.v[3] || sensors.v[4]) && !sensors.v[0] && !sensors.v[1]) {
      lastSteer = 1;
      spinRight(ALIGN_SPEED);
    }
    else if (sensors.v[2] && sensors.v[1] && !sensors.v[3]) {
      lastSteer = -1;
      spinLeft(ALIGN_SPEED);
    }
    else if (sensors.v[2] && sensors.v[3] && !sensors.v[1]) {
      lastSteer = 1;
      spinRight(ALIGN_SPEED);
    }
    else if (sensors.noLine) {
      if (lastSteer <= 0) spinLeft(ALIGN_SPEED);
      else                spinRight(ALIGN_SPEED);
    }
    else {
      if (lastSteer <= 0) spinLeft(ALIGN_SPEED);
      else                spinRight(ALIGN_SPEED);
    }
  }

  stopMotors();
}

void alignThenMove()
{
  alignToCenterStrict();

  stopMotors();
  delay(POST_TURN_SETTLE_MS);

  readSensors();

  if (!isPerfectCenterPattern()) {
    return;
  }

  unsigned long t0 = millis();
  while (millis() - t0 < POST_TURN_FORWARD_MS) {
    readSensors();

    if (isPerfectCenterPattern() ||
        (sensors.v[2] && !sensors.v[0] && !sensors.v[4])) {
      driveForward(POST_TURN_SPEED);
    } else {
      break;
    }
  }

  stopMotors();
}

// ===================== TURNING =====================
void doTurn(char dir)
{
  unsigned long t0 = millis();
  bool centerWentOff = false;

  if (dir == 'S') {
    alignThenMove();
    return;
  }

  if (dir == 'L') {
    lastSteer = -1;

    while (millis() - t0 < TURN_TIMEOUT_MS) {
      readSensors();
      spinLeft(TURN_SPEED);

      if (!sensors.v[2]) centerWentOff = true;

      if (centerWentOff && sensors.v[2] && !sensors.allBlack) {
        break;
      }
    }

    stopMotors();
    alignThenMove();
    return;
  }

  if (dir == 'R') {
    lastSteer = 1;

    while (millis() - t0 < TURN_TIMEOUT_MS) {
      readSensors();
      spinRight(TURN_SPEED);

      if (!sensors.v[2]) centerWentOff = true;

      if (centerWentOff && sensors.v[2] && !sensors.allBlack) {
        break;
      }
    }

    stopMotors();
    alignThenMove();
    return;
  }

  if (dir == 'B') {
    lastSteer = 1;

    while (millis() - t0 < UTURN_TIMEOUT_MS) {
      readSensors();
      spinRight(UTURN_SPEED);

      if (millis() - t0 > 180 && sensors.v[2] && !sensors.allBlack) {
        break;
      }
    }

    stopMotors();
    alignThenMove();
    return;
  }
}

// ===================== DEAD-END RECOVERY =====================
bool persistentNoLine()
{
  return sensors.noLine &&
         (millis() - lastLineSeenMs >= LOST_LINE_TIMEOUT_MS);
}

void deadEndProbeForward()
{
  unsigned long t0 = millis();

  while (millis() - t0 < DEAD_END_PROBE_MS) {
    driveForward(DEAD_END_PROBE_SPEED);
    readSensors();
  }

  stopMotors();
}

void handleTrainingDeadEndRecovery()
{
  stopMotors();
  setStage("TRAINING_DEAD_END");

  unsigned int segMs = (unsigned int)(millis() - segmentStartMs);

  logLine("DEAD_END_DETECTED");
  logLine(String("DEAD_END_RECOVERY_NO_PATH segMs=") + String(segMs));
  recordNode(0, 'B', segMs);

  deadEndProbeForward();
  doTurn('B');

  segmentStartMs = millis();

  lastLineSeenMs = millis();
  lastNodeHandledMs = millis();
  setStage("TRAINING_FOLLOW_LINE");
}

void handleSolvingDeadEndRecovery()
{
  stopMotors();
  setStage("SOLVING_DEAD_END");

  logLine("UNEXPECTED_DEAD_END_DETECTED");

  deadEndProbeForward();
  doTurn('B');

  lastLineSeenMs = millis();
  lastNodeHandledMs = millis();
  setStage("SOLVING_FOLLOW_LINE");
}

// ===================== NO-PID LINE FOLLOW =====================
void followLineNoPID()
{
  readSensors();

  if (!sensors.noLine) {
    lastLineSeenMs = millis();
  }

  if (isPerfectCenterPattern()) {
    lastSteer = 0;
    driveForward(BASE_SPEED);
    return;
  }

  if (sensors.v[2] && sensors.v[1] && !sensors.v[3]) {
    lastSteer = -1;
    spinLeft(ALIGN_SPEED);
    return;
  }

  if (sensors.v[2] && sensors.v[3] && !sensors.v[1]) {
    lastSteer = 1;
    spinRight(ALIGN_SPEED);
    return;
  }

  if (sensors.v[0] || (sensors.v[1] && !sensors.v[2])) {
    lastSteer = -1;
    spinLeft(TURN_SPEED);
    return;
  }

  if (sensors.v[4] || (sensors.v[3] && !sensors.v[2])) {
    lastSteer = 1;
    spinRight(TURN_SPEED);
    return;
  }

  if (sensors.noLine) {
    if (millis() - lastLineSeenMs < LOST_LINE_TIMEOUT_MS) {
      stopMotors();
    } else {
      if (lastSteer <= 0) spinLeft(SEARCH_SPEED);
      else                spinRight(SEARCH_SPEED);
    }
    return;
  }

  if (sensors.v[2]) {
    stopMotors();
    return;
  }

  if (lastSteer <= 0) spinLeft(SEARCH_SPEED);
  else                spinRight(SEARCH_SPEED);
}

// ===================== FINISH DETECTION =====================
bool confirmEndBox()
{
  unsigned long t0 = millis();

  while (millis() - t0 < END_CONFIRM_MS) {
    driveForward(PROBE_SPEED);
    readSensors();
    if (!sensors.allBlack) return false;
  }

  stopMotors();
  return true;
}

// ===================== PHASE CONTROL =====================
void onTrainingComplete()
{
  stopMotors();
  setStage("TRAINING_COMPLETE");

  finalizePendingTrainingRecord(true);

  buildOptimalPath();
  bool saved = savePathsToEEPROM();

  state = READY_TO_SOLVE;
  solveStepIndex = 0;

  logLine("TRAINING_DONE");
  logLine(String("RAW_PATH=") + rawPath);
  logLine(String("OPTIMAL_PATH=") + optimalPath);
  logLine(saved ? "EEPROM_SAVE=OK" : "EEPROM_SAVE=FAILED");

  printTrainingSummary();
  logLine("READY_FOR_SOLVE: send START");
}

void onSolvingComplete()
{
  stopMotors();
  state = READY_TO_SOLVE;
  solveStepIndex = 0;
  setStage("SOLVING_COMPLETE");

  logLine("SOLVING_DONE");
  logLine(String("FINAL_OPTIMAL_PATH=") + optimalPath);
  logLine("READY_FOR_SOLVE: send START again to rerun");
}

// ===================== INTERSECTION INSPECTION =====================
void inspectAndHandleNodeTraining()
{
  finalizePendingTrainingRecord(true);

  setStage("TRAINING_NODE_CHECK");

  bool burstL[PROBE_BURST_N];
  bool burstR[PROBE_BURST_N];
  bool burstS[PROBE_BURST_N];

  driveForward(PROBE_SPEED);
  delay(PROBE_ENTRY_MS);

  readSensors();
  bool entryAllBlack = sensors.allBlack;
  burstL[0] = sensors.v[0] || sensors.v[1];
  burstR[0] = sensors.v[3] || sensors.v[4];
  for (uint8_t i = 1; i < PROBE_BURST_N; i++) {
    readSensors();
    burstL[i] = sensors.v[0] || sensors.v[1];
    burstR[i] = sensors.v[3] || sensors.v[4];
  }

  bool foundLeft  = false;
  bool foundRight = false;
  for (uint8_t i = 0; i < PROBE_BURST_N; i++) {
    foundLeft = foundLeft || burstL[i];
    foundRight = foundRight || burstR[i];
  }

  if (entryAllBlack) {
    if (confirmEndBox()) {
      onTrainingComplete();
      return;
    }
  }

  driveForward(PROBE_SPEED);
  delay(PROBE_STRAIGHT_MS);

  readSensors();
  bool straightAllBlack = sensors.allBlack;
  burstS[0] = sensors.v[1] || sensors.v[2] || sensors.v[3];
  for (uint8_t j = 1; j < PROBE_BURST_N; j++) {
    readSensors();
    burstS[j] = sensors.v[1] || sensors.v[2] || sensors.v[3];
  }

  bool foundStraight = false;
  for (uint8_t j = 0; j < PROBE_BURST_N; j++) {
    foundStraight = foundStraight || burstS[j];
  }

  if (straightAllBlack) {
    if (confirmEndBox()) {
      onTrainingComplete();
      return;
    }
  }

  buildExitCandidatesFromBursts(burstL, burstR, burstS, PROBE_BURST_N);

  uint8_t exitsMask = 0;
  if (foundLeft) {
    exitsMask |= 0x01;
  }
  if (foundStraight) {
    exitsMask |= 0x02;
  }
  if (foundRight) {
    exitsMask |= 0x04;
  }

  char turnTaken = selectTurn(foundLeft, foundStraight, foundRight);
  int choices = (foundLeft ? 1 : 0) + (foundStraight ? 1 : 0) + (foundRight ? 1 : 0);

  unsigned int segMs = (unsigned int)(millis() - segmentStartMs);

  if (choices >= 2) {
    logLine(String("RECORD_DEFER candidates=") + String(candCount) +
            " primary=" + exitsMaskToString(exitsMask) +
            " turn=" + String(turnTaken));

    doTurn(turnTaken);
    startPendingTrainingRecord(exitsMask, turnTaken, segMs);
  } else if (choices == 1) {
    logLine(String("FORCED_TURN=") + turnTaken + " exits=" + exitsMaskToString(exitsMask));
    doTurn(turnTaken);
  } else {
    logLine(String("PROBE_ZERO_EXITS_UNRECORDED segMs=") + String(segMs));
    doTurn(turnTaken);
    segmentStartMs = millis();
  }

  lastLineSeenMs = millis();
  lastNodeHandledMs = millis();
  setStage("TRAINING_FOLLOW_LINE");
}

void inspectAndHandleNodeSolving()
{
  setStage("SOLVING_NODE_CHECK");

  driveForward(PROBE_SPEED);
  delay(PROBE_ENTRY_MS);
  readSensors();

  bool foundLeft  = sensors.v[0] || sensors.v[1];
  bool foundRight = sensors.v[3] || sensors.v[4];

  if (sensors.allBlack) {
    if (confirmEndBox()) {
      onSolvingComplete();
      return;
    }
  }

  driveForward(PROBE_SPEED);
  delay(PROBE_STRAIGHT_MS);
  readSensors();

  bool foundStraight = sensors.v[1] || sensors.v[2] || sensors.v[3];

  if (sensors.allBlack) {
    if (confirmEndBox()) {
      onSolvingComplete();
      return;
    }
  }

  uint8_t exitsMask = 0;
  if (foundLeft)     exitsMask |= 0x01;
  if (foundStraight) exitsMask |= 0x02;
  if (foundRight)    exitsMask |= 0x04;

  int choices = (foundLeft ? 1 : 0) + (foundStraight ? 1 : 0) + (foundRight ? 1 : 0);
  bool realDecisionNode = (choices >= 2 || choices == 0);

  char turnTaken;

  if (realDecisionNode) {
    if (solveStepIndex < optimalPathLen) {
      char requested = optimalPath[solveStepIndex];
      solveStepIndex++;

      if (!turnExists(requested, foundLeft, foundStraight, foundRight)) {
        logLine(String("WARNING_PATH_MISMATCH at step ") +
                String(solveStepIndex - 1) +
                " requested=" + String(requested) +
                " exits=" + exitsMaskToString(exitsMask));

        turnTaken = selectTurn(foundLeft, foundStraight, foundRight);
      } else {
        turnTaken = requested;
      }

      logLine(String("SOLVE_NODE step=") +
              String(solveStepIndex) + "/" + String(optimalPathLen) +
              " exits=" + exitsMaskToString(exitsMask) +
              " turn=" + String(turnTaken));
    } else {
      turnTaken = selectTurn(foundLeft, foundStraight, foundRight);

      logLine(String("WARNING_NO_MORE_OPT_STEPS exits=") +
              exitsMaskToString(exitsMask) +
              " fallback=" + String(turnTaken));
    }
  } else {
    turnTaken = selectTurn(foundLeft, foundStraight, foundRight);
    logLine(String("SOLVE_FORCED_TURN=") + turnTaken +
            " exits=" + exitsMaskToString(exitsMask));
  }

  doTurn(turnTaken);
  lastLineSeenMs = millis();
  lastNodeHandledMs = millis();
  setStage("SOLVING_FOLLOW_LINE");
}

// ===================== START AREA HANDLING =====================
bool leaveStartAreaAndFindLine(RobotState nextState)
{
  driveForward(START_SPEED);
  unsigned long t0 = millis();

  while (millis() - t0 < START_EXIT_TIMEOUT_MS) {
    readSensors();

    if (!sensors.allBlack && (sensors.v[1] || sensors.v[2] || sensors.v[3])) {
      stopMotors();
      alignToCenterStrict();

      segmentStartMs = millis();
      lastLineSeenMs = millis();
      lastNodeHandledMs = millis();
      startBoxSeenMs = 0;

      state = nextState;

      if (nextState == TRAINING) {
        setStage("TRAINING_FOLLOW_LINE");
        logLine("TRAINING_STARTED");
      } else {
        setStage("SOLVING_FOLLOW_LINE");
        logLine("SOLVING_STARTED");
        logLine(String("USING_OPTIMAL_PATH=") + optimalPath);
      }

      return true;
    }
  }

  stopMotors();
  startBoxSeenMs = 0;
  logLine("Could not leave start area cleanly");
  return false;
}

void handleArmTraining()
{
  readSensors();

  if (sensors.allBlack) {
    if (startBoxSeenMs == 0) {
      startBoxSeenMs = millis();
      logLine("TRAIN_START_BOX_DETECTED");
    }

    if (millis() - startBoxSeenMs >= START_BOX_CONFIRM_MS) {
      leaveStartAreaAndFindLine(TRAINING);
    }
  } else {
    startBoxSeenMs = 0;
  }
}

void handleArmSolving()
{
  readSensors();

  if (sensors.allBlack) {
    if (startBoxSeenMs == 0) {
      startBoxSeenMs = millis();
      logLine("SOLVE_START_BOX_DETECTED");
    }

    if (millis() - startBoxSeenMs >= START_BOX_CONFIRM_MS) {
      leaveStartAreaAndFindLine(SOLVING);
    }
  } else {
    startBoxSeenMs = 0;
  }
}

// ===================== TRAINING / SOLVING =====================
void handleTraining()
{
  readSensors();

  if (pendingTrainingRecord) {
    trainingRecordAccumulateWitness();
    finalizePendingTrainingRecord(false);
  }

  if (persistentNoLine()) {
    if (millis() - lastNodeHandledMs >= DEAD_END_GRACE_AFTER_NODE_MS) {
      finalizePendingTrainingRecord(true);
      handleTrainingDeadEndRecovery();
      return;
    }
  }

  if (millis() - lastNodeHandledMs >= NODE_LOCKOUT_MS && nodeCandidateNow()) {
    finalizePendingTrainingRecord(true);
    stopMotors();
    inspectAndHandleNodeTraining();
    return;
  }

  followLineNoPID();
}

void handleSolving()
{
  readSensors();

  if (persistentNoLine()) {
    handleSolvingDeadEndRecovery();
    return;
  }

  if (millis() - lastNodeHandledMs >= NODE_LOCKOUT_MS && nodeCandidateNow()) {
    stopMotors();
    inspectAndHandleNodeSolving();
    return;
  }

  followLineNoPID();
}

// ===================== LED INDICATION =====================
void updateLedPattern()
{
  switch (state) {
    case WAIT_COMMAND:
      digitalWrite(LED_PIN, LOW);
      break;

    case ARM_TRAINING:
      digitalWrite(LED_PIN, (millis() / 300) % 2);
      break;

    case TRAINING:
      digitalWrite(LED_PIN, HIGH);
      break;

    case READY_TO_SOLVE:
      digitalWrite(LED_PIN, (millis() / 120) % 2);
      break;

    case ARM_SOLVING:
      digitalWrite(LED_PIN, (millis() / 150) % 2);
      break;

    case SOLVING:
      digitalWrite(LED_PIN, (millis() / 60) % 2);
      break;
  }
}

// ===================== COMMANDS =====================
void normalizeCommand(char *cmd)
{
  int w = 0;
  for (int i = 0; cmd[i] != '\0'; i++) {
    char c = cmd[i];
    if (c == ' ' || c == '\t') continue;
    cmd[w++] = toupper((unsigned char)c);
  }
  cmd[w] = '\0';
}

void armTrainingPhase()
{
  stopMotors();
  clearRamPaths();
  state = ARM_TRAINING;
  startBoxSeenMs = 0;
  setStage("WAITING_FOR_TRAIN_START_BOX");
  logLine("CMD=TRAIN");
  logLine("Place robot on start box");
}

void armSolvingPhase()
{
  stopMotors();

  if (optimalPathLen <= 0) {
    if (!loadPathsFromEEPROM()) {
      logLine("ERROR: no valid saved optimal path in EEPROM");
      return;
    }
  }

  solveStepIndex = 0;
  state = ARM_SOLVING;
  startBoxSeenMs = 0;
  setStage("WAITING_FOR_SOLVE_START_BOX");
  logLine("CMD=START");
  logLine(String("OPTIMAL_PATH=") + optimalPath);
  logLine("Place robot on start box");
}

void stopAndIdle()
{
  stopMotors();
  state = WAIT_COMMAND;
  setStage("IDLE");
  logLine("STOPPED");
}

void processCommand(char *cmd)
{
  normalizeCommand(cmd);
  if (cmd[0] == '\0') return;

  if (strcmp(cmd, "TRAIN") == 0) {
    armTrainingPhase();
    return;
  }

  if (strcmp(cmd, "START") == 0 || strcmp(cmd, "SOLVE") == 0) {
    armSolvingPhase();
    return;
  }

  if (strcmp(cmd, "STATUS") == 0) {
    reportStatus();
    return;
  }

  if (strcmp(cmd, "DUMP") == 0) {
    printTrainingSummary();
    return;
  }

  if (strcmp(cmd, "LOAD") == 0) {
    if (loadPathsFromEEPROM()) {
      state = READY_TO_SOLVE;
      setStage("PATH_LOADED_FROM_EEPROM");
      logLine("EEPROM_LOAD=OK");
      logLine(String("RAW_PATH=") + rawPath);
      logLine(String("OPTIMAL_PATH=") + optimalPath);
    } else {
      logLine("EEPROM_LOAD=FAILED");
    }
    return;
  }

  if (strcmp(cmd, "CLEAR") == 0) {
    clearRamPaths();
    clearEEPROMPaths();
    state = WAIT_COMMAND;
    setStage("IDLE");
    logLine("RAM_AND_EEPROM_CLEARED");
    return;
  }

  if (strcmp(cmd, "STOP") == 0) {
    stopAndIdle();
    return;
  }

  logLine(String("UNKNOWN_COMMAND=") + cmd);
}

void handleCommandStream(Stream &port, char *buffer, uint8_t &indexRef)
{
  while (port.available()) {
    char c = (char)port.read();

    if (c == '\r') continue;

    if (c == '\n') {
      buffer[indexRef] = '\0';
      if (indexRef > 0) processCommand(buffer);
      indexRef = 0;
      continue;
    }

    if (indexRef < (CMD_BUF_SIZE - 1)) {
      buffer[indexRef++] = c;
    } else {
      indexRef = 0;
    }
  }
}

void pollCommands()
{
  handleCommandStream(BT, btCmdBuf, btCmdIdx);
  handleCommandStream(Serial, usbCmdBuf, usbCmdIdx);
}

// ===================== SETUP =====================
void setupPins()
{
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_IN3, OUTPUT);
  pinMode(R_IN4, OUTPUT);
  pinMode(EN_LEFT, OUTPUT);
  pinMode(EN_RIGHT, OUTPUT);

  for (int i = 0; i < 5; i++) {
    pinMode(SENS_PINS[i], INPUT);
  }

  pinMode(LED_PIN, OUTPUT);
}

void setup()
{
  Serial.begin(USB_BAUD);
  BT.begin(BT_BAUD);

  setupPins();
  stopMotors();
  clearRamPaths();

  digitalWrite(LED_PIN, LOW);
  setStage("IDLE");

  logLine("Maze trainer + solver - LEFT HAND RULE - NO PID");
  logLine("Bluetooth commands: TRAIN, START, STATUS, DUMP, LOAD, CLEAR, STOP");

  if (loadPathsFromEEPROM()) {
    state = READY_TO_SOLVE;
    setStage("EEPROM_PATH_READY");
    logLine("Saved path found in EEPROM");
    logLine(String("RAW_PATH=") + rawPath);
    logLine(String("OPTIMAL_PATH=") + optimalPath);
  } else {
    state = WAIT_COMMAND;
    setStage("IDLE");
    logLine("No valid EEPROM path stored yet");
  }
}

// ===================== LOOP =====================
void loop()
{
  pollCommands();
  updateLedPattern();

  switch (state) {
    case WAIT_COMMAND:
      stopMotors();
      break;

    case ARM_TRAINING:
      handleArmTraining();
      break;

    case TRAINING:
      handleTraining();
      break;

    case READY_TO_SOLVE:
      stopMotors();
      break;

    case ARM_SOLVING:
      handleArmSolving();
      break;

    case SOLVING:
      handleSolving();
      break;
  }
}