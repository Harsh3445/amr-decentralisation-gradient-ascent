// =====================================================
//  ESP-B  —  ROBOT  (coordinate go-to, timed moves)
//  Receives a target (x,y) from ESP-A, plans a path with A*,
//  drives it, and reports progress back to ESP-A.
// =====================================================
#include <WiFi.h>
#include <esp_now.h>
#include <stdarg.h>

// ---------- MOTOR PINS ----------
#define IN1 27
#define IN2 26
#define IN3 25
#define IN4 33

// ---------- MOTOR SIDES ----------
// Your robot turned LEFT when told RIGHT, so the IN1/IN2 motor is really
// the RIGHT wheel. SWAP_SIDES = true fixes that: R now means real right.
const bool SWAP_SIDES   = true;
const bool INVERT_LEFT  = false;   // true if left wheel runs backward on forward
const bool INVERT_RIGHT = false;   // true if right wheel runs backward on forward

// ---------- ESP-A MAC ----------
uint8_t espAMac[] = {0x20, 0x9B, 0xA9, 0x60, 0xE3, 0xF4};

// ---------- YOUR ROBOT MEASUREMENTS ----------
const float WHEEL_DIAMETER_CM = 6.5;
const float WHEEL_BASE_CM     = 14.0;    // distance between the two wheels
const float MOTOR_RPM         = 63.25;   // measured: 1265 pulses / 60 s / 20 PPR
const float CELL_CM           = 10.0;    // 1 grid unit = 10 cm

// ---------- CALIBRATION (1.0 = pure calculation) ----------
// Robot goes too far  -> make DRIVE_FACTOR smaller (e.g. 0.9)
// Robot turns too far -> make TURN_FACTOR smaller
const float DRIVE_FACTOR = 1.0;
const float TURN_FACTOR  = 1.0;

// ---------- CALCULATED TIMINGS ----------
const float SPEED_CM_S  = MOTOR_RPM / 60.0 * PI * WHEEL_DIAMETER_CM;   // ~21.5 cm/s
const float TURN_ARC_CM = PI * WHEEL_BASE_CM / 4.0;                    // ~11.0 cm per 90 deg
const unsigned long CELL_MS   = CELL_CM / SPEED_CM_S * 1000.0 * DRIVE_FACTOR;    // ~465 ms
const unsigned long TURN90_MS = TURN_ARC_CM / SPEED_CM_S * 1000.0 * TURN_FACTOR; // ~511 ms
const unsigned long PAUSE_MS  = 400;     // short stop between moves

// ---------- GRID ----------
// (0,0) = start corner. x grows to the RIGHT, y grows FORWARD.
// Heading: 0 = +Y (forward), 1 = +X (right), 2 = -Y, 3 = -X
const int GRID_W = 10;                  // 10 x 10 cells = 1 m x 1 m
const int GRID_H = 10;
const int CELLS  = GRID_W * GRID_H;
const int STATES = CELLS * 4;           // each cell x 4 headings
const int TURN_COST = 1;                // makes A* prefer fewer turns

const int DX[4] = {0, 1, 0, -1};
const int DY[4] = {1, 0, -1, 0};
const char *DIR_NAME[4] = {"+Y", "+X", "-Y", "-X"};

bool blocked[GRID_W][GRID_H] = {};
int posX = 0, posY = 0, heading = 0;

int pathX[CELLS], pathY[CELLS];
int pathLen = 0, pathIndex = 0;

// A* working memory
static int  gCost[STATES], parentState[STATES];
static bool openSet[STATES], closedSet[STATES];
static int  chain[STATES];

// ---------- MESSAGES (must match ESP-A) ----------
typedef struct {
  char command;   // G = go to, H = home, S = stop, X = obstacle, W = where, C/Q = calibrate
  int8_t x;
  int8_t y;
} RobotCommand;

typedef struct {
  char text[80];
} RobotMessage;

volatile bool newCommand = false;
volatile char pendingCmd = 'S';
volatile int8_t pendingX = 0, pendingY = 0;

// ---------- MOVEMENT STATE ----------
enum Step { IDLE, DRIVING, TURNING, PAUSED, CALIBRATING };
Step step = IDLE;
bool navigating = false;
unsigned long stepStartedAt = 0, stepDuration = 0;
int turnTarget = 0;
int driveCells = 0;

esp_now_peer_info_t peerInfo;

// ---------- REPORT: print here AND send to ESP-A ----------
void report(const char *fmt, ...) {
  RobotMessage m;
  va_list args;
  va_start(args, fmt);
  vsnprintf(m.text, sizeof(m.text), fmt, args);
  va_end(args);
  Serial.println(m.text);
  esp_now_send(espAMac, (uint8_t *)&m, sizeof(m));
}

// ---------- RECEIVE ----------
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(RobotCommand)) return;
  RobotCommand c;
  memcpy(&c, data, sizeof(c));
  pendingCmd = c.command;
  pendingX = c.x;
  pendingY = c.y;
  newCommand = true;
}

// ---------- MOTORS ----------
void driveOne(int in1, int in2, int dir) {     // dir: 1 forward, -1 backward, 0 stop
  digitalWrite(in1, dir > 0 ? HIGH : LOW);
  digitalWrite(in2, dir < 0 ? HIGH : LOW);
}

void setWheels(int left, int right) {
  if (INVERT_LEFT)  left = -left;
  if (INVERT_RIGHT) right = -right;
  if (!SWAP_SIDES) {
    driveOne(IN1, IN2, left);
    driveOne(IN3, IN4, right);
  } else {
    driveOne(IN1, IN2, right);
    driveOne(IN3, IN4, left);
  }
}

void motors(char a) {
  switch (a) {
    case 'F': setWheels( 1,  1); break;
    case 'R': setWheels( 1, -1); break;   // spin right in place
    case 'L': setWheels(-1,  1); break;   // spin left in place
    default:  setWheels( 0,  0); break;
  }
}

void startStep(Step s, char action, unsigned long duration) {
  step = s;
  stepStartedAt = millis();
  stepDuration = duration;
  motors(action);
}

// =====================================================
//  A* PATH PLANNER
//  State = (cell, heading). Moving one cell costs 1,
//  a 90-degree turn costs TURN_COST -> shortest path
//  with the fewest turns.
// =====================================================
bool planPath(int sx, int sy, int sh, int gx, int gy) {
  if (blocked[gx][gy]) return false;

  for (int i = 0; i < STATES; i++) {
    gCost[i] = 30000; parentState[i] = -1;
    openSet[i] = false; closedSet[i] = false;
  }

  int start = (sy * GRID_W + sx) * 4 + sh;
  gCost[start] = 0;
  openSet[start] = true;
  int goalState = -1;

  while (true) {
    // open state with the lowest cost + distance to goal
    int cur = -1, best = 30000;
    for (int i = 0; i < STATES; i++) {
      if (!openSet[i]) continue;
      int c = i / 4;
      int f = gCost[i] + abs(c % GRID_W - gx) + abs(c / GRID_W - gy);
      if (f < best) { best = f; cur = i; }
    }
    if (cur == -1) return false;                 // no route

    int cell = cur / 4, h = cur % 4;
    int cx = cell % GRID_W, cy = cell / GRID_W;
    if (cx == gx && cy == gy) { goalState = cur; break; }

    openSet[cur] = false;
    closedSet[cur] = true;

    int next[3], cost[3], n = 0;
    int nx = cx + DX[h], ny = cy + DY[h];         // move forward one cell
    if (nx >= 0 && nx < GRID_W && ny >= 0 && ny < GRID_H && !blocked[nx][ny]) {
      next[n] = (ny * GRID_W + nx) * 4 + h; cost[n++] = 1;
    }
    next[n] = cell * 4 + (h + 1) % 4; cost[n++] = TURN_COST;   // turn right
    next[n] = cell * 4 + (h + 3) % 4; cost[n++] = TURN_COST;   // turn left

    for (int k = 0; k < n; k++) {
      int s = next[k];
      if (closedSet[s]) continue;
      if (gCost[cur] + cost[k] < gCost[s]) {
        gCost[s] = gCost[cur] + cost[k];
        parentState[s] = cur;
        openSet[s] = true;
      }
    }
  }

  // walk back to the start, keep only the cells visited
  int len = 0;
  for (int s = goalState; s != -1; s = parentState[s]) chain[len++] = s;
  pathLen = 0;
  int lastCell = start / 4;
  for (int i = len - 1; i >= 0; i--) {
    int c = chain[i] / 4;
    if (c != lastCell) {
      pathX[pathLen] = c % GRID_W;
      pathY[pathLen] = c / GRID_W;
      pathLen++;
      lastCell = c;
    }
  }
  return true;
}

// ---------- FOLLOW THE PATH ----------
int dirBetween(int x1, int y1, int x2, int y2) {
  if (y2 > y1) return 0;
  if (x2 > x1) return 1;
  if (y2 < y1) return 2;
  return 3;
}

void startNextMove() {
  if (pathIndex >= pathLen) {
    motors('S');
    step = IDLE;
    navigating = false;
    report("GOAL REACHED at (%d,%d) facing %s", posX, posY, DIR_NAME[heading]);
    return;
  }

  int need = dirBetween(posX, posY, pathX[pathIndex], pathY[pathIndex]);
  int diff = (need - heading + 4) % 4;
  turnTarget = need;

  if (diff == 0) {
    // drive all cells in a straight line in one go
    int run = 1;
    while (pathIndex + run < pathLen &&
           dirBetween(pathX[pathIndex + run - 1], pathY[pathIndex + run - 1],
                      pathX[pathIndex + run], pathY[pathIndex + run]) == need) {
      run++;
    }
    driveCells = run;
    int ex = pathX[pathIndex + run - 1], ey = pathY[pathIndex + run - 1];
    report("Forward %d cm -> (%d,%d)", (int)(run * CELL_CM), ex, ey);
    startStep(DRIVING, 'F', run * CELL_MS);
  } else if (diff == 1) {
    report("Turn RIGHT 90 -> facing %s", DIR_NAME[need]);
    startStep(TURNING, 'R', TURN90_MS);
  } else if (diff == 3) {
    report("Turn LEFT 90 -> facing %s", DIR_NAME[need]);
    startStep(TURNING, 'L', TURN90_MS);
  } else {
    report("Turn around 180 -> facing %s", DIR_NAME[need]);
    startStep(TURNING, 'R', TURN90_MS * 2);
  }
}

void updateMovement() {
  if (step == IDLE) return;
  if (millis() - stepStartedAt < stepDuration) return;

  if (step == CALIBRATING) {
    motors('S');
    step = IDLE;
    report("Calibration move done");
  } else if (step == DRIVING) {
    pathIndex += driveCells;
    posX = pathX[pathIndex - 1];
    posY = pathY[pathIndex - 1];
    startStep(PAUSED, 'S', PAUSE_MS);
  } else if (step == TURNING) {
    heading = turnTarget;
    startStep(PAUSED, 'S', PAUSE_MS);
  } else if (step == PAUSED) {
    startNextMove();
  }
}

// ---------- COMMANDS ----------
void handleCommand(char c, int x, int y) {
  switch (c) {

    case 'G':
      if (navigating || step != IDLE) {
        report("BUSY - wait, or send S first");
      } else if (x < 0 || x >= GRID_W || y < 0 || y >= GRID_H) {
        report("Target (%d,%d) outside grid 0-%d", x, y, GRID_W - 1);
      } else if (x == posX && y == posY) {
        report("Already at (%d,%d)", x, y);
      } else if (!planPath(posX, posY, heading, x, y)) {
        report("NO PATH to (%d,%d)", x, y);
      } else {
        report("Going (%d,%d) -> (%d,%d), %d cells", posX, posY, x, y, pathLen);
        navigating = true;
        pathIndex = 0;
        startNextMove();
      }
      break;

    case 'S':
      motors('S');
      step = IDLE;
      if (navigating) {
        navigating = false;
        report("STOPPED near (%d,%d). Place robot on a cell and send H if unsure.", posX, posY);
      } else {
        report("STOPPED");
      }
      break;

    case 'H':
      motors('S');
      step = IDLE;
      navigating = false;
      posX = 0; posY = 0; heading = 0;
      report("HOME set: (0,0) facing +Y");
      break;

    case 'W':
      report("At (%d,%d) facing %s", posX, posY, DIR_NAME[heading]);
      break;

    case 'X':
      if (x < 0 || x >= GRID_W || y < 0 || y >= GRID_H) {
        report("Cell outside grid");
      } else {
        blocked[x][y] = !blocked[x][y];
        report("Cell (%d,%d) %s", x, y, blocked[x][y] ? "BLOCKED" : "FREE");
      }
      break;

    case 'C':   // calibrate: one cell forward (position not changed)
      if (navigating) { report("BUSY"); break; }
      report("Calibrate: forward %d cm (%lu ms)", (int)CELL_CM, CELL_MS);
      startStep(CALIBRATING, 'F', CELL_MS);
      break;

    case 'Q':   // calibrate: 90 degree right turn (heading not changed)
      if (navigating) { report("BUSY"); break; }
      report("Calibrate: turn right 90 (%lu ms)", TURN90_MS);
      startStep(CALIBRATING, 'R', TURN90_MS);
      break;
  }
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  motors('S');

  WiFi.mode(WIFI_STA);
  WiFi.setChannel(1);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW INIT FAILED");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, espAMac, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ESP-A PEER ADD FAILED (check MAC)");
  }

  Serial.println("===== ESP-B ROBOT READY =====");
  Serial.printf("Speed %.1f cm/s | 1 cell = %lu ms | 90 turn = %lu ms\n",
                SPEED_CM_S, CELL_MS, TURN90_MS);
  Serial.println("Position (0,0) facing +Y");
}

// ---------- LOOP ----------
void loop() {
  if (newCommand) {
    newCommand = false;
    handleCommand(toupper(pendingCmd), pendingX, pendingY);
  }
  updateMovement();
}
