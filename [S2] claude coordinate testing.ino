#include <WiFi.h>
#include <esp_now.h>

// ===== ESP-B MAC ADDRESS =====
uint8_t ESP_B_MAC[] = {
  0x20, 0x9B, 0xA9, 0x61, 0xBB, 0x20
};

typedef struct {
  char command;
} Message;

Message message;
esp_now_peer_info_t peerInfo;

// Set by onAckReceived() whenever ESP-B confirms it got a pulse.
// volatile because it is written from the ESP-NOW callback, not the main loop.
volatile bool ackReceived = false;
volatile char lastAck = 0;

// ===================================================
// CALIBRATION CONSTANTS -- taken from YOUR measurements
// ===================================================
const float UNIT_TIME_S     = 0.5;   // seconds of forward pulse per 1 grid unit  (2 units -> 1.0s, 3 units -> 1.5s)
const float TURN_90_TIME_S  = 0.25;  // seconds of single-motor pulse per 90 degree turn
// ===================================================

const char *FACING_NAME[4] = { "EAST (+X)", "NORTH (+Y)", "WEST (-X)", "SOUTH (-Y)" };
#define EAST  0
#define NORTH 1
#define WEST  2
#define SOUTH 3

long currentX = 0;
long currentY = 0;
int  currentFacing = EAST;

struct RouteStep {
  char command;
  unsigned long durationMs;
  String label;
};

RouteStep pendingRoute[4];
int pendingRouteLen = 0;
long pendingTargetX, pendingTargetY;
int pendingFinalFacing;
bool routeWaitingConfirmation = false;

String inputLine = "";

// ---------------------------------------------------
// Called automatically whenever ESP-B sends anything back to us
// ---------------------------------------------------
void onAckReceived(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(Message)) return;
  Message incoming;
  memcpy(&incoming, data, sizeof(incoming));
  lastAck = incoming.command;
  ackReceived = true;
}

// ---------------------------------------------------
void sendCommand(char cmd) {
  message.command = cmd;
  esp_now_send(ESP_B_MAC, (uint8_t *)&message, sizeof(message));
}

// ---------------------------------------------------
int addTurnIfNeeded(int currentF, int desiredF) {
  if (currentF == desiredF) return currentF;

  int diffRight = ((desiredF - currentF) + 4) % 4;
  char turnCmd;
  int steps;

  if (diffRight <= 2) {
    turnCmd = 'R';
    steps = diffRight;
  } else {
    turnCmd = 'L';
    steps = 4 - diffRight;
  }

  unsigned long dur = (unsigned long)(steps * TURN_90_TIME_S * 1000.0);

  String label = "TURN ";
  label += (turnCmd == 'R') ? "RIGHT" : "LEFT";
  if (steps > 1) { label += " x"; label += steps; }

  pendingRoute[pendingRouteLen++] = { turnCmd, dur, label };
  return desiredF;
}

// ---------------------------------------------------
void planRoute(long targetX, long targetY) {
  long dx = targetX - currentX;
  long dy = targetY - currentY;

  pendingRouteLen = 0;
  int facing = currentFacing;

  Serial.println();
  Serial.println("==================== ROUTE PLAN ====================");
  Serial.print("From: (");
  Serial.print(currentX); Serial.print(", "); Serial.print(currentY);
  Serial.print(")  facing "); Serial.println(FACING_NAME[currentFacing]);
  Serial.print("To:   (");
  Serial.print(targetX); Serial.print(", "); Serial.print(targetY);
  Serial.println(")");
  Serial.println("-----------------------------------------------------");

  if (dx != 0) {
    int desired = (dx > 0) ? EAST : WEST;
    facing = addTurnIfNeeded(facing, desired);
    unsigned long fwdMs = (unsigned long)(fabs((float)dx) * UNIT_TIME_S * 1000.0);
    pendingRoute[pendingRouteLen++] = { 'F', fwdMs, String("FORWARD (X)") };
  }

  if (dy != 0) {
    int desired = (dy > 0) ? NORTH : SOUTH;
    facing = addTurnIfNeeded(facing, desired);
    unsigned long fwdMs = (unsigned long)(fabs((float)dy) * UNIT_TIME_S * 1000.0);
    pendingRoute[pendingRouteLen++] = { 'F', fwdMs, String("FORWARD (Y)") };
  }

  if (pendingRouteLen == 0) {
    Serial.println("Already at that position. Nothing to do.");
    Serial.println("=====================================================");
    return;
  }

  for (int i = 0; i < pendingRouteLen; i++) {
    Serial.print(i + 1);
    Serial.print(") ");
    Serial.print(pendingRoute[i].label);
    Serial.print("  for ");
    Serial.print(pendingRoute[i].durationMs / 1000.0, 2);
    Serial.println("s");
  }

  pendingTargetX = targetX;
  pendingTargetY = targetY;
  pendingFinalFacing = facing;
  routeWaitingConfirmation = true;

  Serial.println("=====================================================");
  Serial.println("Type OK to run this route, or CANCEL to discard it.");
}

// ---------------------------------------------------
// Send one pulse, hold it for its full duration, and report whether
// ESP-B confirmed receiving it -- all visible on THIS Serial Monitor.
// ---------------------------------------------------
void runPulse(char cmd, unsigned long dur) {
  Serial.print("PULSE: ");
  Serial.print(cmd);
  Serial.print(" for ");
  Serial.print(dur);
  Serial.println(" ms");

  ackReceived = false;
  sendCommand(cmd);

  unsigned long waitStart = millis();
  unsigned long ackTimeout = (dur < 300) ? dur : 300;
  while ((millis() - waitStart) < ackTimeout && !ackReceived) {
    delay(2);
  }

  if (ackReceived) {
    Serial.print("  <- ESP-B CONFIRMED RECEIPT of ");
    Serial.println((char)lastAck);
  } else {
    Serial.println("  <- !! NO RESPONSE FROM ESP-B !!");
    Serial.println("     Check: ESP-B is powered on, ESP-B has the latest code");
    Serial.println("     uploaded, and the MAC address in this file matches ESP-B.");
  }

  unsigned long alreadyWaited = millis() - waitStart;
  if (alreadyWaited < dur) {
    delay(dur - alreadyWaited);
  }

  ackReceived = false;
  sendCommand('S');
  delay(1200);   // gap before the next pulse starts
}

void executeRoute() {
  Serial.println();
  Serial.println("---- EXECUTING ROUTE ----");

  for (int i = 0; i < pendingRouteLen; i++) {
    Serial.print("[");
    Serial.print(i + 1);
    Serial.print("/");
    Serial.print(pendingRouteLen);
    Serial.println("]");
    runPulse(pendingRoute[i].command, pendingRoute[i].durationMs);
  }

  currentX = pendingTargetX;
  currentY = pendingTargetY;
  currentFacing = pendingFinalFacing;

  routeWaitingConfirmation = false;
  pendingRouteLen = 0;

  Serial.println("---- ROUTE COMPLETE ----");
  Serial.print("New position: (");
  Serial.print(currentX); Serial.print(", "); Serial.print(currentY);
  Serial.print(")  facing "); Serial.println(FACING_NAME[currentFacing]);
  Serial.println();
  Serial.println("Type: GOTO x,y   for the next target");
}

// ---------------------------------------------------
void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (routeWaitingConfirmation) {
    if (line.equalsIgnoreCase("OK")) {
      executeRoute();
    } else if (line.equalsIgnoreCase("CANCEL")) {
      routeWaitingConfirmation = false;
      pendingRouteLen = 0;
      Serial.println("Route cancelled.");
    } else {
      Serial.println("Type OK to run the route, or CANCEL to discard it.");
    }
    return;
  }

  if (line.startsWith("GOTO")) {
    int spaceIdx = line.indexOf(' ');
    if (spaceIdx == -1) { Serial.println("Format: GOTO x,y  (example: GOTO 2,3)"); return; }

    String coords = line.substring(spaceIdx + 1);
    int commaIdx = coords.indexOf(',');
    if (commaIdx == -1) { Serial.println("Format: GOTO x,y  (example: GOTO 2,3)"); return; }

    long tx = coords.substring(0, commaIdx).toInt();
    long ty = coords.substring(commaIdx + 1).toInt();

    planRoute(tx, ty);
  } else {
    Serial.println("Unknown command. Use: GOTO x,y");
  }
}

// ---------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);
  WiFi.setChannel(1);

  Serial.println("ESP-A starting...");

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW INIT FAILED");
    return;
  }

  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, ESP_B_MAC, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("PEER ADD FAILED");
    return;
  }

  esp_now_register_recv_cb(onAckReceived);

  Serial.println("==============================");
  Serial.println("ESP-A READY (Navigator)");
  Serial.println("Type: GOTO x,y   (example: GOTO 2,3)");
  Serial.println("==============================");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      handleLine(inputLine);
      inputLine = "";
    } else if (c != '\r') {
      inputLine += c;
    }
  }
}
