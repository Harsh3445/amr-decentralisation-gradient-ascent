// =====================================================
//  ESP-A  —  CONTROLLER  (coordinate go-to)
//  Serial Monitor: 115200 baud, line ending = Newline
//
//  2,3      go to (2,3)      (also: 2 3   or   G 2 3)
//  H        robot is at home (0,0) facing +Y
//  S        stop
//  W        where is the robot?
//  X 2,1    mark / unmark cell (2,1) as obstacle
//  C / Q    calibrate: 1 cell forward / 90 deg right
// =====================================================
#include <WiFi.h>
#include <esp_now.h>

uint8_t espBMac[] = {0x20, 0x9B, 0xA9, 0x61, 0xBB, 0x20};

// ---------- MESSAGES (must match ESP-B) ----------
typedef struct {
  char command;
  int8_t x;
  int8_t y;
} RobotCommand;

typedef struct {
  char text[80];
} RobotMessage;

esp_now_peer_info_t peerInfo;

void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) Serial.println("[SEND FAILED - is ESP-B on?]");
}

// messages from the robot
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(RobotMessage)) return;
  RobotMessage m;
  memcpy(&m, data, sizeof(m));
  m.text[sizeof(m.text) - 1] = '\0';
  Serial.print("[ROBOT] ");
  Serial.println(m.text);
}

void sendCommand(char c, int x, int y) {
  RobotCommand cmd;
  cmd.command = c;
  cmd.x = x;
  cmd.y = y;
  esp_now_send(espBMac, (uint8_t *)&cmd, sizeof(cmd));
}

void printHelp() {
  Serial.println("==================================");
  Serial.println(" 2,3    go to cell (2,3)");
  Serial.println(" H      set home (0,0), facing +Y");
  Serial.println(" S      stop     W   where am I");
  Serial.println(" X 2,1  toggle obstacle at (2,1)");
  Serial.println(" C / Q  calibrate 1 cell / 90 turn");
  Serial.println(" 1 cell = 10 cm, grid 0..9");
  Serial.println("==================================");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);
  WiFi.setChannel(1);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW INIT FAILED");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, espBMac, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("PEER ADD FAILED");
    return;
  }

  Serial.println("ESP-A CONTROLLER READY");
  printHelp();
}

void loop() {
  if (!Serial.available()) return;

  String in = Serial.readStringUntil('\n');
  in.trim();
  in.toUpperCase();
  in.replace(",", " ");
  in.replace("(", " ");
  in.replace(")", " ");
  in.trim();
  if (in.length() == 0) return;

  char c = in.charAt(0);
  int x = 0, y = 0;

  // plain coordinates like "2 3" -> go to
  if (isDigit(c)) {
    if (sscanf(in.c_str(), "%d %d", &x, &y) != 2) {
      Serial.println("Type coordinates like: 2,3");
      return;
    }
    c = 'G';
  } else if (c == 'G' || c == 'X') {
    if (sscanf(in.c_str() + 1, "%d %d", &x, &y) != 2) {
      Serial.println("Need two numbers, e.g.  G 2,3");
      return;
    }
  } else if (strchr("HSWCQ", c) == nullptr) {
    Serial.println("Unknown command");
    printHelp();
    return;
  }

  if (c == 'G') Serial.printf("[YOU] go to (%d,%d)\n", x, y);
  else          Serial.printf("[YOU] %s\n", in.c_str());

  sendCommand(c, x, y);
}
