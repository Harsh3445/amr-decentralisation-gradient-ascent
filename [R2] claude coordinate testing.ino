#include <WiFi.h>
#include <esp_now.h>

// L298N pins (same wiring as before)
#define IN1 27   // Left motor
#define IN2 26   // Left motor
#define IN3 25   // Right motor
#define IN4 33   // Right motor

typedef struct {
  char command;
} Message;

Message receivedMessage;

// Broadcast address -- lets ESP-B reply to ESP-A without needing to
// know ESP-A's specific MAC address.
uint8_t broadcastAddress[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
esp_now_peer_info_t broadcastPeer;

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void forward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void backward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// RIGHT motor only -> robot turns LEFT
void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// LEFT motor only -> robot turns RIGHT
void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// Tell ESP-A "I got your command and acted on it"
void sendAck(char cmd) {
  Message ackMsg;
  ackMsg.command = cmd;
  esp_now_send(broadcastAddress, (uint8_t *)&ackMsg, sizeof(ackMsg));
}

void onReceive(const esp_now_recv_info_t *info,
               const uint8_t *data,
               int len) {

  if (len != sizeof(Message)) {
    return;
  }

  memcpy(&receivedMessage, data, sizeof(receivedMessage));

  switch (receivedMessage.command) {
    case 'F': forward();    sendAck('F'); break;
    case 'B': backward();   sendAck('B'); break;
    case 'L': turnLeft();   sendAck('L'); break;
    case 'R': turnRight();  sendAck('R'); break;
    case 'S': stopMotors(); sendAck('S'); break;
    default: break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stopMotors();

  WiFi.mode(WIFI_STA);
  WiFi.setChannel(1);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW INIT FAILED");
    return;
  }

  esp_now_register_recv_cb(onReceive);

  memset(&broadcastPeer, 0, sizeof(broadcastPeer));
  memcpy(broadcastPeer.peer_addr, broadcastAddress, 6);
  broadcastPeer.channel = 1;
  broadcastPeer.encrypt = false;
  esp_now_add_peer(&broadcastPeer);

  Serial.println("==============================");
  Serial.println("ESP-B READY");
  Serial.println("==============================");
}

void loop() {
}
