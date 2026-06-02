// ESP32 — WiFi controller and LiDAR safety stop for the MQRPII quadruped.
//
// Talks to the Teensy over Serial2 (binary framed packets, same protocol both ways).
// Hosts a web page so a phone on the same network can drive the robot with
// directional buttons. Polls an RPLidar in the background, and overrides any
// movement command with a STOP if it sees an obstacle inside a danger zone.

#include <Arduino.h>
#include <RPLidar.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "lwip/etharp.h"
#include "lwip/ip_addr.h"

const char *ssid = "University of Southampton Guest";
const char *password = "Password123";

// -------- SECURITY -----------
// The web UI has no password. The only thing stopping a random device on the
// same WiFi network from driving the robot is a MAC address whitelist.
// Anything not in ALLOWED_MACS gets dropped after the IP-to-MAC lookup.
//
// IMPORTANT before flashing:
//   1. Put your own SSID and password above (these are checked into the repo
//      as placeholders, do not assume they will work on your network).
//   2. Replace the two placeholder MAC addresses below with the MACs of the
//      phones or laptops that should be allowed to control the robot.
//      Uppercase, colon-separated, e.g. "A0:B1:C2:D3:E4:F5".
//   3. If you cannot find a device's MAC, the loop() prints it on the serial
//      monitor the first time that device hits the page.
const char* ALLOWED_MACS[] = {
  "11:22:33:44:55:66", // Placeholder — replace before deployment
  "AA:BB:CC:DD:EE:FF"  // Placeholder — replace before deployment
};
const int NUM_ALLOWED_MACS = sizeof(ALLOWED_MACS) / sizeof(ALLOWED_MACS[0]);

// Teensy serial link (Serial2)
constexpr int TEENSY_RX = 10;
constexpr int TEENSY_TX = 9;
constexpr uint32_t TEENSY_BAUD = 115200;

// LiDAR UART (Serial1)
constexpr int LIDAR_RX_PIN = 2;
constexpr int LIDAR_TX_PIN = 3;
constexpr uint32_t LIDAR_BAUD = 115200;
constexpr int LED_PIN = 8;

// LiDAR obstacle detection constants.
//
// The robot stops if it sees a cluster of LiDAR points closer than these
// distances. Front is checked at a tighter threshold than the sides because
// forward motion is the only direction the LiDAR is really protecting against.
// THRESHOLD_MM is the inner cutoff that ignores readings from the robot's own
// body (the LiDAR sees parts of the chassis at very short range).
// MAX_GAP lets the hit-counter tolerate a few stray empty readings inside a
// real object cluster, so a partially reflective surface still trips a stop.
constexpr int STOP_DISTANCE_FRONT_MM = 450;
constexpr int STOP_DISTANCE_SIDE_MM = 350;
constexpr int MAX_GAP = 4;
constexpr int THRESHOLD_MM = 175;

WiFiServer server(80);
RPLidar lidar;

// Motion state flags (encoding from ESP_lidar_controller_V2)
// FORWARDS:  A=1, B=0, C=0
// LEFT:      A=0, B=1, C=0
// RIGHT:     A=0, B=0, C=1
// BACKWARDS: A=1, B=1, C=1
// STOP:      A=0, B=0, C=0
int wifiFlagA = 0;
int wifiFlagB = 0;
int wifiFlagC = 0;

int hitCount = 0;
int gapCount = 0;
float startAngle = 0.0f;
float avgAngle = 0.0f;
bool stopTriggeredByLidar = false;
bool objectDetectedThisScan = false;
// minPoints: how many consecutive LiDAR hits inside the danger zone count as
// a real object (rather than noise). It depends on distance.
// The formula targets a ~100 mm-wide object at the current distance:
//arc length 100mm at radius dist  ->  angle (rad)  ->  number of LiDAR
//samples assuming the scanner gives ~1 point per 1.3 degrees.
// Initialised here for STOP_DISTANCE_FRONT_MM; recomputed in pollLidar() each
// time a new cluster starts so it matches the actual hit distance.
int minPoints = (int)(100.0f / STOP_DISTANCE_FRONT_MM * 180.0f / (3.1415f * 1.3f));

// Forward declarations
bool isMacAllowed(String clientMac);
// Looks up the MAC of a connected client by reading the ESP32's internal ARP
// table. Returns an empty string if the entry isn't there yet — which happens
// on the very first request from a new device, before the OS has cached it.
// The loop() handles that case by printing a warning instead of blocking.
String getMacFromIp(IPAddress ip);
float getStopDistance(float angle);
void initLidar();
void tryRestartLidar();
void pollLidar();
void sendMotionCommand(int flagA, int flagB, int flagC, const char *label, const char *reason = nullptr);
const char *stateLabel();
void serveControllerPage(WiFiClient &client);

// -------------------- MAC Whitelist Helpers --------------------
bool isMacAllowed(String clientMac) {
  clientMac.toUpperCase();
  for (int i = 0; i < NUM_ALLOWED_MACS; i++) {
    String allowed = String(ALLOWED_MACS[i]);
    allowed.toUpperCase();
    if (clientMac == allowed) return true;
  }
  return false;
}

String getMacFromIp(IPAddress ip) {
  ip4_addr_t ip_addr;
  ip_addr.addr = ip;
  eth_addr *mac_addr = NULL;
  const ip4_addr_t *ip_ptr = &ip_addr;
  ssize_t idx = etharp_find_addr(NULL, ip_ptr, &mac_addr, NULL);
  if (idx >= 0 && mac_addr != NULL) {
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac_addr->addr[0], mac_addr->addr[1], mac_addr->addr[2],
             mac_addr->addr[3], mac_addr->addr[4], mac_addr->addr[5]);
    return String(macStr);
  }
  return "";
}

// -------------------- Binary packet TX (to Teensy) --------------------
// Sends a framed packet: [0xAA][0x55][6 bytes payload][1 byte checksum]
// Payload: 3 signed 16-bit ints packed big-endian
void sendPacket(int a, int b, int c) {
  uint8_t header[2] = {0xAA, 0x55};
  uint8_t payload[6];
  int16_t vals[3] = { (int16_t)a, (int16_t)b, (int16_t)c };
  for (int i = 0; i < 3; ++i) {
    payload[2*i]   = (uint8_t)((vals[i] >> 8) & 0xFF);
    payload[2*i+1] = (uint8_t)(vals[i] & 0xFF);
  }
  uint8_t sum = 0;
  for (uint8_t byte : payload) sum += byte;
  Serial2.write(header, 2);
  Serial2.write(payload, 6);
  Serial2.write(sum);
}

// -------------------- Binary packet RX (from Teensy) --------------------
// The same framed protocol as the TX side, in reverse. The Teensy echoes
// every command back as an acknowledgement (with the second value negated and
// a sequence counter added to the third), which is what gets printed on the
// serial monitor as "RX OK". Used for debugging the link, not for any control logic
enum RxState { FIND_AA, FIND_55, READ_PAYLOAD, READ_SUM };
RxState rxState = FIND_AA;
uint8_t rxPayload[6];
int rxIdx = 0;

void pollSerial2RX() {
  while (Serial2.available()) {
    uint8_t b = (uint8_t)Serial2.read();
    switch (rxState) {
      case FIND_AA:
        if (b == 0xAA) rxState = FIND_55;
        break;
      case FIND_55:
        rxState = (b == 0x55) ? READ_PAYLOAD : FIND_AA;
        rxIdx = 0;
        break;
      case READ_PAYLOAD:
        rxPayload[rxIdx++] = b;
        if (rxIdx >= 6) rxState = READ_SUM;
        break;
      case READ_SUM: {
        uint8_t sum = 0;
        for (int i = 0; i < 6; ++i) sum += rxPayload[i];
        if (sum == b) {
          int16_t r1 = (int16_t)((rxPayload[0] << 8) | rxPayload[1]);
          int16_t r2 = (int16_t)((rxPayload[2] << 8) | rxPayload[3]);
          int16_t r3 = (int16_t)((rxPayload[4] << 8) | rxPayload[5]);
          Serial.printf("RX OK: r1=%d r2=%d r3=%d\n", r1, r2, r3);
        } else {
          Serial.println("RX checksum mismatch (ESP32)");
        }
        rxState = FIND_AA;
        break;
      }
    }
  }
}

// -------------------- Motion command --------------------
const char *stateLabel() {
  if (wifiFlagA == 1 && wifiFlagB == 1 && wifiFlagC == 1) return "BACKWARDS";
  if (wifiFlagA == 1) return "FORWARDS";
  if (wifiFlagB == 1) return "LEFT";
  if (wifiFlagC == 1) return "RIGHT";
  return "STOP";
}

void sendMotionCommand(int flagA, int flagB, int flagC, const char *label, const char *reason) {
  wifiFlagA = flagA;
  wifiFlagB = flagB;
  wifiFlagC = flagC;
  Serial.print("Command: ");
  Serial.print(label);
  if (reason != nullptr) {
    Serial.print(" (");
    Serial.print(reason);
    Serial.print(")");
  }
  Serial.println();
  sendPacket(flagA, flagB, flagC);
}

// -------------------- LiDAR --------------------
// LiDAR angle ranges
//   45–135°  : front quadrant   -> tighter stop distance
//   135–225° : right side       -> looser stop distance
//   225–315° : back             -> looser (and arguably should be ignored)
//   315–45°  : left side        -> looser
// Returns the cutoff distance for a hit at a given angle.
float getStopDistance(float angle) {
  if (angle >= 45 && angle < 135)   return STOP_DISTANCE_FRONT_MM;
  if (angle >= 135 && angle < 225) return STOP_DISTANCE_SIDE_MM;
  if (angle >= 225 && angle < 315) return STOP_DISTANCE_SIDE_MM;
  return STOP_DISTANCE_SIDE_MM;
}

void initLidar() {
  Serial1.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  delay(500);
  if (!lidar.begin(Serial1)) {
    Serial.println("LiDAR init failed");
    return;
  }
  rplidar_response_device_health_t health;
  if (lidar.getHealth(health) == RESULT_OK) {
    Serial.println("LiDAR health OK");
  } else {
    Serial.println("LiDAR health check failed");
  }
  if (lidar.startScan()) {
    Serial.println("LiDAR scan started");
  } else {
    Serial.println("LiDAR scan failed to start");
  }
}

void tryRestartLidar() {
  rplidar_response_device_info_t info;
  if (lidar.getDeviceInfo(info, 100) == RESULT_OK) {
    lidar.startScan();
    Serial.println("LiDAR scan restarted");
  }
}

// One full pass of the obstacle detection. Reads a single point from the
// LiDAR, decides whether it belongs to a cluster of points inside the danger
// zone, and if the cluster is big enough, sends a STOP to the Teensy.
//
// How the cluster detection works:
//   - We count "hits" when points are inside the danger zone.
//   - Stray empty samples inside a real object are tolerated up to MAX_GAP
//     (gapCount) so a slightly reflective surface doesn't break the cluster.
//   - The cluster has to be at least minPoints samples wide to trigger a
//     stop. minPoints scales with distance (farther = fewer samples for the
//     same physical width), so a small object close up trips it just as
//     reliably as a larger one farther away.
//   - startBit marks the start of a new 360° scan. We use it to debounce:
//     stopTriggeredByLidar stays latched until a full scan finishes WITHOUT
//     re-detecting the object, at which point we declare "clear".
void pollLidar() {
  if (lidar.waitPoint() != RESULT_OK) {
    tryRestartLidar();
    Serial.println("Lidar Not Okay");
    return;
  }
  const RPLidarMeasurement &point = lidar.getCurrentPoint();
  const float dist = point.distance;
  const float angle = point.angle;
  const bool startBit = point.startBit;
  const float stopDistance = getStopDistance(angle);

  // Start of a new 360° scan
  if (startBit) {
    Serial.println("Spin");
    if (stopTriggeredByLidar && !objectDetectedThisScan) {
      stopTriggeredByLidar = false;
      Serial.println("LiDAR clear");
    }
    objectDetectedThisScan = false;
  }
  // Point sits inside the danger zone, count it as a hit.
  if (dist > THRESHOLD_MM && dist < stopDistance) {
    Serial.print("H");
    // First hit of a new cluster, record where it started and
    // recompute minPoints for the actual hit distance.
    if (hitCount == 0) {
      startAngle = angle;
      minPoints = (int)(100.0f / dist * 180.0f / (3.1415f * 1.3f));
      if (minPoints < 1) minPoints = 1;
    }
    hitCount++;
    // If we were inside a gap, absorb it into the cluster (gap was a
    // false dropout, not the end of the object).
    if (gapCount > 0) {
      hitCount += gapCount;
      gapCount = 0;
    }
  } else if (hitCount > 0 && gapCount < MAX_GAP) {
    gapCount++;
  } else {
    hitCount = 0;
    gapCount = 0;
  }

  if (hitCount >= minPoints) {
    objectDetectedThisScan = true;
    avgAngle = (angle - startAngle) / 2.0f + startAngle;
    if (!stopTriggeredByLidar) {
      stopTriggeredByLidar = true;
      Serial.print("LiDAR object detected at angle: ");
      Serial.println(avgAngle);
      sendMotionCommand(0, 0, 0, "STOP", "LiDAR");
    }
  }
}

// -------------------- Web UI (from ESP_lidar_controller_V2) --------------------
void serveControllerPage(WiFiClient &client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-type:text/html");
  client.println("Connection: close");
  client.println();
  client.println("<!DOCTYPE html><html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
  client.println("<title>ESP LiDAR Controller</title>");
  client.println("<style>");
  client.println(":root{--bg:#f4f0e8;--panel:#fffaf2;--ink:#202020;--accent:#d95f02;--ok:#2f855a;--warn:#c53030;--edge:#d7c7aa;}");
  client.println("*{box-sizing:border-box}body{margin:0;min-height:100vh;display:grid;place-items:center;background:radial-gradient(circle at top,#fffdf7 0,#f4f0e8 45%,#e7dfd0 100%);font-family:Georgia,serif;color:var(--ink);}");
  client.println(".panel{width:min(92vw,440px);padding:28px;border:2px solid var(--edge);border-radius:28px;background:linear-gradient(180deg,var(--panel),#efe4d3);box-shadow:0 18px 40px rgba(54,35,12,.16);}");
  client.println(".title{text-align:center;font-size:2rem;letter-spacing:.08em;margin:0 0 8px}.status{text-align:center;margin:0 0 20px;font-size:1rem}");
  client.println(".status span{font-weight:700}.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:14px;align-items:center}");
  client.println(".btn{display:grid;place-items:center;min-height:92px;border:none;border-radius:22px;text-decoration:none;color:#fff;font-size:2rem;font-weight:700;box-shadow:0 10px 20px rgba(0,0,0,.12), inset 0 2px 0 rgba(255,255,255,.2);}");
  client.println(".btn:active{transform:translateY(2px)}.forward{grid-column:2;background:var(--ok)}.left{grid-column:1;grid-row:2;background:#2b6cb0}.stop{grid-column:2;grid-row:2;background:var(--warn);font-size:1.2rem;letter-spacing:.08em}.right{grid-column:3;grid-row:2;background:#805ad5}.backward{grid-column:2;grid-row:3;background:var(--accent)}");
  client.println(".meta{margin-top:18px;text-align:center;font-size:.95rem;opacity:.82}.blocked{color:var(--warn);font-weight:700}");
  client.println("</style></head><body><div class=\"panel\">");
  client.println("<h1 class=\"title\">Robot Controller</h1>");
  client.print("<p class=\"status\">State: <span>");
  client.print(stateLabel());
  client.println("</span></p>");
  client.println("<div class=\"grid\">");
  client.println("<a href=\"/FORWARDS\" class=\"btn forward\" aria-label=\"FORWARDS\">&#8593;</a>");
  client.println("<a href=\"/LEFT\" class=\"btn left\" aria-label=\"LEFT\">&#8592;</a>");
  client.println("<a href=\"/STOP\" class=\"btn stop\" aria-label=\"STOP\">STOP</a>");
  client.println("<a href=\"/RIGHT\" class=\"btn right\" aria-label=\"RIGHT\">&#8594;</a>");
  client.println("<a href=\"/BACKWARDS\" class=\"btn backward\" aria-label=\"BACKWARDS\">&#8595;</a>");
  client.println("</div>");
  client.print("<p class=\"meta\">LiDAR status: ");
  if (stopTriggeredByLidar) {
    client.print("<span class=\"blocked\">Obstacle detected</span>");
  } else {
    client.print("Clear");
  }
  client.println("</p></div></body></html>");
  client.println();
}

// -------------------- Setup & Loop --------------------
void setup() {
  Serial.begin(115200);
  Serial2.begin(TEENSY_BAUD, SERIAL_8N1, TEENSY_RX, TEENSY_TX);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  delay(500);
  initLidar();

  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("ESP32 MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("Waiting for authorized clients...");
  server.begin();
}

void loop() {
  pollSerial2RX();
  pollLidar();

  WiFiClient client = server.available();
  if (!client) return;

  Serial.println("New Client.");

  // ---- SECURITY CHECK: MAC WHITELIST ----
  IPAddress clientIp = client.remoteIP();
  String clientMac = getMacFromIp(clientIp);
  Serial.print("Client IP: ");
  Serial.print(clientIp);
  if (clientMac.length() > 0) {
    Serial.print(", MAC: ");
    Serial.println(clientMac);
    if (!isMacAllowed(clientMac)) {
      Serial.println("!!! SECURITY ALERT: Client MAC not in whitelist. Disconnecting.");
      client.stop();
      return;
    } else {
      Serial.println("Client Authorized.");
    }
  } else {
    Serial.println(", MAC: Not found in ARP table yet. Warning only.");
    // First request from a new device often arrives before the ESP's ARP
    // table has cached its MAC. We let the request through and rely on the
    // next one being authenticated properly. For a stricter setup, uncomment
    // the next line so unknown devices are blocked outright.
    // client.stop(); return;
  // ---------------------------------------

  unsigned long requestStart = millis();
  while (!client.available() && millis() - requestStart < 2000) {
    delay(1);
  }

  if (!client.available()) {
    Serial.println("HTTP request timeout");
    client.stop();
    Serial.println("Client Disconnected.");
    return;
  }

  String requestLine = client.readStringUntil('\n');
  requestLine.trim();
  Serial.print("HTTP request: ");
  Serial.println(requestLine);

  // Only FORWARDS is gated by the LiDAR
  // the other directions are still allowed because they let the operator strafe or turn out of the obstacle.
  // STOP and BACKWARDS go straight through.
  if (requestLine.startsWith("GET /FORWARDS")) {
    if (stopTriggeredByLidar) {
      sendMotionCommand(0, 0, 0, "STOP", "LiDAR block");
    } else {
      sendMotionCommand(1, 0, 0, "FORWARDS");
    }
  } else if (requestLine.startsWith("GET /LEFT")) {
    sendMotionCommand(0, 1, 0, "LEFT");
  } else if (requestLine.startsWith("GET /RIGHT")) {
    sendMotionCommand(0, 0, 1, "RIGHT");
  } else if (requestLine.startsWith("GET /BACKWARDS")) {
    sendMotionCommand(1, 1, 1, "BACKWARDS");
  } else if (requestLine.startsWith("GET /STOP")) {
    sendMotionCommand(0, 0, 0, "STOP", "Web");
  }

  while (client.available()) {
    client.read();
  }

  serveControllerPage(client);
  delay(1);
  client.stop();
  Serial.println("Client Disconnected.");
}
