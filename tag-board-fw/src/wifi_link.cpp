#include "wifi_link.h"

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <ctype.h>
#include <errno.h>

#include "freertos/stream_buffer.h"
#include "lwip/sockets.h"

static volatile int g_client = -1;  // 연결된 TCP 소켓 (없으면 -1)
static volatile bool g_on = false;
static volatile uint32_t g_dropped = 0;
static StreamBufferHandle_t g_rx = nullptr;
static uint16_t g_port = 3333;
static volatile bool g_otaOn = false;
static bool (*g_otaAllowed)() = nullptr;

// 접속 상태: 0 꺼짐, 1 공유기 접속 중, 2 공유기 접속됨(STA), 3 보드 자체 WiFi(AP), 4 공유기 끊김(자동 재접속 중)
static volatile uint8_t g_mode = 0;
static const char *MODE_NAME[] = {"off", "connecting", "sta", "ap", "sta_lost"};
static char g_name[24] = "TagBoard";
static char g_host[24] = "tagboard";
static char g_apPass[64] = "";
static uint32_t g_connectStartMs = 0;
static bool g_mdnsOn = false;
static const uint32_t STA_CONNECT_TIMEOUT_MS = 20000;  // 이 안에 공유기에 못 붙으면 보드 자체 WiFi 로
static volatile uint8_t g_lastReason = 0;  // 마지막 공유기 끊김 이유 (201 공유기 없음, 15·204 비밀번호/핸드셰이크 등)

static void startMdns() {
  if (g_mdnsOn) return;
  if (MDNS.begin(g_host)) {  // PC 에서 tagboard-xxxx.local 로 찾는다 (공유기가 준 IP 를 몰라도)
    MDNS.addService("tagboard", "tcp", g_port);
    g_mdnsOn = true;
  }
}

static void startAp() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAP(g_name, g_apPass, 6, 0, 2);  // 채널 6, 숨김 아님, 동시 접속 2
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  g_mode = 3;
  startMdns();
}

// wifiTask 가 200ms 마다 부른다: 접속 완료·시간 초과(→ AP)·끊김 표시
static void superviseStep() {
  bool up = WiFi.status() == WL_CONNECTED;
  if (g_mode == 1) {
    if (up) {
      g_mode = 2;
      startMdns();
    } else if (millis() - g_connectStartMs > STA_CONNECT_TIMEOUT_MS) {
      startAp();
    }
  } else if (g_mode == 2 && !up) {
    g_mode = 4;  // 자동 재접속(setAutoReconnect)이 다시 붙인다. 비행 중에는 AP 로 바꾸지 않는다
  } else if (g_mode == 4 && up) {
    g_mode = 2;
  }
}

static void closeClient() {
  int c = g_client;
  g_client = -1;
  if (c >= 0) {
    shutdown(c, SHUT_RDWR);
    close(c);
  }
}

// 연결을 받고, 받은 바이트를 g_rx 에 넣는다. 새 연결이 오면 이전 연결은 닫는다 (뷰어를 다시 켤 때).
static int openServer() {
  int srv = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
  if (srv < 0) return -1;
  int yes = 1;
  setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
  sockaddr_in addr = {};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(g_port);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(srv, (sockaddr *)&addr, sizeof(addr)) != 0 || listen(srv, 1) != 0) {
    close(srv);
    return -1;
  }
  return srv;
}

static void wifiTask(void *) {
  // 망이 올라오기 전에는 소켓을 못 열 수 있다 → 감시(접속 확인·AP 전환)는 계속하면서 다시 시도
  int srv = -1;
  uint8_t buf[256];
  for (;;) {
    superviseStep();
    if (srv < 0) {
      srv = openServer();
      if (srv < 0) {
        vTaskDelay(pdMS_TO_TICKS(200));
        continue;
      }
    }
    // 비행 중(아밍)에는 업데이트 요청을 아예 보지 않는다 → 재부팅으로 보드 출력이 끊기는 일 방지
    if (g_otaOn && (!g_otaAllowed || g_otaAllowed())) ArduinoOTA.handle();
    fd_set rd;
    FD_ZERO(&rd);
    FD_SET(srv, &rd);
    int c = g_client;
    if (c >= 0) FD_SET(c, &rd);
    timeval tv = {0, 200000};
    int n = select((c > srv ? c : srv) + 1, &rd, nullptr, nullptr, &tv);
    if (n <= 0) continue;
    if (FD_ISSET(srv, &rd)) {
      int nc = accept(srv, nullptr, nullptr);
      if (nc >= 0) {
        int one = 1;
        setsockopt(nc, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        int flags = fcntl(nc, F_GETFL, 0);
        fcntl(nc, F_SETFL, flags | O_NONBLOCK);  // 보내기는 outTask 에서 기다림 한도를 직접 둔다
        closeClient();
        g_client = nc;
      }
    }
    c = g_client;
    if (c >= 0 && FD_ISSET(c, &rd)) {
      int r = recv(c, buf, sizeof(buf), 0);
      if (r > 0) {
        if (g_rx) xStreamBufferSend(g_rx, buf, r, 0);
      } else if (r == 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) {
        closeClient();
      }
    }
  }
}

bool wifiBegin(const char *name, const char *apPass, const char *staSsid, const char *staPass, uint16_t port) {
  WiFi.onEvent([](arduino_event_id_t, arduino_event_info_t info) { g_lastReason = info.wifi_sta_disconnected.reason; },
               ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  g_port = port;
  strlcpy(g_name, name, sizeof(g_name));
  strlcpy(g_apPass, apPass, sizeof(g_apPass));
  size_t i = 0;  // mDNS 이름은 소문자 (TagBoard-B8B4 → tagboard-b8b4)
  for (; g_name[i] && i < sizeof(g_host) - 1; i++) g_host[i] = (char)tolower((unsigned char)g_name[i]);
  g_host[i] = '\0';
  g_rx = xStreamBufferCreate(512, 1);
  if (staSsid && staSsid[0]) {
    WiFi.setHostname(g_host);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);  // 절전하면 지연이 수백 ms 로 튄다
    WiFi.setTxPower(WIFI_POWER_19_5dBm);  // 송신 최대 (약한 신호에서 재전송 줄이기)
    WiFi.setAutoReconnect(true);
    WiFi.begin(staSsid, staPass);
    g_connectStartMs = millis();
    g_mode = 1;
  } else {
    startAp();
  }
  g_on = true;
  xTaskCreatePinnedToCore(wifiTask, "wifi", 4096, nullptr, 2, nullptr, 0);
  return true;
}

int wifiReason() { return g_lastReason; }
float wifiTxDbm() { return g_mode ? WiFi.getTxPower() / 4.0f : 0; }

// 주변 WiFi 목록 (2.4GHz 만 보인다). JSON 배열로 out 에. 블로킹 2~3초 — 비행 전 점검용
int wifiScan(char *out, size_t n) {
  int cnt = WiFi.scanNetworks(false, true);
  size_t len = snprintf(out, n, "[");
  for (int i = 0; i < cnt && i < 15 && len < n - 80; i++) {
    char ssid[33];
    strlcpy(ssid, WiFi.SSID(i).c_str(), sizeof(ssid));
    for (char *c = ssid; *c; c++)
      if (*c == '"' || *c == '\\') *c = '_';
    len += snprintf(out + len, n - len, "%s{\"ssid\":\"%s\",\"rssi\":%d,\"ch\":%d,\"auth\":%d}", i ? "," : "", ssid,
                    WiFi.RSSI(i), WiFi.channel(i), (int)WiFi.encryptionType(i));
  }
  snprintf(out + len, n - len, "]");
  WiFi.scanDelete();
  return cnt;
}

const char *wifiMode() { return MODE_NAME[g_mode < 5 ? g_mode : 0]; }
const char *wifiHost() { return g_host; }
int wifiRssi() { return (g_mode == 2) ? WiFi.RSSI() : 0; }
void wifiIp(char *out, size_t n) {
  IPAddress ip = (g_mode == 3) ? WiFi.softAPIP() : WiFi.localIP();
  snprintf(out, n, "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
}

void wifiOtaBegin(const char *hostname, const char *pass, bool (*allowed)()) {
  if (!g_on) return;
  g_otaAllowed = allowed;
  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.setPassword(pass);
  ArduinoOTA.setMdnsEnabled(false);  // mDNS 는 위에서 따로 (tagboard-xxxx.local)
  ArduinoOTA.begin();
  g_otaOn = true;
}

bool wifiOn() { return g_on; }
bool wifiClientConnected() { return g_client >= 0; }
int wifiStations() { return g_mode == 3 ? WiFi.softAPgetStationNum() : 0; }
uint32_t wifiDropped() { return g_dropped; }

bool wifiSend(const uint8_t *data, size_t len, uint32_t timeoutMs) {
  int c = g_client;
  if (c < 0) return false;
  uint32_t t0 = millis();
  size_t off = 0;
  while (off < len) {
    int w = send(c, data + off, len - off, MSG_DONTWAIT);
    if (w > 0) {
      off += w;
      continue;
    }
    if (w < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {  // 끊김
      closeClient();
      return false;
    }
    if (millis() - t0 > timeoutMs) {
      // 시작 전이면 줄을 통째로 버린다. 보내던 중이면 줄바꿈만 보내 다음 줄과 붙지 않게 한다
      g_dropped++;
      if (off > 0) send(c, "\n", 1, MSG_DONTWAIT);
      return false;
    }
    vTaskDelay(1);
  }
  return true;
}

size_t wifiRead(uint8_t *buf, size_t max) { return g_rx ? xStreamBufferReceive(g_rx, buf, max, 0) : 0; }
