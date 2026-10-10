#include "ble_link.h"

#include <Arduino.h>
#include <NimBLEDevice.h>

#include "freertos/stream_buffer.h"

static const char *NUS_SERVICE = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *NUS_RX = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *NUS_TX = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

static NimBLECharacteristic *g_tx = nullptr;
static volatile uint16_t g_conn = BLE_HS_CONN_HANDLE_NONE;
static volatile bool g_subscribed = false;
static volatile uint16_t g_mtu = 23;
static volatile uint32_t g_dropped = 0;
static StreamBufferHandle_t g_rx = nullptr;
static char g_name[20] = "";
static bool g_started = false;

static void makeName() {
  if (g_name[0]) return;
  uint64_t mac = ESP.getEfuseMac();  // 하위 바이트부터: 마지막 두 바이트가 MAC 끝
  snprintf(g_name, sizeof(g_name), "TagBoard-%02X%02X", (uint8_t)(mac >> 32), (uint8_t)(mac >> 40));
}

class ServerCb : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *s, ble_gap_conn_desc *d) override {
    g_conn = d->conn_handle;
    g_mtu = 23;
    // 연결 간격·DLE·PHY 변경을 여기서 요청하지 않는다. macOS가 같은 시점에 자체 갱신을 시작해
    // 두 절차가 겹치면 ESP32-S3 컨트롤러가 assert(llc_con_upd.c 488)를 내고 링크가 멈춘다 (20261003 실측).
    (void)s;
    (void)d;
  }
  void onDisconnect(NimBLEServer *, ble_gap_conn_desc *) override {
    g_conn = BLE_HS_CONN_HANDLE_NONE;
    g_subscribed = false;
  }
  void onMTUChange(uint16_t mtu, ble_gap_conn_desc *) override { g_mtu = mtu; }
};

class TxCb : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic *, ble_gap_conn_desc *, uint16_t subValue) override {
    g_subscribed = subValue != 0;
  }
};

class RxCb : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c, ble_gap_conn_desc *) override {
    NimBLEAttValue v = c->getValue();
    if (g_rx) xStreamBufferSend(g_rx, v.data(), v.length(), 0);  // 쓰는 쪽은 NimBLE 호스트 태스크 하나
  }
};

void bleBegin() {
  makeName();
  g_started = true;
  g_rx = xStreamBufferCreate(512, 1);

  NimBLEDevice::init(g_name);
  NimBLEDevice::setMTU(247);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEServer *server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCb());
  server->advertiseOnDisconnect(true);

  NimBLEService *svc = server->createService(NUS_SERVICE);
  g_tx = svc->createCharacteristic(NUS_TX, NIMBLE_PROPERTY::NOTIFY);
  g_tx->setCallbacks(new TxCb());
  NimBLECharacteristic *rx = svc->createCharacteristic(NUS_RX, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  rx->setCallbacks(new RxCb());
  svc->start();

  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE);
  adv->setScanResponse(true);  // 128비트 UUID 때문에 이름은 스캔 응답에 실린다
  adv->start();
}

const char *bleName() {
  makeName();
  return g_name;
}
bool bleStarted() { return g_started; }
bool bleConnected() { return g_conn != BLE_HS_CONN_HANDLE_NONE && g_subscribed; }
uint16_t bleMtu() { return g_mtu; }
uint32_t bleDropped() { return g_dropped; }
int bleBuffersFree() { return g_started ? os_msys_num_free() : 0; }
int bleBuffersTotal() { return g_started ? os_msys_count() : 0; }

// 알림을 몰아 보내면 수 초 뒤 링크가 멈추고(데이터가 더 안 나감) 결국 끊기는 것을 20261003 실측으로
// 확인했다. 그래서 (1) 스택 몫 버퍼를 남기고 (2) 알림 사이에 최소 간격을 둔다.
// 간격 4ms. 진짜 원인(연결 직후 파라미터 요청 충돌, 내부 RAM 고갈)을 고친 뒤 실측으로 정한 값.
static const int MSYS_RESERVE = 8;
static const uint32_t MIN_GAP_US = 4000;
static uint32_t g_lastNotifyUs = 0;

static bool notifyChunk(const uint8_t *data, size_t len) {
  if (micros() - g_lastNotifyUs < MIN_GAP_US) return false;
  if (os_msys_num_free() < MSYS_RESERVE) return false;
  os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
  if (!om || ble_gattc_notify_custom(g_conn, g_tx->getHandle(), om) != 0) return false;  // om은 성공·실패 모두 소비됨
  g_lastNotifyUs = micros();
  return true;
}

bool bleSend(const uint8_t *data, size_t len, uint32_t timeoutMs) {
  if (!bleConnected() || !g_tx) return false;
  const size_t chunk = (g_mtu > 23 ? g_mtu : 23) - 3;
  const uint32_t t0 = millis();
  size_t off = 0;
  while (off < len) {
    if (!bleConnected()) return false;
    size_t n = min(chunk, len - off);
    if (notifyChunk(data + off, n)) {
      off += n;
      continue;
    }
    // 시작 전이면 줄을 통째로 버린다. 보내던 중이면 조금 더 기다리고, 그래도 안 되면
    // 줄바꿈만 보내 끊긴 조각이 다음 줄과 붙지 않게 한다 (받는 쪽에서 깨진 줄로 걸러진다).
    uint32_t waited = millis() - t0;
    if (off == 0 && waited > timeoutMs) {
      g_dropped++;
      return false;
    }
    if (off > 0 && waited > timeoutMs + 1000) {
      g_dropped++;
      for (int i = 0; i < 50 && bleConnected() && !notifyChunk((const uint8_t *)"\n", 1); i++) vTaskDelay(2);
      return false;
    }
    vTaskDelay(1);
  }
  return true;
}

size_t bleRead(uint8_t *buf, size_t max) {
  return g_rx ? xStreamBufferReceive(g_rx, buf, max, 0) : 0;
}
