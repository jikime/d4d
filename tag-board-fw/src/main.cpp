// 태그인식 보드 펌웨어 — XIAO ESP32S3 Sense
//
// core 0: MTF-01P(Micolink) 수신 → flow JSON (5Hz)
// core 1: 카메라 그레이스케일 프레임 → AprilTag 검출 → 의미 해석·이동 판단 → tag JSON
//
// USB 시리얼 명령: snap | size <mm> | info | help
// 출력은 한 줄에 JSON 하나. 사람이 읽기 좋은 형태는 tools/tag_monitor.py 사용.

#include <Arduino.h>
#include <Preferences.h>
#include <math.h>

#include "freertos/message_buffer.h"

#include "esp_camera.h"
#include "esp_log.h"
#include "esp_pthread.h"
#include "img_converters.h"
#include "mbedtls/base64.h"

#include "apriltag.h"
#include "tag36h11.h"
#include "common/image_u8.h"
#include "common/zarray.h"
#include "common/homography.h"

#include "alt_hold.h"
#include "approach.h"
#include "tour.h"
#include "ble_link.h"
#include "config.h"
#include "gimbal.h"
#include "tag_size_cal.h"
#include "wifi_link.h"
#include "msp.h"
#include "mic.h"
#include "mtf01p.h"
#include "rc_authority.h"

SET_LOOP_TASK_STACK_SIZE(16 * 1024);

// ---------------------------------------------------------------------------
// XIAO ESP32S3 Sense 카메라 핀
// ---------------------------------------------------------------------------
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 10
#define SIOD_GPIO_NUM 40
#define SIOC_GPIO_NUM 39
#define Y9_GPIO_NUM 48
#define Y8_GPIO_NUM 11
#define Y7_GPIO_NUM 12
#define Y6_GPIO_NUM 14
#define Y5_GPIO_NUM 16
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 17
#define Y2_GPIO_NUM 15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM 47
#define PCLK_GPIO_NUM 13

// ---------------------------------------------------------------------------
// 출력 (USB 시리얼)
// arduino-esp32 2.0.x HWCDC는 USB 인터럽트가 도는 코어(core 1)가 아닌 곳에서 write하면
// 64바이트 패킷 일부를 잃는다. 그래서 모든 줄을 메시지 버퍼에 넣고 core 1의 출력 태스크만 쓴다.
// USB가 안 꽂혀 있거나 호스트가 안 읽어도 멈추지 않도록, 자리가 없으면 줄을 버린다.
// ---------------------------------------------------------------------------
static SemaphoreHandle_t g_outMutex;
static SemaphoreHandle_t g_emitMutex;
static MessageBufferHandle_t g_outBuf;
static volatile uint32_t g_droppedLines = 0;

static void emitLine(const char *s) {
  // FreeRTOS 메시지 버퍼는 쓰는 쪽이 하나여야 안전하다. 여러 태스크가 쓰므로 뮤텍스로 묶는다.
  if (xSemaphoreTake(g_emitMutex, pdMS_TO_TICKS(5)) != pdTRUE) {
    g_droppedLines++;
    return;
  }
  if (xMessageBufferSend(g_outBuf, s, strlen(s), 0) == 0) g_droppedLines++;
  xSemaphoreGive(g_emitMutex);
}

// USB에 한 줄 쓴다. PC가 읽어 가는 중이면 자리가 날 때까지 잠깐 기다리고,
// 아무도 읽지 않아 버퍼가 계속 차 있으면(포트 닫힘) 기다리지 않고 건너뛴다 → BLE 출력이 막히지 않게.
static bool g_usbDraining = true;

static void usbWriteLine(const char *line, size_t n) {
  xSemaphoreTake(g_outMutex, portMAX_DELAY);
  uint32_t t0 = millis();
  while (Serial.availableForWrite() < (int)n) {
    if (!g_usbDraining || millis() - t0 > 40) break;
    vTaskDelay(1);
  }
  if (Serial.availableForWrite() >= (int)n) {
    Serial.write((const uint8_t *)line, n);
    g_usbDraining = true;
  } else {
    if (g_usbDraining) g_droppedLines++;  // 읽던 중 막힌 경우만 센다 (아무도 안 읽어 건너뛴 줄은 정상)
    g_usbDraining = false;                // 다음부터는 자리가 날 때까지 기다리지 않음
  }
  xSemaphoreGive(g_outMutex);
}

// BLE 로만 자주 나오는 상태 줄을 솎는다 (`ble lite`, config.h BLE_LITE_*). USB 는 모든 줄을 그대로 받는다.
static volatile bool g_bleLite = BLE_LITE_DEFAULT;
// 느린 통로(BLE, 약한 WiFi)로는 자주 나오는 상태 줄을 최소 간격으로 솎는다. 통로마다 마지막 시각을 따로 둔다.
// 20261011: 행사장 WiFi(-80dBm)로 보드 TCP 처리량이 약 2~10KB/s 라 상태 줄(약 5KB/s)만으로 사진이 밀렸다.
static volatile bool g_tcpLite = WIFI_LITE_DEFAULT;
static bool liteSkip(const char *line, uint32_t *lastMs) {
  static const char P[] = "{\"type\":\"";
  if (strncmp(line, P, sizeof(P) - 1) != 0) return false;  // 사진 조각 등 JSON 이 아닌 줄은 통과
  const char *t = line + sizeof(P) - 1;
  struct Rate { const char *type; uint32_t minMs; };
  static const Rate rates[] = {
      {"approach\"", BLE_LITE_APPROACH_MS}, {"flow\"", BLE_LITE_FLOW_MS}, {"tag\"", BLE_LITE_TAG_MS},
      {"vision\"", BLE_LITE_VISION_MS},     {"msp\"", BLE_LITE_MSP_MS},   {"mic\"", BLE_LITE_MIC_MS},
  };
  uint32_t now = millis();
  for (size_t i = 0; i < sizeof(rates) / sizeof(rates[0]); i++) {
    if (strncmp(t, rates[i].type, strlen(rates[i].type)) != 0) continue;
    if (lastMs[i] && now - lastMs[i] < rates[i].minMs) return true;
    lastMs[i] = now;
    return false;
  }
  return false;  // 이벤트·명령 응답·fc 등은 그대로
}
static bool bleLiteSkip(const char *line) {
  static uint32_t last[6] = {};
  return g_bleLite && liteSkip(line, last);
}
static bool tcpLiteSkip(const char *line) {
  static uint32_t last[6] = {};
  return g_tcpLite && liteSkip(line, last);
}

// 명령이 들어온 통로와, 사진·녹음처럼 JSON 이 아닌 큰 줄을 보낼 통로 (요청한 쪽으로만).
// 예: WiFi 뷰어가 사진을 받는 동안 BLE 로 같은 40KB 를 밀어 넣으면 outTask 가 BLE 를 기다리며 막힌다.
enum : uint8_t { SRC_USB = 1, SRC_BLE = 2, SRC_TCP = 4, SRC_ALL = 7 };
static volatile uint8_t g_cmdSource = SRC_ALL;
static volatile uint8_t g_rawDest = SRC_ALL;

// 같은 줄을 USB·BLE·WiFi(연결돼 있으면) 로 보낸다
static void outTask(void *) {
  static char line[1400];
  for (;;) {
    size_t n = xMessageBufferReceive(g_outBuf, line, sizeof(line) - 1, portMAX_DELAY);
    if (n == 0) continue;
    line[n] = '\0';
    bool raw = line[0] != '{';  // SNAP_BEGIN·base64·SNAP_END·MIC_* 등
    bool bin = n > 6 && memcmp(line, "SNAPC ", 6) == 0;  // 사진 원본 조각: 머리줄 + 바이트 (끝 줄바꿈 없음)
    uint8_t dest = raw ? g_rawDest : SRC_ALL;
    if (bin) dest &= SRC_TCP;  // 원본 조각은 TCP 로만 (USB·BLE 는 글자 통로)
    bool bleSkip = !(dest & SRC_BLE) || (bleConnected() && bleLiteSkip(line));
    if (!bin) line[n++] = '\n';
    if (dest & SRC_USB) usbWriteLine(line, n);
    if ((dest & SRC_TCP) && wifiClientConnected() && !(raw ? false : tcpLiteSkip(line)))
      wifiSend((const uint8_t *)line, n, n > 500 ? 1000 : 100);
    // 짧은 상태 줄은 0.1초, 긴 줄(스냅샷 조각)은 1.5초까지 자리를 기다린다
    if (bleConnected() && !bleSkip) bleSend((const uint8_t *)line, n, n > 500 ? 1500 : 200);
  }
}

// 스냅샷처럼 잃으면 안 되는 줄은 메시지 버퍼에 자리가 날 때까지 기다린다
static bool emitLineWait(const char *s, size_t n, uint32_t timeoutMs) {
  if (xSemaphoreTake(g_emitMutex, pdMS_TO_TICKS(timeoutMs)) != pdTRUE) return false;
  bool ok = xMessageBufferSend(g_outBuf, s, n, pdMS_TO_TICKS(timeoutMs)) == n;
  xSemaphoreGive(g_emitMutex);
  if (!ok) g_droppedLines++;
  return ok;
}

// ---------------------------------------------------------------------------
// MTF-01P 상태 (core 0 태스크가 쓰고, core 1이 읽는다)
// ---------------------------------------------------------------------------
struct FlowState {
  MtfSample s{};
  uint32_t lastRxMs = 0;
  uint32_t bytes = 0;
  uint32_t packets = 0;
  uint32_t errors = 0;
  float upCmS = 0.0f;
  bool hasSample = false;
  const char *proto = "none";  // MTF-01P 출력 형식: micolink | msp
  const char *heightSrc = "none";  // 높이 출처: mtf(보드 직결) | fc(FC 거리센서) | sim | none
};

static FlowState g_flow;
static portMUX_TYPE g_flowMux = portMUX_INITIALIZER_UNLOCKED;

// 시뮬레이션 고도 (브라우저가 `simh <cm>`로 보냄)
static volatile float g_simHeightCm = 0;
static volatile uint32_t g_simAtMs = 0;

static bool simActive() { return g_simAtMs != 0 && (millis() - g_simAtMs) < SIM_TIMEOUT_MS; }

// FC 거리센서 높이 (MSP_SONAR_ALTITUDE, cm). MTF-01P를 FC UART4에 연결한 구성(20261010~)에서 mspTask가 채운다.
static volatile int32_t g_fcAltCm = -1;
static volatile uint32_t g_fcAltAtMs = 0;

// 시뮬레이션 중에는 높이만 가상 값으로 바꾼다. 미션·decimate·스틱 계산이 모두 이 값을 쓴다.
static FlowState flowSnapshot() {
  portENTER_CRITICAL(&g_flowMux);
  FlowState f = g_flow;
  portEXIT_CRITICAL(&g_flowMux);
  uint32_t altAt = g_fcAltAtMs;
  if (simActive()) {
    f.s.distance_mm = (uint32_t)lroundf(g_simHeightCm * 10.0f);
    f.s.tof_status = 1;
    f.hasSample = true;
    f.lastRxMs = millis();
    f.heightSrc = "sim";
  } else if (f.hasSample && (millis() - f.lastRxMs) < FLOW_STALE_MS) {
    f.heightSrc = "mtf";
  } else if (altAt && (millis() - altAt) < FLOW_STALE_MS) {
    // 보드에 MTF-01P가 직접 연결돼 있지 않으면 FC가 받은 거리센서 값을 쓴다 (속도 정보는 없음)
    int32_t alt = g_fcAltCm;
    f.s.distance_mm = alt > 0 ? (uint32_t)alt * 10 : 0;
    f.s.tof_status = alt > 0 ? 1 : 0;
    f.hasSample = true;
    f.lastRxMs = altAt;
    f.heightSrc = "fc";
  }
  return f;
}

static bool flowConnected(const FlowState &f) {
  return f.hasSample && (millis() - f.lastRxMs) < FLOW_STALE_MS;
}

static bool heightValid(const FlowState &f) {
  return flowConnected(f) && f.s.tof_status == 1 && f.s.distance_mm > 0;
}

// `mtfraw` 명령: MTF-01P UART로 들어오는 다음 64바이트를 hex로 보여준다 (프로토콜·배선 진단용)
static volatile bool g_mtfRawRequest = false;

static void flowTask(void *) {
  MtfReader parser;  // Micolink 과 MSP(INAV 출력 설정)를 자동 구분
  uint8_t raw[64];
  size_t rawLen = 0;
  MtfSample smp;
  uint32_t prevTimeMs = 0, prevDistMm = 0;
  bool havePrev = false;
  uint32_t lastReport = 0;
  char line[400];

  for (;;) {
    while (Serial1.available()) {
      uint8_t b = Serial1.read();
      if (g_mtfRawRequest) {
        raw[rawLen++] = b;
        if (rawLen == sizeof(raw)) {
          char hex[2 * sizeof(raw) + 1], line[220];
          for (size_t i = 0; i < rawLen; i++) snprintf(hex + 2 * i, 3, "%02x", raw[i]);
          snprintf(line, sizeof(line), "{\"type\":\"mtf_raw\",\"baud\":%d,\"hex\":\"%s\"}", MTF_BAUD, hex);
          emitLine(line);
          rawLen = 0;
          g_mtfRawRequest = false;
        }
      }
      bool done = parser.feed(b, micros(), smp);
      portENTER_CRITICAL(&g_flowMux);
      g_flow.bytes++;
      g_flow.errors = parser.errors();
      g_flow.proto = parser.proto;
      if (done) {
        float up = g_flow.upCmS;
        if (smp.tof_status == 1 && smp.distance_mm > 0) {
          if (havePrev && smp.time_ms > prevTimeMs) {
            float dt = (smp.time_ms - prevTimeMs) / 1000.0f;
            float v = ((float)smp.distance_mm - (float)prevDistMm) / 10.0f / dt;
            up = 0.8f * up + 0.2f * v;  // 거리 미분은 잡음이 커서 저역통과
          }
          prevTimeMs = smp.time_ms;
          prevDistMm = smp.distance_mm;
          havePrev = true;
        } else {
          havePrev = false;
        }
        g_flow.s = smp;
        g_flow.upCmS = up;
        g_flow.lastRxMs = millis();
        g_flow.packets = parser.packets();
        g_flow.hasSample = true;
      }
      portEXIT_CRITICAL(&g_flowMux);
    }

    uint32_t now = millis();
    if (now - lastReport >= FLOW_REPORT_MS) {
      lastReport = now;
      FlowState f = flowSnapshot();
      bool conn = flowConnected(f);
      bool hv = heightValid(f);
      float hM = f.s.distance_mm / 1000.0f;
      int rxLevel = digitalRead(MTF_RX_PIN);
      if (conn && hv && strcmp(f.heightSrc, "fc") == 0) {
        // MTF-01P가 FC에 연결된 구성: 높이만 FC에서 받는다 (속도·품질은 FC가 처리)
        snprintf(line, sizeof(line),
                 "{\"type\":\"flow\",\"connected\":true,\"forward_cm_s\":null,\"right_cm_s\":null,"
                 "\"up_cm_s\":null,\"height_cm\":%.1f,\"height_src\":\"fc\",\"bytes\":%lu,\"packets\":%lu,"
                 "\"errors\":%lu,\"rx_level\":%d,\"axes_verified\":%s,\"sim\":false,\"proto\":\"%s\"}",
                 f.s.distance_mm / 10.0f, (unsigned long)f.bytes, (unsigned long)f.packets,
                 (unsigned long)f.errors, rxLevel, AXES_VERIFIED ? "true" : "false", f.proto);
      } else if (conn && hv) {
        // 1m 기준 속도 × 실제 높이(m) = cm/s. 축 매핑은 기존 RIDER-SERIAL과 동일(x→forward, y→right)
        snprintf(line, sizeof(line),
                 "{\"type\":\"flow\",\"connected\":true,\"forward_cm_s\":%.1f,\"right_cm_s\":%.1f,"
                 "\"up_cm_s\":%.1f,\"height_cm\":%.1f,\"height_src\":\"%s\",\"raw_x\":%d,\"raw_y\":%d,\"quality\":%u,"
                 "\"flow_status\":%u,\"tof_status\":%u,\"bytes\":%lu,\"packets\":%lu,\"errors\":%lu,"
                 "\"rx_level\":%d,\"axes_verified\":%s,\"sim\":%s,\"proto\":\"%s\"}",
                 f.s.flow_vel_x * hM, f.s.flow_vel_y * hM, f.upCmS, f.s.distance_mm / 10.0f, f.heightSrc,
                 f.s.flow_vel_x, f.s.flow_vel_y, f.s.flow_quality, f.s.flow_status, f.s.tof_status,
                 (unsigned long)f.bytes, (unsigned long)f.packets, (unsigned long)f.errors, rxLevel,
                 AXES_VERIFIED ? "true" : "false", simActive() ? "true" : "false", f.proto);
      } else {
        snprintf(line, sizeof(line),
                 "{\"type\":\"flow\",\"connected\":%s,\"forward_cm_s\":null,\"right_cm_s\":null,"
                 "\"up_cm_s\":null,\"height_cm\":null,\"raw_x\":%d,\"raw_y\":%d,\"quality\":%u,"
                 "\"flow_status\":%u,\"tof_status\":%u,\"bytes\":%lu,\"packets\":%lu,\"errors\":%lu,"
                 "\"rx_level\":%d,\"axes_verified\":%s,\"sim\":%s,\"proto\":\"%s\"}",
                 conn ? "true" : "false", conn ? f.s.flow_vel_x : 0, conn ? f.s.flow_vel_y : 0,
                 conn ? f.s.flow_quality : 0, conn ? f.s.flow_status : 0, conn ? f.s.tof_status : 0,
                 (unsigned long)f.bytes, (unsigned long)f.packets, (unsigned long)f.errors, rxLevel,
                 AXES_VERIFIED ? "true" : "false", simActive() ? "true" : "false", f.proto);
      }
      emitLine(line);
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// ---------------------------------------------------------------------------
// 카메라
// ---------------------------------------------------------------------------
static bool g_cameraOk = false;
static const char *g_sensorName = "none";
static int g_frameW = 0, g_frameH = 0;

static bool initCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;
  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;
  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;
  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;
  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM;
  c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.frame_size = CAM_FRAMESIZE;
  c.pixel_format = PIXFORMAT_GRAYSCALE;  // AprilTag 입력은 8비트 회색조
  c.grab_mode = CAMERA_GRAB_LATEST;      // 처리 중 밀린 프레임 대신 항상 최신 프레임
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.fb_count = 2;
  c.jpeg_quality = 12;

  if (esp_camera_init(&c) != ESP_OK) return false;

  sensor_t *s = esp_camera_sensor_get();
  if (s && CAM_EXP_FAST_DEFAULT) {  // 이동 중 흐림 방지 (`exp auto` 로 되돌릴 수 있다)
    s->set_exposure_ctrl(s, 0);
    s->set_aec_value(s, CAM_EXP_FAST_AEC);
    s->set_gain_ctrl(s, 0);
    s->set_agc_gain(s, CAM_EXP_FAST_GAIN);
  }
  if (s) {
    switch (s->id.PID) {
      case OV2640_PID: g_sensorName = "OV2640"; break;
      case OV3660_PID: g_sensorName = "OV3660"; break;
      case OV5640_PID: g_sensorName = "OV5640"; break;
      default: g_sensorName = "unknown"; break;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// AprilTag
// ---------------------------------------------------------------------------
static apriltag_family_t *g_tf = nullptr;
static apriltag_detector_t *g_td = nullptr;
static Preferences g_prefs;
static float g_fx = 0;  // 초점거리(px), 화각에서 계산
static bool g_snapRequested = false;

static const size_t TAG_COUNT = sizeof(TAG_TABLE) / sizeof(TAG_TABLE[0]);
static TagInfo g_tags[TAG_COUNT];  // TAG_TABLE + NVS 덮어쓰기

static const TagInfo *findTag(int id) {
  for (size_t i = 0; i < TAG_COUNT; i++)
    if (g_tags[i].id == id) return &g_tags[i];
  return nullptr;
}

static bool isMissionTag(const TagInfo *t) { return t && strcmp(t->code, "MISSION_TARGET") == 0; }

// 태그 크기 자동 보정 (src/tag_size_cal.h). 거리센서 높이로 실제 한 변을 재서, 안정되면 그 값을 쓴다.
static TagSizeCal g_tagCal[TAG_COUNT];
static bool g_tagCalOn = TAGCAL_ENABLED_DEFAULT;
static int tagIndex(const TagInfo *t) { return t ? (int)(t - g_tags) : -1; }
// 한 번이라도 수렴했는가 (잠깐 흐트러져도 마지막 수렴값을 계속 쓴다 — TagSizeCal::held)
static bool tagCalReady(int idx) { return idx >= 0 && g_tagCal[idx].held() > 0; }
// 거리·위치 계산에 쓰는 크기: 자동 보정이 수렴한 적이 있으면 마지막 수렴값, 아니면 설정값(config.h·NVS)
static float tagSizeUsedMm(const TagInfo *t, const char **src) {
  int idx = tagIndex(t);
  if (g_tagCalOn && tagCalReady(idx)) {
    *src = "auto";
    return g_tagCal[idx].held();
  }
  *src = "config";
  return (t ? *t : TAG_UNASSIGNED).size_mm;
}
// 네 꼭짓점이 모두 화면 가장자리에서 edge px 이상 안쪽인가 (잘린 태그는 크기 표본으로 쓰지 않는다)
static bool cornersInside(const apriltag_detection_t *det, float edge) {
  for (int i = 0; i < 4; i++)
    if (det->p[i][0] < edge || det->p[i][0] > g_frameW - edge || det->p[i][1] < edge || det->p[i][1] > g_frameH - edge)
      return false;
  return true;
}

static void nvsKey(char *out, char kind, int id) { snprintf(out, 8, "%c%d", kind, id); }

static void loadTags() {
  char k[8];
  for (size_t i = 0; i < TAG_COUNT; i++) {
    g_tags[i] = TAG_TABLE[i];
    int id = g_tags[i].id;
    nvsKey(k, 's', id);
    g_tags[i].size_mm = g_prefs.getUShort(k, g_tags[i].size_mm);
    nvsKey(k, 'x', id);
    g_tags[i].land_x_mm = g_prefs.getShort(k, g_tags[i].land_x_mm);
    nvsKey(k, 'y', id);
    g_tags[i].land_y_mm = g_prefs.getShort(k, g_tags[i].land_y_mm);
  }
}

static void initDetector() {
  // workerpool 스레드는 pthread로 만들어진다. 기본 스택(3KB)은 검출에 부족할 수 있다.
  esp_pthread_cfg_t cfg = esp_pthread_get_default_config();
  cfg.stack_size = 16 * 1024;
  esp_pthread_set_cfg(&cfg);

  g_tf = tag36h11_create();
  g_td = apriltag_detector_create();
  apriltag_detector_add_family_bits(g_td, g_tf, 1);  // 1비트 오류 정정 (587개 코드, 표 약 1MB → PSRAM)
  g_td->quad_decimate = DEC_START;
  g_td->quad_sigma = 0.0f;
  g_td->refine_edges = 1;
  g_td->decode_sharpening = 0.25f;
  g_td->nthreads = TAG_NTHREADS;
  g_td->debug = 0;
}

// 화면 기준 (오른쪽, 위쪽) 벡터를 장착 방향에 맞춰 기체 기준 (오른쪽, 앞쪽)으로 돌린다
static void imageToBody(float imgRight, float imgUp, float &bodyRight, float &bodyFwd) {
  switch (CAM_ROTATE_DEG) {
    case 90: bodyRight = imgUp; bodyFwd = -imgRight; break;
    case 180: bodyRight = -imgRight; bodyFwd = -imgUp; break;
    case 270: bodyRight = -imgUp; bodyFwd = imgRight; break;
    default: bodyRight = imgRight; bodyFwd = imgUp; break;
  }
}

static void describeMove(float right, float fwd, char *out, size_t n) {
  char a[48] = "", b[48] = "";
  if (fabsf(right) >= 0.5f) snprintf(a, sizeof(a), "%s %.1fcm", right > 0 ? "오른쪽" : "왼쪽", fabsf(right));
  if (fabsf(fwd) >= 0.5f) snprintf(b, sizeof(b), "%s %.1fcm", fwd > 0 ? "앞쪽" : "뒤쪽", fabsf(fwd));
  snprintf(out, n, "%s%s%s", a, (a[0] && b[0]) ? ", " : "", b);
}

// 태그 하나를 관측한 결과. 착륙 지점은 태그 중심에서 land_x/y 만큼 떨어진 점이다.
struct TagObs {
  const apriltag_detection_t *det;
  const TagInfo *info;  // nullptr = 미등록
  float side;           // 검은 정사각형 한 변(px), 네 변 평균
  float rotDeg;         // 인쇄물 +x 방향의 화면 회전(시계방향 +)
  float landPx[2];      // 착륙 지점의 화면 좌표
  float imgRight, imgUp, bodyRight, bodyFwd, offCm, distCm;
  bool centered;
  bool geomOk;    // 카메라 각도로 기체 기준 위치를 구함
  bool byHeight;  // 높이로 지면 교점을 구함 (false = 태그 크기로 잰 거리)
  float rayDown;  // 착륙 지점 광선의 아래 성분 (1 = 바로 아래)
  float sizeMm;   // 계산에 쓴 태그 한 변 (자동 보정 또는 설정값)
  const char *sizeSrc;
};

static TagObs observe(const apriltag_detection_t *det, const GimbalPose &gp, float heightCm, bool calAllowed) {
  TagObs o = {};
  o.det = det;
  o.info = findTag(det->id);
  const TagInfo &t = o.info ? *o.info : TAG_UNASSIGNED;

  for (int i = 0; i < 4; i++) {
    int j = (i + 1) % 4;
    o.side += hypotf(det->p[j][0] - det->p[i][0], det->p[j][1] - det->p[i][1]);
  }
  o.side /= 4.0f;
  o.rotDeg = atan2f(det->p[1][1] - det->p[0][1], det->p[1][0] - det->p[0][0]) * 180.0f / (float)M_PI;

  // 크기 자동 보정 표본: 실제 한 변 = 화면 한 변 × 거리센서 높이 / fx (조건은 loop 가 판단해 calAllowed 로 준다)
  int idx = tagIndex(o.info);
  if (calAllowed && idx >= 0 && heightCm > 0 && det->decision_margin >= TAGCAL_MIN_MARGIN &&
      cornersInside(det, TAGCAL_EDGE_PX) &&
      g_tagCal[idx].add(tagSizeSampleMm(o.side, heightCm, TAGCAL_CAM_ABOVE_SENSOR_CM, g_fx)))
    g_tagCal[idx].refresh(TAGCAL_MIN_N, TAGCAL_MAX_SPREAD);
  o.sizeMm = tagSizeUsedMm(o.info, &o.sizeSrc);

  // 호모그래피 좌표: 검은 정사각형이 [-1,1]², +x = 인쇄물 오른쪽, +y = 인쇄물 아래쪽.
  // 착륙 지점(mm, 인쇄물 위쪽 +)을 그대로 투영하므로 원근·회전이 반영된다.
  float half = o.sizeMm / 2.0f;
  homography_project(det->H, t.land_x_mm / half, -t.land_y_mm / half, &o.landPx[0], &o.landPx[1]);

  // 화면 평면 오프셋(참고용): 태그 자신의 크기로 cm/px 를 구한다
  float cmPerPx = (o.sizeMm / 10.0f) / o.side;
  o.imgRight = (o.landPx[0] - g_frameW / 2.0f) * cmPerPx;
  o.imgUp = -(o.landPx[1] - g_frameH / 2.0f) * cmPerPx;
  o.distCm = g_fx * (o.sizeMm / 10.0f) / o.side;

  // 기체 기준 위치: 착륙 지점 광선을 카메라 팬·틸트만큼 돌려 구한다 (src/gimbal_geom.h).
  // 팬 0°·틸트 0°(바로 아래)면 위의 cm/px 방식과 같은 값이 나온다.
  float nRight, nFwd;
  imageToBody((o.landPx[0] - g_frameW / 2.0f) / g_fx, -(o.landPx[1] - g_frameH / 2.0f) / g_fx, nRight, nFwd);
  GroundFix g = groundFix(camRayToBody(nFwd, nRight, gp.pan, gp.tilt), o.distCm, heightCm, gp.tilt,
                          GIMBAL_OBLIQUE_DEG, GIMBAL_MIN_DOWN);
  o.geomOk = g.ok && gp.known;
  o.byHeight = g.byHeight;
  o.rayDown = g.downFrac;
  o.bodyRight = g.right;
  o.bodyFwd = g.fwd;
  o.offCm = hypotf(o.bodyRight, o.bodyFwd);
  o.centered = o.geomOk && o.offCm <= CENTER_TOL_CM;
  return o;
}

static void heightJson(const FlowState &f, char *out, size_t n) {
  if (heightValid(f)) snprintf(out, n, "%.1f", f.s.distance_mm / 10.0f);
  else snprintf(out, n, "null");
}

static void reportTag(const TagObs &o) {
  const TagInfo &t = o.info ? *o.info : TAG_UNASSIGNED;
  char line[900];
  snprintf(line, sizeof(line),
           "{\"type\":\"tag\",\"family\":\"%s\",\"id\":%d,\"registered\":%s,\"meaning\":\"%s\",\"meaning_ko\":\"%s\","
           "\"hamming\":%d,\"margin\":%.1f,\"cx_px\":%.1f,\"cy_px\":%.1f,\"side_px\":%.1f,\"rot_deg\":%.1f,"
           "\"tag_size_mm\":%.0f,\"size_src\":\"%s\",\"size_cfg_mm\":%u,\"cal_n\":%d,\"cal_spread\":%.3f,\"land_x_mm\":%d,\"land_y_mm\":%d,\"land_px\":[%.1f,%.1f],"
           "\"img_right_cm\":%.1f,\"img_up_cm\":%.1f,\"dist_est_cm\":%.0f}",
           TAG_FAMILY_NAME, o.det->id, o.info ? "true" : "false", t.code, t.ko, o.det->hamming,
           o.det->decision_margin, o.det->c[0], o.det->c[1], o.side, o.rotDeg, o.sizeMm, o.sizeSrc, t.size_mm, tagIndex(o.info) >= 0 ? g_tagCal[tagIndex(o.info)].count() : 0,
           tagIndex(o.info) >= 0 ? g_tagCal[tagIndex(o.info)].spread() : 1.0f, t.land_x_mm,
           t.land_y_mm, o.landPx[0], o.landPx[1], o.imgRight, o.imgUp, o.distCm);
  emitLine(line);
}

// ---------------------------------------------------------------------------
// 미션 단계 (노션 20261001): 정렬 → 50cm → 3초 유지 → 100cm → 호버링
// 카메라 루프(core 1)가 프레임마다 갱신하고, MSP 태스크(core 0)가 읽어 스틱 값을 만든다.
// ---------------------------------------------------------------------------
enum Phase { PH_ALIGN, PH_GOTO_LOW, PH_HOLD_LOW, PH_GOTO_HIGH, PH_HOVER };
static const char *PHASE_CODE[] = {"ALIGN", "GOTO_50", "HOLD_50", "GOTO_100", "HOVER"};

// approach 파라미터 tag_vel_alpha 사본. updateMission(카메라 루프)이 읽고 `approach set`이 갱신한다.
static volatile float g_tagVelAlpha = 0.5f;

struct CtrlState {
  bool seen = false;      // 마지막 프레임에 미션 태그가 보였나
  uint32_t atMs = 0;      // 마지막으로 본 시각
  float bodyRight = 0, bodyFwd = 0;
  bool centered = false;
  int phase = PH_ALIGN;
  uint32_t holdMs = 0;    // 50cm 유지 누적 시간
  // 태그 위치 미분으로 구한 기체 속도 (cm/s, + = 기체가 오른쪽/앞으로 이동). 광류 교차 확인용, 3~4Hz
  bool tagVelValid = false;
  float tagVelRight = 0, tagVelFwd = 0;
  float distCm = 0;       // 태그 크기로 잰 카메라~태그 거리 (지상 시험에서 높이 대용)
  uint32_t captureMs = 0; // 그 프레임을 받은 시각 (칼만 지연 보정: 지연 = 지금 − 이 값 + kf_latency_ms)
  // 투어(-41): 이 프레임에서 본 미션 태그 전부 (태그가 없어도 프레임마다 갱신)
  uint8_t nSeen = 0;
  TourSeen seenTags[TOUR_MAX_SEEN];
  uint32_t seenAtMs = 0, seenCaptureMs = 0;
};
static CtrlState g_ctrl;
static volatile uint32_t g_frameCaptureMs = 0;  // 카메라 루프가 프레임을 받은 시각 (인식 처리 시간 전)
static portMUX_TYPE g_ctrlMux = portMUX_INITIALIZER_UNLOCKED;

static CtrlState ctrlSnapshot() {
  portENTER_CRITICAL(&g_ctrlMux);
  CtrlState c = g_ctrl;
  portEXIT_CRITICAL(&g_ctrlMux);
  return c;
}

static bool inBand(const FlowState &f, float targetCm) {
  return heightValid(f) && fabsf(f.s.distance_mm / 10.0f - targetCm) <= MISSION_BAND_CM;
}

// 태그를 놓치면 단계는 그대로 두고 진행하지 않는다 (명령은 MSP 태스크에서 중립)
static void updateMission(const TagObs *t, const FlowState &f, uint32_t dtMs) {
  CtrlState c = ctrlSnapshot();
  bool wasSeen = c.seen;
  c.seen = t != nullptr;
  if (t) {
    uint32_t now = millis();
    // 목표가 기체 기준으로 오른쪽으로 움직였다 = 기체가 왼쪽으로 이동했다 → 부호 반전
    if (wasSeen && c.atMs && now > c.atMs && now - c.atMs < 1000) {
      float dt = (now - c.atMs) / 1000.0f;
      float rawR = -(t->bodyRight - c.bodyRight) / dt;
      float rawF = -(t->bodyFwd - c.bodyFwd) / dt;
      // 3~6fps 위치 미분은 잡음이 크다 → 지수평활 (연속으로 보일 때만 이전 값과 섞는다)
      float a = g_tagVelAlpha;
      c.tagVelRight = c.tagVelValid ? c.tagVelRight + a * (rawR - c.tagVelRight) : rawR;
      c.tagVelFwd = c.tagVelValid ? c.tagVelFwd + a * (rawF - c.tagVelFwd) : rawF;
      c.tagVelValid = true;
    } else {
      c.tagVelValid = false;
    }
    c.atMs = now;
    c.captureMs = g_frameCaptureMs;
    c.bodyRight = t->bodyRight;
    c.bodyFwd = t->bodyFwd;
    c.centered = t->centered;
    c.distCm = t->distCm;
    switch (c.phase) {
      case PH_ALIGN:
        if (c.centered) c.phase = PH_GOTO_LOW;
        break;
      case PH_GOTO_LOW:
        if (inBand(f, MISSION_LOW_CM)) {
          c.phase = PH_HOLD_LOW;
          c.holdMs = 0;
        }
        break;
      case PH_HOLD_LOW:
        if (inBand(f, MISSION_LOW_CM)) {
          c.holdMs += dtMs;
          if (c.holdMs >= MISSION_HOLD_MS) c.phase = PH_GOTO_HIGH;
        } else if (heightValid(f)) {
          c.phase = PH_GOTO_LOW;  // 범위를 벗어나면 다시 맞춘다
        }
        break;
      case PH_GOTO_HIGH:
        if (inBand(f, MISSION_HIGH_CM)) c.phase = PH_HOVER;
        break;
      default:
        break;
    }
  } else {
    c.tagVelValid = false;
  }
  portENTER_CRITICAL(&g_ctrlMux);
  g_ctrl = c;
  portEXIT_CRITICAL(&g_ctrlMux);
}

static void publishSeen(const TourSeen *s, int n) {
  uint32_t now = millis(), cap = g_frameCaptureMs;
  portENTER_CRITICAL(&g_ctrlMux);
  g_ctrl.nSeen = (uint8_t)n;
  for (int i = 0; i < n && i < TOUR_MAX_SEEN; i++) g_ctrl.seenTags[i] = s[i];
  g_ctrl.seenAtMs = now;
  g_ctrl.seenCaptureMs = cap;
  portEXIT_CRITICAL(&g_ctrlMux);
}

static void resetMission() {
  portENTER_CRITICAL(&g_ctrlMux);
  g_ctrl.phase = PH_ALIGN;
  g_ctrl.holdMs = 0;
  portEXIT_CRITICAL(&g_ctrlMux);
}

static void phaseText(const CtrlState &c, const FlowState &f, char *out, size_t n) {
  float h = f.s.distance_mm / 10.0f;
  bool hv = heightValid(f);
  switch (c.phase) {
    case PH_ALIGN:
      snprintf(out, n, "%s", c.centered ? "정렬 완료 → 50cm로 이동 시작" : "착륙 지점 위로 수평 이동");
      break;
    case PH_GOTO_LOW:
      if (hv) snprintf(out, n, "50cm로 고도 이동 (현재 %.0fcm)", h);
      else snprintf(out, n, "%s", "50cm로 이동해야 하나 높이 정보 없음(MTF-01P 확인)");
      break;
    case PH_HOLD_LOW:
      snprintf(out, n, "50cm 유지 %.1f / %.1f초", c.holdMs / 1000.0f, MISSION_HOLD_MS / 1000.0f);
      break;
    case PH_GOTO_HIGH:
      if (hv) snprintf(out, n, "100cm로 상승 (현재 %.0fcm)", h);
      else snprintf(out, n, "%s", "100cm로 상승해야 하나 높이 정보 없음(MTF-01P 확인)");
      break;
    default:
      snprintf(out, n, "%s", "미션 완료: 100cm 호버링 유지");
      break;
  }
}

// 이번 프레임의 판단: 보이는 미션 태그 중 화면에 가장 크게 보이는 것(px당 오차가 가장 작음)을 쓴다
static void reportTarget(const TagObs &o, int candidates, uint32_t frameMs, const GimbalPose &gp,
                         bool controlUse) {
  FlowState f = flowSnapshot();
  CtrlState c = ctrlSnapshot();
  char move[112], hint[200], h[16];
  const char *decision;
  if (!controlUse) {  // 카메라가 움직이는 중이거나 각도를 몰라 위치를 믿을 수 없음 → 제어에 쓰지 않음
    decision = o.geomOk ? "GIMBAL_MOVING" : "NO_GEOMETRY";
    snprintf(move, sizeof(move), "%s", o.geomOk ? "카메라 이동 중 — 판단 보류" : "카메라 각도로 위치 계산 불가 — 판단 보류");
  } else if (o.centered) {
    decision = "OVER_TARGET";
    snprintf(move, sizeof(move), "%s", "이동 불필요");
  } else {
    decision = "MOVE";
    describeMove(o.bodyRight, o.bodyFwd, move, sizeof(move));
  }
  phaseText(c, f, hint, sizeof(hint));
  heightJson(f, h, sizeof(h));

  char line[1100];
  snprintf(line, sizeof(line),
           "{\"type\":\"target\",\"id\":%d,\"tag_size_mm\":%u,\"side_px\":%.1f,\"candidates\":%d,"
           "\"img_right_cm\":%.1f,\"img_up_cm\":%.1f,\"body_right_cm\":%.1f,\"body_fwd_cm\":%.1f,"
           "\"offset_cm\":%.1f,\"dist_est_cm\":%.0f,\"height_cm\":%s,\"height_src\":\"%s\",\"decision\":\"%s\","
           "\"move_ko\":\"%s\",\"next_ko\":\"%s\",\"phase\":\"%s\",\"decimate\":%.1f,\"axes_verified\":%s,"
           "\"gimbal\":{\"pan\":%.1f,\"tilt\":%.1f,\"settled\":%s},\"range_src\":\"%s\",\"ray_down\":%.2f,"
           "\"control_use\":%s,\"frame_ms\":%lu}",
           o.det->id, o.info->size_mm, o.side, candidates, o.imgRight, o.imgUp, o.bodyRight, o.bodyFwd, o.offCm,
           o.distCm, h, f.heightSrc, decision, move, hint, PHASE_CODE[c.phase], g_td->quad_decimate,
           AXES_VERIFIED ? "true" : "false", gp.pan, gp.tilt, gp.settled ? "true" : "false",
           o.byHeight ? "height" : "tag_size", o.rayDown, controlUse ? "true" : "false", (unsigned long)frameMs);
  emitLine(line);
}

// ---------------------------------------------------------------------------
// FC 연결: MSP v1 over UART (Serial2)
// MSP_SET_RAW_RC(200)로 조종 채널 값을 보내고, MSP_RC(105)로 FC가 실제 쓰는 채널 값을 읽어 확인한다.
// INAV에서 MSP RC Override 모드가 켜져 있고 msp_override_channels에 포함된 채널만 적용된다.
// ---------------------------------------------------------------------------
struct MspStats {
  uint32_t sent = 0, acks = 0, rcReplies = 0, errors = 0, motorAcks = 0;
  uint32_t lastRxMs = 0;
  uint32_t lastRawRcTxMs = 0;
  bool rawRcTxActive = false;
  bool lastRawRcKill = false;
  uint16_t out[MSP_CHANNELS] = {};
  uint16_t fcRc[MSP_CHANNELS] = {};
  uint8_t fcRcCount = 0;
  uint32_t fcRcAtMs = 0;         // 마지막 MSP_RC 응답 시각 (approach 줄의 fc_rp 신선도)
  uint32_t txHz = 0;            // 지난 1초 동안 실제로 보낸 SET_RAW_RC 수
  uint32_t txCountWindow = 0;
  uint32_t txWindowStartMs = 0;
  // FC 가 실제로 받아 처리한 비율의 증거: 지난 1초 동안의 SET_RAW_RC ack 수 · MSP_RC 응답 수.
  // INAV 는 MSP 포트당 10ms 에 명령 하나만 처리하므로(msp_serial.c "process one command at a time")
  // 보드가 초당 100개 넘게 보내면 프레임이 버려진다. ack_hz ≈ tx_hz 여야 정상.
  uint32_t ackHz = 0, rcHz = 0;
  uint32_t ackCountWindow = 0, rcCountWindow = 0;
};
static MspStats g_msp;
static portMUX_TYPE g_mspMux = portMUX_INITIALIZER_UNLOCKED;

// 1초 창이 끝나면 tx/ack/rc 초당 횟수를 확정한다 (g_mspMux 안에서 호출). 창 길이는 20ms 단위로
// 끊기므로 실제 경과 시간으로 정규화해 50 이 55 로 보이는 일을 막는다.
static void rateWindowStep(uint32_t now) {
  uint32_t elapsed = now - g_msp.txWindowStartMs;
  if (elapsed < 1000) return;
  if (g_msp.txWindowStartMs == 0) {  // 첫 창: 부팅 뒤 경과 시간이 섞이므로 버린다
    g_msp.txCountWindow = g_msp.ackCountWindow = g_msp.rcCountWindow = 0;
    g_msp.txWindowStartMs = now;
    return;
  }
  g_msp.txHz = (g_msp.txCountWindow * 1000u + elapsed / 2) / elapsed;
  g_msp.ackHz = (g_msp.ackCountWindow * 1000u + elapsed / 2) / elapsed;
  g_msp.rcHz = (g_msp.rcCountWindow * 1000u + elapsed / 2) / elapsed;
  g_msp.txCountWindow = g_msp.ackCountWindow = g_msp.rcCountWindow = 0;
  g_msp.txWindowStartMs = now;
}

// FC 채널 설정 (rcmap · msp_override_channels). 부팅 뒤 FC_CFG_POLL_MS 마다 읽는다.
// rcmap[논리] = 프레임 슬롯. 논리 순서 롤 0, 피치 1, 요 2, 스로틀 3 (INAV rx.c).
struct FcChannelConfig {
  uint8_t rcmap[4] = RCMAP_DEFAULT;
  bool rcmapFromFc = false;
  uint32_t rcmapAtMs = 0;
  int32_t overrideMask = -1;     // -1 = 아직 못 읽음
  uint32_t maskAtMs = 0;
  uint32_t maskQuerySentMs = 0;  // 0 = 우리가 보낸 0x1003 질의가 없음
  // 마지막으로 보낸 0x1003 질의 (응답에 이름이 없다): 0 마스크, 1 rc_expo, 2 deadband, 3 yaw_rate, 4 rc_yaw_expo, 5 yaw_deadband
  uint8_t settingQuery = 0;
  int16_t rcExpo = -1;           // FC rc_expo (접근 제어 기울기 출력의 곡선 역변환에 쓴다)
  int16_t deadband = -1;         // FC deadband (µs)
  uint32_t curveAtMs = 0;
  // 요 곡선 (투어 기수 맞추기: °/s → µs). yaw_rate 는 10°/s 단위
  int16_t yawRate = -1, yawExpo = -1, yawDeadband = -1;
  // FC MSP RC OVERRIDE 모드 배정 (MSP_MODE_RANGES, 10초마다). 보드의 AUTO 판정을 FC 와 같게 맞춘다
  bool ovrFromFc = false;        // FC 에서 배정을 찾았나 (없으면 config.h 기본 범위)
  uint8_t ovrCh = MSP_CH_AUTHORITY;
  uint16_t ovrMinUs = MSP_AUTH_AUTO_MIN_US, ovrMaxUs = MSP_AUTH_AUTO_MAX_US;
  uint32_t ovrAtMs = 0;
};
static FcChannelConfig g_fcCfg;  // g_mspMux 로 보호
static bool fcTakesThrottle(const FcChannelConfig &cfg, uint32_t now) {
  return cfg.overrideMask >= 0 && (cfg.overrideMask & OVERRIDE_MASK_THROTTLE_BIT) && (now - cfg.maskAtMs) < 60000;
}

static inline uint8_t frameSlot(uint8_t logicalCh) { return g_fcCfg.rcmap[logicalCh & 3]; }
static bool overrideMaskOk(int32_t mask) {
  // 롤·피치는 들어 있어야 한다 (요는 선택). 스로틀 비트는 고도 유지(-43)를 켠 빌드에서만 허용
  bool thrOk = ALT_HOLD_ENABLED || !(mask & OVERRIDE_MASK_THROTTLE_BIT);
  return mask >= 0 && thrOk && (mask & 1) && (mask & 2);
}
static void rcmapString(const uint8_t *rcmap, char *out) {
  // 슬롯 i 에 들어가는 논리 채널의 글자 (A 롤, E 피치, R 요, T 스로틀)
  static const char LETTERS[] = "AERT";
  for (int slot = 0; slot < 4; slot++) {
    out[slot] = '?';
    for (int logical = 0; logical < 4; logical++)
      if (rcmap[logical] == slot) out[slot] = LETTERS[logical];
  }
  out[4] = 0;
}

// 무충격 핸드오버: 수동일 때 조종기 롤·피치·요(MSP_RC)를 따라가고, AUTO 로 바뀌는 순간의 값을 기준점으로
// 삼아 handover_slew 로 1500 에 수렴시킨다. 제어기 오프셋은 이 기준점 위에 더한다.
// AUTO 중 MSP_RC 는 우리가 덮어쓴 값을 돌려주므로 그때는 추적하지 않는다.
struct Handover {
  float base[3] = {1500, 1500, 1500};   // 롤·피치·요 기준점 (µs)
  uint16_t pilot[3] = {1500, 1500, 1500};
  bool pilotValid = false;
  bool wasAuto = false;
  uint32_t autoSinceMs = 0;
  uint32_t lastTransitionMs = 0;
};
static Handover g_handover;  // mspTask 만 쓴다

// MTF-01 수직 속도 (기록·경고 전용 — 스로틀 제어에 쓰지 않는다)
struct VzTrack {
  float lastHeightCm = 0;
  uint32_t lastAtMs = 0;
  float vzCmS = 0;
  bool valid = false;
  uint32_t validAtMs = 0;
  uint32_t lastWarnMs = 0;
  uint32_t warnCount = 0;
};
static VzTrack g_vz;
static portMUX_TYPE g_vzMux = portMUX_INITIALIZER_UNLOCKED;
static RcAuthorityGate g_rcAuthority(MSP_CH_AUTHORITY, MSP_AUTH_AUTO_MIN_US, MSP_AUTH_AUTO_MAX_US,
                                     MSP_AUTH_TIMEOUT_MS);

struct RcAuthoritySnapshot {
  bool fresh;
  bool automatic;
  uint16_t channelUs;
  bool armSwitchDisarmed;
  bool throttleLow;
  uint16_t armSwitchUs;
  uint16_t throttleUs;
};

static RcAuthoritySnapshot rcAuthoritySnapshot(uint32_t nowMs) {
  portENTER_CRITICAL(&g_mspMux);
  g_rcAuthority.expire(nowMs);
  bool fresh = g_rcAuthority.fresh(nowMs);
  bool haveSafetyChannels = fresh && g_msp.fcRcCount > FC_RC_CH_ARM;
  uint16_t armUs = haveSafetyChannels ? g_msp.fcRc[FC_RC_CH_ARM] : 0;
  uint16_t throttleUs = haveSafetyChannels ? g_msp.fcRc[FC_RC_CH_THROTTLE] : 0;
  RcAuthoritySnapshot s = {
    fresh,
    g_rcAuthority.authority(nowMs) == RcAuthority::Auto,
    g_rcAuthority.channelUs(),
    haveSafetyChannels && armUs >= 750 && armUs <= FC_RC_SAFE_MAX_US,
    haveSafetyChannels && throttleUs >= 750 && throttleUs <= FC_RC_SAFE_MAX_US,
    armUs,
    throttleUs,
  };
  portEXIT_CRITICAL(&g_mspMux);
  return s;
}

// FC 상태 (MSP2_INAV_STATUS · MSP2_INAV_ANALOG · MSP_MOTOR)
struct FcStatus {
  uint32_t armingFlags = 0;
  uint32_t statusAtMs = 0;
  uint16_t vbatCv = 0;      // 0.01V
  int16_t ampsCa = 0;       // 0.01A
  uint8_t cells = 0;
  uint32_t analogAtMs = 0;
  uint16_t motor[4] = {0, 0, 0, 0};
  uint32_t motorAtMs = 0;
};
static FcStatus g_fc;

// FC 광류·자세 (접근 제어 입력). MTF-01P는 FC UART4에 있으므로 속도는 FC 값으로 추정한다.
struct FcFlow {
  uint8_t quality = 0;
  int16_t flowX = 0, flowY = 0;  // °/s (INAV 가 정렬·보정한 뒤의 값)
  int16_t bodyX = 0, bodyY = 0;  // 자이로 °/s
  uint32_t atMs = 0;
};
struct FcAttitude {
  int16_t rollDeci = 0, pitchDeci = 0;  // 0.1°
  int16_t yawDeg = 0;
  uint32_t atMs = 0;
};
static FcFlow g_fcFlow;
static FcAttitude g_fcAtt;
// 자세 기록 (10Hz × 1.6초) — 사진을 찍은 순간의 기울기로 태그 위치를 보정 (처리 지연 0.1~0.6초)
struct AttSample { uint32_t atMs; int16_t rollDeci, pitchDeci, yawDeg; };
static AttSample g_attHist[16];
static uint8_t g_attHead = 0;
// t 시각의 자세 (앞뒤 표본 사이 선형 보간, 기수 방향은 짧은 쪽으로). 300ms 안에 표본이 없으면 false. g_mspMux 밖에서 부른다
static bool attitudeAt(uint32_t t, float &rollDeg, float &pitchDeg, float *yawDeg = nullptr) {
  AttSample h[16];
  portENTER_CRITICAL(&g_mspMux);
  memcpy(h, g_attHist, sizeof(h));
  portEXIT_CRITICAL(&g_mspMux);
  const AttSample *before = nullptr, *after = nullptr;
  for (const auto &s : h) {
    if (!s.atMs) continue;
    if ((int32_t)(s.atMs - t) <= 0) {
      if (!before || (int32_t)(s.atMs - before->atMs) > 0) before = &s;
    } else if (!after || (int32_t)(s.atMs - after->atMs) < 0) {
      after = &s;
    }
  }
  if (before && after && after->atMs != before->atMs) {
    float k = (float)(t - before->atMs) / (float)(after->atMs - before->atMs);
    rollDeg = 0.1f * (before->rollDeci + k * (after->rollDeci - before->rollDeci));
    pitchDeg = 0.1f * (before->pitchDeci + k * (after->pitchDeci - before->pitchDeci));
    if (yawDeg) *yawDeg = tourWrap360(before->yawDeg + k * tourWrap180((float)(after->yawDeg - before->yawDeg)));
    return (int32_t)(t - before->atMs) < 300;
  }
  const AttSample *s = before ? before : after;
  if (!s || (uint32_t)abs((int32_t)(s->atMs - t)) > 300) return false;
  rollDeg = 0.1f * s->rollDeci;
  pitchDeg = 0.1f * s->pitchDeci;
  if (yawDeg) *yawDeg = (float)s->yawDeg;
  return true;
}

// INAV runtime_config.h armingFlag_e (bit → 이름)
static const struct { uint8_t bit; const char *name; } ARMING_BLOCKERS[] = {
  {6, "GEOZONE"}, {7, "FAILSAFE_SYSTEM"}, {8, "NOT_LEVEL"}, {9, "SENSORS_CALIBRATING"},
  {10, "SYSTEM_OVERLOADED"}, {11, "NAVIGATION_UNSAFE"}, {12, "COMPASS_NOT_CALIBRATED"},
  {13, "ACC_NOT_CALIBRATED"}, {14, "ARM_SWITCH"}, {15, "HARDWARE_FAILURE"}, {16, "BOXFAILSAFE"},
  {18, "RC_LINK"}, {19, "THROTTLE"}, {20, "CLI"}, {21, "CMS_MENU"}, {22, "OSD_MENU"},
  {23, "ROLLPITCH_NOT_CENTERED"}, {24, "SERVO_AUTOTRIM"}, {25, "OOM"}, {26, "INVALID_SETTING"},
  {27, "PWM_OUTPUT_ERROR"}, {28, "NO_PREARM"}, {29, "DSHOT_BEEPER"}, {30, "LANDING_DETECTED"},
};
static const uint32_t ARMED_FLAG = 1u << 2;

// fcmsp 직접 질의: 명령 하나를 보내고 같은 번호의 응답을 그대로 보여준다 (진단용)
struct FcQuery {
  bool pending = false, sent = false, done = false;
  uint16_t cmd = 0;
  uint8_t payload[64];
  uint16_t payloadLen = 0;
  uint32_t sentAtMs = 0;
  bool error = false;
  uint16_t replyLen = 0;
  uint8_t reply[512];
};
static FcQuery g_query;
static portMUX_TYPE g_queryMux = portMUX_INITIALIZER_UNLOCKED;

struct ControlFlags {
  bool mspEnabled;
  bool kill;
  uint8_t killReleaseFrames;
};
static ControlFlags g_control = {MSP_ENABLED_DEFAULT, false, 0};
static portMUX_TYPE g_controlMux = portMUX_INITIALIZER_UNLOCKED;

static ControlFlags controlFlagsSnapshot() {
  portENTER_CRITICAL(&g_controlMux);
  ControlFlags flags = g_control;
  portEXIT_CRITICAL(&g_controlMux);
  return flags;
}

static void setMspEnabled(bool enabled) {
  portENTER_CRITICAL(&g_controlMux);
  g_control.mspEnabled = enabled;
  portEXIT_CRITICAL(&g_controlMux);
}

static void setKill(bool kill) {
  portENTER_CRITICAL(&g_controlMux);
  if (!kill && g_control.kill) g_control.killReleaseFrames = 10;
  g_control.kill = kill;
  if (kill) g_control.killReleaseFrames = 0;
  portEXIT_CRITICAL(&g_controlMux);
}

static void consumeKillReleaseFrame() {
  portENTER_CRITICAL(&g_controlMux);
  if (g_control.killReleaseFrames) g_control.killReleaseFrames--;
  portEXIT_CRITICAL(&g_controlMux);
}

static inline uint16_t rd16(const uint8_t *p) { return p[0] | (p[1] << 8); }
static inline uint32_t rd32(const uint8_t *p) { return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24); }

static void handleMspReply(const MspReply &r) {
  portENTER_CRITICAL(&g_mspMux);
  if (!r.ckOk || r.error) {
    g_msp.errors++;
  } else {
    g_msp.lastRxMs = millis();
    if (r.cmd == MSP_SET_RAW_RC) {
      g_msp.acks++;
      g_msp.ackCountWindow++;
    } else if (r.cmd == MSP_SET_MOTOR) {
      g_msp.motorAcks++;
    } else if (r.cmd == MSP_RC) {
      g_msp.rcReplies++;
      g_msp.rcCountWindow++;
      uint8_t n = min<uint16_t>(r.size / 2, MSP_CHANNELS);
      for (uint8_t i = 0; i < n; i++) g_msp.fcRc[i] = r.payload[2 * i] | (r.payload[2 * i + 1] << 8);
      g_msp.fcRcCount = n;
      g_msp.fcRcAtMs = millis();
      g_rcAuthority.observe(g_msp.fcRc, n, millis());
    } else if (r.cmd == MSP2_INAV_STATUS && r.size >= 13) {
      g_fc.armingFlags = rd32(r.payload + 9);  // cycle·i2c·sensors·load(각 u16) + profiles(u8) 다음
      g_fc.statusAtMs = millis();
    } else if (r.cmd == MSP2_INAV_ANALOG && r.size >= 5) {
      g_fc.cells = r.payload[0] >> 4;
      g_fc.vbatCv = rd16(r.payload + 1);
      g_fc.ampsCa = (int16_t)rd16(r.payload + 3);
      g_fc.analogAtMs = millis();
    } else if (r.cmd == MSP_MOTOR && r.size >= 8) {
      for (int i = 0; i < 4; i++) g_fc.motor[i] = rd16(r.payload + 2 * i);
      g_fc.motorAtMs = millis();
    } else if (r.cmd == MSP_SONAR_ALTITUDE && r.size >= 4) {
      g_fcAltCm = (int32_t)rd32(r.payload);
      g_fcAltAtMs = millis();
    } else if (r.cmd == MSP2_INAV_OPTICAL_FLOW && r.size >= 9) {
      g_fcFlow.quality = r.payload[0];
      g_fcFlow.flowX = (int16_t)rd16(r.payload + 1);
      g_fcFlow.flowY = (int16_t)rd16(r.payload + 3);
      g_fcFlow.bodyX = (int16_t)rd16(r.payload + 5);
      g_fcFlow.bodyY = (int16_t)rd16(r.payload + 7);
      g_fcFlow.atMs = millis();
    } else if (r.cmd == MSP_ATTITUDE && r.size >= 6) {
      g_fcAtt.rollDeci = (int16_t)rd16(r.payload);
      g_fcAtt.pitchDeci = (int16_t)rd16(r.payload + 2);
      g_fcAtt.yawDeg = (int16_t)rd16(r.payload + 4);
      g_fcAtt.atMs = millis();
      g_attHist[g_attHead] = {g_fcAtt.atMs, g_fcAtt.rollDeci, g_fcAtt.pitchDeci, g_fcAtt.yawDeg};
      g_attHead = (g_attHead + 1) % 16;
    } else if (r.cmd == MSP_MODE_RANGES && r.size >= 4) {
      uint8_t ci;
      uint16_t lo, hi;
      bool found = mspFindModeRange(r.payload, r.size, INAV_BOX_MSP_RC_OVERRIDE, ci, lo, hi);
      if (found && g_rcAuthority.setRange(ci, lo, hi)) {
        g_fcCfg.ovrCh = ci;
        g_fcCfg.ovrMinUs = lo;
        g_fcCfg.ovrMaxUs = hi;
      }
      g_fcCfg.ovrFromFc = found;
      g_fcCfg.ovrAtMs = millis();
    } else if (r.cmd == MSP_RX_MAP && r.size >= 4) {
      // 0..3 의 순열이어야 한다. 아니면 기본값 유지
      bool seen[4] = {false, false, false, false}, ok = true;
      for (int i = 0; i < 4; i++) {
        if (r.payload[i] > 3 || seen[r.payload[i]]) ok = false;
        else seen[r.payload[i]] = true;
      }
      if (ok) {
        memcpy(g_fcCfg.rcmap, r.payload, 4);
        g_fcCfg.rcmapFromFc = true;
      }
      g_fcCfg.rcmapAtMs = millis();
    } else if (r.cmd == MSP2_COMMON_SETTING && g_fcCfg.maskQuerySentMs && r.size >= 1 && r.size <= 4) {
      // 우리가 보낸 msp_override_channels 질의의 응답 (값 크기는 설정 타입에 따라 1·2·4바이트)
      uint32_t v = 0;
      for (uint16_t i = 0; i < r.size; i++) v |= (uint32_t)r.payload[i] << (8 * i);
      if (g_fcCfg.settingQuery == 1) {
        g_fcCfg.rcExpo = (int16_t)v;
        g_fcCfg.curveAtMs = millis();
      } else if (g_fcCfg.settingQuery == 2) {
        g_fcCfg.deadband = (int16_t)v;
        g_fcCfg.curveAtMs = millis();
      } else if (g_fcCfg.settingQuery == 3) {
        g_fcCfg.yawRate = (int16_t)v;
      } else if (g_fcCfg.settingQuery == 4) {
        g_fcCfg.yawExpo = (int16_t)v;
      } else if (g_fcCfg.settingQuery == 5) {
        g_fcCfg.yawDeadband = (int16_t)v;
      } else {
        g_fcCfg.overrideMask = (int32_t)v;
        g_fcCfg.maskAtMs = millis();
      }
      g_fcCfg.maskQuerySentMs = 0;
    }
  }
  portEXIT_CRITICAL(&g_mspMux);
  // 진단 질의 응답 (오류 응답도 그대로 보여준다)
  portENTER_CRITICAL(&g_queryMux);
  if (g_query.sent && !g_query.done && r.cmd == g_query.cmd) {
    g_query.error = r.error || !r.ckOk;
    g_query.replyLen = min<uint16_t>(r.size, sizeof(g_query.reply));
    memcpy(g_query.reply, r.payload, g_query.replyLen);
    g_query.done = true;
  }
  portEXIT_CRITICAL(&g_queryMux);
}

static uint16_t stick(float us, int maxUs) {
  float v = fmaxf(-maxUs, fminf(maxUs, us));
  return (uint16_t)lroundf(1500.0f + v);
}

// ---------------------------------------------------------------------------
// 접근 제어 (src/approach.h). mspTask 가 20Hz 로 step 하고, computeSticks 가 결과를 쓴다.
// 모드: off(계산 안 함) · shadow(계산·로그만, FC에는 중립) · on(FC로 출력) · legacy(기존 4µs/cm 위치 제어)
// ---------------------------------------------------------------------------
enum ApproachMode { APM_OFF = 0, APM_SHADOW = 1, APM_ON = 2, APM_LEGACY = 3 };
static const char *APPROACH_MODE_NAME[] = {"off", "shadow", "on", "legacy"};
static volatile int g_approachMode = APPROACH_MODE_DEFAULT;
static ApproachController g_approach;
// 비행 속도 기본값 (config.h AP_*) — 부팅 때와 `approach reset` 때 적용
static void applyFlightSpeed(ApproachParams &p) {
  p.maxSpeedCmS = AP_MAX_SPEED_CMS;
  p.slowRadiusCm = AP_SLOW_RADIUS_CM;
  p.maxAccelCmS2 = AP_PROFILE_ACCEL;
  p.kfSearchSpeedCmS = AP_SEARCH_SPEED_CMS;
  p.maxAccelCmdCmS2 = AP_MAX_ACCEL_CMD;
  p.overspeedCmS = AP_OVERSPEED_CMS;
}
// 고도 유지·착륙 (src/alt_hold.h). mspTask 가 매 주기 step (g_approachMux 로 출력 보호)
static AltHold g_alt;
static AltHoldOutput g_altOut;
static volatile bool g_altLandCmd = false;  // `alt land` 명령 (수동으로 바뀌면 지운다)
static bool g_altSeeded = false;            // 부팅 뒤 FC 스로틀을 한 번 읽었다 (그 전에는 프레임을 보내지 않는다)

// 태그 투어 (src/tour.h). mspTask 가 step 하고, 명령이 켜고 끈다·파라미터를 바꾼다 (g_approachMux)
static TagTour g_tour;
static volatile bool g_tourOn = TOUR_DEFAULT_ON;
static TourOutput g_tourOut;          // 마지막 출력 (로그용)
static float g_tourYawUs = 0;         // 요 스틱 오프셋 (변화율 제한 후, µs)
// 왕복 이동(-53): 정해진 기울기 출력. 머무는 동안 접근 제어가 낸 평균 명령(=수평 유지에 필요한 기울기)을 더한다
static bool g_tourOl = false;
static float g_tourOlRollUs = 0, g_tourOlPitchUs = 0;
static float g_tourTrimRollUs = 0, g_tourTrimPitchUs = 0;
static const char *g_tourYawSrc = "default";  // 요 곡선 출처: fc | default
static volatile int g_tourFocusId = -1;       // 카메라 루프: ROI·대표 태그를 이 ID 로 (-1 = 가장 크게 보이는 태그)
static volatile bool g_tourScanFull = false;  // 카메라 루프: 다음 태그 찾기 → 가끔 전체 화면 인식
static ApproachInput g_approachIn;    // 마지막 입력 (로그용)
static ApproachOutput g_approachOut;  // 마지막 출력
static uint32_t g_approachStepMs = 0;
static float g_approachTagVelR = 0, g_approachTagVelF = 0;
static bool g_approachTagVelValid = false;
static const char *g_approachHSrc = "none";  // 이번 step 에 쓴 높이 출처: rangefinder | tag | none
static float g_approachOf[2] = {0, 0};       // 광류 속도 (오른쪽, 앞) — vel_src 와 무관하게 교차 확인용
static bool g_approachOfValid = false;
static portMUX_TYPE g_approachMux = portMUX_INITIALIZER_UNLOCKED;

static void approachStep(const RcAuthoritySnapshot &authority, const ControlFlags &flags, float dtS) {
  uint32_t now = millis();
  CtrlState c = ctrlSnapshot();
  FlowState f = flowSnapshot();
  FcFlow fl;
  FcAttitude att;
  FcChannelConfig cfg;
  portENTER_CRITICAL(&g_mspMux);
  fl = g_fcFlow;
  att = g_fcAtt;
  cfg = g_fcCfg;
  portEXIT_CRITICAL(&g_mspMux);

  portENTER_CRITICAL(&g_approachMux);
  ApproachParams p = g_approach.p;
  portEXIT_CRITICAL(&g_approachMux);

  ApproachInput in;
  in.nowMs = now;
  in.enabled = g_approachMode == APM_ON || g_approachMode == APM_SHADOW;
  in.authorityAuto = authority.automatic;
  in.kill = flags.kill;
  // FC 마스크: 최근에 읽었고 스로틀 비트가 없어야 출력한다 (요구 2)
  in.maskKnown = cfg.overrideMask >= 0 && (now - cfg.maskAtMs) < FC_MASK_VALID_MS;
  in.maskOk = in.maskKnown && overrideMaskOk(cfg.overrideMask);
  bool curveFresh = cfg.curveAtMs && (now - cfg.curveAtMs) < 180000;  // 각 값은 50초마다 다시 읽는다
  in.fcRcExpo = curveFresh ? cfg.rcExpo : -1;     // 모르면 파라미터 rc_expo(80)
  in.fcDeadbandUs = curveFresh ? cfg.deadband : -1;

  bool hv = heightValid(f);
  float hCm = hv ? f.s.distance_mm / 10.0f : 0.0f;
  uint32_t hAt = f.lastRxMs;
  const char *hSrc = hv ? "rangefinder" : "none";
  bool tagRecent = c.seen && c.atMs && (now - c.atMs) <= p.tagStaleMs;
  if (!hv && p.heightSrc != 0 && tagRecent && c.distCm > 0) {
    // 지상 시험: 거리센서가 무효(작업대 위)면 카메라~태그 거리(태그 크기 추정)를 높이로 쓴다
    hv = true;
    hCm = c.distCm;
    hAt = c.atMs;
    hSrc = "tag";
  }
  in.heightValid = hv;
  in.heightAtMs = hAt;
  in.heightCm = hCm;

  // 위치 오차: 카메라 → 무게중심 오프셋, 선택적으로 기체 기울기 보정 (부호는 실측으로 확정)
  in.tagValid = c.seen && c.atMs != 0;
  in.tagAtMs = c.atMs;
  in.tagCaptureMs = c.captureMs;
  in.applied = g_approachMode == APM_ON;  // shadow 는 계산만 → FC 에는 기준점(→1500)만 간다
  in.baseRollUs = g_handover.base[0] - 1500.0f;
  in.basePitchUs = g_handover.base[1] - 1500.0f;
  // MANUAL 중 FC 가 쓰는 값 = 조종자 스틱 (MSP_RC). 칼만이 수동 중에도 기울기를 알아 외란을 미리 배운다
  {
    MspStats ms;
    portENTER_CRITICAL(&g_mspMux);
    ms = g_msp;
    portEXIT_CRITICAL(&g_mspMux);
    bool rcFresh = ms.fcRcAtMs && (now - ms.fcRcAtMs) < 200 && ms.fcRcCount > FC_RC_CH_PITCH;
    in.pilotValid = !authority.automatic && authority.fresh && rcFresh;
    in.pilotRollUs = rcFresh ? (float)ms.fcRc[FC_RC_CH_ROLL] - 1500.0f : 0.0f;
    in.pilotPitchUs = rcFresh ? (float)ms.fcRc[FC_RC_CH_PITCH] - 1500.0f : 0.0f;
  }
  float errR = c.bodyRight + p.camOffsetRightCm;
  float errF = c.bodyFwd + p.camOffsetFwdCm;
  bool attFresh = att.atMs && (now - att.atMs) < 300;
  // 촬영 순간 자세로 보정 (자세 기록 보간). 기록이 없으면 지금 자세
  float capRoll = att.rollDeci * 0.1f, capPitch = att.pitchDeci * 0.1f;
  bool capOk = c.captureMs ? attitudeAt(c.captureMs, capRoll, capPitch) : attFresh;
  if (p.attComp != 0 && capOk && hv) {
    errR -= p.attRollSign * hCm * tanf(capRoll * (float)M_PI / 180.0f);
    errF -= p.attPitchSign * hCm * tanf(capPitch * (float)M_PI / 180.0f);
  }
  // 칼만 가속도 입력: 지금 FC 가 잰 실제 자세 (스틱과 같은 부호 — 20261011 비행 확인)
  in.attValid = attFresh;
  in.attAtMs = att.atMs;
  in.attRollDeg = att.rollDeci * 0.1f;
  in.attPitchDeg = att.pitchDeci * 0.1f;
  in.errRightCm = errR;
  in.errFwdCm = errF;

  // 기체가 돌면 칼만이 들고 있는 태그 위치·속도도 기체 기준으로 돌린다 (FC 기수 10Hz). 한 번에 45° 넘으면 이상값
  {
    static uint32_t lastYawAt = 0;
    static int16_t lastYaw = 0;
    if (att.atMs && att.atMs != lastYawAt) {
      if (lastYawAt && att.atMs - lastYawAt < 500 && p.velSrc == 2) {
        float d = tourWrap180((float)(att.yawDeg - lastYaw));
        if (fabsf(d) < 45.0f) g_approach.rotateFrame(d);
      }
      lastYawAt = att.atMs;
      lastYaw = att.yawDeg;
    }
  }

  // 태그 투어 (-41): 따라갈 태그·목표점·요는 투어가 정한다
  static TourInput ti;  // mspTask 스택 절약
  ti = TourInput();
  ti.nowMs = now;
  bool altReady;
  bool altThrottle;
  {
    portENTER_CRITICAL(&g_approachMux);
    altReady = g_altOut.ready;
    portEXIT_CRITICAL(&g_approachMux);
    portENTER_CRITICAL(&g_mspMux);
    altThrottle = fcTakesThrottle(g_fcCfg, now);
    portEXIT_CRITICAL(&g_mspMux);
  }
  // 보드가 스로틀을 쓰는 설정이면 이륙이 끝나 고도 유지 중일 때만 투어 (이륙 중에는 태그 위에서 오르기만)
  ti.active = g_tourOn && in.enabled && authority.automatic && !flags.kill && (!altThrottle || altReady);
  {
    // 이륙 태그: 이륙 중(바닥 대기·상승) 처음 본 미션 태그 = 왕복 미션의 A. 수동이면 지운다
    static int homeId = -1;
    AltState as;
    portENTER_CRITICAL(&g_approachMux);
    as = g_altOut.state;
    portEXIT_CRITICAL(&g_approachMux);
    if (!authority.automatic || as == AltState::Ground) homeId = -1;
    if (as == AltState::Takeoff && homeId < 0 && c.nSeen > 0 && c.seenAtMs && now - c.seenAtMs < 600) {
      int best = 0;
      for (int i = 1; i < c.nSeen && i < TOUR_MAX_SEEN; i++)
        if (hypotf(c.seenTags[i].rightCm, c.seenTags[i].fwdCm) < hypotf(c.seenTags[best].rightCm, c.seenTags[best].fwdCm))
          best = i;
      homeId = c.seenTags[best].id;
    }
    ti.homeId = homeId;
  }
  ti.frameAtMs = c.seenAtMs;
  ti.n = c.nSeen;
  for (int i = 0; i < c.nSeen && i < TOUR_MAX_SEEN; i++) ti.tags[i] = c.seenTags[i];
  ti.yawValid = attFresh;
  ti.yawDeg = att.yawDeg;
  float sr = 0, sp = 0, sy = 0;
  bool seenAttOk = c.seenCaptureMs && attitudeAt(c.seenCaptureMs, sr, sp, &sy);
  ti.yawCapValid = seenAttOk;
  ti.yawCapDeg = sy;
  portENTER_CRITICAL(&g_approachMux);
  ti.goalKnown = g_approachOut.distValid;
  ti.distToGoalCm = g_approachOut.distCm;
  portEXIT_CRITICAL(&g_approachMux);
  TourOutput to = g_tour.step(ti, dtS);
  if (to.state != TourState::Off) {
    in.tagValid = to.targetInFrame;
    in.tagAtMs = to.targetInFrame ? c.seenAtMs : 0;
    in.tagCaptureMs = c.seenCaptureMs;
    float tr = to.targetR + p.camOffsetRightCm, tf = to.targetF + p.camOffsetFwdCm;
    if (p.attComp != 0 && seenAttOk && hv) {  // 촬영 순간 기울기만큼 카메라가 본 곳이 밀린 것을 뺀다
      tr -= p.attRollSign * hCm * tanf(sr * (float)M_PI / 180.0f);
      tf -= p.attPitchSign * hCm * tanf(sp * (float)M_PI / 180.0f);
    }
    in.errRightCm = tr;
    in.errFwdCm = tf;
    in.goalRightCm = to.goalR;
    in.goalFwdCm = to.goalF;
    in.speedCapCmS = to.speedCapCmS;
    in.kfLostMs = to.kfLostMs;
    if (to.resetKf) g_approach.reset();       // 정해진 기울기 이동 뒤: 칼만 처음부터 (예측 속도를 믿지 않는다)
    else if (to.retarget) g_approach.retarget();  // 다음 태그: 칼만 위치만 새로 (속도는 이어서)
  }
  {
    // 착륙 중: 내려가다 태그가 안 보여도(너무 가까움) 칼만 예측으로 그 자리를 지킨다 → 최대한 수직 하강
    AltState as;
    portENTER_CRITICAL(&g_approachMux);
    as = g_altOut.state;
    portEXIT_CRITICAL(&g_approachMux);
    if (as == AltState::Landing || as == AltState::Touchdown) {
      in.kfLostMs = ALT_LAND_KF_LOST_MS;
      in.speedCapCmS = ALT_LAND_SPEED_CMS;
    }
  }
  g_tourFocusId = to.state != TourState::Off ? to.focusId : -1;
  g_tourScanFull = to.scanFull;
  // 요: °/s → INAV 요 곡선 역변환 (FC 에서 읽은 yaw_rate·rc_yaw_expo·yaw_deadband, 못 읽었으면 INAV 기본값)
  float yawUs = 0;
  bool yawFc = cfg.yawRate > 0 && cfg.yawExpo >= 0 && cfg.yawDeadband >= 0;
  if (to.yawOn) {
    float rateMax = yawFc ? cfg.yawRate * 10.0f : TOUR_FC_YAW_RATE_DPS;
    float expo = yawFc ? (float)cfg.yawExpo : TOUR_FC_YAW_EXPO;
    float db = yawFc ? (float)cfg.yawDeadband : TOUR_FC_YAW_DEADBAND;
    if (expo > 99) expo = 99;
    float lim = g_tour.p.yawUsMax;
    yawUs = fmaxf(-lim, fminf(lim, g_tour.p.yawSign * tiltToStickUs(to.yawRateDegS, expo, db, rateMax)));
  }
  float yawStep = TOUR_YAW_SLEW_US_S * dtS;
  float yawCmd = g_tourYawUs + fmaxf(-yawStep, fminf(yawStep, yawUs - g_tourYawUs));

  // 속도 추정
  //  vel_src 0: (유량 − 자이로)[rad/s] × 높이[cm]. 축 대응·부호는 파라미터 (실측 전)
  //  vel_src 1: 카메라로 잰 태그 상대 속도 (−Δ오차/Δt, 평활). 기체가 서 있고 태그가 움직여도 상대 속도가 된다
  bool flowFresh = fl.atMs && (now - fl.atMs) < p.flowStaleMs;
  in.flowQuality = fl.quality;
  // 광류 속도는 출처와 관계없이 계산해 기록한다 (카메라 속도와 비교해 광류 축·부호를 검증)
  bool ofValid = flowFresh && hv;
  float ofR = 0, ofF = 0;
  if (ofValid) {
    float rA = (fl.flowX - fl.bodyX) * (float)M_PI / 180.0f, rB = (fl.flowY - fl.bodyY) * (float)M_PI / 180.0f;
    ofF = p.flowFwdSign * (p.flowSwapXY != 0 ? rB : rA) * hCm;
    ofR = p.flowRightSign * (p.flowSwapXY != 0 ? rA : rB) * hCm;
  }
  if (p.velSrc != 0) {
    in.velFromTag = true;
    in.velValid = c.tagVelValid && tagRecent && hv;
    in.velAtMs = c.atMs;
    in.velRightCmS = c.tagVelRight;
    in.velFwdCmS = c.tagVelFwd;
  } else {
  in.velValid = flowFresh && hv;
  in.velAtMs = fl.atMs;
  if (in.velValid) {
    float rateA = (fl.flowX - fl.bodyX) * (float)M_PI / 180.0f;
    float rateB = (fl.flowY - fl.bodyY) * (float)M_PI / 180.0f;
    float fwdRate = p.flowSwapXY != 0 ? rateB : rateA;
    float rightRate = p.flowSwapXY != 0 ? rateA : rateB;
    in.velFwdCmS = p.flowFwdSign * fwdRate * hCm;
    in.velRightCmS = p.flowRightSign * rightRate * hCm;
  }
  }

  ApproachOutput out = g_approach.step(in, dtS);
  // 머무는 동안 접근 제어 명령 평균 = 그 자리에 서 있는 데 필요한 기울기 (수평 보정 오차·바람)
  if (to.state == TourState::Dwell && !out.fault) {
    g_tourTrimRollUs += 0.01f * (out.rollUs - g_tourTrimRollUs);
    g_tourTrimPitchUs += 0.01f * (out.pitchUs - g_tourTrimPitchUs);
  }
  float olRoll = 0, olPitch = 0;
  if (to.openLoop) {
    float expo = in.fcRcExpo >= 0 ? (float)in.fcRcExpo : p.rcExpo;
    float db = in.fcDeadbandUs >= 0 ? (float)in.fcDeadbandUs : p.rcDeadbandUs;
    olRoll = g_tourTrimRollUs;
    olPitch = g_tourTrimPitchUs + p.pitchSign * tiltToStickUs(to.olTiltFwdDeg, expo, db, p.maxAngleDeg);
  }
  portENTER_CRITICAL(&g_approachMux);
  g_tourOl = to.openLoop && to.state != TourState::Off;
  g_tourOlRollUs = olRoll;
  g_tourOlPitchUs = olPitch;
  g_tourOut = to;
  g_tourYawUs = yawCmd;
  g_tourYawSrc = yawFc ? "fc" : "default";
  g_approachIn = in;
  g_approachOut = out;
  g_approachStepMs = now;
  g_approachTagVelR = c.tagVelRight;
  g_approachTagVelF = c.tagVelFwd;
  g_approachTagVelValid = c.tagVelValid;
  g_approachHSrc = hSrc;
  g_approachOf[0] = ofR;
  g_approachOf[1] = ofF;
  g_approachOfValid = ofValid;
  portEXIT_CRITICAL(&g_approachMux);
}

// MTF-01 수직 속도: 높이 샘플이 바뀔 때마다 Δh/Δt 를 EMA 로 평활. |vz| > VZ_WARN_CM_S 면 경고 줄만 낸다.
// 보드는 어떤 경우에도 스로틀 슬롯 값을 바꾸지 않는다 (MSP_THROTTLE_SAFE_US 고정).
static void vzTrackStep() {
  FlowState f = flowSnapshot();
  uint32_t now = millis();
  bool hv = heightValid(f);
  portENTER_CRITICAL(&g_vzMux);
  VzTrack v = g_vz;
  portEXIT_CRITICAL(&g_vzMux);
  if (hv && f.lastRxMs != v.lastAtMs) {
    float h = f.s.distance_mm / 10.0f;
    if (v.lastAtMs && f.lastRxMs > v.lastAtMs) {
      float dt = (f.lastRxMs - v.lastAtMs) / 1000.0f;
      if (dt >= 0.02f && dt <= 1.0f) {
        float raw = (h - v.lastHeightCm) / dt;
        v.vzCmS = v.valid ? v.vzCmS + 0.3f * (raw - v.vzCmS) : raw;
        v.valid = true;
        v.validAtMs = now;
      }
    }
    v.lastHeightCm = h;
    v.lastAtMs = f.lastRxMs;
  }
  if (v.valid && (now - v.validAtMs) > 600) v.valid = false;  // 높이가 끊기면 vz 도 무효 (수동 스로틀에는 영향 없음)
  bool warn = v.valid && fabsf(v.vzCmS) > VZ_WARN_CM_S;
  if (warn && (now - v.lastWarnMs) >= VZ_WARN_REPEAT_MS) {
    v.lastWarnMs = now;
    v.warnCount++;
    char line[200];
    snprintf(line, sizeof(line),
             "{\"type\":\"warn\",\"code\":\"vz\",\"vz_cm_s\":%.1f,\"limit_cm_s\":%.0f,\"height_cm\":%.1f,"
             "\"height_src\":\"%s\",\"throttle_action\":\"none\"}",
             v.vzCmS, VZ_WARN_CM_S, v.lastHeightCm, f.heightSrc);
    emitLine(line);
  }
  portENTER_CRITICAL(&g_vzMux);
  g_vz = v;
  portEXIT_CRITICAL(&g_vzMux);
}

static VzTrack vzSnapshot() {
  portENTER_CRITICAL(&g_vzMux);
  VzTrack v = g_vz;
  portEXIT_CRITICAL(&g_vzMux);
  return v;
}

// 요구 11: 거리, 오차 X/Y, 목표속도, 추정속도, 롤/피치 명령, 태그 상태, CH6 모드, 오류 상태
static void emitApproach(const RcAuthoritySnapshot &authority) {
  ApproachInput in;
  ApproachOutput out;
  float tvR, tvF;
  bool tvValid;
  const char *hSrc;
  int mode = g_approachMode;
  portENTER_CRITICAL(&g_approachMux);
  in = g_approachIn;
  out = g_approachOut;
  tvR = g_approachTagVelR;
  tvF = g_approachTagVelF;
  tvValid = g_approachTagVelValid;
  hSrc = g_approachHSrc;
  float ofR = g_approachOf[0], ofF = g_approachOf[1];
  bool ofValid = g_approachOfValid;
  portEXIT_CRITICAL(&g_approachMux);
  char of_s[32] = "null,null";
  if (ofValid) snprintf(of_s, sizeof(of_s), "%.1f,%.1f", ofR, ofF);
  FcAttitude att;
  portENTER_CRITICAL(&g_mspMux);
  att = g_fcAtt;
  portEXIT_CRITICAL(&g_mspMux);
  uint32_t now = millis();
  bool tagFresh = in.tagValid && (now - in.tagAtMs) <= g_approach.p.tagStaleMs;
  char tagVel[48] = "null,null";
  if (tvValid) snprintf(tagVel, sizeof(tagVel), "%.1f,%.1f", tvR, tvF);
  char att_s[40] = "null,null";
  if (att.atMs && now - att.atMs < 500) snprintf(att_s, sizeof(att_s), "%.1f,%.1f", att.rollDeci * 0.1f, att.pitchDeci * 0.1f);
  VzTrack vz = vzSnapshot();
  char vz_s[16] = "null";
  if (vz.valid) snprintf(vz_s, sizeof(vz_s), "%.1f", vz.vzCmS);
  MspStats m;
  portENTER_CRITICAL(&g_mspMux);
  m = g_msp;
  portEXIT_CRITICAL(&g_mspMux);
  bool rcTx = m.rawRcTxActive;
  // FC(INAV)가 실제로 쓰는 롤·피치 (MSP_RC, 약 16Hz 로 물음). msp 줄은 1초에 한 번이라 짧은 명령을 놓친다 →
  // 5Hz approach 줄에도 실어 보드 명령과 같은 시점의 FC 값을 비교할 수 있게 한다
  char fcRp[40] = "null";
  if (m.fcRcCount > FC_RC_CH_PITCH && m.fcRcAtMs && now - m.fcRcAtMs < 1000)
    snprintf(fcRp, sizeof(fcRp), "[%u,%u],\"fc_rp_age_ms\":%lu", m.fcRc[FC_RC_CH_ROLL], m.fcRc[FC_RC_CH_PITCH],
             (unsigned long)(now - m.fcRcAtMs));
  // 칼만(vel_src 2) 추정: 태그 위치 e·기체 속도 v·외란 b [오른쪽, 앞], 측정 지연, 예측(태그 놓침) 중인지
  char kf_s[160] = "null";
  if (g_approach.p.velSrc == 2)
    snprintf(kf_s, sizeof(kf_s), "{\"on\":%s,\"pred\":%s,\"e\":[%.1f,%.1f],\"v\":[%.1f,%.1f],\"b\":[%.1f,%.1f],\"lat\":%.0f}",
             out.kfOn ? "true" : "false", out.predicting ? "true" : "false", out.kfE[0], out.kfE[1], out.kfV[0], out.kfV[1],
             out.kfB[0], out.kfB[1], out.kfLatMs);
  char line[1300];  // outTask 줄 버퍼(1400)보다 작게
  snprintf(line, sizeof(line),
           "{\"type\":\"approach\",\"mode\":\"%s\",\"state\":\"%s\",\"fault\":%s%s%s,\"dist_cm\":%.1f,"
           "\"err_right_cm\":%.1f,\"err_fwd_cm\":%.1f,\"target_speed\":%.1f,\"v_target\":[%.1f,%.1f],"
           "\"v_flow\":[%.1f,%.1f],\"v_flow_valid\":%s,\"flow_q\":%u,\"v_tag\":[%s],\"roll_us\":%.1f,"
           "\"pitch_us\":%.1f,\"tag\":\"%s\",\"authority\":\"%s\",\"ch6\":%u,\"height_cm\":%.1f,"
           "\"height_valid\":%s,\"att\":[%s],\"arrive_held_ms\":%lu,\"neutral\":%s,\"overspeed\":%s,"
           "\"vz_cm_s\":%s,\"vz_warn\":%s,\"mask_ok\":%s,\"out_rpy\":[%u,%u,%u],\"thr_slot_us\":%u,\"rc_tx\":%s,"
           "\"vel_src\":\"%s\",\"h_src\":\"%s\",\"out\":\"%s\",\"accel\":[%.1f,%.1f],\"tilt\":[%.2f,%.2f],"
           "\"expo\":%d,\"v_of\":[%s],\"fc_rp\":%s,\"kf\":%s}",
           APPROACH_MODE_NAME[mode & 3], approachStateName(out.state), out.fault ? "\"" : "",
           out.fault ? out.fault : "null", out.fault ? "\"" : "", out.distCm, in.errRightCm, in.errFwdCm,
           out.targetSpeedCmS, out.vTargetRightCmS, out.vTargetFwdCmS, in.velRightCmS, in.velFwdCmS,
           in.velValid ? "true" : "false", in.flowQuality, tagVel, out.rollUs, out.pitchUs,
           tagFresh ? "fresh" : (in.tagValid ? "stale" : "none"), authority.automatic ? "auto" : "manual",
           authority.channelUs, in.heightCm, in.heightValid ? "true" : "false", att_s,
           (unsigned long)out.arriveHeldMs, out.neutral ? "true" : "false", out.overspeed ? "true" : "false",
           vz_s, (vz.valid && fabsf(vz.vzCmS) > VZ_WARN_CM_S) ? "true" : "false",
           in.maskKnown ? (in.maskOk ? "true" : "false") : "null",
           rcTx ? m.out[frameSlot(FC_RC_CH_ROLL)] : 0, rcTx ? m.out[frameSlot(FC_RC_CH_PITCH)] : 0,
           rcTx ? m.out[frameSlot(FC_RC_CH_YAW)] : 0, rcTx ? m.out[frameSlot(FC_RC_CH_THROTTLE)] : 0,
           rcTx ? "true" : "false", g_approach.p.velSrc == 2 ? "kf" : (in.velFromTag ? "tag" : "flow"), hSrc,
           g_approach.p.outTilt != 0 ? "tilt" : "us", out.accelRightCmS2, out.accelFwdCmS2, out.tiltRightDeg,
           out.tiltFwdDeg, in.fcRcExpo, of_s, fcRp, kf_s);
  emitLine(line);
}

// 고도 유지 한 주기 (50Hz). 수동 중에는 조종자 스로틀·높이를 읽고, AUTO 에서는 스로틀을 계산한다
static void altStep(const RcAuthoritySnapshot &authority, const ControlFlags &flags, float dtS) {
  uint32_t now = millis();
  MspStats ms;
  portENTER_CRITICAL(&g_mspMux);
  ms = g_msp;
  portEXIT_CRITICAL(&g_mspMux);
  bool rcFresh = ms.fcRcAtMs && (now - ms.fcRcAtMs) < 300 && ms.fcRcCount > FC_RC_CH_THROTTLE;
  if (!g_altSeeded && rcFresh) {
    // 부팅 뒤 첫 FC 스로틀. 보드가 아직 프레임을 안 보냈으니 이 값은 조종기 값이다 (비행 중 재부팅해도 튀지 않게)
    AltHoldInput seed;
    seed.nowMs = now;
    seed.pilotThrValid = true;
    seed.pilotThrUs = ms.fcRc[FC_RC_CH_THROTTLE];
    g_alt.step(seed, dtS);
    g_altSeeded = true;
  }
  FlowState f = flowSnapshot();
  VzTrack vz = vzSnapshot();
  AltHoldInput in;
  in.nowMs = now;
  in.autoOn = authority.automatic && !flags.kill;
  in.pilotThrValid = !authority.automatic && authority.fresh && rcFresh;  // AUTO 에서는 MSP_RC 가 보드 값이다
  in.pilotThrUs = ms.fcRc[FC_RC_CH_THROTTLE];
  in.hValid = heightValid(f);
  in.hAtMs = f.lastRxMs;
  in.hCm = f.s.distance_mm / 10.0f;
  in.vzValid = vz.valid;
  in.vzCmS = vz.vzCmS;
  bool tourLand;
  portENTER_CRITICAL(&g_approachMux);
  tourLand = g_tourOut.landRequest;
  portEXIT_CRITICAL(&g_approachMux);
  if (!in.autoOn) g_altLandCmd = false;
  in.landRequest = g_altLandCmd || tourLand;
  {
    FcStatus fc;
    portENTER_CRITICAL(&g_mspMux);
    fc = g_fc;
    portEXIT_CRITICAL(&g_mspMux);
    in.armed = fc.statusAtMs && (now - fc.statusAtMs) <= FC_STATUS_SAFE_TIMEOUT_MS && (fc.armingFlags & ARMED_FLAG);
  }
  AltHoldOutput o = g_alt.step(in, dtS);
  portENTER_CRITICAL(&g_approachMux);
  g_altOut = o;
  portEXIT_CRITICAL(&g_approachMux);
}

// 고도 상태 2Hz
static void emitAlt() {
  AltHoldOutput o;
  portENTER_CRITICAL(&g_approachMux);
  o = g_altOut;
  portEXIT_CRITICAL(&g_approachMux);
  FcChannelConfig cfg;
  portENTER_CRITICAL(&g_mspMux);
  cfg = g_fcCfg;
  portEXIT_CRITICAL(&g_mspMux);
  FlowState f = flowSnapshot();
  VzTrack vz = vzSnapshot();
  char h[12] = "null", v[12] = "null";
  if (heightValid(f)) snprintf(h, sizeof(h), "%.1f", f.s.distance_mm / 10.0f);
  if (vz.valid) snprintf(v, sizeof(v), "%.1f", vz.vzCmS);
  char line[520];
  snprintf(line, sizeof(line),
           "{\"type\":\"alt\",\"state\":\"%s\",\"fault\":%s%s%s,\"fc_throttle\":%s,\"known\":%s,\"thr\":%.0f,"
           "\"base\":%.0f,\"target\":%.1f,\"h\":%s,\"vz\":%s,\"err\":%.1f,\"p\":%.0f,\"d\":%.0f,\"i\":%.0f,"
           "\"landing\":%s,\"touchdown\":%s,\"disarm_req\":%s,\"hf\":%.1f,\"vzf\":%.1f,\"spikes\":%lu,\"takeoff_fail\":%s,\"hover\":%.0f}",
           altStateName(o.state), o.fault ? "\"" : "", o.fault ? o.fault : "null", o.fault ? "\"" : "",
           fcTakesThrottle(cfg, millis()) ? "true" : "false", o.known ? "true" : "false", o.thrUs, o.baseUs,
           o.targetCm, h, v, o.errCm, o.pUs, o.dUs, o.iUs,
           (o.state == AltState::Landing || g_altLandCmd) ? "true" : "false", o.touchdown ? "true" : "false",
           o.disarmRequest ? "true" : "false", o.hF, o.vzF, (unsigned long)o.spikes, o.takeoffFail ? "true" : "false",
           o.hoverUs);
  emitLine(line);
}

// 투어 상태 2Hz. 화면·기록용
static void emitTour() {
  TourOutput to;
  float yawUs;
  const char *yawSrc;
  portENTER_CRITICAL(&g_approachMux);
  to = g_tourOut;
  yawUs = g_tourYawUs;
  yawSrc = g_tourYawSrc;
  portEXIT_CRITICAL(&g_approachMux);
  FcAttitude att;
  FcChannelConfig cfg;
  portENTER_CRITICAL(&g_mspMux);
  att = g_fcAtt;
  cfg = g_fcCfg;
  portEXIT_CRITICAL(&g_mspMux);
  bool attFresh = att.atMs && (millis() - att.atMs) < 500;
  char travel[12] = "null", yawNow[8] = "null";
  if (to.travelKnown) snprintf(travel, sizeof(travel), "%.1f", to.travelDeg);
  if (attFresh) snprintf(yawNow, sizeof(yawNow), "%d", att.yawDeg);
  char line[900];
  snprintf(line, sizeof(line),
           "{\"type\":\"tour\",\"on\":%s,\"paused\":%s,\"state\":\"%s\",\"state_ko\":\"%s\",\"reason\":\"%s\",\"target\":%d,"
           "\"last\":%d,\"act\":\"%s\",\"act_ko\":\"%s\",\"turn\":%d,\"in_frame\":%s,\"tgt\":[%.1f,%.1f],"
           "\"goal\":[%.1f,%.1f],\"visited\":%lu,\"land\":%s,\"dwell_ms\":%lu,\"dwell_need_ms\":%lu,\"elapsed_ms\":%lu,\"yaw_on\":%s,"
           "\"yaw_err\":%.1f,\"yaw_rate\":%.1f,\"yaw_us\":%.0f,\"yaw\":%s,\"travel\":%s,\"yaw_curve\":[%d,%d,%d],"
           "\"yaw_curve_src\":\"%s\",\"focus\":%d,\"scan_full\":%s,\"ol\":%s,\"ol_tilt\":%.1f}",
           g_tourOn ? "true" : "false", to.paused ? "true" : "false", tourStateName(to.state), tourStateKo(to.state), to.reason, to.targetId,
           to.lastId, to.act->code, to.act->ko, to.act->turnDeg, to.targetInFrame ? "true" : "false", to.targetR,
           to.targetF, to.goalR, to.goalF, (unsigned long)to.visited, to.landRequest ? "true" : "false", (unsigned long)to.dwellMs, (unsigned long)to.dwellNeedMs,
           (unsigned long)to.elapsedMs, to.yawOn ? "true" : "false", to.yawErrDeg, to.yawRateDegS, yawUs, yawNow,
           travel, cfg.yawRate, cfg.yawExpo, cfg.yawDeadband, yawSrc, to.focusId, to.scanFull ? "true" : "false",
           to.openLoop ? "true" : "false", to.olTiltFwdDeg);
  emitLine(line);
}

static uint16_t clampUs(float us) { return (uint16_t)lroundf(fmaxf(1000.0f, fminf(2000.0f, us))); }

// 핸드오버 기준점 갱신 (mspTask 가 매 주기 한 번 호출). 수동이면 조종기 스틱을 그대로 따라가고,
// AUTO 로 바뀌는 순간의 값을 고정한 뒤 handover_slew 로 1500 에 수렴시킨다.
static void handoverStep(Handover &ho, const RcAuthoritySnapshot &authority, float dtS) {
  uint32_t now = millis();
  if (!authority.automatic) {
    // MSP_RC 가 신선하면 조종기 값 (AUTO 가 아니므로 덮어쓰기 전 값이다)
    portENTER_CRITICAL(&g_mspMux);
    bool have = authority.fresh && g_msp.fcRcCount > FC_RC_CH_THROTTLE;
    if (have) {
      ho.pilot[0] = g_msp.fcRc[FC_RC_CH_ROLL];
      ho.pilot[1] = g_msp.fcRc[FC_RC_CH_PITCH];
      ho.pilot[2] = g_msp.fcRc[FC_RC_CH_YAW];
    }
    portEXIT_CRITICAL(&g_mspMux);
    ho.pilotValid = have;
    for (int i = 0; i < 3; i++) ho.base[i] = have ? ho.pilot[i] : 1500.0f;
    if (ho.wasAuto) ho.lastTransitionMs = now;
    ho.wasAuto = false;
    return;
  }
  if (!ho.wasAuto) {
    // 수동 → AUTO: 기준점은 마지막 조종기 값. 프레임은 이미 이 값으로 나가고 있었다.
    ho.wasAuto = true;
    ho.autoSinceMs = now;
    ho.lastTransitionMs = now;
  }
  float slew = g_approach.p.handoverSlewUsPerS * dtS;
  for (int i = 0; i < 3; i++) {
    float d = 1500.0f - ho.base[i];
    if (d > slew) d = slew;
    if (d < -slew) d = -slew;
    ho.base[i] += d;
  }
}

// 판단 → 스틱 값. 프레임 슬롯은 FC rcmap 을 따른다.
//   롤·피치 = 핸드오버 기준점 + 제어기 오프셋, 요 = 기준점(→1500), 스로틀 슬롯 = MSP_THROTTLE_SAFE_US 고정.
// allowThrottle: 실제 FC로는 CTRL_THROTTLE_ENABLED일 때만(false), 시뮬레이터로는 항상 허용한다.
static void computeSticks(uint16_t ch[MSP_CHANNELS], bool allowThrottle, bool kill, const Handover &ho) {
  for (int i = 0; i < MSP_CHANNELS; i++) ch[i] = 1500;
  const uint8_t sR = frameSlot(FC_RC_CH_ROLL), sP = frameSlot(FC_RC_CH_PITCH);
  const uint8_t sY = frameSlot(FC_RC_CH_YAW), sT = frameSlot(FC_RC_CH_THROTTLE);
  ch[sR] = clampUs(ho.base[0]);
  ch[sP] = clampUs(ho.base[1]);
  ch[sY] = clampUs(ho.base[2]);
  if (!allowThrottle) ch[sT] = MSP_THROTTLE_SAFE_US;  // 스로틀은 절대 중간값으로 보내지 않는다 (요구 2)
  // 고도 유지(-43): FC 가 보드 스로틀을 쓰는 설정이면 1000 이 아니라 고도 제어 값 (AUTO 순간 조종자 스로틀 기준).
  // 조종자 스로틀을 한 번도 못 읽었으면 보낼 값이 없다 → 1000 (이때는 프레임 자체를 늦게 보낸다: mspTask)
  AltHoldOutput alt;
  portENTER_CRITICAL(&g_approachMux);
  alt = g_altOut;
  portEXIT_CRITICAL(&g_approachMux);
  FcChannelConfig cfgT;
  portENTER_CRITICAL(&g_mspMux);
  cfgT = g_fcCfg;
  portEXIT_CRITICAL(&g_mspMux);
  if (ALT_HOLD_ENABLED && fcTakesThrottle(cfgT, millis()) && alt.known) ch[sT] = clampUs(alt.thrUs);
  // 착륙 마지막(15cm 아래)·착지 뒤: 수평 이동 없이 기체를 수평으로 (롤·피치·요 = 기준점 → 1500)
  bool levelOnly = alt.touchdown || (alt.state == AltState::Landing && heightValid(flowSnapshot()) &&
                                     flowSnapshot().s.distance_mm / 10.0f < ALT_LEVEL_BELOW_CM);
  ch[MSP_CH_KILL] = kill ? KILL_ON_US : KILL_OFF_US;  // 태그·판단과 무관하게 항상
  CtrlState c = ctrlSnapshot();
  FlowState f = flowSnapshot();
  int mode = g_approachMode;

  if (mode == APM_ON) {
    // 접근 제어 출력. step 이 최근(2주기 안)에 돌았을 때만 — 아니면 기준점만
    ApproachOutput out;
    uint32_t stepMs;
    float yawUs;
    bool yawOn;
    portENTER_CRITICAL(&g_approachMux);
    out = g_approachOut;
    stepMs = g_approachStepMs;
    yawUs = g_tourYawUs;
    yawOn = g_tourOut.state != TourState::Off;  // 투어 중에는 요도 보드가 (돌지 않을 때는 0 → 기수 유지)
    portEXIT_CRITICAL(&g_approachMux);
    bool ol;
    float olR, olP;
    portENTER_CRITICAL(&g_approachMux);
    ol = g_tourOl;
    olR = g_tourOlRollUs;
    olP = g_tourOlPitchUs;
    portEXIT_CRITICAL(&g_approachMux);
    if (stepMs && (millis() - stepMs) < (2000 / MSP_SEND_HZ) && !levelOnly) {
      // 왕복 이동 중에는 정해진 기울기 (2° 는 이 FC rc_expo 80 에서 약 113µs → 평소 한계 80µs 대신 기울기 한계 200µs)
      float mx = ol ? g_approach.p.tiltCmdMaxUs : g_approach.p.cmdMaxUs;
      float rUs = ol ? olR : out.rollUs, pUs = ol ? olP : out.pitchUs;
      ch[sR] = clampUs(ho.base[0] + fmaxf(-mx, fminf(mx, rUs)));
      ch[sP] = clampUs(ho.base[1] + fmaxf(-mx, fminf(mx, pUs)));
      if (yawOn) ch[sY] = clampUs(ho.base[2] + yawUs);
    }
    return;  // 접근 제어는 수평(+투어 요)만. 스로틀은 보내지 않는다
  }
  if (mode != APM_LEGACY) return;  // off·shadow: 롤·피치 기준점(→중립)

  bool fresh = c.seen && (millis() - c.atMs) < CTRL_TARGET_STALE_MS;
  if (!fresh) return;

  if (!c.centered) {
    ch[sR] = clampUs(ho.base[0] + fmaxf(-CTRL_XY_MAX_US, fminf(CTRL_XY_MAX_US, CTRL_XY_US_PER_CM * c.bodyRight)));
    ch[sP] = clampUs(ho.base[1] + fmaxf(-CTRL_XY_MAX_US, fminf(CTRL_XY_MAX_US, CTRL_XY_US_PER_CM * c.bodyFwd)));
  }
  if (allowThrottle && heightValid(f) && c.phase != PH_ALIGN) {
    float target = (c.phase == PH_GOTO_LOW || c.phase == PH_HOLD_LOW) ? MISSION_LOW_CM : MISSION_HIGH_CM;
    float err = target - f.s.distance_mm / 10.0f;  // + 이면 올라가야 함
    ch[sT] = stick(CTRL_Z_US_PER_CM * err, CTRL_Z_MAX_US);
  }
}

static void emitMspStatus() {
  MspStats m;
  portENTER_CRITICAL(&g_mspMux);
  m = g_msp;
  portEXIT_CRITICAL(&g_mspMux);
  uint32_t now = millis();
  RcAuthoritySnapshot authority = rcAuthoritySnapshot(now);
  ControlFlags flags = controlFlagsSnapshot();
  CtrlState c = ctrlSnapshot();
  bool link = m.lastRxMs && (now - m.lastRxMs) < MSP_LINK_TIMEOUT_MS;
  bool rcTx = m.rawRcTxActive;
  bool killTx = rcTx && m.lastRawRcKill;
  char out[64] = "", rc[64] = "", ch5[8] = "null", ch6[8] = "null", throttle[8] = "null";
  if (authority.fresh) {
    snprintf(ch5, sizeof(ch5), "%u", authority.armSwitchUs);
    snprintf(ch6, sizeof(ch6), "%u", authority.channelUs);
    snprintf(throttle, sizeof(throttle), "%u", authority.throttleUs);
  }
  size_t n = 0;
  for (int i = 0; rcTx && i < MSP_CHANNELS; i++) n += snprintf(out + n, sizeof(out) - n, "%s%u", i ? "," : "", m.out[i]);
  n = 0;
  for (int i = 0; link && i < m.fcRcCount; i++) n += snprintf(rc + n, sizeof(rc) - n, "%s%u", i ? "," : "", m.fcRc[i]);
  // 채널 설정·핸드오버·vz (요구 2·3·5 증빙)
  FcChannelConfig cfg;
  portENTER_CRITICAL(&g_mspMux);
  cfg = g_fcCfg;
  portEXIT_CRITICAL(&g_mspMux);
  char rcmap_s[5];
  rcmapString(cfg.rcmap, rcmap_s);
  char mask_s[12] = "null", maskOk_s[8] = "null";
  bool maskKnown = cfg.overrideMask >= 0 && (now - cfg.maskAtMs) < FC_MASK_VALID_MS;
  if (cfg.overrideMask >= 0) snprintf(mask_s, sizeof(mask_s), "%ld", (long)cfg.overrideMask);
  if (maskKnown) snprintf(maskOk_s, sizeof(maskOk_s), "%s", overrideMaskOk(cfg.overrideMask) ? "true" : "false");
  VzTrack vz = vzSnapshot();
  char vz_s[16] = "null";
  if (vz.valid) snprintf(vz_s, sizeof(vz_s), "%.1f", vz.vzCmS);
  const Handover &ho = g_handover;
  char line[1000];
  snprintf(line, sizeof(line),
           "{\"type\":\"msp\",\"enabled\":%s,\"fc_link\":%s,\"sent\":%lu,\"acks\":%lu,\"rc_replies\":%lu,"
           "\"errors\":%lu,\"out\":[%s],\"fc_rc\":[%s],\"authority\":\"%s\",\"authority_fresh\":%s,"
           "\"ch5\":%s,\"ch6\":%s,\"rc_throttle\":%s,\"rc_tx\":%s,\"phase\":\"%s\",\"throttle_ctrl\":%s,"
           "\"kill\":%s,\"kill_tx\":%s,\"tx_hz\":%lu,\"ack_hz\":%lu,\"rc_hz\":%lu,\"tx_always\":%s,"
           "\"rcmap\":\"%s\",\"rcmap_src\":\"%s\","
           "\"fc_mask\":%s,\"mask_ok\":%s,\"mask_recommended\":%u,\"thr_slot\":%u,\"thr_slot_us\":%u,"
           "\"base_rpy\":[%.0f,%.0f,%.0f],\"pilot_rpy\":[%u,%u,%u],\"pilot_valid\":%s,\"auto_since_ms\":%lu,"
           "\"vz_cm_s\":%s,\"vz_warn\":%s,\"vz_warn_count\":%lu,\"ovr_ch\":%u,\"ovr_us\":[%u,%u],\"ovr_src\":\"%s\"}",
           flags.mspEnabled ? "true" : "false", link ? "true" : "false", (unsigned long)m.sent, (unsigned long)m.acks,
           (unsigned long)m.rcReplies, (unsigned long)m.errors, out, rc,
           authority.automatic ? "auto" : "manual", authority.fresh ? "true" : "false",
           ch5, ch6, throttle, rcTx ? "true" : "false", PHASE_CODE[c.phase],
           CTRL_THROTTLE_ENABLED ? "true" : "false", flags.kill ? "true" : "false",
           killTx ? "true" : "false", (unsigned long)m.txHz, (unsigned long)m.ackHz, (unsigned long)m.rcHz,
           MSP_TX_ALWAYS ? "true" : "false", rcmap_s,
           cfg.rcmapFromFc ? "fc" : "default", mask_s, maskOk_s, (unsigned)OVERRIDE_MASK_RECOMMENDED,
           frameSlot(FC_RC_CH_THROTTLE), rcTx ? m.out[frameSlot(FC_RC_CH_THROTTLE)] : 0,
           ho.base[0], ho.base[1], ho.base[2], ho.pilot[0], ho.pilot[1], ho.pilot[2],
           ho.pilotValid ? "true" : "false",
           (unsigned long)(authority.automatic && ho.wasAuto ? now - ho.autoSinceMs : 0),
           vz_s, (vz.valid && fabsf(vz.vzCmS) > VZ_WARN_CM_S) ? "true" : "false", (unsigned long)vz.warnCount,
           (unsigned)cfg.ovrCh + 1, (unsigned)cfg.ovrMinUs, (unsigned)cfg.ovrMaxUs + 1, cfg.ovrFromFc ? "fc" : "default");
  emitLine(line);
}

// CH6 전환 이벤트 (요구 5 증빙): 전환 시각, 마지막 프레임 송신 뒤 경과, 그 프레임의 롤·피치·요, 조종기 값.
// frame_age_ms 가 한 주기(20ms) 안이면 "통신 초기화 지연 없음", out_rpy ≈ pilot_rpy 면 "연속적".
static void emitAuthorityEvent(const RcAuthoritySnapshot &authority, const Handover &ho) {
  MspStats m;
  portENTER_CRITICAL(&g_mspMux);
  m = g_msp;
  portEXIT_CRITICAL(&g_mspMux);
  uint32_t now = millis();
  char fcrc[64] = "";
  size_t n = 0;
  for (int i = 0; i < m.fcRcCount && i < MSP_CHANNELS; i++) n += snprintf(fcrc + n, sizeof(fcrc) - n, "%s%u", i ? "," : "", m.fcRc[i]);
  char line[400];
  snprintf(line, sizeof(line),
           "{\"type\":\"authority\",\"to\":\"%s\",\"ch6\":%u,\"t\":%lu,\"frame_age_ms\":%ld,\"tx_hz\":%lu,"
           "\"out_rpy\":[%u,%u,%u],\"thr_slot_us\":%u,\"pilot_rpy\":[%u,%u,%u],\"pilot_valid\":%s,\"fc_rc\":[%s],"
           "\"approach\":\"%s\"}",
           authority.automatic ? "auto" : "manual", authority.channelUs, (unsigned long)now,
           m.lastRawRcTxMs ? (long)(now - m.lastRawRcTxMs) : -1L, (unsigned long)m.txHz,
           m.out[frameSlot(FC_RC_CH_ROLL)], m.out[frameSlot(FC_RC_CH_PITCH)], m.out[frameSlot(FC_RC_CH_YAW)],
           m.out[frameSlot(FC_RC_CH_THROTTLE)], ho.pilot[0], ho.pilot[1], ho.pilot[2],
           ho.pilotValid ? "true" : "false", fcrc, APPROACH_MODE_NAME[g_approachMode & 3]);
  emitLine(line);
}

// rcmap · msp_override_channels 읽기 (FC_CFG_POLL_MS 마다). 진단 fcmsp 질의가 진행 중이면 건너뛴다.
static void fcConfigPollStep() {
  static uint32_t lastMs = 0;
  uint32_t now = millis();
  if (lastMs && now - lastMs < FC_CFG_POLL_MS) return;
  lastMs = now;
  bool queryBusy;
  portENTER_CRITICAL(&g_queryMux);
  queryBusy = g_query.pending;
  portEXIT_CRITICAL(&g_queryMux);
  if (queryBusy) return;
  uint8_t f1[8];
  // rcmap 은 매번, 모드 배정(MSP RC OVERRIDE 범위)은 5번에 1번(10초마다)
  static uint8_t rangeTurn = 0;
  if (rangeTurn++ % 5 == 0) Serial2.write(f1, mspEncode(MSP_MODE_RANGES, nullptr, 0, f1));
  else Serial2.write(f1, mspEncode(MSP_RX_MAP, nullptr, 0, f1));
  // 0x1003 응답에는 설정 이름이 없어서 한 번에 하나만 묻는다. 마스크는 5번 중 4번(2초마다), 곡선 값은
  // 5번에 1번(rc_expo·deadband·yaw_rate·rc_yaw_expo·yaw_deadband 차례로 각 50초마다). 진단 질의가 겹치면 건너뛰므로
  // 마스크 유효 기간은 넉넉히 둔다.
  static uint8_t turn = 0, curveTurn = 0;
  static const char *CURVE_NAMES[] = {"rc_expo", "deadband", "yaw_rate", "rc_yaw_expo", "yaw_deadband"};
  const char *name = "msp_override_channels";
  uint8_t which = 0;
  if (turn++ % 5 == 4) {
    which = 1 + (curveTurn++ % 5);
    name = CURVE_NAMES[which - 1];
  }
  size_t len = strlen(name) + 1;
  uint8_t f2[48];
  Serial2.write(f2, mspEncodeV2(MSP2_COMMON_SETTING, (const uint8_t *)name, len, f2));
  portENTER_CRITICAL(&g_mspMux);
  g_fcCfg.maskQuerySentMs = now;
  g_fcCfg.settingQuery = which;
  portEXIT_CRITICAL(&g_mspMux);
}

// FC 상태 1Hz: 아밍 여부·거부 사유·배터리·모터 출력
static void emitFcStatus() {
  FcStatus f;
  portENTER_CRITICAL(&g_mspMux);
  f = g_fc;
  portEXIT_CRITICAL(&g_mspMux);
  uint32_t now = millis();
  bool st = f.statusAtMs && now - f.statusAtMs < 2000;
  bool an = f.analogAtMs && now - f.analogAtMs < 2000;
  bool mo = f.motorAtMs && now - f.motorAtMs < 2000;
  ControlFlags flags = controlFlagsSnapshot();
  char blockers[260] = "";
  size_t n = 0;
  for (const auto &b : ARMING_BLOCKERS)
    if (st && (f.armingFlags & (1u << b.bit)) && n < sizeof(blockers) - 32)
      n += snprintf(blockers + n, sizeof(blockers) - n, "%s\"%s\"", n ? "," : "", b.name);
  char line[520];
  snprintf(line, sizeof(line),
           "{\"type\":\"fc\",\"status\":%s,\"armed\":%s,\"arming_flags\":\"0x%08lX\",\"blockers\":[%s],"
           "\"battery\":%s,\"vbat\":%.2f,\"cells\":%u,\"amps\":%.2f,\"motor\":[%u,%u,%u,%u],\"motor_ok\":%s,"
           "\"kill\":%s}",
           st ? "true" : "false", (st && (f.armingFlags & ARMED_FLAG)) ? "true" : "false",
           (unsigned long)f.armingFlags, blockers, an ? "true" : "false", f.vbatCv / 100.0f, f.cells,
           f.ampsCa / 100.0f, f.motor[0], f.motor[1], f.motor[2], f.motor[3], mo ? "true" : "false",
           flags.kill ? "true" : "false");
  emitLine(line);
}

// 진단 질의 처리: mspTask가 보내고, 응답이 오거나 1초가 지나면 결과를 출력
static void fcQueryStep() {
  portENTER_CRITICAL(&g_queryMux);
  FcQuery q = g_query;
  portEXIT_CRITICAL(&g_queryMux);
  if (q.pending && !q.sent) {
    uint8_t frame[9 + sizeof(q.payload)];
    size_t len = q.cmd > 255 ? mspEncodeV2(q.cmd, q.payload, q.payloadLen, frame)
                             : mspEncode((uint8_t)q.cmd, q.payload, (uint8_t)q.payloadLen, frame);
    Serial2.write(frame, len);
    portENTER_CRITICAL(&g_queryMux);
    g_query.sent = true;
    g_query.sentAtMs = millis();
    portEXIT_CRITICAL(&g_queryMux);
    return;
  }
  if (q.sent && (q.done || millis() - q.sentAtMs > 1000)) {
    static char line[1300];
    size_t n = snprintf(line, sizeof(line), "{\"type\":\"fcmsp\",\"cmd\":%u,\"ok\":%s,\"error\":%s,\"size\":%u,\"hex\":\"",
                        q.cmd, q.done ? "true" : "false", q.error ? "true" : "false", q.replyLen);
    for (uint16_t i = 0; q.done && i < q.replyLen && n < sizeof(line) - 8; i++) n += snprintf(line + n, sizeof(line) - n, "%02x", q.reply[i]);
    snprintf(line + n, sizeof(line) - n, "\"}");
    emitLine(line);
    portENTER_CRITICAL(&g_queryMux);
    g_query.pending = g_query.sent = g_query.done = false;
    portEXIT_CRITICAL(&g_queryMux);
  }
}

// ---------------------------------------------------------------------------
// 모터 시험: mspTask가 20Hz로 MSP_SET_MOTOR를 보내고, 시간이 다 되면 정지(1000)를 여러 번 보낸다.
// ---------------------------------------------------------------------------
struct MotorTest {
  bool active = false;
  uint8_t mask = 0;        // bit0 = M1 … bit3 = M4
  uint16_t value = 1000;
  uint32_t untilMs = 0;
  uint8_t stopFrames = 0;  // 남은 정지 프레임 수
  uint32_t ackStart = 0;
  uint32_t generation = 0; // 다른 코어의 중지 요청 뒤 오래된 RUN 결정을 폐기
};
static MotorTest g_motor;
static portMUX_TYPE g_motorMux = portMUX_INITIALIZER_UNLOCKED;

static bool motorCommandBusy() {
  portENTER_CRITICAL(&g_motorMux);
  bool busy = g_motor.active || g_motor.stopFrames != 0;
  portEXIT_CRITICAL(&g_motorMux);
  return busy;
}

static bool fcLinkAlive() {
  portENTER_CRITICAL(&g_mspMux);
  uint32_t last = g_msp.lastRxMs;
  portEXIT_CRITICAL(&g_mspMux);
  return last && (millis() - last) < MSP_LINK_TIMEOUT_MS;
}

static bool sendMotorFrame(uint8_t mask, uint16_t value) {
  uint8_t payload[16], frame[6 + 16];
  for (int i = 0; i < 8; i++) {
    uint16_t v = (i < MOTOR_COUNT && (mask & (1 << i))) ? value : 1000;
    payload[2 * i] = v & 0xFF;
    payload[2 * i + 1] = v >> 8;
  }
  size_t frameLen = mspEncode(MSP_SET_MOTOR, payload, sizeof(payload), frame);
  return Serial2.write(frame, frameLen) == frameLen;
}

static void motorTestStep() {
  uint32_t now = millis();
  RcAuthoritySnapshot authority = rcAuthoritySnapshot(now);
  ControlFlags flags = controlFlagsSnapshot();
  portENTER_CRITICAL(&g_mspMux);
  FcStatus fc = g_fc;
  portEXIT_CRITICAL(&g_mspMux);
  bool fcStatusFresh = fc.statusAtMs && (now - fc.statusAtMs) <= FC_STATUS_SAFE_TIMEOUT_MS;
  bool armed = fcStatusFresh && (fc.armingFlags & ARMED_FLAG);
  const char *abortReason = !authority.automatic ? "manual_authority"
                            : flags.kill         ? "kill"
                            : !authority.armSwitchDisarmed ? "arm_switch"
                            : !authority.throttleLow       ? "throttle"
                            : !fcStatusFresh     ? "fc_status_stale"
                            : armed              ? "fc_armed"
                                                 : nullptr;

  enum Action { NONE, RUN, STOP } action = NONE;
  uint8_t mask = 0;
  uint16_t value = 1000;
  uint32_t ackStart = 0;
  uint32_t generation = 0;
  bool aborted = false;
  bool stopped = false;

  portENTER_CRITICAL(&g_motorMux);
  if (g_motor.active) {
    if (abortReason || (int32_t)(now - g_motor.untilMs) >= 0) {
      aborted = abortReason != nullptr;
      g_motor.active = false;
      g_motor.stopFrames = 10;  // 종료·권한 상실 → 정지 신호를 0.5초 동안 반복
      g_motor.generation++;
    } else {
      action = RUN;
      mask = g_motor.mask;
      value = g_motor.value;
    }
  }
  if (action == NONE && g_motor.stopFrames) {
    action = STOP;
  }
  ackStart = g_motor.ackStart;
  generation = g_motor.generation;
  portEXIT_CRITICAL(&g_motorMux);

  if (action == RUN) {
    portENTER_CRITICAL(&g_motorMux);
    bool stillCurrent = g_motor.active && g_motor.generation == generation;
    bool stopRequested = !stillCurrent && g_motor.stopFrames != 0;
    portEXIT_CRITICAL(&g_motorMux);
    if (!stillCurrent) action = stopRequested ? STOP : NONE;
  }

  bool written = action == RUN ? sendMotorFrame(mask, value)
                               : action == STOP ? sendMotorFrame(0, 1000)
                                                : true;
  if (!written) {
    portENTER_CRITICAL(&g_mspMux);
    g_msp.errors++;
    portEXIT_CRITICAL(&g_mspMux);
    if (action == RUN) {
      portENTER_CRITICAL(&g_motorMux);
      if (g_motor.active && g_motor.generation == generation) {
        g_motor.active = false;
        g_motor.stopFrames = 10;
        g_motor.generation++;
      }
      portEXIT_CRITICAL(&g_motorMux);
      aborted = true;
      abortReason = "uart_write";
    }
  } else if (action == STOP) {
    portENTER_CRITICAL(&g_motorMux);
    if (g_motor.stopFrames) stopped = --g_motor.stopFrames == 0;
    ackStart = g_motor.ackStart;
    portEXIT_CRITICAL(&g_motorMux);
  }

  if (aborted) {
    char line[160];
    snprintf(line, sizeof(line), "{\"type\":\"motortest\",\"state\":\"stopping\",\"reason\":\"%s\"}",
             abortReason);
    emitLine(line);
  }
  if (stopped) {
    char line[120];
    portENTER_CRITICAL(&g_mspMux);
    uint32_t acks = g_msp.motorAcks;
    portEXIT_CRITICAL(&g_mspMux);
    snprintf(line, sizeof(line), "{\"type\":\"motortest\",\"state\":\"stopped\",\"fc_acks\":%lu}",
             (unsigned long)(acks - ackStart));
    emitLine(line);
  }
}

static void mspTask(void *) {
  MspReplyParser parser;
  // 모터 시험 중 보드가 재부팅되면 FC는 마지막 모터 값을 계속 유지한다 → 켜질 때마다 정지부터 보낸다
  portENTER_CRITICAL(&g_motorMux);
  g_motor.stopFrames = 10;
  g_motor.generation++;
  portEXIT_CRITICAL(&g_motorMux);
  TickType_t wake = xTaskGetTickCount();
  uint32_t cycle = 0, lastReport = 0;
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000 / MSP_SEND_HZ));
    MspReply r;
    while (Serial2.available())
      if (parser.feed((uint8_t)Serial2.read(), r)) handleMspReply(r);
    motorTestStep();  // msp off 여도 모터 시험·정지 신호는 보낸다

    fcQueryStep();
    fcConfigPollStep();  // rcmap · msp_override_channels (2s 마다)
    // 질의 예산: INAV 는 포트당 10ms 에 명령 하나(100/s). SET_RAW_RC 50/s 외 나머지는 ≤45/s (config.h 참고)
    switch (cycle % FC_POLL_PERIOD) {  // 상태 2Hz(아밍 감지), 배터리·모터 출력 1Hz
      case 7:
      case 32: { uint8_t f[12]; Serial2.write(f, mspEncodeV2(MSP2_INAV_STATUS, nullptr, 0, f)); break; }
      case 23: { uint8_t f[12]; Serial2.write(f, mspEncodeV2(MSP2_INAV_ANALOG, nullptr, 0, f)); break; }
      case 41: { uint8_t f[8]; Serial2.write(f, mspEncode(MSP_MOTOR, nullptr, 0, f)); break; }
    }
    if (cycle % FC_ALT_POLL_EVERY == 0) {  // MTF-01P가 FC에 연결된 구성의 높이 (10Hz)
      uint8_t f[8];
      Serial2.write(f, mspEncode(MSP_SONAR_ALTITUDE, nullptr, 0, f));
    }
    if (g_approachMode != APM_OFF) {  // 접근 제어 입력: 광류 20Hz, 자세 10Hz
      if (cycle % FC_FLOW_POLL_EVERY == 0) {
        uint8_t f[12];
        Serial2.write(f, mspEncodeV2(MSP2_INAV_OPTICAL_FLOW, nullptr, 0, f));
      }
      if (cycle % FC_ATT_POLL_EVERY == 0) {
        uint8_t f[8];
        Serial2.write(f, mspEncode(MSP_ATTITUDE, nullptr, 0, f));
      }
    }
    if (cycle % MSP_POLL_EVERY == 0) {
      // MANUAL에서도 계속 조회해야 CH6=1000 복귀를 감지할 수 있다.
      uint8_t f[8];
      Serial2.write(f, mspEncode(MSP_RC, nullptr, 0, f));
    }

    RcAuthoritySnapshot authority = rcAuthoritySnapshot(millis());
    ControlFlags flags = controlFlagsSnapshot();
    const float dtS = 1.0f / MSP_SEND_HZ;
    vzTrackStep();  // MTF-01 수직 속도 기록·경고 (스로틀에는 영향 없음)
    // CH6 전환 감지 → 이벤트. 전환 직전 프레임(수동 추적값)이 이미 FC 에 가 있다.
    bool prevAuto = g_handover.wasAuto;
    handoverStep(g_handover, authority, dtS);
    if (prevAuto != authority.automatic) emitAuthorityEvent(authority, g_handover);
    altStep(authority, flags, dtS);
    if (cycle % ALT_REPORT_EVERY == 11) emitAlt();
    if (g_approachMode == APM_ON || g_approachMode == APM_SHADOW) {
      approachStep(authority, flags, dtS);  // 수동 중에도 계산·로그는 계속 (FAULT manual_authority)
      if (cycle % APPROACH_REPORT_EVERY == 0) emitApproach(authority);
      if (g_tourOn && cycle % TOUR_REPORT_EVERY == 3) emitTour();
    }
    bool motorBusy = motorCommandBusy();
    bool releasePending = flags.killReleaseFrames != 0;
    portENTER_CRITICAL(&g_mspMux);
    g_msp.rawRcTxActive = false;
    portEXIT_CRITICAL(&g_mspMux);
    bool rawRcAllowed = flags.kill || (!motorBusy && (flags.mspEnabled || releasePending));
    // 부팅 뒤 FC 스로틀을 읽기 전에는 보내지 않는다: FC 가 보드 스로틀을 쓰는 설정에서 비행 중 재부팅되면
    // 첫 프레임의 스로틀 1000 이 그대로 적용될 수 있다 (MSP 가 끊기면 INAV 는 0.9초 뒤 조종기로 돌아가 있다)
    if (ALT_HOLD_ENABLED && !g_altSeeded && !flags.kill) rawRcAllowed = false;
    // MSP_TX_ALWAYS: CH6 위치와 무관하게 매 주기 보낸다. FC 는 CH6=AUTO 일 때만 마스크 채널에 적용하고,
    // 수동이면 프레임을 무시한다(INAV rx.c). 모터 시험 중·msp off 일 때만 멈춘다.
    if ((MSP_TX_ALWAYS || authority.automatic) && rawRcAllowed) {
      uint16_t ch[MSP_CHANNELS];
      computeSticks(ch, CTRL_THROTTLE_ENABLED, flags.kill, g_handover);
      uint8_t frame[6 + 2 * MSP_CHANNELS];
      size_t frameLen = mspEncodeRawRc(ch, MSP_CHANNELS, frame);
      bool written = Serial2.write(frame, frameLen) == frameLen;
      uint32_t nowTx = millis();
      portENTER_CRITICAL(&g_mspMux);
      if (written) {
        g_msp.sent++;
        g_msp.lastRawRcTxMs = nowTx;
        g_msp.rawRcTxActive = true;
        g_msp.lastRawRcKill = flags.kill;
        memcpy(g_msp.out, ch, sizeof(ch));
        g_msp.txCountWindow++;
      } else {
        g_msp.errors++;
      }
      rateWindowStep(nowTx);
      portEXIT_CRITICAL(&g_mspMux);
      if (written && releasePending) consumeKillReleaseFrame();
    } else {
      portENTER_CRITICAL(&g_mspMux);
      rateWindowStep(millis());
      portEXIT_CRITICAL(&g_mspMux);
    }

    if (simActive()) {
      // 시뮬레이터 출력은 실제 FC 제어권과 분리한다. 이 블록은 UART에 SET_RAW_RC를 보내지 않는다.
      uint16_t sc[MSP_CHANNELS];
      computeSticks(sc, true, flags.kill, g_handover);
      CtrlState c = ctrlSnapshot();
      char line[200];
      snprintf(line, sizeof(line),
               "{\"type\":\"rc\",\"t\":%lu,\"ch\":[%u,%u,%u,%u],\"seen\":%s,\"centered\":%s,"
               "\"phase\":\"%s\",\"sim_h\":%.1f}",
               (unsigned long)millis(), sc[frameSlot(FC_RC_CH_ROLL)], sc[frameSlot(FC_RC_CH_PITCH)],
               sc[frameSlot(FC_RC_CH_THROTTLE)], sc[frameSlot(FC_RC_CH_YAW)],
               c.seen ? "true" : "false", c.centered ? "true" : "false", PHASE_CODE[c.phase],
               (float)g_simHeightCm);
      emitLine(line);
    }

    cycle++;
    uint32_t now = millis();
    if (now - lastReport >= 1000) {
      lastReport = now;
      emitMspStatus();
      emitFcStatus();
    }
  }
}

// ---------------------------------------------------------------------------
// decimate 자동 전환
// ---------------------------------------------------------------------------
// 오름차순. 20261011 실측(VGA, 태그 없음): 2.0 → 540ms(1.9fps), 3.0 → 255ms, 4.0 → 160ms(5fps).
// 17cm 태그는 1m 에서 약 105px 이라 5~6 으로도 한 변 20px 이상 → 더 빠르게
static const float DEC_LEVELS[] = {2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
static bool g_decAuto = DEC_AUTO_DEFAULT;
static const char *g_decReason = "start";

// 축소 후 한 변이 minSide 이상 남는 가장 큰 decimate
static float pickDecimate(float sidePx, float minSide) {
  float best = DEC_LEVELS[0];
  for (float d : DEC_LEVELS)
    if (sidePx / d >= minSide) best = d;
  return best;
}

struct RoiState { bool on; float cx, cy, side; };
static RoiState g_roi = {false, 0, 0, 0};
static bool g_roiUsedLast = false;
static float g_lastTargetSizeMm = 0;  // 마지막으로 본 미션 태그의 쓰는 크기 (자동 보정 포함) — 찾을 때 예측용
static uint32_t g_lastTargetSeenMs = 0;  // 마지막으로 미션 태그를 본 시각 — 최근이면 느린 넓은 탐색(2.0)을 건너뛴다

static uint16_t smallestMissionTagMm() {
  uint16_t m = 0;
  for (size_t i = 0; i < TAG_COUNT; i++)
    if (isMissionTag(&g_tags[i]) && (m == 0 || g_tags[i].size_mm < m)) m = g_tags[i].size_mm;
  return m ? m : TAG_UNKNOWN_SIZE_MM;
}

// targetSide: 이번 프레임 목표 태그 크기(px), 없으면 0
static void autoDecimate(float targetSide, uint32_t missStreak) {
  if (!g_decAuto) return;
  float cur = g_td->quad_decimate;
  float next = cur;

  if (targetSide > 0) {
    // 보이는 태그 기준: 놓치기 전에 낮추고, 여유가 충분할 때만 올린다
    float keep = pickDecimate(targetSide, DEC_KEEP_SIDE_PX);
    float raise = pickDecimate(targetSide, DEC_RAISE_SIDE_PX);
    if (cur > keep) next = keep;
    else if (raise > cur) next = raise;
    g_decReason = "tag_px";
  } else if (missStreak >= DEC_LOST_FRAMES) {
    FlowState f = flowSnapshot();
    bool recent = g_lastTargetSeenMs && millis() - g_lastTargetSeenMs < DEC_RECENT_MS;
    if (missStreak % DEC_SWEEP_EVERY == 0 && !recent) {
      // 높이 예측이 틀렸을 수도 있으니 주기적으로 가장 넓게 찾아본다 (방금 본 태그는 크기를 알므로 예측으로 충분)
      next = DEC_LEVELS[0];
      g_decReason = "sweep";
    } else if (heightValid(f)) {
      // 찾는 태그가 이 높이에서 몇 px로 보일지 예측. 본 적 있으면 그 태그 크기(자동 보정값), 없으면 가장 작은 미션 태그
      float sizeMm = g_lastTargetSizeMm > 0 ? g_lastTargetSizeMm : (float)smallestMissionTagMm();
      float predicted = g_fx * sizeMm / (float)f.s.distance_mm;
      next = pickDecimate(predicted, DEC_RAISE_SIDE_PX);
      g_decReason = "mtf_height";
    } else {
      next = DEC_LEVELS[0];
      g_decReason = "search";
    }
  }
  g_td->quad_decimate = next;
}

// ---------------------------------------------------------------------------
// 스냅샷: 현재 그레이스케일 프레임을 JPEG → base64 로 보낸다 (패널이 화면에 어떻게 보이는지 확인용)
// ---------------------------------------------------------------------------
// 카메라 루프는 JPEG·base64 만 만들고(수십 ms), 줄 나눠 보내기는 snapTask 가 한다.
// 예전에는 루프가 직접 보내 BLE 에서 사진 한 장(약 12초) 동안 태그 인식이 멈췄다 (20261010 밤).
struct SnapJob {
  uint8_t *data;  // base64 글자, 또는 binary 면 JPEG 원본
  size_t len;
  bool binary;    // WiFi(TCP) 스트림: base64 없이 원본(약 25% 작다) — 조각마다 "SNAPC <n>\n" + n 바이트
  char head[64];
};
static QueueHandle_t g_snapQ = nullptr;
static volatile bool g_snapBusy = false;  // 보내는 중에는 새 사진을 만들지 않는다 (요청은 남겨 둠)
static bool g_snapSmall = false;           // `snap small`: 320×240 (BLE 용)
// `snap stream <ms>`: 요청 없이 작은 사진을 계속 보낸다 (요청·응답 왕복 지연 없이 거의 실시간 영상).
// 앞 사진을 다 보낸 뒤에만 다음 것을 만들므로 통로가 느리면 저절로 간격이 벌어진다. 0 = 끔
static volatile uint32_t g_streamMs = 0;
static volatile uint8_t g_streamDest = 0;
static uint32_t g_streamLastMs = 0;

static void snapTask(void *) {
  SnapJob job;
  for (;;) {
    if (xQueueReceive(g_snapQ, &job, portMAX_DELAY) != pdTRUE) continue;
    // SNAP_CHUNK 씩 끊어 보낸다. 받는 쪽은 SNAP_END까지 이어 붙인다.
    // base64: 조각이 한 줄. binary: 조각마다 "SNAPC <n>\n" 머리 + 원본 n 바이트를 한 메시지로 (사이에 상태 줄이 끼어도 분리됨)
    static char chunk[SNAP_CHUNK + 16];
    bool ok = emitLineWait(job.head, strlen(job.head), 3000);
    for (size_t off = 0; ok && off < job.len; off += SNAP_CHUNK) {
      size_t n = min((size_t)SNAP_CHUNK, job.len - off);
      // 출력 버퍼에 상태 줄 자리를 남긴다: 느린 BLE 에서 사진 조각이 버퍼를 채우면 상태 줄이 버려진다
      uint32_t t0 = millis();
      while (xMessageBufferSpacesAvailable(g_outBuf) < n + SNAP_RESERVE_BYTES && millis() - t0 < 5000)
        vTaskDelay(pdMS_TO_TICKS(10));
      if (job.binary) {
        int h = snprintf(chunk, sizeof(chunk), "SNAPC %u\n", (unsigned)n);
        memcpy(chunk + h, job.data + off, n);
        ok = emitLineWait(chunk, h + n, 5000);
      } else {
        ok = emitLineWait((const char *)job.data + off, n, 5000);
      }
    }
    emitLineWait("SNAP_END", 8, 3000);
    free(job.data);
    g_snapBusy = false;
  }
}

static void snapError(const char *reason) {
  char line[96];
  snprintf(line, sizeof(line), "{\"type\":\"snap_error\",\"reason\":\"%s\"}", reason);
  emitLine(line);
  g_snapBusy = false;
}

static void sendSnapshot(camera_fb_t *fb, bool small, bool binary = false) {
  g_snapBusy = true;
  uint8_t *jpg = nullptr;
  size_t jpgLen = 0;
  int w = fb->width, h = fb->height;
  bool encoded;
  if (small && fb->format == PIXFORMAT_GRAYSCALE) {
    // 2×2 평균으로 절반 크기 → 태그·글자가 보일 만큼은 남고 크기는 약 1/6
    w = fb->width / 2;
    h = fb->height / 2;
    uint8_t *half = (uint8_t *)ps_malloc((size_t)w * h);
    if (!half) return snapError("out of memory");
    for (int y = 0; y < h; y++) {
      const uint8_t *r0 = fb->buf + (size_t)2 * y * fb->width, *r1 = r0 + fb->width;
      uint8_t *o = half + (size_t)y * w;
      for (int x = 0; x < w; x++) o[x] = (r0[2 * x] + r0[2 * x + 1] + r1[2 * x] + r1[2 * x + 1] + 2) >> 2;
    }
    encoded = fmt2jpg(half, (size_t)w * h, w, h, PIXFORMAT_GRAYSCALE, SNAP_SMALL_QUALITY, &jpg, &jpgLen);
    free(half);
  } else {
    encoded = frame2jpg(fb, SNAP_JPEG_QUALITY, &jpg, &jpgLen);
  }
  if (!encoded) return snapError("jpeg encode failed");
  SnapJob job;
  job.binary = binary;
  if (binary) {
    job.data = jpg;  // snapTask 가 free
    job.len = jpgLen;
    snprintf(job.head, sizeof(job.head), "SNAPB %u %d %d", (unsigned)jpgLen, w, h);
  } else {
    job.len = 0;
    mbedtls_base64_encode(nullptr, 0, &job.len, jpg, jpgLen);
    job.data = (uint8_t *)ps_malloc(job.len + 1);
    if (!job.data) {
      free(jpg);
      return snapError("out of memory");
    }
    mbedtls_base64_encode(job.data, job.len + 1, &job.len, jpg, jpgLen);
    free(jpg);
    snprintf(job.head, sizeof(job.head), "SNAP_BEGIN %u %d %d %u", (unsigned)jpgLen, w, h, (unsigned)job.len);
  }
  if (!g_snapQ || xQueueSend(g_snapQ, &job, 0) != pdTRUE) {
    free(job.data);
    snapError("busy");
  }
}

// ---------------------------------------------------------------------------
// 부팅/정보 메시지, 명령 처리
// ---------------------------------------------------------------------------
static TaskHandle_t g_mspTaskHandle = nullptr;

// 부팅 원인 (esp_reset_reason). 시험 중 보드가 혼자 재부팅되면 원인을 가린다
static const char *resetReasonName() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "poweron";
    case ESP_RST_EXT: return "ext";
    case ESP_RST_SW: return "sw";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "int_wdt";
    case ESP_RST_TASK_WDT: return "task_wdt";
    case ESP_RST_WDT: return "wdt";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_SDIO: return "sdio";
    default: return "other";  // USB 직렬 포트를 열 때의 칩 리셋(rst:0x15)도 여기에 해당할 수 있다
  }
}

static void emitInfo(const char *type) {
  RcAuthoritySnapshot authority = rcAuthoritySnapshot(millis());
  ControlFlags flags = controlFlagsSnapshot();
  portENTER_CRITICAL(&g_mspMux);
  MspStats msp = g_msp;
  portEXIT_CRITICAL(&g_mspMux);
  bool rcTx = msp.rawRcTxActive;
  bool killTx = rcTx && msp.lastRawRcKill;
  char ch6[8] = "null";
  if (authority.fresh) snprintf(ch6, sizeof(ch6), "%u", authority.channelUs);
  FcChannelConfig cfg;
  portENTER_CRITICAL(&g_mspMux);
  cfg = g_fcCfg;
  portEXIT_CRITICAL(&g_mspMux);
  char rcmap_s[5], mask_s[12] = "null";
  rcmapString(cfg.rcmap, rcmap_s);
  if (cfg.overrideMask >= 0) snprintf(mask_s, sizeof(mask_s), "%ld", (long)cfg.overrideMask);
  char wifiIpStr[16];
  wifiIp(wifiIpStr, sizeof(wifiIpStr));
  char line[1250];  // outTask 줄 버퍼(1400)보다 작게
  snprintf(line, sizeof(line),
           "{\"type\":\"%s\",\"build\":\"%s\",\"wifi\":%s,\"wifi_mode\":\"%s\",\"wifi_ip\":\"%s\",\"wifi_client\":%s,\"wifi_dropped\":%lu,"
           "\"camera\":%s,\"camera_sensor\":\"%s\","
           "\"frame\":\"%dx%d\",\"tag_family\":\"%s\",\"tags_registered\":%u,\"decimate\":%.1f,"
           "\"dec_mode\":\"%s\",\"psram_free\":%u,\"heap_free\":%u,\"rear\":\"USB+BLE+MSP\",\"rx\":%d,\"tx\":%d,"
           "\"fc_tx\":%d,\"fc_rx\":%d,\"msp\":%s,\"authority\":\"%s\",\"authority_fresh\":%s,\"ch6\":%s,"
           "\"rc_tx\":%s,\"kill\":%s,\"kill_tx\":%s,\"throttle_ctrl\":%s,\"sim\":%s,\"ble\":\"%s\","
           "\"ble_connected\":%s,\"ble_mtu\":%u,\"ble_dropped\":%lu,\"ble_buf\":\"%d/%d\",\"ram_min\":%u,"
           "\"ram_largest\":%u,\"dropped_lines\":%lu,\"stack_free\":{\"msp\":%u,\"loop\":%u},\"reset\":\"%s\",\"gimbal\":%s,\"height_src\":\"%s\",\"approach\":\"%s\","
           "\"tx_hz_target\":%d,\"tx_always\":%s,\"rcmap\":\"%s\",\"fc_mask\":%s,\"mask_ok\":%s,\"thr_slot\":%u}",
           type, FW_BUILD, wifiOn() ? "true" : "false", wifiMode(), wifiIpStr, wifiClientConnected() ? "true" : "false",
           (unsigned long)wifiDropped(), g_cameraOk ? "true" : "false", g_sensorName, g_frameW, g_frameH, TAG_FAMILY_NAME,
           (unsigned)TAG_COUNT, g_td ? g_td->quad_decimate : 0.0f, g_decAuto ? "auto" : "manual",
           (unsigned)ESP.getFreePsram(), (unsigned)ESP.getFreeHeap(), MTF_RX_PIN, MTF_TX_PIN, FC_TX_PIN, FC_RX_PIN,
           flags.mspEnabled ? "true" : "false", authority.automatic ? "auto" : "manual",
           authority.fresh ? "true" : "false", ch6, rcTx ? "true" : "false",
           flags.kill ? "true" : "false", killTx ? "true" : "false",
           CTRL_THROTTLE_ENABLED ? "true" : "false", simActive() ? "true" : "false",
           bleName(), bleConnected() ? "true" : "false", bleMtu(), (unsigned long)bleDropped(), bleBuffersFree(),
           bleBuffersTotal(), (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
           (unsigned long)g_droppedLines, g_mspTaskHandle ? (unsigned)uxTaskGetStackHighWaterMark(g_mspTaskHandle) : 0u, (unsigned)uxTaskGetStackHighWaterMark(nullptr), resetReasonName(), GIMBAL_ENABLED ? "true" : "false", flowSnapshot().heightSrc,
           APPROACH_MODE_NAME[g_approachMode & 3], MSP_SEND_HZ, MSP_TX_ALWAYS ? "true" : "false", rcmap_s, mask_s,
           cfg.overrideMask < 0 ? "null" : (overrideMaskOk(cfg.overrideMask) ? "true" : "false"),
           frameSlot(FC_RC_CH_THROTTLE));
  emitLine(line);
}

static void emitTags() {
  char line[900];
  size_t n = snprintf(line, sizeof(line), "{\"type\":\"tags\",\"tags\":[");
  for (size_t i = 0; i < TAG_COUNT && n < sizeof(line); i++) {
    const TagInfo &t = g_tags[i];
    n += snprintf(line + n, sizeof(line) - n,
                  "%s{\"id\":%d,\"size_mm\":%u,\"land_x_mm\":%d,\"land_y_mm\":%d,\"meaning\":\"%s\"}",
                  i ? "," : "", t.id, t.size_mm, t.land_x_mm, t.land_y_mm, t.code);
  }
  if (n < sizeof(line)) snprintf(line + n, sizeof(line) - n, "]}");
  emitLine(line);
}

static void cmdError(const char *reason) {
  char line[200];
  snprintf(line, sizeof(line), "{\"type\":\"cmd_error\",\"reason\":\"%s\"}", reason);
  emitLine(line);
}

static bool otaAllowed();
static uint8_t g_radio = RADIO_DEFAULT;  // 0 both, 1 wifi, 2 ble (NVS "radio")
static const char *RADIO_NAME[] = {"both", "wifi", "ble"};

static void handleCommand(const char *buf) {
  int id, a, b;
  char k[8];
  if (strncmp(buf, "snap stream", 11) == 0) {
    const char *a = buf + 11;
    while (*a == ' ') a++;
    int ms = (!*a || strcmp(a, "off") == 0) ? 0 : atoi(a);
    if (ms > 0 && ms < 100) ms = 100;
    g_streamDest = g_cmdSource;
    g_streamLastMs = 0;
    g_streamMs = ms > 0 ? ms : 0;
    char line[96];
    snprintf(line, sizeof(line), "{\"type\":\"snap_stream\",\"ms\":%lu}", (unsigned long)g_streamMs);
    emitLine(line);
  } else if (strcmp(buf, "snap") == 0 || strcmp(buf, "snap small") == 0) {
    g_snapSmall = buf[4] != '\0';
    g_rawDest = g_cmdSource;  // 사진 줄은 요청한 통로로만
    g_snapRequested = true;
  } else if (strcmp(buf, "radio") == 0 || strncmp(buf, "radio ", 6) == 0) {
    const char *a = buf[5] ? buf + 6 : "";
    int m = -1;
    for (int i = 0; i < 3; i++)
      if (strcmp(a, RADIO_NAME[i]) == 0) m = i;
    if (a[0] && m < 0) return cmdError("형식: radio [wifi|ble|both]");
    char line[160];
    if (m >= 0 && m != g_radio) {
      if (!otaAllowed()) return cmdError("FC 아밍 중 — 무선 설정을 바꾸지 않음 (재시작이 필요)");
      g_prefs.putUChar("radio", (uint8_t)m);
      snprintf(line, sizeof(line), "{\"type\":\"radio\",\"mode\":\"%s\",\"restart\":true}", RADIO_NAME[m]);
      emitLine(line);
      delay(500);  // 응답이 나갈 시간
      ESP.restart();
    }
    snprintf(line, sizeof(line), "{\"type\":\"radio\",\"mode\":\"%s\",\"ble\":%s,\"wifi\":%s}", RADIO_NAME[g_radio],
             bleStarted() ? "true" : "false", wifiOn() ? "true" : "false");
    emitLine(line);
  } else if (strcmp(buf, "wifi") == 0 || strncmp(buf, "wifi ", 5) == 0) {
    // wifi | wifi sta <이름> <비밀번호> (공유기 저장 → 재시작) | wifi ap (공유기 지우고 보드 자체 WiFi → 재시작)
    const char *a = buf[4] ? buf + 5 : "";
    bool sta = strncmp(a, "sta ", 4) == 0, ap = strcmp(a, "ap") == 0;
    if (strcmp(a, "lite on") == 0 || strcmp(a, "lite off") == 0) {
      g_tcpLite = strcmp(a, "lite on") == 0;
      a = "";  // 아래 상태 출력으로
    } else if (strcmp(a, "scan") == 0) {
      static char nets[1200];
      int cnt = wifiScan(nets, sizeof(nets));
      static char line[1300];
      snprintf(line, sizeof(line), "{\"type\":\"wifi_scan\",\"count\":%d,\"nets\":%s}", cnt, nets);
      emitLine(line);
      return;
    }
    if (a[0] && !sta && !ap) return cmdError("형식: wifi | wifi scan | wifi sta <이름> <비밀번호> | wifi ap");
    if (sta || ap) {
      char ssid[33] = "";
      const char *pass = "";
      if (sta) {
        const char *p = a + 4, *sp = strchr(p, ' ');
        size_t n = sp ? (size_t)(sp - p) : strlen(p);
        if (n == 0 || n > 32) return cmdError("공유기 이름은 1~32자");
        memcpy(ssid, p, n);
        ssid[n] = '\0';
        pass = sp ? sp + 1 : "";
        if (strlen(pass) > 64 || (pass[0] && strlen(pass) < 8)) return cmdError("비밀번호는 8~64자 (없으면 생략)");
      }
      if (!otaAllowed()) return cmdError("FC 아밍 중 — WiFi 설정을 바꾸지 않음 (재시작이 필요)");
      g_prefs.putString("sta_ssid", ssid);
      g_prefs.putString("sta_pass", pass);
      char line[160];
      snprintf(line, sizeof(line), "{\"type\":\"wifi_set\",\"mode\":\"%s\",\"ssid\":\"%s\",\"restart\":true}",
               sta ? "sta" : "ap", ssid);  // 비밀번호는 출력하지 않는다
      emitLine(line);
      delay(500);
      ESP.restart();
    }
    char ip[16], ssid[33] = "", line[300];
    wifiIp(ip, sizeof(ip));
    g_prefs.getString("sta_ssid", ssid, sizeof(ssid));
    snprintf(line, sizeof(line),
             "{\"type\":\"wifi\",\"on\":%s,\"mode\":\"%s\",\"ip\":\"%s\",\"host\":\"%s.local\",\"port\":%d,"
             "\"sta_ssid\":\"%s\",\"rssi\":%d,\"tx_dbm\":%.1f,\"lite\":%s,\"reason\":%d,\"ap_ssid\":\"%s\",\"stations\":%d,"
             "\"client\":%s,\"dropped\":%lu}",
             wifiOn() ? "true" : "false", wifiMode(), ip, wifiHost(), WIFI_TCP_PORT, ssid, wifiRssi(), wifiTxDbm(),
             g_tcpLite ? "true" : "false", wifiReason(), bleName(),
             wifiStations(), wifiClientConnected() ? "true" : "false", (unsigned long)wifiDropped());
    emitLine(line);
  } else if (strcmp(buf, "ble lite on") == 0 || strcmp(buf, "ble lite off") == 0 || strcmp(buf, "ble") == 0) {
    if (buf[3]) g_bleLite = strcmp(buf, "ble lite on") == 0;
    char line[160];
    snprintf(line, sizeof(line),
             "{\"type\":\"ble\",\"connected\":%s,\"mtu\":%u,\"lite\":%s,\"dropped\":%lu}", bleConnected() ? "true" : "false",
             (unsigned)bleMtu(), g_bleLite ? "true" : "false", (unsigned long)bleDropped());
    emitLine(line);
  } else if (sscanf(buf, "size %d %d", &id, &a) == 2) {
    TagInfo *t = const_cast<TagInfo *>(findTag(id));
    if (!t) return cmdError("등록되지 않은 ID. config.h의 TAG_TABLE에 먼저 추가");
    if (a < 10 || a > 2000) return cmdError("size는 10~2000(mm)");
    t->size_mm = (uint16_t)a;
    nvsKey(k, 's', id);
    g_prefs.putUShort(k, t->size_mm);
    emitTags();
  } else if (sscanf(buf, "land %d %d %d", &id, &a, &b) == 3) {
    TagInfo *t = const_cast<TagInfo *>(findTag(id));
    if (!t) return cmdError("등록되지 않은 ID. config.h의 TAG_TABLE에 먼저 추가");
    if (abs(a) > 5000 || abs(b) > 5000) return cmdError("land는 -5000~5000(mm)");
    t->land_x_mm = (int16_t)a;
    t->land_y_mm = (int16_t)b;
    nvsKey(k, 'x', id);
    g_prefs.putShort(k, t->land_x_mm);
    nvsKey(k, 'y', id);
    g_prefs.putShort(k, t->land_y_mm);
    emitTags();
  } else if (sscanf(buf, "reset %d", &id) == 1) {
    for (char kind : {'s', 'x', 'y'}) {
      nvsKey(k, kind, id);
      g_prefs.remove(k);
    }
    loadTags();
    emitTags();
  } else if (strcmp(buf, "tags") == 0) {
    emitTags();
  } else if (strcmp(buf, "dec auto") == 0 && g_td) {
    g_decAuto = true;
    emitInfo("info");
  } else if (strncmp(buf, "dec", 3) == 0 && !g_td) {
    cmdError("카메라가 없어 decimate를 바꿀 수 없음");
  } else if (strncmp(buf, "dec ", 4) == 0) {
    float d = atof(buf + 4);
    if (d < 1.0f || d > 4.0f) return cmdError("dec는 auto 또는 1.0~4.0");
    g_decAuto = false;
    g_td->quad_decimate = d;
    g_decReason = "manual";
    emitInfo("info");
  } else if (strcmp(buf, "msp on") == 0 || strcmp(buf, "msp off") == 0) {
    setMspEnabled(buf[5] == 'n');
    emitMspStatus();
  } else if (strncmp(buf, "simh ", 5) == 0) {
    float h = atof(buf + 5);
    if (h < 5.0f || h > 1000.0f) return cmdError("simh는 5~1000(cm)");
    g_simHeightCm = h;
    g_simAtMs = millis() ? millis() : 1;
  } else if (strcmp(buf, "sim off") == 0) {
    g_simAtMs = 0;
    emitInfo("info");
  } else if (strcmp(buf, "mission reset") == 0) {
    resetMission();
    emitMspStatus();
  } else if (strcmp(buf, "kill") == 0 || strcmp(buf, "kill on") == 0) {
    setKill(true);  // AUTO일 때 ch7=2000 → INAV FAILSAFE(DROP). MANUAL에서는 송신하지 않는다.
    portENTER_CRITICAL(&g_motorMux);
    g_motor.active = false;
    g_motor.stopFrames = 10;
    g_motor.generation++;
    portEXIT_CRITICAL(&g_motorMux);
    emitLine("{\"type\":\"kill\",\"state\":\"on\"}");
  } else if (strcmp(buf, "kill off") == 0) {
    setKill(false);  // FAILSAFE 해제. 재아밍은 ARM 스위치를 내렸다 올려야 한다 (INAV: 3초간 정상 수신 필요)
    emitLine("{\"type\":\"kill\",\"state\":\"off\"}");
  } else if (strncmp(buf, "fcmsp ", 6) == 0) {
    // fcmsp <명령번호> [hex payload]   예: fcmsp 0x2000   (255 초과면 MSP v2)
    char *end = nullptr;
    long cmd = strtol(buf + 6, &end, 0);
    if (cmd <= 0 || cmd > 0xFFFF) return cmdError("형식: fcmsp <명령번호> [hex]");
    if (cmd == MSP_SET_RAW_RC || cmd == MSP_SET_MOTOR || cmd == MSP_REBOOT)
      return cmdError("이 명령은 fcmsp로 보낼 수 없음 (motortest·kill 사용)");
    uint8_t pl[64];
    uint16_t plen = 0;
    while (end && *end == ' ') end++;
    for (const char *h = end; h && h[0] && h[1] && plen < sizeof(pl); h += 2) {
      char byte[3] = {h[0], h[1], 0};
      pl[plen++] = (uint8_t)strtol(byte, nullptr, 16);
    }
    portENTER_CRITICAL(&g_queryMux);
    bool busy = g_query.pending;
    if (!busy) {
      g_query.cmd = (uint16_t)cmd;
      memcpy(g_query.payload, pl, plen);
      g_query.payloadLen = plen;
      g_query.sent = g_query.done = false;
      g_query.pending = true;
    }
    portEXIT_CRITICAL(&g_queryMux);
    if (busy) return cmdError("이전 fcmsp 응답을 기다리는 중");
  } else if (strcmp(buf, "motorstop") == 0) {
    portENTER_CRITICAL(&g_motorMux);
    g_motor.active = false;
    g_motor.stopFrames = 10;
    g_motor.generation++;
    portEXIT_CRITICAL(&g_motorMux);
    emitLine("{\"type\":\"motortest\",\"state\":\"stopping\"}");
  } else if (strncmp(buf, "motortest ", 10) == 0) {
    // motortest <1~4|all> <퍼센트> <초>   예: motortest 1 5 3
    char which[8] = "";
    float pct = 0, sec = 0;
    if (sscanf(buf + 10, "%7s %f %f", which, &pct, &sec) != 3) return cmdError("형식: motortest <1~4|all> <퍼센트> <초>");
    uint8_t mask = strcmp(which, "all") == 0 ? 0x0F : (atoi(which) >= 1 && atoi(which) <= MOTOR_COUNT ? 1 << (atoi(which) - 1) : 0);
    if (!mask) return cmdError("모터 번호는 1~4 또는 all");
    if (pct <= 0 || pct > MOTOR_TEST_MAX_PCT) return cmdError("출력은 0 초과 ~ 15%");
    if (sec <= 0 || sec * 1000 > MOTOR_TEST_MAX_MS) return cmdError("시간은 0 초과 ~ 5초");
    if (!fcLinkAlive()) return cmdError("FC 응답 없음 — UART5 MSP와 배선 확인");
    uint32_t now = millis();
    RcAuthoritySnapshot authority = rcAuthoritySnapshot(now);
    if (!authority.automatic) return cmdError("CH6=AUTO(1000)에서만 모터 시험 가능");
    if (!authority.armSwitchDisarmed) return cmdError("CH5=DISARM(1000)에서만 모터 시험 가능");
    if (!authority.throttleLow) return cmdError("스로틀 1100 이하에서만 모터 시험 가능");
    if (controlFlagsSnapshot().kill) return cmdError("kill 해제 후 모터 시험 가능");
    portENTER_CRITICAL(&g_mspMux);
    FcStatus fc = g_fc;
    portEXIT_CRITICAL(&g_mspMux);
    if (!fc.statusAtMs || now - fc.statusAtMs > FC_STATUS_SAFE_TIMEOUT_MS)
      return cmdError("최신 FC 아밍 상태 없음 — 시험 거부");
    if (fc.armingFlags & ARMED_FLAG) return cmdError("FC가 아밍됨 — 모터 시험 거부");
    if (motorCommandBusy()) return cmdError("이전 모터 시험·정지 처리 중");
    uint16_t value = (uint16_t)lroundf(1000.0f + pct * 10.0f);
    portENTER_CRITICAL(&g_mspMux);
    uint32_t acks = g_msp.motorAcks;
    portEXIT_CRITICAL(&g_mspMux);
    portENTER_CRITICAL(&g_motorMux);
    g_motor.mask = mask;
    g_motor.value = value;
    g_motor.untilMs = now + (uint32_t)(sec * 1000);
    g_motor.stopFrames = 0;
    g_motor.ackStart = acks;
    g_motor.active = true;
    g_motor.generation++;
    portEXIT_CRITICAL(&g_motorMux);
    char line[160];
    snprintf(line, sizeof(line), "{\"type\":\"motortest\",\"state\":\"running\",\"motors\":\"%s\",\"value\":%u,\"ms\":%lu}",
             which, value, (unsigned long)(sec * 1000));
    emitLine(line);
  } else if (strncmp(buf, "rec ", 4) == 0) {
    float sec = atof(buf + 4);
    if (sec < 0.2f || sec * 1000 > MIC_MAX_REC_MS) return cmdError("녹음은 0.2~3초");
    if (!micRequestRecord((uint32_t)(sec * 1000))) return cmdError("녹음 중 — 끝난 뒤 다시");
    g_rawDest = g_cmdSource;  // 녹음 줄(MIC_*)도 요청한 통로로만
  } else if (strncmp(buf, "voicecmd ", 9) == 0) {
    const char *kw = buf + 9;
    for (const auto &v : VOICE_COMMANDS) {
      if (strcmp(v.keyword, kw) != 0) continue;
      char list[96];
      strlcpy(list, v.commands, sizeof(list));
      char *save = nullptr;
      for (char *c = strtok_r(list, ";", &save); c; c = strtok_r(nullptr, ";", &save)) {
        if (strncmp(c, "motortest", 9) == 0 || strncmp(c, "voicecmd", 8) == 0 || strncmp(c, "fcmsp", 5) == 0)
          continue;  // 음성으로는 금지
        handleCommand(c);
      }
      char line[200];
      snprintf(line, sizeof(line), "{\"type\":\"voice\",\"keyword\":\"%s\",\"ko\":\"%s\",\"commands\":\"%s\"}",
               v.keyword, v.ko, v.commands);
      emitLine(line);
      return;
    }
    cmdError("등록되지 않은 음성 키워드 (config.h VOICE_COMMANDS)");
  } else if (strcmp(buf, "tagcal") == 0 || strncmp(buf, "tagcal ", 7) == 0) {
    // tagcal | tagcal on|off|reset | tagcal save <id> (수렴한 크기를 NVS 에 저장)
    const char *a = buf[6] ? buf + 7 : "";
    if (strcmp(a, "on") == 0) g_tagCalOn = true;
    else if (strcmp(a, "off") == 0) g_tagCalOn = false;
    else if (strcmp(a, "reset") == 0) for (auto &cc : g_tagCal) cc.reset();
    else if (sscanf(a, "save %d", &id) == 1) {
      TagInfo *t = const_cast<TagInfo *>(findTag(id));
      if (!t) return cmdError("등록되지 않은 ID");
      if (!tagCalReady(tagIndex(t))) return cmdError("아직 크기 보정이 수렴하지 않음 (거리센서 높이가 있는 상태로 태그를 더 보여 줄 것)");
      t->size_mm = (uint16_t)lroundf(g_tagCal[tagIndex(t)].held());
      nvsKey(k, 's', id);
      g_prefs.putUShort(k, t->size_mm);
    } else if (a[0]) {
      return cmdError("형식: tagcal [on|off|reset|save <id>]");
    }
    char line[700];
    size_t n = snprintf(line, sizeof(line), "{\"type\":\"tagcal\",\"on\":%s,\"tags\":[", g_tagCalOn ? "true" : "false");
    for (size_t i = 0; i < TAG_COUNT && n < sizeof(line); i++) {
      const char *src;
      float used = tagSizeUsedMm(&g_tags[i], &src);
      n += snprintf(line + n, sizeof(line) - n,
                    "%s{\"id\":%d,\"cfg_mm\":%u,\"est_mm\":%.1f,\"held_mm\":%.1f,\"n\":%d,\"spread\":%.3f,\"used_mm\":%.1f,"
                    "\"src\":\"%s\"}",
                    i ? "," : "", g_tags[i].id, g_tags[i].size_mm, g_tagCal[i].median(), g_tagCal[i].held(),
                    g_tagCal[i].count(), g_tagCal[i].spread(), used, src);
    }
    if (n < sizeof(line)) snprintf(line + n, sizeof(line) - n, "]}");
    emitLine(line);
  } else if (strcmp(buf, "exp") == 0 || strncmp(buf, "exp ", 4) == 0) {
    // 카메라 노출: exp | exp auto [ae -2~2] | exp fast | exp <노출 0~1200> [게인 0~30]
    // 손으로 움직이면 자동 노출(실내에서 길어짐) 때문에 영상이 번져 태그를 놓친다 → 짧은 노출 + 높은 게인
    sensor_t *s = esp_camera_sensor_get();
    if (!s) return cmdError("카메라 없음");
    const char *a = buf[3] ? buf + 4 : "";
    int v = 0, g = -1;
    if (strncmp(a, "auto", 4) == 0) {
      int ae = 0;
      sscanf(a + 4, "%d", &ae);
      s->set_exposure_ctrl(s, 1);
      s->set_gain_ctrl(s, 1);
      s->set_ae_level(s, ae < -2 ? -2 : (ae > 2 ? 2 : ae));
    } else if (strcmp(a, "fast") == 0) {
      s->set_exposure_ctrl(s, 0);
      s->set_aec_value(s, CAM_EXP_FAST_AEC);
      s->set_gain_ctrl(s, 0);
      s->set_agc_gain(s, CAM_EXP_FAST_GAIN);
    } else if (a[0] && sscanf(a, "%d %d", &v, &g) >= 1) {
      if (v < 0 || v > 1200 || g > 30) return cmdError("형식: exp <노출 0~1200> [게인 0~30]");
      s->set_exposure_ctrl(s, 0);
      s->set_aec_value(s, v);
      if (g >= 0) {
        s->set_gain_ctrl(s, 0);
        s->set_agc_gain(s, g);
      }
    } else if (a[0]) {
      return cmdError("형식: exp | exp auto [ae] | exp fast | exp <노출 0~1200> [게인 0~30]");
    }
    char line[200];
    snprintf(line, sizeof(line),
             "{\"type\":\"exp\",\"aec_auto\":%s,\"aec_value\":%d,\"agc_auto\":%s,\"agc_gain\":%d,\"ae_level\":%d}",
             s->status.aec ? "true" : "false", s->status.aec_value, s->status.agc ? "true" : "false",
             s->status.agc_gain, s->status.ae_level);
    emitLine(line);
  } else if (strcmp(buf, "cam") == 0 || strncmp(buf, "cam ", 4) == 0) {
    // cam | cam <팬°> <틸트°> | cam down | cam front | cam off | cam us <팬µs> <틸트µs>
    const char *a = buf[3] ? buf + 4 : "";
    float p, t;
    int pu, tu;
    bool ok = true;
    if (!a[0]) ok = true;
    else if (strcmp(a, "down") == 0) ok = gimbalSet(0, 0);
    else if (strcmp(a, "front") == 0) ok = gimbalSet(0, GIMBAL_FRONT_TILT_DEG);
    else if (strcmp(a, "off") == 0) ok = gimbalOff();
    else if (sscanf(a, "us %d %d", &pu, &tu) == 2) ok = gimbalRawUs(pu, tu);
    else if (sscanf(a, "%f %f", &p, &t) == 2) ok = gimbalSet(p, t);
    else return cmdError("형식: cam | cam <팬°> <틸트°> | cam down | cam front | cam off | cam us <팬µs> <틸트µs>");
    if (!ok) return cmdError("카메라 서보 비활성 (config.h GIMBAL_ENABLED)");
    char line[320];
    gimbalJson(line, sizeof(line));
    emitLine(line);
  } else if (strcmp(buf, "mtfraw") == 0) {
    g_mtfRawRequest = true;
  } else if (strcmp(buf, "approach") == 0 || strncmp(buf, "approach ", 9) == 0) {
    // approach | approach on|shadow|legacy|off | approach set <이름> <값> | approach reset | approach params
    const char *a = buf[8] ? buf + 9 : "";
    char name[24];
    float v;
    if (!a[0] || strcmp(a, "params") == 0) {
      char pj[1250];
      approachParamsJson(g_approach.p, pj, sizeof(pj));
      char line[1350];
      snprintf(line, sizeof(line), "{\"type\":\"approach_params\",\"mode\":\"%s\",\"params\":%s}",
               APPROACH_MODE_NAME[g_approachMode & 3], pj);
      emitLine(line);
      return;
    }
    // 지상 시험 ↔ 비행 입력 전환. bench = 높이·속도를 카메라에서(작업대에서 손으로 태그를 움직여 확인) + 출력 켬.
    // flight = 거리센서·FC 광류(기본값). 모드는 그대로 둔다.
    bool bench = strcmp(a, "bench") == 0, flight = strcmp(a, "flight") == 0;
    if (bench || flight) {
      portENTER_CRITICAL(&g_approachMux);
      g_approach.p.velSrc = bench ? 1.0f : 2.0f;  // 비행 = 칼만 (광류는 `approach set vel_src 0`)
      g_approach.p.heightSrc = bench ? 1.0f : 0.0f;
      portEXIT_CRITICAL(&g_approachMux);
      if (flight) {
        emitLine("{\"type\":\"approach_set\",\"name\":\"flight\",\"vel_src\":\"kf\",\"height_src\":\"rangefinder\"}");
        return;
      }
      emitLine("{\"type\":\"approach_set\",\"name\":\"bench\",\"vel_src\":\"tag\",\"height_src\":\"tag_fallback\","
               "\"note\":\"지상 시험 전용. 비행 전 approach flight\"}");
      a = "on";  // 이어서 출력 모드로
    }
    int newMode = -1;
    if (strcmp(a, "off") == 0) newMode = APM_OFF;
    else if (strcmp(a, "shadow") == 0) newMode = APM_SHADOW;
    else if (strcmp(a, "on") == 0) newMode = APM_ON;
    else if (strcmp(a, "legacy") == 0) newMode = APM_LEGACY;
    if (newMode >= 0) {
      portENTER_CRITICAL(&g_approachMux);
      g_approach.reset();  // 모드 전환 시 적분·명령 0 에서 시작
      g_approachOut = ApproachOutput();
      g_approachMode = newMode;
      portEXIT_CRITICAL(&g_approachMux);
      // on/legacy 는 FC 마스크가 확인돼야 실제로 출력된다. 아니면 제어기가 fault mask/mask_unknown 으로 막는다.
      FcChannelConfig cfg;
      portENTER_CRITICAL(&g_mspMux);
      cfg = g_fcCfg;
      portEXIT_CRITICAL(&g_mspMux);
      const char *warn = nullptr;
      if (newMode == APM_ON || newMode == APM_LEGACY) {
        if (cfg.overrideMask < 0) warn = "FC msp_override_channels 를 아직 못 읽음 — 출력은 fault mask_unknown 으로 막힘";
        else if (!overrideMaskOk(cfg.overrideMask)) warn = "FC 마스크에 스로틀 비트(8)가 있거나 롤·피치가 없음 — 출력 차단. CLI: set msp_override_channels = 7 (보드 킬까지 71)";
      }
      char line[320];
      snprintf(line, sizeof(line), "{\"type\":\"approach_mode\",\"mode\":\"%s\",\"fc_mask\":%ld,\"warning\":%s%s%s}",
               APPROACH_MODE_NAME[newMode], (long)cfg.overrideMask, warn ? "\"" : "", warn ? warn : "null", warn ? "\"" : "");
      emitLine(line);
      return;
    }
    if (strcmp(a, "reset") == 0) {
      portENTER_CRITICAL(&g_approachMux);
      g_approach.reset();
      g_approachOut = ApproachOutput();
      portEXIT_CRITICAL(&g_approachMux);
      emitLine("{\"type\":\"approach_mode\",\"reset\":true}");
      return;
    }
    if (sscanf(a, "set %23s %f", name, &v) == 2) {
      ApproachParams p = g_approach.p;
      if (!approachSetParam(p, name, v)) return cmdError("approach set: 모르는 이름이거나 범위 밖 (approach params 로 목록)");
      portENTER_CRITICAL(&g_approachMux);
      g_approach.p = p;
      portEXIT_CRITICAL(&g_approachMux);
      g_tagVelAlpha = p.tagVelAlpha;
      char line[120];
      snprintf(line, sizeof(line), "{\"type\":\"approach_set\",\"name\":\"%s\",\"value\":%.3f}", name, v);
      emitLine(line);
      return;
    }
    cmdError("형식: approach [on|shadow|legacy|off|bench|flight|reset|params|set <이름> <값>]");
  } else if (strcmp(buf, "alt") == 0 || strncmp(buf, "alt ", 4) == 0) {
    // alt | alt land | alt cancel | alt target <cm> | alt params | alt set <이름> <값>
    const char *a = buf[3] ? buf + 4 : "";
    char name[24];
    float v;
    if (!a[0]) {
      emitAlt();
      return;
    }
    if (strcmp(a, "land") == 0) {
      AltHoldOutput o;
      portENTER_CRITICAL(&g_approachMux);
      o = g_altOut;
      portEXIT_CRITICAL(&g_approachMux);
      FcChannelConfig cfg;
      portENTER_CRITICAL(&g_mspMux);
      cfg = g_fcCfg;
      portEXIT_CRITICAL(&g_mspMux);
      if (!fcTakesThrottle(cfg, millis())) return cmdError("alt land: FC 가 보드 스로틀을 쓰지 않음 (msp_override_channels 에 스로틀 비트 8 필요)");
      if (o.state == AltState::Manual) return cmdError("alt land: CH6 AUTO 에서만");
      g_altLandCmd = true;
      emitLine("{\"type\":\"alt_mode\",\"land\":true}");
      return;
    }
    if (strcmp(a, "cancel") == 0) {  // 착륙 취소 → 지금 높이에서 유지 (착지 뒤에는 안 됨: 수동으로)
      g_altLandCmd = false;
      AltHoldOutput o;
      portENTER_CRITICAL(&g_approachMux);
      o = g_altOut;
      portEXIT_CRITICAL(&g_approachMux);
      FlowState f = flowSnapshot();
      if (o.state == AltState::Landing && heightValid(f)) {
        portENTER_CRITICAL(&g_approachMux);
        g_alt.cancelLanding(f.s.distance_mm / 10.0f);
        portEXIT_CRITICAL(&g_approachMux);
      }
      emitLine("{\"type\":\"alt_mode\",\"land\":false}");
      return;
    }
    if (sscanf(a, "target %f", &v) == 1) {
      portENTER_CRITICAL(&g_approachMux);
      g_alt.setTarget(v);
      portEXIT_CRITICAL(&g_approachMux);
      char line[80];
      snprintf(line, sizeof(line), "{\"type\":\"alt_mode\",\"target\":%.1f}", g_alt.targetCm());
      emitLine(line);
      return;
    }
    if (strcmp(a, "params") == 0) {
      char pj[600];
      altParamsJson(g_alt.p, pj, sizeof(pj));
      char line[700];
      snprintf(line, sizeof(line), "{\"type\":\"alt_params\",\"params\":%s}", pj);
      emitLine(line);
      return;
    }
    if (sscanf(a, "set %23s %f", name, &v) == 2) {
      AltHoldParams p = g_alt.p;
      if (!altSetParam(p, name, v)) return cmdError("alt set: 모르는 이름이거나 범위 밖 (alt params 로 목록)");
      portENTER_CRITICAL(&g_approachMux);
      g_alt.p = p;
      portEXIT_CRITICAL(&g_approachMux);
      char line[120];
      snprintf(line, sizeof(line), "{\"type\":\"alt_set\",\"name\":\"%s\",\"value\":%.3f}", name, v);
      emitLine(line);
      return;
    }
    cmdError("형식: alt [land|cancel|target <cm>|params|set <이름> <값>]");
  } else if (strcmp(buf, "tour") == 0 || strncmp(buf, "tour ", 5) == 0) {
    // tour | tour on|off|reset|params|acts | tour set <이름> <값>
    const char *a = buf[4] ? buf + 5 : "";
    char name[24];
    float v;
    if (!a[0]) {
      emitTour();
      return;
    }
    if (strcmp(a, "on") == 0 || strcmp(a, "off") == 0 || strcmp(a, "reset") == 0) {
      bool on = strcmp(a, "reset") == 0 ? g_tourOn : strcmp(a, "on") == 0;
      portENTER_CRITICAL(&g_approachMux);
      g_tourOn = on;
      g_tour.reset();
      g_tourOut = TourOutput();
      portEXIT_CRITICAL(&g_approachMux);
      const char *warn = nullptr;
      if (on && g_approachMode != APM_ON) warn = "approach 가 on 이 아님 — 투어는 계산만 하고 FC 로 나가지 않는다 (approach on)";
      char line[300];
      snprintf(line, sizeof(line), "{\"type\":\"tour_mode\",\"on\":%s,\"reset\":true,\"warning\":%s%s%s}",
               on ? "true" : "false", warn ? "\"" : "", warn ? warn : "null", warn ? "\"" : "");
      emitLine(line);
      return;
    }
    if (strcmp(a, "params") == 0) {
      char pj[700];
      tourParamsJson(g_tour.p, pj, sizeof(pj));
      char line[800];
      snprintf(line, sizeof(line), "{\"type\":\"tour_params\",\"on\":%s,\"params\":%s}", g_tourOn ? "true" : "false", pj);
      emitLine(line);
      return;
    }
    if (strcmp(a, "acts") == 0) {  // 태그 ID → 행동 표
      char line[1100];
      size_t n = snprintf(line, sizeof(line), "{\"type\":\"tour_acts\",\"acts\":[");
      for (size_t i = 0; i < sizeof(TOUR_ACTIONS) / sizeof(TOUR_ACTIONS[0]) && n < sizeof(line) - 160; i++) {
        const TourActionDef &d = TOUR_ACTIONS[i];
        n += snprintf(line + n, sizeof(line) - n, "%s{\"id\":%d,\"act\":\"%s\",\"turn\":%d,\"dwell_s\":%u,\"ko\":\"%s\"}",
                      i ? "," : "", d.id, d.code, d.turnDeg, d.dwellS, d.ko);
      }
      snprintf(line + n, sizeof(line) - n, "],\"other\":\"%s\"}", TOUR_ACTION_DEFAULT.ko);
      emitLine(line);
      return;
    }
    if (sscanf(a, "set %23s %f", name, &v) == 2) {
      TourParams p = g_tour.p;
      if (!tourSetParam(p, name, v)) return cmdError("tour set: 모르는 이름이거나 범위 밖 (tour params 로 목록)");
      portENTER_CRITICAL(&g_approachMux);
      g_tour.p = p;
      portEXIT_CRITICAL(&g_approachMux);
      char line[120];
      snprintf(line, sizeof(line), "{\"type\":\"tour_set\",\"name\":\"%s\",\"value\":%.3f}", name, v);
      emitLine(line);
      return;
    }
    cmdError("형식: tour [on|off|reset|params|acts|set <이름> <값>]");
  } else if (strcmp(buf, "info") == 0) {
    emitInfo("info");
  } else if (buf[0]) {
    emitLine("{\"type\":\"help\",\"commands\":[\"snap [small]\",\"snap stream <ms>|off\",\"ble [lite on|off]\",\"wifi [sta <이름> <비밀번호>|ap]\",\"radio [wifi|ble|both]\",\"tags\",\"size <id> <mm>\",\"land <id> <x_mm> <y_mm>\","
             "\"reset <id>\",\"dec auto\",\"dec <1.0~4.0>\",\"msp on|off\",\"mission reset\",\"simh <cm>\",\"sim off\",\"mtfraw\",\"motortest <1~4|all> <pct> <sec>\",\"motorstop\",\"kill\",\"kill off\",\"fcmsp <cmd> [hex]\",\"rec <sec>\",\"voicecmd <키워드>\",\"cam [<팬> <틸트>|down|front|off|us <p> <t>]\",\"exp [auto [ae]|fast|<노출> [게인]]\",\"tagcal [on|off|reset|save <id>]\",\"approach [on|shadow|legacy|off|bench|flight|reset|params|set <k> <v>]\",\"tour [on|off|reset|params|acts|set <k> <v>]\",\"alt [land|cancel|target <cm>|params|set <k> <v>]\",\"info\"]}");
  }
}

// 바이트를 줄로 모은다. USB와 BLE가 각자 하나씩 가진다.
struct LineAssembler {
  char buf[128];  // `wifi sta <이름> <비밀번호>` 가 들어갈 만큼
  size_t len = 0;
  void feed(char ch) {
    if (ch == '\r') return;
    if (ch != '\n') {
      if (len < sizeof(buf) - 1) buf[len++] = ch;
      return;
    }
    buf[len] = 0;
    len = 0;
    handleCommand(buf);  // 카메라가 없어도 simh·msp·info 등은 동작해야 한다
  }
};

static void handleCommands() {
  static LineAssembler usb, ble, tcp;
  g_cmdSource = SRC_USB;
  while (Serial.available()) usb.feed((char)Serial.read());
  uint8_t tmp[64];
  size_t n;
  g_cmdSource = SRC_BLE;
  while ((n = bleRead(tmp, sizeof(tmp))) > 0)
    for (size_t i = 0; i < n; i++) ble.feed((char)tmp[i]);
  g_cmdSource = SRC_TCP;
  while ((n = wifiRead(tmp, sizeof(tmp))) > 0)
    for (size_t i = 0; i < n; i++) tcp.feed((char)tmp[i]);
  g_cmdSource = SRC_ALL;
}

// ---------------------------------------------------------------------------

// WiFi 펌웨어 업데이트 허용 조건: FC 가 디스아밍으로 확인됐거나, FC 상태가 5초 넘게 없을 때(작업대에서 FC 없이).
// 아밍 중 업데이트 → 재부팅으로 보드 출력이 끊기므로 받지 않는다.
static bool otaAllowed() {
  FcStatus f;
  portENTER_CRITICAL(&g_mspMux);
  f = g_fc;
  portEXIT_CRITICAL(&g_mspMux);
  uint32_t now = millis();
  if (f.statusAtMs && now - f.statusAtMs < 2000) return !(f.armingFlags & ARMED_FLAG);
  return !f.statusAtMs || now - f.statusAtMs > 5000;
}

void setup() {
  // 20261011 코어 덤프(-34 panic): 카메라 드라이버 태스크(cam_task)가 "FB-SIZE: x != y"(센서 프레임 크기 오류) 로그를
  // 처음 찍으면서 콘솔(UART0) 쓰기 잠금을 그때 만들었고, 그 잠금이 PSRAM(0x3d9a…)에 잡혀 스핀락에서 멈췄다.
  // (1) IDF 로그를 꺼서 드라이버 오류 로그가 콘솔로 가지 않게, (2) 콘솔 잠금은 내부 RAM 이 넉넉한 지금 미리 만든다.
  esp_log_level_set("*", ESP_LOG_NONE);
  printf("\n");
  fflush(stdout);
  applyFlightSpeed(g_approach.p);  // 비행 속도 기본값 (config.h AP_*) — 태스크 시작 전에
  Serial.setTxBufferSize(8192);
  Serial.begin(115200);  // HWCDC 인터럽트가 이 코어(core 1)에 붙는다
  g_outMutex = xSemaphoreCreateMutex();
  g_emitMutex = xSemaphoreCreateMutex();
  // 출력 버퍼 8KB 는 PSRAM 에 (내부 RAM 절약). 버퍼 구조체(스핀락 포함)는 내부 RAM 의 정적 변수
  static StaticMessageBuffer_t outBufStruct;
  static uint8_t *outBufStorage = (uint8_t *)heap_caps_malloc(8192 + 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  g_outBuf = outBufStorage ? xMessageBufferCreateStatic(8192, outBufStorage, &outBufStruct) : xMessageBufferCreate(8192);
  xTaskCreatePinnedToCore(outTask, "out", 4096, nullptr, 4, nullptr, 1);
  // 스냅샷 줄 보내기 (카메라 루프가 멈추지 않게 따로). 낮은 우선순위
  g_snapQ = xQueueCreate(1, sizeof(SnapJob));
  xTaskCreatePinnedToCore(snapTask, "snap", 3072, nullptr, 1, nullptr, 0);
  g_prefs.begin("tagfw", false);
  g_radio = g_prefs.getUChar("radio", RADIO_DEFAULT);
  if (g_radio > 2) g_radio = RADIO_DEFAULT;
  if (g_radio != 1) bleBegin();  // USB 대신 BLE로도 같은 데이터·명령을 주고받는다 (radio ble·both)
  if (g_radio != 2 && WIFI_ENABLED_DEFAULT) {
    // 저장된 공유기(`wifi sta <이름> <비밀번호>`)가 있으면 거기 붙고(PC 는 인터넷 유지), 없거나 20초 안에 못 붙으면
    // 보드 자체 WiFi. 비밀번호는 소스가 아니라 보드 NVS 에만 둔다
    char staSsid[33] = "", staPass[65] = "";
    g_prefs.getString("sta_ssid", staSsid, sizeof(staSsid));
    g_prefs.getString("sta_pass", staPass, sizeof(staPass));
    if (wifiBegin(bleName(), WIFI_AP_PASS, staSsid, staPass, WIFI_TCP_PORT))
      wifiOtaBegin(bleName(), WIFI_AP_PASS, otaAllowed);  // 다음부터 펌웨어는 WiFi 로 (USB 불필요)
  }
  if (!micBegin(emitLine, emitLineWait)) emitLine("{\"type\":\"mic\",\"ok\":false,\"reason\":\"I2S 시작 실패\"}");

  loadTags();

  pinMode(MTF_RX_PIN, INPUT);
  Serial1.setRxBufferSize(2048);
  Serial1.begin(MTF_BAUD, SERIAL_8N1, MTF_RX_PIN, MTF_TX_PIN);
  xTaskCreatePinnedToCore(flowTask, "flow", 6144, nullptr, 3, nullptr, 0);

  Serial2.setRxBufferSize(1024);
  Serial2.begin(FC_BAUD, SERIAL_8N1, FC_RX_PIN, FC_TX_PIN);
  // 접근 제어 보고(1000B 줄 + 실수 서식)를 이 태스크가 만든다 → 6KB 는 여유가 적어 10KB (info 의 stack_free 로 확인)
  xTaskCreatePinnedToCore(mspTask, "msp", 10240, nullptr, 3, &g_mspTaskHandle, 0);

  g_cameraOk = initCamera();
  if (g_cameraOk) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      g_frameW = fb->width;
      g_frameH = fb->height;
      esp_camera_fb_return(fb);
    }
    g_fx = (g_frameW / 2.0f) / tanf(CAM_HFOV_DEG * 0.5f * (float)M_PI / 180.0f);
    // AprilTag는 프레임마다 작은 메모리를 수천 번 할당한다. 기본 기준(4KB 미만 → 내부 RAM)이면
    // 검출 중 내부 RAM 여유가 수백 바이트까지 떨어져 BLE 컨트롤러가 연결용 메모리를 못 받는다
    // (20261003 실측: 최저 324B, "BLE_INIT: Malloc failed"). 이후 할당은 32바이트 이상이면 PSRAM으로.
    heap_caps_malloc_extmem_enable(32);
    initDetector();
  }
  // 카메라 XCLK(LEDC 타이머 0) 설정 뒤에 서보 PWM(타이머 1)을 켠다
  if (!gimbalBegin()) emitLine("{\"type\":\"gimbal\",\"ok\":false,\"reason\":\"서보 PWM(LEDC) 시작 실패\"}");
  delay(300);  // USB 호스트가 포트를 다시 붙잡을 시간
  emitInfo("boot");
  emitTags();
}

void loop() {
  handleCommands();

  if (!g_cameraOk) {
    static uint32_t last = 0;
    if (millis() - last > 2000) {
      last = millis();
      emitLine("{\"type\":\"vision\",\"camera\":false,\"decision\":\"NO_CAMERA\"}");
    }
    delay(20);
    return;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    delay(5);
    return;
  }
  g_frameCaptureMs = millis();

  if (g_snapRequested && !g_snapBusy) {
    g_snapRequested = false;
    sendSnapshot(fb, g_snapSmall);
  } else if (g_streamMs && !g_snapBusy && millis() - g_streamLastMs >= g_streamMs &&
             (g_streamDest != SRC_TCP || wifiClientConnected())) {
    g_streamLastMs = millis();
    g_rawDest = g_streamDest;
    sendSnapshot(fb, true, g_streamDest == SRC_TCP);  // WiFi 스트림은 원본(binary), USB·BLE 는 base64
  }

  // 이 프레임을 해석할 카메라 각도·높이 (프레임을 받은 직후 값)
  GimbalPose gp = gimbalPose();
  FlowState fs = flowSnapshot();
  float heightCm = heightValid(fs) ? fs.s.distance_mm / 10.0f : -1.0f;
  // 크기 자동 보정 조건: 거리센서 높이(시뮬레이션 제외)·카메라 바로 아래·기체가 거의 수평
  bool calAllowed = false;
  if (g_tagCalOn && heightCm >= TAGCAL_MIN_H_CM && heightCm <= TAGCAL_MAX_H_CM && strcmp(fs.heightSrc, "sim") != 0 &&
      gp.known && gp.settled && fabsf(gp.tilt) < 3.0f) {
    FcAttitude att;
    portENTER_CRITICAL(&g_mspMux);
    att = g_fcAtt;
    portEXIT_CRITICAL(&g_mspMux);
    calAllowed = att.atMs && (millis() - att.atMs) < 1200 && abs(att.rollDeci) <= TAGCAL_MAX_ATT_DECI &&
                 abs(att.pitchDeci) <= TAGCAL_MAX_ATT_DECI;
  }

  float usedDec = g_td->quad_decimate;
  uint32_t t0 = millis();
  // 동적 ROI (임베디드 AprilTag 관행 — OpenMV): 방금 본 태그 주변만 찾는다. 인식 시간은 찾는 화소 수에 거의 비례.
  // 20261011: 8.5cm 태그(38~60px)를 전체 화면 decimate 2 로 찾느라 0.56s/장(1.6fps) → 영상·제어 지연이 같이 커졌다.
  // 못 찾으면 다음 프레임은 전체 화면으로 다시 찾는다.
  int rx0 = 0, ry0 = 0, rw = fb->width, rh = fb->height;
  bool roiUsed = false;
  float fullDec = g_td->quad_decimate;
  // 투어가 다음 태그를 찾는 중이면 TOUR_FULL_EVERY 프레임에 한 번은 전체 화면 (ROI 는 떠나는 태그만 본다)
  static uint32_t tourFullTick = 0;
  bool tourFull = g_tourScanFull && (++tourFullTick % TOUR_FULL_EVERY == 0);
  if (ROI_ENABLED && g_roi.on && g_roi.side > 0 && !tourFull) {
    int half = (int)fmaxf(ROI_MIN_HALF_PX, ROI_SIDE_MULT * g_roi.side);
    rx0 = (int)g_roi.cx - half;
    ry0 = (int)g_roi.cy - half;
    if (rx0 < 0) rx0 = 0;
    if (ry0 < 0) ry0 = 0;
    rw = min(2 * half, (int)fb->width - rx0) & ~3;
    rh = min(2 * half, (int)fb->height - ry0) & ~3;
    if (rw >= 64 && rh >= 64) {
      roiUsed = true;
      // ROI 안에서는 태그 한 변이 ROI_MIN_SIDE_PX 이상 남는 가장 큰 축소 (1, 1.5, 2, 3, 4)
      static const float lv[] = {1.0f, 1.5f, 2.0f, 3.0f, 4.0f};
      float d = 1.0f;
      for (float x : lv)
        if (g_roi.side / x >= ROI_MIN_SIDE_PX) d = x;
      g_td->quad_decimate = d;
      usedDec = d;
    } else {
      rx0 = ry0 = 0;
      rw = fb->width;
      rh = fb->height;
    }
  }
  image_u8_t im = {.width = (int32_t)rw, .height = (int32_t)rh, .stride = (int32_t)fb->width,
                   .buf = fb->buf + (size_t)ry0 * fb->width + rx0};
  zarray_t *dets = apriltag_detector_detect(g_td, &im);
  if (roiUsed) {
    g_td->quad_decimate = fullDec;  // 전체 화면용 값 복원 (놓쳤을 때 autoDecimate 가 이어서)
    // ROI 좌표 → 전체 화면 좌표 (중심·꼭짓점·호모그래피)
    for (int i = 0; i < zarray_size(dets); i++) {
      apriltag_detection_t *det;
      zarray_get(dets, i, &det);
      det->c[0] += rx0;
      det->c[1] += ry0;
      for (int k = 0; k < 4; k++) {
        det->p[k][0] += rx0;
        det->p[k][1] += ry0;
      }
      for (int j = 0; j < 3; j++) {
        MATD_EL(det->H, 0, j) += rx0 * MATD_EL(det->H, 2, j);
        MATD_EL(det->H, 1, j) += ry0 * MATD_EL(det->H, 2, j);
      }
    }
  }
  uint32_t frameMs = millis() - t0;

  int accepted = 0, candidates = 0;
  char ids[96] = "";
  TagObs best = {};
  TourSeen seen[TOUR_MAX_SEEN];
  int nSeen = 0;
  int focusId = g_tourFocusId;
  for (int i = 0; i < zarray_size(dets); i++) {
    apriltag_detection_t *det;
    zarray_get(dets, i, &det);
    if (det->decision_margin < TAG_MIN_MARGIN) continue;
    TagObs o = observe(det, gp, heightCm, calAllowed);
    reportTag(o);
    if (isMissionTag(o.info)) {
      candidates++;
      // 대표 태그: 투어가 따라가는 태그가 있으면 그것, 없으면 가장 크게 보이는 것 (ROI·target 줄·legacy 제어)
      bool f = focusId >= 0 && det->id == focusId, bf = best.det && focusId >= 0 && best.det->id == focusId;
      if (!best.det || (f && !bf) || (f == bf && o.side > best.side)) best = o;
      if (o.geomOk && gp.settled && nSeen < TOUR_MAX_SEEN) {
        // 태그 화살표(인쇄물 위쪽) 방향: 화면에서 시계 rot° → 화면 (오른쪽, 위) = (sin, cos) → 기체 기준 각
        float ur, uf, a = o.rotDeg * (float)M_PI / 180.0f;
        imageToBody(sinf(a), cosf(a), ur, uf);
        TourSeen &s = seen[nSeen++];
        s.id = det->id;
        s.rightCm = o.bodyRight;
        s.fwdCm = o.bodyFwd;
        s.upDeg = atan2f(ur, uf) * 180.0f / (float)M_PI;
      }
    }
    size_t l = strlen(ids);
    snprintf(ids + l, sizeof(ids) - l, "%s%d", accepted ? "," : "", det->id);
    accepted++;
  }
  static uint32_t lastFrameAt = millis();
  uint32_t nowMs = millis();
  // 카메라가 움직이는 중이거나 위치를 못 구했으면 태그를 못 본 것으로 처리 → 스틱 중립
  bool controlUse = best.det && best.geomOk && gp.settled;
  updateMission(controlUse ? &best : nullptr, flowSnapshot(), nowMs - lastFrameAt);
  publishSeen(seen, nSeen);
  lastFrameAt = nowMs;
  if (best.det) reportTarget(best, candidates, frameMs, gp, controlUse);  // det 가 살아 있는 동안 출력
  // 좌표 방식(-40): 인식할 때마다 짧은 위치 줄 (약 130바이트). 맥북은 사진 없이 이 좌표로 태그 위치·궤적·추정 위치를 그린다.
  // 영상(snap stream)과 따로 늘 보낸다 — 어느 방식을 쓸지는 뷰어가 고른다.
  {
    ApproachOutput ao;
    portENTER_CRITICAL(&g_approachMux);
    ao = g_approachOut;
    portEXIT_CRITICAL(&g_approachMux);
    char pl[220];
    if (best.det)
      snprintf(pl, sizeof(pl),
               "{\"type\":\"pos\",\"id\":%d,\"cx\":%.0f,\"cy\":%.0f,\"s\":%.0f,\"rot\":%.0f,\"r\":%.1f,\"f\":%.1f,"
               "\"ke\":[%.1f,%.1f],\"kp\":%d,\"roi\":%d,\"dms\":%lu,\"h\":%.0f}",
               best.det->id, best.det->c[0], best.det->c[1], best.side, best.rotDeg, best.bodyRight, best.bodyFwd,
               ao.kfE[0], ao.kfE[1], ao.predicting ? 1 : 0, roiUsed ? 1 : 0, (unsigned long)frameMs, heightCm);
    else
      snprintf(pl, sizeof(pl), "{\"type\":\"pos\",\"id\":-1,\"ke\":[%.1f,%.1f],\"kp\":%d,\"kon\":%d,\"roi\":%d,\"dms\":%lu,\"h\":%.0f}",
               ao.kfE[0], ao.kfE[1], ao.predicting ? 1 : 0, ao.kfOn ? 1 : 0, roiUsed ? 1 : 0, (unsigned long)frameMs, heightCm);
    emitLine(pl);
  }

  static uint32_t missStreak = 0;
  missStreak = best.det ? 0 : missStreak + 1;
  float targetSide = best.det ? best.side : 0.0f;
  int targetId = best.det ? best.det->id : -1;
  if (best.det) {
    g_lastTargetSizeMm = best.sizeMm;
    g_lastTargetSeenMs = millis();
    g_roi = {true, (float)best.det->c[0], (float)best.det->c[1], best.side};  // 다음 프레임은 이 주변만
  } else {
    g_roi.on = false;  // 못 찾음 → 다음 프레임은 전체 화면
  }
  g_roiUsedLast = roiUsed;

  apriltag_detections_destroy(dets);
  esp_camera_fb_return(fb);

  autoDecimate(targetSide, missStreak);

  // 비전 상태: 2Hz
  static uint32_t lastStatus = 0, frames = 0, lastFpsMs = 0;
  static float fps = 0;
  frames++;
  uint32_t now = millis();
  if (now - lastFpsMs >= 1000) {
    fps = frames * 1000.0f / (now - lastFpsMs);
    frames = 0;
    lastFpsMs = now;
  }
  if (now - lastStatus >= 500) {
    lastStatus = now;
    char line[440];
    snprintf(line, sizeof(line),
             "{\"type\":\"vision\",\"camera\":true,\"fps\":%.1f,\"detect_ms\":%lu,\"tags\":%d,\"ids\":[%s],"
             "\"target_id\":%d,\"decimate_used\":%.1f,\"decimate_next\":%.1f,\"dec_mode\":\"%s\","
             "\"dec_reason\":\"%s\",\"phase\":\"%s\",\"decision\":\"%s\",\"gimbal\":[%.0f,%.0f,%s],"
             "\"height_src\":\"%s\",\"roi\":%s}",
             fps, (unsigned long)frameMs, accepted, ids, targetId, usedDec, g_td->quad_decimate,
             g_decAuto ? "auto" : "manual", g_decReason, PHASE_CODE[ctrlSnapshot().phase],
             targetId >= 0 ? "TARGET_SEEN" : "NO_TARGET", gp.pan, gp.tilt, gp.settled ? "true" : "false",
             fs.heightSrc, g_roiUsedLast ? "true" : "false");
    emitLine(line);
  }
}
