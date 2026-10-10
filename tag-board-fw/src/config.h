#pragma once
#include <stdint.h>

// ---------------------------------------------------------------------------
// 빌드 식별자 (부팅 메시지에 출력)
// ---------------------------------------------------------------------------
#define FW_BUILD "TAGVISION-20261011-51"

// ---------------------------------------------------------------------------
// MTF-01P (115200 8N1, 100Hz) — Micolink·MSP(INAV 출력) 자동 구분. 기존 RIDER-SERIAL 펌웨어와 같은 핀
// ---------------------------------------------------------------------------
#define MTF_RX_PIN 44  // XIAO D7 ← MTF-01P TX
#define MTF_TX_PIN 43  // XIAO D6 → MTF-01P RX
#define MTF_BAUD 115200
#define FLOW_REPORT_MS 200      // flow JSON 출력 주기 (5Hz)
#define FLOW_STALE_MS 300       // 이 시간 동안 패킷이 없으면 connected=false

// ---------------------------------------------------------------------------
// FC 연결 (MSP over UART) — MicoAir H743 V2 의 UART5 커넥터
// XIAO D0(GPIO1) → RX5, XIAO D1(GPIO2) ← TX5, GND 공통. INAV Ports 탭에서 UART5 MSP 켜기
// ---------------------------------------------------------------------------
#define FC_TX_PIN 1             // XIAO D0 → FC RX5
#define FC_RX_PIN 2             // XIAO D1 ← FC TX5
#define FC_BAUD 115200          // INAV MSP 기본 속도
#define MSP_ENABLED_DEFAULT true
// 2026-10-10 엔지니어 요청: CH6 위치와 무관하게 SET_RAW_RC 를 50Hz 로 계속 보낸다. INAV 는 MSP RC OVERRIDE
// 모드(CH6)가 꺼져 있으면 프레임을 무시하므로(rx.c: IS_RC_MODE_ACTIVE(BOXMSPRCOVERRIDE) 일 때만 적용),
// AUTO 로 바뀌는 순간 이미 최신 프레임이 FC 안에 있어 통신 대기·초기화 지연이 없다.
#define MSP_SEND_HZ 50          // INAV msp_override 는 50Hz 로 채널을 갱신한다 (DELAY_50_HZ)
#define MSP_TX_ALWAYS true      // false 면 예전처럼 CH6=AUTO 일 때만 보낸다
// ★ FC 처리 한도: INAV 9.1 msp_serial.c 는 MSP 포트당 SERIAL 태스크 한 번(10ms)에 명령 **하나** 만 처리한다
//   ("process one command at a time so as not to block") → 포트당 최대 100 명령/s. SET_RAW_RC 50/s 를 빼면
//   나머지 질의는 모두 합쳐 50/s 미만, 여유를 두어 ≤40/s 로 맞춘다. -25 는 152/s 를 보내 FC 가 절반을 버렸다
//   (ack 55%, fcmsp 응답 유실). msp 줄의 ack_hz ≈ tx_hz 로 확인한다.
//   현재 예산(접근 shadow/on): RC 16.7 + 광류 10 + 높이 10 + 자세 2 + 상태 2 + 배터리 1 + 모터 1 + 설정 1 ≈ 44/s → 총 94/s
#define MSP_POLL_EVERY 3        // MSP_RC 요청 16.7Hz → CH6 전환을 ≤60ms 안에 감지 (FC 적용은 즉시, 이건 보드 인지)
#define MSP_LINK_TIMEOUT_MS 1000
#define MSP_CHANNELS 8          // 롤·피치·스로틀·요 + AUX 4개 (AUX는 INAV 마스크에 넣지 않으면 무시됨)

// MSP_SET_RAW_RC 프레임의 채널 위치는 수신기 순서(INAV rcmap)다. INAV 9.1 rx/msp_override.c 는
// 논리 채널 i 의 값을 frame[rcmap[i]] 에서 읽는다. 기본 AETR 이면 rcmap = {0,1,3,2}:
//   frame[0]=롤, frame[1]=피치, frame[2]=스로틀, frame[3]=요.
// 부팅 뒤 MSP_RX_MAP(64) 으로 실제 rcmap 을 읽어 쓰고, 못 읽으면 아래 기본값을 쓴다.
#define RCMAP_DEFAULT {0, 1, 3, 2}
// FC_CFG_POLL_MS 마다 rcmap 과 msp_override_channels 를 다시 읽는다 (CLI 로 바꾼 뒤 반영 확인용)
#define FC_CFG_POLL_MS 2000
// 마스크를 이 시간 안에 다시 읽지 못하면 접근 제어 출력을 막는다(mask_unknown). 진단 질의와 겹쳐 건너뛰는 경우를 감안해 넉넉히
#define FC_MASK_VALID_MS 10000

// INAV 논리 채널 순서 (rcData / MSP_RC / msp_override_channels 비트): 롤 0, 피치 1, 요 2, 스로틀 3.
// → 롤·피치·요만 덮어쓰는 마스크는 7, 스로틀 비트는 8. 기체 rcmap 과 무관하게 논리 비트다.
#define FC_RC_CH_ROLL 0
#define FC_RC_CH_PITCH 1
#define FC_RC_CH_YAW 2
#define FC_RC_CH_THROTTLE 3
#define OVERRIDE_MASK_THROTTLE_BIT (1u << FC_RC_CH_THROTTLE)
#define OVERRIDE_MASK_RECOMMENDED 7u   // 롤+피치+요. 보드 킬(ch7)까지 쓰려면 7+64 = 71
#define FC_RC_CH_ARM 4
#define FC_RC_SAFE_MAX_US 1100

// 제어권 스위치: FC MSP_RC의 CH6(인덱스 5)가 1000이면 AUTO, 2000이면 MANUAL.
// INAV의 MSP RC OVERRIDE 활성 범위와 같은 900~1700만 AUTO이며 그 밖과 응답 누락은 MANUAL이다.
#define MSP_CH_AUTHORITY 5
#define MSP_AUTH_AUTO_MIN_US 900
#define MSP_AUTH_AUTO_MAX_US 1700
#define MSP_AUTH_TIMEOUT_MS 250

// 킬(정지) 채널: 보드가 이 채널을 KILL_ON_US로 보내면 INAV FAILSAFE 모드(DROP)가 켜져 즉시 디스아밍된다.
// 20261010 FC 설정 기준: ch5=ARM, ch6=MSP RC OVERRIDE(900~1700, 스위치 올림), ch8=ANGLE(항상).
// 그래서 킬은 ch7(인덱스 6). INAV 설정 필요: aux 3 27 2 1700 2100 / set msp_override_channels = 71
// 71은 롤(1)+피치(2)+요(4)+킬 ch7(64)이며, 보드는 끌 수만 있고 아밍은 못 한다. 마스크가 7 이면 보드 킬은
// FC 에 반영되지 않는다(조종기 CH7 킬만 동작).
#define MSP_CH_KILL 6
#define KILL_ON_US 2000
#define KILL_OFF_US 1000

// FC 상태 읽기: 50Hz 주기 중 이 간격마다 상태(2Hz)·배터리·모터 출력(각 1Hz)을 번갈아 요청 (FC 처리 한도 때문)
#define FC_POLL_PERIOD 50
#define FC_STATUS_SAFE_TIMEOUT_MS 1000  // 모터 시험은 이보다 오래된 아밍 상태로 시작·지속하지 않음

// 높이: 20261010부터 MTF-01P는 FC UART4에 연결한다(INAV rangefinder·opflow = MSP).
// 보드에 MTF-01P 데이터가 직접 들어오지 않으면 FC의 거리센서 값(MSP_SONAR_ALTITUDE, cm)을 대신 쓴다.
// 50Hz 주기 중 이 간격마다 한 번 요청 → 5 = 10Hz (FLOW_STALE_MS 300 안에 응답 하나를 잃어도 유지되는 최소)
#define FC_ALT_POLL_EVERY 5
// MTF-01 수직 속도 경고: |vz| 가 이 값을 넘으면 경고만 기록한다. 보드는 스로틀(CH3)을 절대 건드리지 않는다.
#define VZ_WARN_CM_S 10.0f
#define VZ_WARN_REPEAT_MS 1000

// 접근 제어 (src/approach.h): 광류·자세를 FC에서 읽는 주기 (50Hz 주기 중 간격)
// -40: 자세 10Hz(칼만 가속도 입력·촬영 순간 기울기 보정), 광류 2Hz(비행 속도는 칼만 — 광류는 기록용). 예산 합은 그대로
#define FC_FLOW_POLL_EVERY 25   // MSP2_INAV_OPTICAL_FLOW 2Hz
#define FC_ATT_POLL_EVERY 5     // MSP_ATTITUDE 10Hz
// 부팅 시 접근 제어 모드: 0 off · 1 shadow(계산·로그만, FC에는 중립) · 2 on · 3 legacy(아래 4µs/cm 위치 제어)
// 20261011 사용자 요청: Mac 에서 누르지 않아도 자동 → 부팅부터 on. 실제로 FC 가 보드 값을 쓰는지는 조종기 CH6
// (FC MSP RC OVERRIDE 범위) 가 정한다 — AUTO 로 내리면 보드가 조정, MANUAL 로 올리면 조종기. 재부팅해도 on.
// `approach on|shadow|legacy|off`로 바꿀 수 있다 (재부팅하면 이 값으로 돌아옴).
#define APPROACH_MODE_DEFAULT 2
// 비행 속도 (20261011 사용자 요청: 태그로 되돌아가는 게 너무 느림 → 적당히 빠르게). 부팅 때 적용 (approach.h 기본값은
// PC 시험 기준이라 그대로). 실측 기체 응답(명령 → 기울기 0.6s 지연·78%)으로 시뮬레이션해 고른 값:
//   10cm 안 도달 중앙 5.2 → 4.3s, 느린 10% 7.1 → 5.0s, 반대편 넘침 중앙 5 → 8cm (큰 10% 12 → 15cm).
//   속도 25cm/s·감속 60cm 는 넘침 큰 10% 37cm, 속도 이득 Kv 2.0 은 넘침 2배라 쓰지 않음 (Kv 는 1.5 그대로)
#define AP_MAX_SPEED_CMS 20.0f      // 최대 접근 속도 (이전 10)
#define AP_SLOW_RADIUS_CM 50.0f     // 이 거리부터 비례 감속 (이전 25) — 빠를수록 일찍 감속
#define AP_PROFILE_ACCEL 30.0f      // 목표 속도 변화율 (cm/s², 이전 10)
#define AP_SEARCH_SPEED_CMS 12.0f   // 태그를 놓쳐 예측 위치로 되돌아갈 때 (이전 6)
#define AP_MAX_ACCEL_CMD 80.0f      // 가속도 명령 상한 (cm/s² ≈ 4.7°, 이전 50)
#define AP_OVERSPEED_CMS 24.0f      // 실측 속도가 이보다 크면 제동 (최대 속도 × 1.2)
#define APPROACH_REPORT_EVERY 10 // approach JSON 5Hz (50Hz 주기 기준)

// 착륙 지점 오프셋(cm) → 스틱 값(µs). 1500 중립에서 ±만큼 움직인다. (approach legacy 모드에서만 사용)
// INAV 쪽 비행 모드에 따라 의미가 다르다: ANGLE은 기울기, POSHOLD는 속도 명령.
#define CTRL_XY_US_PER_CM 4.0f  // 1cm 벗어나면 4µs
#define CTRL_XY_MAX_US 80       // 최대 ±80µs (전체 ±500의 16%)
#define CTRL_TARGET_STALE_MS 1000 // 이 시간 동안 새 판단이 없으면 중립

// 스로틀(고도) 제어: INAV ALTHOLD처럼 "스로틀 중앙 = 고도 유지" 모드에서만 안전하다.
// false면 스로틀은 항상 1500을 보낸다. INAV msp_override_channels에 스로틀 비트를 넣지 말 것.
#define CTRL_THROTTLE_ENABLED false
// FC로 보내는 SET_RAW_RC 프레임의 스로틀 슬롯 값. 보드는 스로틀을 제어하지 않으므로 항상 이 값을 보낸다.
// 1500(중간)을 보내면 FC의 msp_override_channels 마스크에 스로틀 비트(8)가 잘못 들어가 있을 때
// CH6=AUTO 전환 순간 스로틀이 절반으로 뛴다 (2026-10-10 엔지니어 보고). min_check(1100) 아래 값으로 고정한다.
// 조종기 값을 되돌려 주는 방식은 쓰지 않는다: 마스크에 스로틀이 있으면 MSP_RC 가 이미 덮어쓴 값을 돌려줘 되먹임된다.
// 이 값은 마스크가 올바르면(스로틀 비트 없음) FC 가 무시한다. 마스크에 스로틀이 있으면 접근 제어기는 fault "mask" 로
// 출력을 멈추고, 보드가 `msp` JSON 의 mask_ok=false 로 알린다.
#define MSP_THROTTLE_SAFE_US 1000
#define CTRL_Z_US_PER_CM 3.0f
#define CTRL_Z_MAX_US 60

// 미션 단계 (노션 20261001): 정렬 → 50cm 하강 → 3초 유지 → 100cm 상승 → 호버링
#define MISSION_LOW_CM 50.0f
#define MISSION_HIGH_CM 100.0f
#define MISSION_HOLD_MS 3000
#define MISSION_BAND_CM 5.0f    // 목표 고도 ± 이 범위면 도달로 본다

// 모터 시험 (`motortest`): FC가 아밍되지 않았을 때 MSP_SET_MOTOR로 모터를 직접 돌린다.
// 프로펠러를 반드시 뗀 상태에서만 쓴다. 시간이 지나면 보드가 스스로 정지 신호를 보낸다(BLE가 끊겨도).
#define MOTOR_TEST_MAX_PCT 15     // 출력 상한 (1000 + 15% = 1150)
#define MOTOR_TEST_MAX_MS 5000    // 한 번에 최대 5초
#define MOTOR_COUNT 4

// 시뮬레이션 모드: 브라우저(sim/index.html)가 `simh <cm>`로 가상 고도를 보내는 동안
// MTF-01P 높이 대신 그 값을 쓰고, 스틱 값을 USB로 20Hz `rc` 메시지로 내보낸다.
// 이 시간 동안 simh가 없으면 자동으로 실제 센서로 돌아간다.
#define SIM_TIMEOUT_MS 2000

// ---------------------------------------------------------------------------
// 마이크 INMP441 (I2S). VDD→3.3V, GND→GND, L/R→GND(왼쪽 채널)
// ---------------------------------------------------------------------------
#define MIC_SCK_PIN 4             // D3 ← INMP441 SCK(BCLK)
#define MIC_WS_PIN 5              // D4 ← INMP441 WS(LRCLK)
#define MIC_SD_PIN 6              // D5 ← INMP441 SD(데이터)
#define MIC_RATE 16000            // 음성 인식 표준 16kHz
#define MIC_GAIN 16               // 24비트 → 16비트 변환 시 이득 (×16 = +24dB). 소리가 작으면 올린다
#define MIC_REPORT_MS 1000        // 소리 크기 보고 주기
#define MIC_MAX_REC_MS 3000       // 한 번 녹음 최대 3초

// ---------------------------------------------------------------------------
// 음성 명령: 인식기(PC 한국어 인식 또는 보드 키워드 모델)가 알아들은 키워드를
// `voicecmd <키워드>`로 넘기면 아래 명령을 차례로 실행한다(세미콜론으로 구분).
// 안전: 모터 시험·아밍처럼 기체를 움직이는 명령은 넣지 않는다 (펌웨어가 motortest·voicecmd는 거부).
// ---------------------------------------------------------------------------
struct VoiceCommand {
  const char *keyword;   // 인식기가 보내는 이름
  const char *ko;        // 사람이 말할 말 (인식기 학습·안내용)
  const char *commands;  // 실행할 보드 명령
};

static const VoiceCommand VOICE_COMMANDS[] = {
  {"stop", "정지 / 멈춰", "motorstop;msp off"},  // 모터 시험 중지 + 보드 조종 해제(조종기로)
  {"resume", "다시 시작", "msp on"},
  {"status", "상태", "info"},
  {"snap", "사진", "snap"},
  {"mission_reset", "미션 처음부터", "mission reset"},
  {"cam_down", "카메라 아래", "cam down"},
  {"cam_front", "카메라 앞", "cam front"},
};

// ---------------------------------------------------------------------------
// 카메라 (XIAO ESP32S3 Sense 확장 모듈)
// ---------------------------------------------------------------------------
// VGA 그레이스케일. 더 빠르게 하려면 FRAMESIZE_HVGA(480x320), 더 멀리 보려면 FRAMESIZE_SVGA.
#define CAM_FRAMESIZE FRAMESIZE_VGA
// 수평 화각: OV2640 대각 66° 를 4:3 으로 환산한 55°. 20261003 실측(태그 304px ↔ MTF-01P 41cm)과 일치
#define CAM_HFOV_DEG 55.0f
// `exp fast`: 손으로 움직이거나 비행할 때 흐림을 줄이는 수동 노출(0~1200, 줄 단위)과 게인(0~30).
// 20261010 밤 실측: 자동 노출에서 손으로 움직이면 화면 전체가 번져 태그 인식이 20초 중 6/101 로 떨어졌다.
// 같은 실내 조명에서 자동 노출 ≈ 노출 150 (밝기 126). 노출 20·게인 8 이면 밝기 108 로 노출 시간이 약 1/7.5 → 흐림도 약 1/7.5
#define CAM_EXP_FAST_AEC 20
#define CAM_EXP_FAST_GAIN 8
// -50: 부팅 때부터 짧은 노출. -41 이후 재부팅마다 자동 노출로 돌아가(뷰어가 exp fast 를 안 보냄) 이동 중 태그가 번져
// 화면 가운데 근처에서만 잡혔다 (20261011 수동 비행: 다음 태그를 처음 본 곳이 화면 중심 0~6cm)
#define CAM_EXP_FAST_DEFAULT true

// 스냅샷 (`snap` / `snap small`). 보내기는 별도 태스크가 해서, 사진을 보내는 동안에도 태그 인식이 멈추지 않는다.
// 20261010 밤: BLE 로 640×480 사진(base64 약 40KB)이 약 12초 걸렸고 그동안 카메라 루프가 멈춰 인식이 끊겼다.
#define SNAP_JPEG_QUALITY 85
#define SNAP_SMALL_QUALITY 40       // `snap small`: 2×2 평균 320×240 — BLE 용 (사진 약 1/6)
#define SNAP_CHUNK 1000             // base64 한 줄 길이
#define SNAP_RESERVE_BYTES 2048     // 사진 조각을 넣을 때 출력 버퍼(8KB)에 남겨 둘 자리 → 상태 줄이 버려지지 않게

// BLE 출력 줄이기 (`ble lite on|off`). 실측 BLE 처리량 약 6KB/s 인데 보드 출력은 태그가 보이면 10KB/s 가 넘는다.
// 켜면 BLE 로만 아래 줄을 최소 간격으로 솎아 보낸다 (USB 는 그대로). 명령 응답·이벤트·사진은 그대로 보낸다.
#define BLE_LITE_DEFAULT true
#define BLE_LITE_APPROACH_MS 500    // 2Hz (fc_rp 포함 → INAV 값 2Hz)
#define BLE_LITE_FLOW_MS 1000
#define BLE_LITE_TAG_MS 1000
#define BLE_LITE_VISION_MS 1000
#define BLE_LITE_MSP_MS 2000
#define BLE_LITE_MIC_MS 5000

// 보드 자체 WiFi (AP). 이름은 BLE 이름(TagBoard-XXXX), 주소 192.168.4.1, TCP WIFI_TCP_PORT 로 USB 와 같은 줄·명령.
// 20261011: BLE 로는 사진이 약 13초에 한 장 → WiFi 로 인식 주기(약 3~5장/s)만큼 받는다.
// 비밀번호(WPA2, 8자 이상)를 바꾸려면 여기. 열린 AP 면 근처 누구나 보드에 명령(모터 시험·FC 질의)을 보낼 수 있다.
#define WIFI_ENABLED_DEFAULT true
#define WIFI_AP_PASS "tagboard2026"
#define WIFI_TCP_PORT 3333
#define WIFI_LITE_DEFAULT true  // WiFi 로도 상태 줄을 BLE_LITE_* 간격으로 솎는다 (`wifi lite on|off`)
// 무선 선택 (`radio wifi|ble|both`, NVS 저장, 바꾸면 재시작). 20261011 실측: WiFi+BLE 를 함께 켜면 내부 RAM 이
// 9KB 남고 최저 24B 까지 떨어졌다(카메라·태스크 포함). 내부 RAM 이 바닥나면 작은 할당이 PSRAM 으로 가서 스핀락 panic
// (-34 코어 덤프와 같은 경로) → 기본은 WiFi 만. 0 both, 1 wifi, 2 ble
#define RADIO_DEFAULT 1

// 태그 크기 자동 보정 (src/tag_size_cal.h): 실제 한 변 = 화면 한 변(px) × 거리센서 높이 / fx.
// 20261010 밤: ID 1 을 200mm 로 가정했는데 실제는 약 90~100mm 라 거리가 2배로 계산돼 출력이 막혔다.
// 태그가 바닥에 있고, 카메라가 아래를 보고, 기체가 거의 수평일 때만 표본을 모은다.
#define TAGCAL_ENABLED_DEFAULT true
#define TAGCAL_MIN_H_CM 25.0f          // 거리센서 높이 범위 (너무 가까우면 태그가 화면을 넘침)
#define TAGCAL_MAX_H_CM 300.0f
#define TAGCAL_MAX_ATT_DECI 60         // 기체 롤·피치 6.0° 이내
#define TAGCAL_MIN_MARGIN 40.0f        // AprilTag decision_margin 이상
#define TAGCAL_EDGE_PX 8.0f            // 네 꼭짓점이 화면 가장자리에서 이만큼 안쪽
#define TAGCAL_CAM_ABOVE_SENSOR_CM 0.0f  // 카메라 렌즈가 거리센서보다 위에 있는 거리 (기체에서 실측해 넣는다)
#define TAGCAL_MIN_N 8                 // 이만큼 모이고
#define TAGCAL_MAX_SPREAD 0.06f        // 중앙값 대비 퍼짐(MAD)이 6% 이하면 그 값을 쓴다

// ---------------------------------------------------------------------------
// 카메라 팬·틸트 (서보 2개, 50Hz PWM) — 20261010 요청: 운행 중 카메라 방향 전환
// 각도 규약 (기체 기준): 팬 0° = 앞, + = 오른쪽. 틸트 0° = 바로 아래(기존 장착 자세), + = 앞으로 들어 올림, 90° = 정면 수평.
// 기계 구조 가정: 아래 서보가 팬(수직축), 그 위 서보가 틸트. 계산은 src/gimbal_geom.h.
// 서보 전원: FC 5V (LED·BUZZER 또는 GPS 커넥터의 5V·GND). XIAO 3V3에 연결하지 않는다. GND는 공통.
// 서보가 없는 고정 장착이면 GIMBAL_ENABLED false → 항상 팬 0°·틸트 0°로 계산한다.
// ---------------------------------------------------------------------------
#define GIMBAL_ENABLED true
#define GIMBAL_PAN_PIN 7              // XIAO D8 → 팬 서보 신호
#define GIMBAL_TILT_PIN 8             // XIAO D9 → 틸트 서보 신호
#define GIMBAL_PAN_LEDC_CH 2          // LEDC 채널 2·3 = 타이머 1 (카메라 XCLK가 타이머 0·채널 0을 씀)
#define GIMBAL_TILT_LEDC_CH 3
#define GIMBAL_PWM_HZ 50
#define GIMBAL_PWM_BITS 14            // ESP32-S3 LEDC 최대 해상도 (20ms / 16384 ≈ 1.2µs)
#define GIMBAL_US_MIN 500             // 서보에 보낼 펄스 한계
#define GIMBAL_US_MAX 2500
// 장착 보정: `cam us <팬µs> <틸트µs>`로 실제 0° 위치(앞·바로 아래)의 펄스를 찾아 넣는다
#define GIMBAL_PAN_CENTER_US 1500     // 팬 0°
#define GIMBAL_TILT_DOWN_US 1500      // 틸트 0°
#define GIMBAL_PAN_US_PER_DEG 10.0f   // 표준 서보 약 10µs/°. 돌아가는 방향이 반대면 음수로
#define GIMBAL_TILT_US_PER_DEG 10.0f
#define GIMBAL_PAN_MIN_DEG -90.0f
#define GIMBAL_PAN_MAX_DEG 90.0f
#define GIMBAL_TILT_MIN_DEG -30.0f
#define GIMBAL_TILT_MAX_DEG 90.0f
#define GIMBAL_FRONT_TILT_DEG 60.0f   // `cam front`: 앞쪽 30° 아래를 봄 (멀리 있는 태그 찾기)
#define GIMBAL_SLEW_DEG_S 120.0f      // 최대 회전 속도 (급하게 돌면 영상이 번짐)
#define GIMBAL_UPDATE_HZ 50
#define GIMBAL_SETTLE_MS 250          // 멈춘 뒤 이 시간이 지나야 그 각도로 계산한 위치를 제어에 쓴다
#define GIMBAL_OBLIQUE_DEG 20.0f      // 틸트가 이보다 크면 태그 크기 대신 높이로 지면 교점을 구한다
#define GIMBAL_MIN_DOWN 0.17f         // 광선이 수평 아래 10°보다 얕으면 지면 교점을 쓰지 않음

// ---------------------------------------------------------------------------
// AprilTag
// ---------------------------------------------------------------------------
#define TAG_FAMILY_NAME "tag36h11"
#define TAG_NTHREADS 2
#define TAG_MIN_MARGIN 30.0f    // decision_margin이 이보다 낮으면 오인식 가능성으로 보고 버림
#define TAG_UNKNOWN_SIZE_MM 200 // 표에 없는 태그의 거리·오프셋 계산용 가정값

// ---------------------------------------------------------------------------
// decimate 자동 전환
// decimate가 클수록 빠르지만 작은(먼) 태그를 놓친다. 20261003 실측(VGA):
//   2.0 ≈ 640ms, 3.0 ≈ 150~290ms, 4.0 ≈ 100~160ms
// PC 시뮬레이션에서 인식 가능한 최소 크기는 축소 후 한 변 9~11px.
// 실제 흔들림·기울기를 감안해 그 2배인 20px 이상이 되도록 고른다.
// ---------------------------------------------------------------------------
#define DEC_AUTO_DEFAULT true
#define DEC_START 3.0f          // 부팅 직후 값
#define DEC_KEEP_SIDE_PX 20.0f  // 축소 후 한 변이 이보다 작아지면 decimate를 낮춘다
#define DEC_RAISE_SIDE_PX 24.0f // 올릴 때는 여유를 더 둔다 (경계에서 왕복 전환 방지)
#define DEC_LOST_FRAMES 2       // 태그를 이만큼 연속으로 놓치면 MTF-01P 높이로 크기를 예측
#define DEC_SWEEP_EVERY 8       // 놓친 상태에서 이 프레임마다 한 번은 2.0으로 넓게 찾는다 (2.0 은 한 장 약 0.54초 → 드물게)
#define DEC_RECENT_MS 3000     // 미션 태그를 이 시간 안에 봤으면 넓은 탐색 없이 크기 예측으로만 찾는다 (20261011)
// 동적 ROI (-40): 태그를 봤으면 다음 프레임은 그 주변(한 변의 ROI_SIDE_MULT 배, 최소 ROI_MIN_HALF_PX)만 찾는다
#define ROI_ENABLED true
#define ROI_SIDE_MULT 2.5f      // ROI 반폭 = 태그 한 변 × 2.5 (프레임 사이 이동 여유)
#define ROI_MIN_HALF_PX 90
#define ROI_MIN_SIDE_PX 24.0f   // ROI 안 축소 뒤에도 태그 한 변이 이 이상

// 착륙 지점이 화면 중앙에서 이 거리(cm) 안이면 "착륙 지점 바로 위"로 판단
#define CENTER_TOL_CM 3.0f

// 카메라 영상 → 기체 축 매핑 (카메라가 아래를 보고, 화면 위쪽이 기체 앞쪽이라고 가정)
// 장착 방향을 실측하기 전까지 axes_verified=false 로 출력한다.
#define CAM_ROTATE_DEG 0        // 0 / 90 / 180 / 270
#define AXES_VERIFIED false

// ---------------------------------------------------------------------------
// 태그 표: ID → 크기·착륙 지점·의미
// AprilTag 자체에는 ID 숫자만 들어 있다. 의미와 크기는 이 표에서 정한다.
//
// size_mm  : 검은 정사각형 한 변 (흰 여백 제외)
// land_x/y : 이 태그 중심에서 착륙 지점까지 거리(mm). 인쇄물을 글자 읽듯 똑바로 놓고 봤을 때
//            오른쪽 +x, 위쪽 +y. 태그 중심이 곧 착륙 지점이면 0, 0.
//
// 중첩 태그: 큰 태그는 멀리서, 작은 태그는 가까이서 보인다. 두 태그의 land_x/y를
// 같은 물리적 지점을 가리키게 맞추면, 어느 태그가 보이든 같은 착륙 지점으로 안내한다.
// 시리얼 `size <id> <mm>`, `land <id> <x> <y>`로 재빌드 없이 바꿀 수 있다(NVS 저장).
// ---------------------------------------------------------------------------
struct TagInfo {
  int id;
  uint16_t size_mm;
  int16_t land_x_mm;
  int16_t land_y_mm;
  const char *code;
  const char *ko;
};

#define MISSION_KO "미션 목표 지점: 착륙 지점 위로 이동 → 바닥 기준 50cm에서 3초 유지 → 100cm로 복귀 후 호버링"

static const TagInfo TAG_TABLE[] = {
  // 노션 계획서(20261001): 첫 시험 태그 ID 0, 한 변 100mm → 저고도용 작은 태그
  {0, 100, 0, 0, "MISSION_TARGET", MISSION_KO},
  // 20261003: 제작한 200mm 패널 ID 10 → 고고도용 큰 태그. 작은 태그 옆에 두면 land_x/y를 설정
  {10, 200, 0, 0, "MISSION_TARGET", MISSION_KO},
  // 20261010 저녁 지상 시험에 쓴 종이 출력 태그. 크기 실측 전(200mm 가정) → `size 1 <mm>` 로 보정(NVS 저장)
  {1, 200, 0, 0, "MISSION_TARGET", MISSION_KO},
  // 20261011 태그 투어(-41): ID 가 행동을 정한다 (src/tour.h TOUR_ACTIONS). 0·1 전진, 2 왼쪽, 3 오른쪽, 4 되돌기, 5 끝.
  // 크기는 ID 1 실측(약 88mm)과 같게 인쇄한다고 가정 → 다르면 `size <id> <mm>` 또는 자동 보정(tagcal)
  {2, 88, 0, 0, "MISSION_TARGET", MISSION_KO},
  {3, 88, 0, 0, "MISSION_TARGET", MISSION_KO},
  {4, 88, 0, 0, "MISSION_TARGET", MISSION_KO},
  {5, 88, 0, 0, "MISSION_TARGET", MISSION_KO},
};

// ---------------------------------------------------------------------------
// 태그 투어 (-41, src/tour.h): 태그 위 → 기수를 화살표 방향으로 → 머무름 → 화살표 방향으로 다음 태그 → 반복
// `tour on|off` (approach on + CH6 AUTO 일 때 동작). -44 부터 기본 켜짐 — 재부팅 뒤 꺼져 있어 AUTO 가 머무르기만 했다
// ---------------------------------------------------------------------------
#define TOUR_DEFAULT_ON true    // -44: AUTO = 투어 (재부팅해도 켜져 있게. 끄려면 tour off)
#define TOUR_FULL_EVERY 3          // 다음 태그 찾는 동안 3프레임에 1번은 ROI 없이 전체 화면 인식
#define TOUR_REPORT_EVERY 25       // tour JSON 2Hz (50Hz 주기 기준)
#define TOUR_YAW_SLEW_US_S 300.0f  // 요 스틱 변화율 한계
// FC 요 곡선을 아직 못 읽었을 때 (INAV 기본값): yaw_rate 20(=200°/s), rc_yaw_expo 20, yaw_deadband 5
#define TOUR_FC_YAW_RATE_DPS 200.0f
#define TOUR_FC_YAW_EXPO 20.0f
#define TOUR_FC_YAW_DEADBAND 5.0f

// ---------------------------------------------------------------------------
// 고도 유지·착륙 (-43, src/alt_hold.h): FC msp_override_channels 에 스로틀(비트 8)이 있을 때만 FC 가 보드 스로틀을 쓴다.
// 그때 보드는 AUTO 에서 1000 이 아니라 "AUTO 순간 조종자 스로틀 + 거리센서 보정"을 보낸다. 착지 뒤 1000 →
// INAV nav_disarm_on_landing(모든 모드에서 동작, INAV 9.1 navigation.c)이 시동을 끈다.
// ---------------------------------------------------------------------------
#define ALT_HOLD_ENABLED true      // false 면 스로틀이 든 FC 마스크를 거부 (예전 동작: 접근 제어 fault mask)
#define ALT_LEVEL_BELOW_CM 15.0f   // 착륙 중 이 높이 아래에서는 롤·피치·요를 수평(중립)으로 — 기울어 닿지 않게
#define ALT_LAND_KF_LOST_MS 60000  // 착륙 중 태그가 안 보여도 칼만 예측으로 수평 위치 유지 (수직 하강)
#define ALT_LAND_SPEED_CMS 5.0f    // 착륙 중 수평 보정 속도 상한
#define ALT_REPORT_EVERY 25        // alt JSON 2Hz

static const TagInfo TAG_UNASSIGNED = {
  -1, TAG_UNKNOWN_SIZE_MM, 0, 0, "UNASSIGNED", "의미 미지정 태그: config.h의 TAG_TABLE에 ID를 등록해야 함"
};
