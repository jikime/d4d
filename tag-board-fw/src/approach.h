#pragma once
// AprilTag 저속 접근 제어 (하드웨어 의존 없음 — test/approach_host_test.cpp 에서 PC로 시험한다)
//
//   위치 오차(cm, 기체 기준) ─ 거리 프로파일 ─▶ 목표 속도(≤10cm/s) ─ 가속도 제한 ─▶
//   속도 P(+가속도 FF) ─▶ 가속도 → 기울기 → INAV 곡선 역변환(µs) ─ 변화율·크기 제한 ─▶ MSP_SET_RAW_RC 오프셋
//   (out_tilt 0 이면 예전처럼 속도 P(I) → µs 비례)
//
// 전제: INAV ANGLE 모드(스틱 = 기울기 = 가속도). FC 자세 PID는 바꾸지 않는다.
// 속도는 FC 광류(MSP2_INAV_OPTICAL_FLOW)로 추정한다. 광류가 없으면 접근하지 않는다.
// 모든 게인·부호·한계는 ApproachParams 로 두고, 검증 전 값은 "후보값"이다.
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "tag_kf.h"

struct ApproachParams {
  // ---- 속도 프로파일 (요구 3·4) ----
  float maxSpeedCmS = 10.0f;     // 수평 속도 상한
  float slowRadiusCm = 25.0f;    // 이 거리부터 비례 감속
  float stopRadiusCm = 5.0f;     // 이 안에서는 목표 속도 0
  float maxAccelCmS2 = 10.0f;    // 목표 속도 변화율 상한 (0.10 m/s², 시험용 후보값)

  // ---- 속도 루프 (요구 5) — 후보값, 지상·저속 비행에서 조정 ----
  float velKpUsPerCmS = 2.0f;    // 속도 오차 1cm/s 당 µs
  float velKiUsPerCm = 0.0f;     // 0 = P만. 필요할 때만 켠다
  float velIMaxUs = 20.0f;       // 적분 항 한계
  float cmdMaxUs = 80.0f;        // 롤·피치 오프셋 한계 (±)
  float cmdSlewUsPerS = 100.0f;  // 명령 변화율 한계 (요구 6)
  float rollSign = 1.0f;         // + 오프셋 = 오른쪽 이동으로 검증되면 1, 반대면 -1
  float pitchSign = 1.0f;        // + 오프셋 = 앞 이동으로 검증되면 1, 반대면 -1

  // ---- 도착 판정 (요구 10) ----
  float arriveTolCm = 5.0f;
  float arriveVelCmS = 3.0f;
  uint32_t arriveHoldMs = 2000;

  // ---- 안전 (요구 9) ----
  uint32_t tagStaleMs = 600;     // 3~4fps 카메라 → 프레임 2장 분량
  uint32_t flowStaleMs = 300;
  uint8_t flowMinQuality = 40;   // 오늘 50cm 실측 56~62. 118cm에서 34
  uint32_t heightStaleMs = 300;
  float heightMinCm = 20.0f;
  float heightMaxCm = 300.0f;     // 위아래 이동·착륙 시험 범위 (20cm 아래는 수평 명령 중립). 이전 150
  float maxVelCmS = 60.0f;       // 이보다 빠르면 추정 이상 또는 외란 → 중지

  // ---- 추정 파라미터 (main.cpp 가 입력을 만들 때 사용) — 전부 실측 전 ----
  float flowSwapXY = 0;          // 1 이면 FC 유량 X↔Y 를 바꿔 앞/오른쪽에 대응
  float flowFwdSign = 1.0f;      // 앞으로 이동할 때 추정 속도가 음수로 나오면 -1
  float flowRightSign = 1.0f;
  // 1: 촬영 순간 FC 자세로 카메라 지면점 이동을 보정 (-40 기본). 기체가 오른쪽으로 θ 기울면 아래를 보던 카메라가 왼쪽을
  // 봐서 태그가 h·tanθ 만큼 오른쪽으로 보인다 → 뺀다. ArduPilot 정밀착륙도 촬영 시점 자세로 시선을 보정한다
  float attComp = 1;
  float attRollSign = 1.0f;      // 기체를 오른쪽으로 기울였을 때 보정 뒤 오차가 더 흔들리면 -1
  float attPitchSign = 1.0f;
  float camOffsetRightCm = 0;    // 카메라 중심 → 기체 무게중심 (cm). 카메라가 왼쪽에 있으면 +
  float camOffsetFwdCm = 0;      // 카메라가 뒤에 있으면 +

  // ---- 수동 스로틀 유지형 수평 추적 시험 (2026-10-10 엔지니어 요청) ----
  float deadbandNeutral = 1;     // 1: 정지 반경 안·도착 뒤에는 롤·피치를 중립으로 (속도 유지 제동 안 함). 0: 속도 0 유지
  float lostNeutral = 1;         // 1: 태그 유실·신뢰도 저하 시 즉시 중립으로 복귀. 0: 광류로 속도 0 제동(HOLD)
  float overspeedCmS = 12.0f;    // 실측 수평 속도가 이보다 크면 그 스텝의 목표 속도를 0 으로 (0.10 m/s 상한 보강)
  float handoverSlewUsPerS = 100.0f;  // AUTO 진입 시 조종기 마지막 스틱값 → 1500 복귀 속도 (main.cpp 가 사용)

  // ---- 지상 시험 (2026-10-10 저녁 요청): 기체는 작업대에 두고 태그를 손으로 움직여 FC 값 변화를 확인 ----
  // 비행 기본값은 둘 다 0. `approach bench` 가 1 로, `approach flight` 가 0 으로 되돌린다.
  // 0: FC 광류 속도. 1: 카메라로 잰 태그 상대 속도(위치 미분 + 평활, 지상 시험). 2: 칼만(명령 기울기 + 카메라, 비행 기본 -39~)
  float velSrc = 2;
  float heightSrc = 0;           // 0: 거리센서만. 1: 거리센서가 무효면 태그 크기로 잰 카메라~태그 거리를 높이로 사용
  float tagVelAlpha = 0.5f;      // 카메라 속도 지수평활 계수 (1 = 평활 없음). 3~6fps 미분 잡음 억제

  // ---- 기울기 기반 출력 (2026-10-10 밤): INAV 스틱 곡선을 거꾸로 계산 ----
  // INAV 9.1 fc_core.c getAxisRcCommand: rcCommand = rcLookup(applyDeadbandRescaled(µs−1500, deadband), rc_expo)
  // pid.c pidRcCommandToAngle: 목표 기울기 = rcCommand/500 × max_angle_inclination
  // µs 비례 출력(out_tilt 0)은 이 FC(rc_expo 80)에서 80µs 가 약 1.6° 에 불과해 반응이 약했다.
  float outTilt = 1;             // 1: 속도 오차 → 가속도 → 기울기 → INAV 곡선 역변환 → µs. 0: 기존 µs 비례(vel_kp)
  float velKpAccel = 1.5f;       // 속도 오차 1cm/s 당 가속도 명령 (1/s). 현실 모델 비교로 1.5 선택(최대 10.8cm/s, 지나침 0)
  float accelFf = 0.0f;          // 목표 속도 변화율 피드포워드 비율. 자세·광류 지연 때문에 1 이면 속도가 13.5cm/s 까지 넘쳐 기본 0
  float maxAccelCmdCmS2 = 50.0f; // 가속도 명령 상한 (50cm/s² ≈ 2.9°)
  float rcExpo = 80;             // FC rc_expo (FC 에서 읽으면 그 값을 쓴다)
  float rcDeadbandUs = 2;        // FC deadband (FC 에서 읽으면 그 값을 쓴다)
  float maxAngleDeg = 45;        // FC max_angle_inclination_rll/pit ÷ 10
  float tiltCmdMaxUs = 200;      // 기울기 모드 최종 µs 한계 (±)
  float tiltSlewUsPerS = 400;    // 기울기 모드 명령 변화율 한계

  // ---- vel_src 2: 칼만 추정 (2026-10-11, 광류 없이 카메라 + 명령 기울기) — tag_kf.h ----
  // 비행 중 광류 품질이 22~38 로 기준(40) 미만 → 보드가 계속 중립 → AUTO 에서 기체가 흘렀다. 그래서 속도를
  // "명령한 기울기로 예측 + 카메라 태그 위치로 보정"하는 칼만 필터로 구한다. 태그가 화면 밖으로 나가도 kf_lost_ms 동안
  // 예측 위치로 되돌아간다(kf_search_speed 이하). 값은 PC 시뮬레이션(3~5fps, 지연 0.2~0.5s, 바람)으로 정함.
  float kfTauS = 0.33f;          // 명령 기울기 → 실제 기울기 시정수 (INAV ANGLE)
  float kfQa = 25.0f;            // 모르는 가속도 잡음 (cm/s²) — 보드가 조종할 때
  float kfQaManual = 150.0f;     // 조종기가 조종할 때 (기울기를 모름)
  float kfQb = 3.0f;             // 외란 가속도 b 의 변화 (cm/s² /√s)
  float kfMeasCm = 1.5f;         // 태그 위치 측정 잡음 (cm)
  float kfMeasPerM = 1.0f;       // 높이 1m 당 추가 잡음 (cm)
  float kfLatencyMs = 40;        // 촬영 → 프레임 수신 사이 추가 지연 (처리 지연은 따로 잰다)
  float kfGate = 5.0f;           // 혁신이 이 σ 배수를 넘으면 그 측정을 버린다
  uint32_t kfLostMs = 3000;      // 태그를 놓친 뒤 예측으로 되돌아가는 최대 시간
  float kfSearchSpeedCmS = 6.0f; // 예측으로 되돌아갈 때 속도 상한
  float kfBiasFf = 1.0f;         // 1: 추정한 외란 가속도를 명령에서 뺀다 (바람·트림 상쇄)
  // 20261011 비행 실측: 명령 → FC 실제 자세가 약 0.6s 늦고 크기는 78% → 명령으로 만든 모델(τ 0.33s, 100%)과 달라
  // 태그 주변을 ±30cm 로 흔들렸다. 그래서 (1) FC 가 잰 실제 자세를 가속도 입력으로 쓰고, (2) 명령이 실제로 먹는 시점
  // (kf_pred_s 뒤)의 위치·속도를 예측해 그 값으로 명령을 계산한다(예측기 — 지연 보상).
  float kfAttInput = 1;          // 1: FC 실제 자세(MSP_ATTITUDE)를 가속도 입력으로 (없거나 오래되면 명령 모델)
  float kfPredS = 0.5f;          // 명령이 실제 기울기가 되기까지 걸리는 시간만큼 앞을 내다보고 명령 (0 = 끔)
  // 공기 저항 1/s (-41). 태그 사이를 예측만으로 갈 때 필요. 소형 쿼드 로터 항력은 보통 0.2~0.5 → 중간값, 실측 전
  float kfDrag = 0.3f;
};

// ---- INAV 스틱 곡선 (fc/rc_curves.c, common/maths.c) ----
// rcLookup: 변위 d(0~500) → rcCommand. t = d/100, (2500 + expo·(t²−25))·t/25
inline float inavRcLookup(float d, float expo) {
  float t = d / 100.0f;
  return (2500.0f + expo * (t * t - 25.0f)) * t / 25.0f;
}
// 역함수 (expo < 100 이면 단조 증가 → 이분법)
inline float inavRcLookupInverse(float r, float expo) {
  if (r <= 0) return 0;
  if (r >= 500) return 500;
  float lo = 0, hi = 500;
  for (int i = 0; i < 30; i++) {
    float mid = 0.5f * (lo + hi);
    if (inavRcLookup(mid, expo) < r) lo = mid;
    else hi = mid;
  }
  return 0.5f * (lo + hi);
}
// 원하는 기울기(°) → 1500 기준 스틱 오프셋(µs). applyDeadbandRescaled 의 역: raw = db + d·(500−db)/500
inline float tiltToStickUs(float tiltDeg, float expo, float deadbandUs, float maxAngleDeg) {
  if (maxAngleDeg <= 0) return 0;
  float r = fabsf(tiltDeg) / maxAngleDeg * 500.0f;
  float d = inavRcLookupInverse(r, expo);
  if (d <= 0) return 0;
  float raw = deadbandUs + d * (500.0f - deadbandUs) / 500.0f;
  return tiltDeg < 0 ? -raw : raw;
}
// 스틱 오프셋(µs) → INAV 가 만드는 목표 기울기(°) (시뮬레이션·로그용 정방향)
inline float stickUsToTilt(float us, float expo, float deadbandUs, float maxAngleDeg) {
  float a = fabsf(us);
  if (a < deadbandUs) return 0;
  float d = (a - deadbandUs) * 500.0f / (500.0f - deadbandUs);
  if (d > 500) d = 500;
  float tilt = inavRcLookup(d, expo) / 500.0f * maxAngleDeg;
  return us < 0 ? -tilt : tilt;
}

struct ApproachInput {
  uint32_t nowMs = 0;
  bool enabled = false;          // approach on
  bool authorityAuto = false;    // CH6 = AUTO (신선한 응답)
  bool kill = false;

  bool tagValid = false;         // 이번/최근 프레임에 미션 태그가 보였고 기하 계산이 됐다
  uint32_t tagAtMs = 0;
  float errRightCm = 0;          // 목표 지점 위치 (기체 기준). + = 목표가 오른쪽
  float errFwdCm = 0;            // + = 목표가 앞

  bool velValid = false;         // 광류 기반 속도 추정 가능
  uint32_t velAtMs = 0;
  float velRightCmS = 0;         // 기체 속도. + = 오른쪽으로 이동 중
  float velFwdCmS = 0;           // + = 앞으로 이동 중
  uint8_t flowQuality = 0;
  bool velFromTag = false;       // 속도가 카메라 미분 → 광류 품질 대신 태그 신선도(tagStaleMs)로 판단
  int16_t fcRcExpo = -1;         // FC 에서 읽은 rc_expo (-1 = 모름 → 파라미터 값)
  int16_t fcDeadbandUs = -1;     // FC 에서 읽은 deadband

  bool heightValid = false;
  uint32_t heightAtMs = 0;
  float heightCm = 0;

  // FC msp_override_channels 확인 결과. 스로틀 비트가 들어 있거나 아직 못 읽었으면 출력하지 않는다.
  bool maskKnown = true;
  bool maskOk = true;

  // vel_src 2 (칼만): 태그 사진을 찍은 시각(0 = 모름 → tagAtMs), FC 가 보드 값을 실제로 쓰는가(approach on + CH6 AUTO)
  uint32_t tagCaptureMs = 0;
  bool applied = true;
  // AUTO 진입 직후 FC 가 함께 받는 핸드오버 기준점(조종기 마지막 스틱 → 1500 으로 천천히). 1500 기준 오프셋 µs
  float baseRollUs = 0, basePitchUs = 0;
  // 조종기 수동(MANUAL) 중 FC 가 쓰는 조종자 스틱 (MSP_RC, 1500 기준 오프셋 µs). 알면 수동 중에도 기울기를 알아
  // 외란(수평 보정 오차·바람)을 미리 배운다 → AUTO 로 넘어가는 순간부터 정확히 예측
  bool pilotValid = false;
  float pilotRollUs = 0, pilotPitchUs = 0;
  // FC 가 잰 실제 자세 (°). 부호는 스틱과 같다(롤 + = 오른쪽으로 기움, 피치 + = 앞으로 숙임 — 20261011 비행으로 확인)
  bool attValid = false;
  uint32_t attAtMs = 0;
  float attRollDeg = 0, attPitchDeg = 0;
  // 투어(tour.h): 목표점 = 태그 + 이 오프셋 (기체 기준 cm). 이동 속도 상한·칼만 예측 허용 시간 (0 = 파라미터 값)
  float goalRightCm = 0, goalFwdCm = 0;
  float speedCapCmS = 0;
  uint32_t kfLostMs = 0;
};

enum class ApproachState : uint8_t { Idle, Approach, Arriving, Arrived, Hold, Fault };

inline const char *approachStateName(ApproachState s) {
  switch (s) {
    case ApproachState::Idle: return "IDLE";
    case ApproachState::Approach: return "APPROACH";
    case ApproachState::Arriving: return "ARRIVING";
    case ApproachState::Arrived: return "ARRIVED";
    case ApproachState::Hold: return "HOLD";
    default: return "FAULT";
  }
}

struct ApproachOutput {
  ApproachState state = ApproachState::Idle;
  const char *fault = nullptr;   // FAULT·HOLD 사유 (nullptr = 없음)
  float distCm = 0;              // 목표까지 수평 거리 (태그 유효할 때)
  bool distValid = false;        // distCm 을 이번 step 에 계산했다
  float targetSpeedCmS = 0;      // 프로파일이 정한 속도 크기 (가속도 제한 전)
  float vTargetRightCmS = 0;     // 가속도 제한 후 목표 속도
  float vTargetFwdCmS = 0;
  float rollUs = 0;              // 1500 기준 오프셋
  float pitchUs = 0;
  uint32_t arriveHeldMs = 0;
  bool neutral = false;          // deadband·도착·유실로 명령을 중립으로 수렴시키는 중
  bool overspeed = false;        // 실측 속도가 overspeedCmS 초과 → 목표 0 으로 제동 중
  float accelRightCmS2 = 0;      // 기울기 모드 가속도 명령
  float accelFwdCmS2 = 0;
  float tiltRightDeg = 0;        // 그 가속도에 필요한 기울기 (변화율 제한 전)
  float tiltFwdDeg = 0;
  // vel_src 2: 추정값 (로그·화면용)
  bool kfOn = false;
  bool predicting = false;       // 태그를 놓쳐 예측 위치로 되돌아가는 중
  float kfE[2] = {0, 0}, kfV[2] = {0, 0}, kfB[2] = {0, 0};  // [오른쪽, 앞]
  float kfLatMs = 0;
};

class ApproachController {
 public:
  ApproachParams p;

  // 거리 → 목표 속도 크기. 25cm 이상 10cm/s, 5~25cm 는 거리에 비례(10 × d/25), 5cm 이하 0.
  // 정지 반경에서 0 으로 수렴하는 램프(d−5 비례)를 쓰면 경계 바깥에서 멈춰 버려 영원히 들어오지 못한다.
  // 거리 비례면 5cm 경계에서 2cm/s 로 진입한 뒤 0 이 되고, 가속도 제한이 그 단차를 부드럽게 만든다.
  float targetSpeed(float distCm, float vmax = -1) const {
    if (vmax < 0) vmax = p.maxSpeedCmS;
    if (distCm >= p.slowRadiusCm) return vmax;
    if (distCm <= p.stopRadiusCm) return 0.0f;
    if (p.slowRadiusCm <= 1e-3f) return 0.0f;
    return vmax * distCm / p.slowRadiusCm;
  }

  void reset() {
    vtR_ = vtF_ = 0;
    cmdR_ = cmdF_ = 0;
    iR_ = iF_ = 0;
    heldMs_ = 0;
    arrived_ = false;
    kfR_.reset();
    kfF_.reset();
    kfTiltR_ = kfTiltF_ = 0;
    kfLastTagAtMs_ = kfLastUpdMs_ = 0;
    kfRebase_ = false;
  }

  ApproachOutput step(const ApproachInput &inRaw, float dtS) {
    ApproachOutput o;
    if (dtS <= 0) dtS = 0.05f;
    if (dtS > 0.5f) dtS = 0.5f;
    ApproachInput in = inRaw;
    bool predicting = false;
    if (p.velSrc == 2) kfStep(in, dtS, o, predicting);  // 위치·속도를 칼만 추정값으로 (태그를 놓쳤으면 예측 위치)
    if (in.tagValid) {  // 투어: 태그 옆 목표점 (기본 0 = 태그 중심)
      in.errRightCm += in.goalRightCm;
      in.errFwdCm += in.goalFwdCm;
    }

    bool tagFresh = in.tagValid && (uint32_t)(in.nowMs - in.tagAtMs) <= p.tagStaleMs;
    bool heightOk = in.heightValid && (uint32_t)(in.nowMs - in.heightAtMs) <= p.heightStaleMs &&
                    in.heightCm >= p.heightMinCm && in.heightCm <= p.heightMaxCm;
    uint32_t velStaleMs = in.velFromTag ? p.tagStaleMs : p.flowStaleMs;  // 카메라는 3~6fps
    bool velFresh = in.velValid && (uint32_t)(in.nowMs - in.velAtMs) <= velStaleMs &&
                    (in.velFromTag || in.flowQuality >= p.flowMinQuality);
    float vMag = hypotf(in.velRightCmS, in.velFwdCmS);

    const char *fault = nullptr;
    if (!in.enabled) fault = "disabled";
    else if (!in.authorityAuto) fault = "manual_authority";
    else if (in.kill) fault = "kill";
    else if (!in.maskKnown) fault = "mask_unknown";
    else if (!in.maskOk) fault = "mask";
    else if (!heightOk) fault = "height";
    else if (!velFresh) fault = "flow";
    // 너무 빠름: 속도를 믿을 수 없으면 멈춘다. 칼만이 태그를 보며 낸 속도(예측 아님)는 실제로 밀리는 것이므로 멈추지
    // 않고 제동한다 (overspeed). -52 이륙: 떠오르며 뒤로 60~77cm/s 밀렸는데 출력이 0 이 돼 85cm 더 밀림
    else if (vMag > p.maxVelCmS && (p.velSrc != 2 || predicting)) fault = "velocity";

    if (fault) {
      // 속도 추정을 믿을 수 없거나 권한이 없다 → 목표 0, 명령은 변화율 한계로 중립 복귀
      o.state = in.enabled ? ApproachState::Fault : ApproachState::Idle;
      o.fault = fault;
      vtR_ = vtF_ = 0;
      iR_ = iF_ = 0;
      heldMs_ = 0;
      arrived_ = false;
      slewTo(cmdR_, 0.0f, dtS);
      slewTo(cmdF_, 0.0f, dtS);
      fill(o, in);
      return o;
    }

    float desR = 0, desF = 0;
    if (tagFresh) {
      o.distCm = hypotf(in.errRightCm, in.errFwdCm);
      o.distValid = true;
      // 투어 이동(speed_cap)은 평소 상한(max_speed)보다 빠를 수 있다 (-50: 조종자 25~40cm/s)
      o.targetSpeedCmS = targetSpeed(o.distCm, in.speedCapCmS > 0 ? in.speedCapCmS : p.maxSpeedCmS);
      float predCap = in.speedCapCmS > 0 ? in.speedCapCmS : p.kfSearchSpeedCmS;
      if (predicting && o.targetSpeedCmS > predCap) o.targetSpeedCmS = predCap;
      if (o.distCm > 1e-3f) {
        desR = in.errRightCm / o.distCm * o.targetSpeedCmS;
        desF = in.errFwdCm / o.distCm * o.targetSpeedCmS;
      }
      // 도착 판정: 오차·속도가 허용 범위에 arriveHoldMs 동안 머물러야 한다 (예측만으로는 도착으로 보지 않는다)
      if (!predicting && o.distCm <= p.arriveTolCm && vMag <= p.arriveVelCmS) {
        heldMs_ += (uint32_t)(dtS * 1000.0f);
        if (heldMs_ >= p.arriveHoldMs) arrived_ = true;
      } else {
        heldMs_ = 0;
        if (o.distCm > p.arriveTolCm) arrived_ = false;  // 밀려났으면 다시 접근
      }
      if (arrived_) {
        desR = desF = 0;
        o.state = ApproachState::Arrived;
      } else {
        o.state = o.distCm < p.slowRadiusCm ? ApproachState::Arriving : ApproachState::Approach;
      }
    } else {
      // 태그를 놓침 → lostNeutral: 즉시 중립 복귀(조종자가 넘겨받음) / 아니면 광류로 속도 0 제동
      o.state = ApproachState::Hold;
      o.fault = "tag_lost";
      heldMs_ = 0;
      if (p.lostNeutral != 0) {
        neutralStep(o, dtS);
        fill(o, in);
        return o;
      }
    }

    // 정지 반경 안·도착 뒤 → deadbandNeutral 이면 롤·피치 중립 (수평 이동 멈춤, 고도는 조종자)
    if (tagFresh && p.deadbandNeutral != 0 && p.velSrc != 2 && (arrived_ || o.distCm <= p.stopRadiusCm)) {
      neutralStep(o, dtS);
      fill(o, in);
      return o;
    }

    // 실측 속도가 상한을 넘으면 이 스텝의 목표를 0 으로 → 속도 루프가 제동한다
    if (vMag > fmaxf(p.overspeedCmS, in.speedCapCmS + 5.0f)) {
      desR = desF = 0;
      o.overspeed = true;
    }

    // 가속도 제한 (벡터 크기 기준)
    float dR = desR - vtR_, dF = desF - vtF_;
    float dMag = hypotf(dR, dF);
    float dMax = p.maxAccelCmS2 * dtS;
    if (dMag > dMax && dMag > 1e-6f) {
      dR *= dMax / dMag;
      dF *= dMax / dMag;
    }
    vtR_ += dR;
    vtF_ += dF;

    // 속도 루프 (P + 선택적 I)
    float eR = vtR_ - in.velRightCmS, eF = vtF_ - in.velFwdCmS;
    if (p.velKiUsPerCm > 0) {
      iR_ = clampf(iR_ + p.velKiUsPerCm * eR * dtS, -p.velIMaxUs, p.velIMaxUs);
      iF_ = clampf(iF_ + p.velKiUsPerCm * eF * dtS, -p.velIMaxUs, p.velIMaxUs);
    } else {
      iR_ = iF_ = 0;
    }
    float rawR, rawF;
    if (p.outTilt != 0) {
      // 가속도 = 속도 P + 목표 속도 변화율(피드포워드) → 기울기 = atan(a/g) → INAV 곡선 역변환
      float expo = in.fcRcExpo >= 0 ? (float)in.fcRcExpo : p.rcExpo;
      float db = in.fcDeadbandUs >= 0 ? (float)in.fcDeadbandUs : p.rcDeadbandUs;
      float bR = (p.velSrc == 2 && kfR_.init) ? p.kfBiasFf * kfR_.b : 0.0f;  // 추정 외란 상쇄
      float bF = (p.velSrc == 2 && kfF_.init) ? p.kfBiasFf * kfF_.b : 0.0f;
      float aR = clampf(p.velKpAccel * eR + p.accelFf * dR / dtS - bR, -p.maxAccelCmdCmS2, p.maxAccelCmdCmS2);
      float aF = clampf(p.velKpAccel * eF + p.accelFf * dF / dtS - bF, -p.maxAccelCmdCmS2, p.maxAccelCmdCmS2);
      o.accelRightCmS2 = aR;
      o.accelFwdCmS2 = aF;
      o.tiltRightDeg = atanf(aR / 981.0f) * 57.2957795f;
      o.tiltFwdDeg = atanf(aF / 981.0f) * 57.2957795f;
      rawR = clampf(tiltToStickUs(o.tiltRightDeg, expo, db, p.maxAngleDeg), -p.tiltCmdMaxUs, p.tiltCmdMaxUs);
      rawF = clampf(tiltToStickUs(o.tiltFwdDeg, expo, db, p.maxAngleDeg), -p.tiltCmdMaxUs, p.tiltCmdMaxUs);
    } else {
      rawR = clampf(p.velKpUsPerCmS * eR + iR_, -p.cmdMaxUs, p.cmdMaxUs);
      rawF = clampf(p.velKpUsPerCmS * eF + iF_, -p.cmdMaxUs, p.cmdMaxUs);
    }
    slewTo(cmdR_, rawR, dtS);
    slewTo(cmdF_, rawF, dtS);
    fill(o, in);
    return o;
  }

  // 목표 태그가 바뀜 (투어). 다음 태그 프레임에서 칼만 위치만 새 태그로 다시 잡는다 — 속도·외란은 이어 쓴다
  void retarget() {
    kfRebase_ = true;
    heldMs_ = 0;
    arrived_ = false;
  }

  // 기체가 오른쪽(시계)으로 dpsiDeg 돌았다 → 바닥에 고정된 위치·속도는 기체 기준으로 반시계로 돈다.
  // 외란 b 는 기체에 붙은 값(수평 보정 오차)이 대부분이라 돌리지 않는다.
  void rotateFrame(float dpsiDeg) {
    float a = dpsiDeg * (float)M_PI / 180.0f, c = cosf(a), s = sinf(a);
    auto rot = [c, s](float &r, float &f) {
      float nr = r * c - f * s, nf = r * s + f * c;
      r = nr;
      f = nf;
    };
    rot(kfR_.e, kfF_.e);
    rot(kfR_.v, kfF_.v);
    rot(vtR_, vtF_);
  }

  // 현재 명령 (오프셋 µs, 부호 적용 전)
  float cmdRightUs() const { return cmdR_; }
  float cmdFwdUs() const { return cmdF_; }

 private:
  static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

  // 목표 속도·적분을 비우고 명령을 변화율 한계로 0(1500) 에 수렴시킨다
  void neutralStep(ApproachOutput &o, float dtS) {
    vtR_ = vtF_ = 0;
    iR_ = iF_ = 0;
    slewTo(cmdR_, 0.0f, dtS);
    slewTo(cmdF_, 0.0f, dtS);
    o.neutral = true;
  }

  void slewTo(float &cmd, float target, float dtS) const {
    bool tilt = p.outTilt != 0;
    float maxStep = (tilt ? p.tiltSlewUsPerS : p.cmdSlewUsPerS) * dtS;
    float lim = tilt ? p.tiltCmdMaxUs : p.cmdMaxUs;
    float d = clampf(target - cmd, -maxStep, maxStep);
    cmd = clampf(cmd + d, -lim, lim);
  }

  void fill(ApproachOutput &o, const ApproachInput &in) const {
    (void)in;
    o.vTargetRightCmS = vtR_;
    o.vTargetFwdCmS = vtF_;
    o.rollUs = p.rollSign * cmdR_;
    o.pitchUs = p.pitchSign * cmdF_;
    o.arriveHeldMs = heldMs_;
  }

  // vel_src 2: 칼만 예측(명령 기울기 → 가속도) + 새 태그 프레임이면 지연을 넣어 갱신. 입력의 위치·속도를 추정값으로 바꾼다.
  void kfStep(ApproachInput &in, float dtS, ApproachOutput &o, bool &predicting) {
    float expo = in.fcRcExpo >= 0 ? (float)in.fcRcExpo : p.rcExpo;
    float db = in.fcDeadbandUs >= 0 ? (float)in.fcDeadbandUs : p.rcDeadbandUs;
    uint32_t lostMs = in.kfLostMs ? in.kfLostMs : p.kfLostMs;  // 투어 이동 중에는 더 길게
    bool fcUsesBoard = in.authorityAuto && !in.kill;  // FC 가 보드 프레임(기준점 + 명령)을 쓴다
    bool applied = fcUsesBoard && in.applied && in.enabled;
    bool known = fcUsesBoard || in.pilotValid;        // 지금 FC 가 쓰는 스틱을 아는가
    // 실제 기울기 ≈ 스틱 기울기의 1차 지연. AUTO = 핸드오버 기준점 + 보드 명령, MANUAL = 조종자 스틱. 모르면 0, 잡음 크게
    float stR = fcUsesBoard ? p.rollSign * in.baseRollUs + (applied ? cmdR_ : 0.0f) : p.rollSign * in.pilotRollUs;
    float stF = fcUsesBoard ? p.pitchSign * in.basePitchUs + (applied ? cmdF_ : 0.0f) : p.pitchSign * in.pilotPitchUs;
    float tgtR = known ? stickUsToTilt(stR, expo, db, p.maxAngleDeg) : 0.0f;
    float tgtF = known ? stickUsToTilt(stF, expo, db, p.maxAngleDeg) : 0.0f;
    float k = p.kfTauS > 1e-3f ? dtS / p.kfTauS : 1.0f;
    if (k > 1) k = 1;
    kfTiltR_ += (tgtR - kfTiltR_) * k;
    kfTiltF_ += (tgtF - kfTiltF_) * k;
    float uR = known ? 981.0f * tanf(kfTiltR_ / 57.2957795f) : 0.0f;
    float uF = known ? 981.0f * tanf(kfTiltF_ / 57.2957795f) : 0.0f;
    bool attOk = p.kfAttInput != 0 && in.attValid && (uint32_t)(in.nowMs - in.attAtMs) <= 300;
    if (attOk) {  // 실제 자세 → 가속도 (지연·크기 차이가 없다). 조종기 수동 중에도 그대로 쓸 수 있다
      uR = 981.0f * tanf(p.rollSign * in.attRollDeg / 57.2957795f);
      uF = 981.0f * tanf(p.pitchSign * in.attPitchDeg / 57.2957795f);
      known = true;
    }
    kfU_[0] = uR;
    kfU_[1] = uF;
    float qa = known ? p.kfQa : p.kfQaManual;
    kfR_.drag = kfF_.drag = p.kfDrag;
    kfR_.predict(dtS, uR, qa, p.kfQb);
    kfF_.predict(dtS, uF, qa, p.kfQb);
    // 새 태그 프레임이면 갱신 (지연 L = 지금 − 촬영 시각 + 수신 지연)
    if (in.tagValid && in.tagAtMs && in.tagAtMs != kfLastTagAtMs_) {
      kfLastTagAtMs_ = in.tagAtMs;
      uint32_t cap = in.tagCaptureMs ? in.tagCaptureMs : in.tagAtMs;
      float L = (float)(uint32_t)(in.nowMs - cap) / 1000.0f + p.kfLatencyMs / 1000.0f;
      if (L < 0) L = 0;
      if (L > 1.0f) L = 1.0f;
      float r = p.kfMeasCm + p.kfMeasPerM * (in.heightValid ? in.heightCm / 100.0f : 1.0f);
      uint32_t resetMs = lostMs > kfResetMs ? lostMs : kfResetMs;
      if (kfRebase_ && kfR_.init && kfF_.init && (uint32_t)(in.nowMs - kfLastUpdMs_) <= resetMs) {
        kfR_.rebase(in.errRightCm, L, uR, r);  // 다음 태그: 위치만 새로, 속도는 이어서
        kfF_.rebase(in.errFwdCm, L, uF, r);
      } else if (!kfR_.init || !kfF_.init || (uint32_t)(in.nowMs - kfLastUpdMs_) > resetMs) {
        kfR_.start(in.errRightCm, r);  // 처음이거나 오래 끊겼으면 측정으로 다시 시작
        kfF_.start(in.errFwdCm, r);
        // 외란(수평 보정 오차·바람)은 잘 안 바뀐다 → AUTO 에서 배운 값으로 시작
        if (kfBiasKnown_) {
          kfR_.b = kfBiasR_;
          kfF_.b = kfBiasF_;
          kfR_.P[2][2] = kfF_.P[2][2] = 5.0f * 5.0f;
        }
      } else {
        kfR_.update(in.errRightCm, L, uR, r, p.kfGate);
        kfF_.update(in.errFwdCm, L, uF, r, p.kfGate);
        if (known) {  // 스틱을 모르는 구간의 b 는 조종 입력이 섞여 기억하지 않는다
          kfBiasR_ = kfR_.b;
          kfBiasF_ = kfF_.b;
          kfBiasKnown_ = true;
        }
      }
      kfLastUpdMs_ = in.nowMs;
      kfRebase_ = false;
      o.kfLatMs = L * 1000.0f;
    }
    bool alive = kfR_.init && kfF_.init && (uint32_t)(in.nowMs - kfLastUpdMs_) <= lostMs;
    bool tagFreshRaw = in.tagValid && (uint32_t)(in.nowMs - in.tagAtMs) <= p.tagStaleMs;
    o.kfOn = alive;
    o.kfE[0] = kfR_.e;
    o.kfE[1] = kfF_.e;
    o.kfV[0] = kfR_.v;
    o.kfV[1] = kfF_.v;
    o.kfB[0] = kfR_.b;
    o.kfB[1] = kfF_.b;
    in.velFromTag = true;
    in.velValid = alive;
    in.velAtMs = in.nowMs;
    if (alive) {
      // 지금 위치·속도(카메라 지연 보정) → 명령이 실제 기울기가 되는 kf_pred_s 뒤로 더 내다본다
      float T = p.kfPredS > 0 ? p.kfPredS : 0.0f;
      float aR = kfU_[0] + kfR_.b - p.kfDrag * kfR_.v, aF = kfU_[1] + kfF_.b - p.kfDrag * kfF_.v;
      in.velRightCmS = kfR_.v + aR * T;
      in.velFwdCmS = kfF_.v + aF * T;
      in.errRightCm = kfR_.e - kfR_.v * T - 0.5f * aR * T * T;
      in.errFwdCm = kfF_.e - kfF_.v * T - 0.5f * aF * T * T;
      if (!tagFreshRaw) {
        // 태그가 화면 밖 → 예측 위치로 되돌아간다 (kf_lost_ms 까지)
        predicting = true;
        in.tagValid = true;
        in.tagAtMs = in.nowMs;
      }
    }
    o.predicting = predicting;
  }

  float vtR_ = 0, vtF_ = 0;    // 가속도 제한된 목표 속도
  float cmdR_ = 0, cmdF_ = 0;  // 변화율 제한된 명령
  float iR_ = 0, iF_ = 0;      // 적분 항
  uint32_t heldMs_ = 0;
  bool arrived_ = false;
  // vel_src 2 칼만
  TagKF1 kfR_, kfF_;
  float kfTiltR_ = 0, kfTiltF_ = 0;
  uint32_t kfLastTagAtMs_ = 0, kfLastUpdMs_ = 0;
  float kfBiasR_ = 0, kfBiasF_ = 0;  // AUTO 에서 배운 외란 (다시 시작할 때 초기값)
  float kfU_[2] = {0, 0};            // 이번 스텝의 가속도 입력 (오른쪽, 앞)
  bool kfBiasKnown_ = false;
  bool kfRebase_ = false;            // 다음 태그 측정에서 위치만 다시 잡기 (투어 목표 변경)
  static const uint32_t kfResetMs = 10000;
};

// 이름으로 파라미터 바꾸기 (`approach set <이름> <값>`). 성공하면 true.
inline bool approachSetParam(ApproachParams &p, const char *name, float v) {
  struct Entry { const char *n; float *f; uint32_t *u; uint8_t *b; };
  Entry table[] = {
    {"max_speed", &p.maxSpeedCmS, nullptr, nullptr},
    {"slow_radius", &p.slowRadiusCm, nullptr, nullptr},
    {"stop_radius", &p.stopRadiusCm, nullptr, nullptr},
    {"max_accel", &p.maxAccelCmS2, nullptr, nullptr},
    {"vel_kp", &p.velKpUsPerCmS, nullptr, nullptr},
    {"vel_ki", &p.velKiUsPerCm, nullptr, nullptr},
    {"vel_imax", &p.velIMaxUs, nullptr, nullptr},
    {"cmd_max", &p.cmdMaxUs, nullptr, nullptr},
    {"cmd_slew", &p.cmdSlewUsPerS, nullptr, nullptr},
    {"roll_sign", &p.rollSign, nullptr, nullptr},
    {"pitch_sign", &p.pitchSign, nullptr, nullptr},
    {"arrive_tol", &p.arriveTolCm, nullptr, nullptr},
    {"arrive_vel", &p.arriveVelCmS, nullptr, nullptr},
    {"arrive_hold_ms", nullptr, &p.arriveHoldMs, nullptr},
    {"tag_stale_ms", nullptr, &p.tagStaleMs, nullptr},
    {"flow_stale_ms", nullptr, &p.flowStaleMs, nullptr},
    {"flow_min_q", nullptr, nullptr, &p.flowMinQuality},
    {"height_stale_ms", nullptr, &p.heightStaleMs, nullptr},
    {"height_min", &p.heightMinCm, nullptr, nullptr},
    {"height_max", &p.heightMaxCm, nullptr, nullptr},
    {"max_vel", &p.maxVelCmS, nullptr, nullptr},
    {"flow_swap", &p.flowSwapXY, nullptr, nullptr},
    {"flow_fwd_sign", &p.flowFwdSign, nullptr, nullptr},
    {"flow_right_sign", &p.flowRightSign, nullptr, nullptr},
    {"att_comp", &p.attComp, nullptr, nullptr},
    {"att_roll_sign", &p.attRollSign, nullptr, nullptr},
    {"att_pitch_sign", &p.attPitchSign, nullptr, nullptr},
    {"cam_off_right", &p.camOffsetRightCm, nullptr, nullptr},
    {"cam_off_fwd", &p.camOffsetFwdCm, nullptr, nullptr},
    {"deadband_neutral", &p.deadbandNeutral, nullptr, nullptr},
    {"lost_neutral", &p.lostNeutral, nullptr, nullptr},
    {"overspeed", &p.overspeedCmS, nullptr, nullptr},
    {"handover_slew", &p.handoverSlewUsPerS, nullptr, nullptr},
    {"vel_src", &p.velSrc, nullptr, nullptr},
    {"height_src", &p.heightSrc, nullptr, nullptr},
    {"tag_vel_alpha", &p.tagVelAlpha, nullptr, nullptr},
    {"out_tilt", &p.outTilt, nullptr, nullptr},
    {"vel_kp_accel", &p.velKpAccel, nullptr, nullptr},
    {"accel_ff", &p.accelFf, nullptr, nullptr},
    {"max_accel_cmd", &p.maxAccelCmdCmS2, nullptr, nullptr},
    {"rc_expo", &p.rcExpo, nullptr, nullptr},
    {"rc_deadband", &p.rcDeadbandUs, nullptr, nullptr},
    {"max_angle", &p.maxAngleDeg, nullptr, nullptr},
    {"tilt_cmd_max", &p.tiltCmdMaxUs, nullptr, nullptr},
    {"tilt_slew", &p.tiltSlewUsPerS, nullptr, nullptr},
    {"kf_tau", &p.kfTauS, nullptr, nullptr},
    {"kf_qa", &p.kfQa, nullptr, nullptr},
    {"kf_qa_manual", &p.kfQaManual, nullptr, nullptr},
    {"kf_qb", &p.kfQb, nullptr, nullptr},
    {"kf_meas", &p.kfMeasCm, nullptr, nullptr},
    {"kf_meas_per_m", &p.kfMeasPerM, nullptr, nullptr},
    {"kf_latency_ms", &p.kfLatencyMs, nullptr, nullptr},
    {"kf_gate", &p.kfGate, nullptr, nullptr},
    {"kf_lost_ms", nullptr, &p.kfLostMs, nullptr},
    {"kf_search_speed", &p.kfSearchSpeedCmS, nullptr, nullptr},
    {"kf_bias_ff", &p.kfBiasFf, nullptr, nullptr},
    {"kf_att_input", &p.kfAttInput, nullptr, nullptr},
    {"kf_pred_s", &p.kfPredS, nullptr, nullptr},
    {"kf_drag", &p.kfDrag, nullptr, nullptr},
  };
  if (strcmp(name, "tag_vel_alpha") == 0 && (v <= 0 || v > 1)) return false;
  if ((strcmp(name, "height_src") == 0 || strcmp(name, "out_tilt") == 0) && v != 0 && v != 1) return false;
  if (strcmp(name, "vel_src") == 0 && v != 0 && v != 1 && v != 2) return false;  // 0 광류 · 1 카메라 미분 · 2 칼만
  if (strcmp(name, "kf_lost_ms") == 0 && (v < 0 || v > 10000)) return false;
  if (strcmp(name, "kf_drag") == 0 && (v < 0 || v > 2)) return false;
  if ((strcmp(name, "kf_tau") == 0 || strcmp(name, "kf_qa") == 0 || strcmp(name, "kf_meas") == 0) && v <= 0) return false;
  if (strcmp(name, "rc_expo") == 0 && (v < 0 || v > 99)) return false;  // 100 이면 0 근처 기울기가 0 → 역변환 불가
  if (strcmp(name, "max_accel_cmd") == 0 && (v < 0 || v > 300)) return false;
  if (strcmp(name, "tilt_cmd_max") == 0 && (v < 0 || v > 500)) return false;
  for (const Entry &e : table) {
    if (strcmp(e.n, name) != 0) continue;
    if (e.f) *e.f = v;
    else if (e.u) *e.u = v < 0 ? 0u : (uint32_t)v;
    else if (e.b) *e.b = v < 0 ? 0 : (v > 255 ? 255 : (uint8_t)v);
    return true;
  }
  return false;
}

// 파라미터 전체를 JSON 객체 문자열로. out 은 1200 바이트 이상.
inline int approachParamsJson(const ApproachParams &p, char *out, size_t n) {
  return snprintf(out, n,
                  "{\"max_speed\":%.1f,\"slow_radius\":%.1f,\"stop_radius\":%.1f,\"max_accel\":%.1f,"
                  "\"vel_kp\":%.2f,\"vel_ki\":%.2f,\"vel_imax\":%.1f,\"cmd_max\":%.0f,\"cmd_slew\":%.0f,"
                  "\"roll_sign\":%.0f,\"pitch_sign\":%.0f,\"arrive_tol\":%.1f,\"arrive_vel\":%.1f,"
                  "\"arrive_hold_ms\":%lu,\"tag_stale_ms\":%lu,\"flow_stale_ms\":%lu,\"flow_min_q\":%u,"
                  "\"height_stale_ms\":%lu,\"height_min\":%.0f,\"height_max\":%.0f,\"max_vel\":%.0f,"
                  "\"flow_swap\":%.0f,\"flow_fwd_sign\":%.0f,\"flow_right_sign\":%.0f,\"att_comp\":%.0f,"
                  "\"att_roll_sign\":%.0f,\"att_pitch_sign\":%.0f,\"cam_off_right\":%.1f,\"cam_off_fwd\":%.1f,"
                  "\"deadband_neutral\":%.0f,\"lost_neutral\":%.0f,\"overspeed\":%.1f,\"handover_slew\":%.0f,"
                  "\"vel_src\":%.0f,\"height_src\":%.0f,\"tag_vel_alpha\":%.2f,\"out_tilt\":%.0f,"
                  "\"vel_kp_accel\":%.2f,\"accel_ff\":%.2f,\"max_accel_cmd\":%.0f,\"rc_expo\":%.0f,"
                  "\"rc_deadband\":%.0f,\"max_angle\":%.0f,\"tilt_cmd_max\":%.0f,\"tilt_slew\":%.0f,"
                  "\"kf_tau\":%.2f,\"kf_qa\":%.0f,\"kf_qa_manual\":%.0f,\"kf_qb\":%.1f,\"kf_meas\":%.1f,"
                  "\"kf_meas_per_m\":%.1f,\"kf_latency_ms\":%.0f,\"kf_gate\":%.1f,\"kf_lost_ms\":%lu,"
                  "\"kf_search_speed\":%.1f,\"kf_bias_ff\":%.1f,\"kf_att_input\":%.0f,\"kf_pred_s\":%.2f,\"kf_drag\":%.2f}",
                  p.maxSpeedCmS, p.slowRadiusCm, p.stopRadiusCm, p.maxAccelCmS2, p.velKpUsPerCmS,
                  p.velKiUsPerCm, p.velIMaxUs, p.cmdMaxUs, p.cmdSlewUsPerS, p.rollSign, p.pitchSign,
                  p.arriveTolCm, p.arriveVelCmS, (unsigned long)p.arriveHoldMs, (unsigned long)p.tagStaleMs,
                  (unsigned long)p.flowStaleMs, p.flowMinQuality, (unsigned long)p.heightStaleMs,
                  p.heightMinCm, p.heightMaxCm, p.maxVelCmS, p.flowSwapXY, p.flowFwdSign, p.flowRightSign,
                  p.attComp, p.attRollSign, p.attPitchSign, p.camOffsetRightCm, p.camOffsetFwdCm,
                  p.deadbandNeutral, p.lostNeutral, p.overspeedCmS, p.handoverSlewUsPerS, p.velSrc, p.heightSrc,
                  p.tagVelAlpha, p.outTilt, p.velKpAccel, p.accelFf, p.maxAccelCmdCmS2, p.rcExpo, p.rcDeadbandUs,
                  p.maxAngleDeg, p.tiltCmdMaxUs, p.tiltSlewUsPerS, p.kfTauS, p.kfQa, p.kfQaManual, p.kfQb,
                  p.kfMeasCm, p.kfMeasPerM, p.kfLatencyMs, p.kfGate, (unsigned long)p.kfLostMs, p.kfSearchSpeedCmS,
                  p.kfBiasFf, p.kfAttInput, p.kfPredS, p.kfDrag);
}
