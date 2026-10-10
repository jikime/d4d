#pragma once
// AprilTag 저속 접근 제어 (하드웨어 의존 없음 — test/approach_host_test.cpp 에서 PC로 시험한다)
//
//   위치 오차(cm, 기체 기준) ─ 거리 프로파일 ─▶ 목표 속도(≤10cm/s) ─ 가속도 제한 ─▶
//   속도 P(I) ─▶ 롤·피치 명령(µs) ─ 변화율·크기 제한 ─▶ MSP_SET_RAW_RC 오프셋
//
// 전제: INAV ANGLE 모드(스틱 = 기울기 = 가속도). FC 자세 PID는 바꾸지 않는다.
// 속도는 FC 광류(MSP2_INAV_OPTICAL_FLOW)로 추정한다. 광류가 없으면 접근하지 않는다.
// 모든 게인·부호·한계는 ApproachParams 로 두고, 검증 전 값은 "후보값"이다.
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

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
  float heightMaxCm = 150.0f;
  float maxVelCmS = 60.0f;       // 이보다 빠르면 추정 이상 또는 외란 → 중지

  // ---- 추정 파라미터 (main.cpp 가 입력을 만들 때 사용) — 전부 실측 전 ----
  float flowSwapXY = 0;          // 1 이면 FC 유량 X↔Y 를 바꿔 앞/오른쪽에 대응
  float flowFwdSign = 1.0f;      // 앞으로 이동할 때 추정 속도가 음수로 나오면 -1
  float flowRightSign = 1.0f;
  float attComp = 0;             // 1 이면 FC 자세(롤·피치)로 카메라 지면점 이동을 보정
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
  float velSrc = 0;              // 0: FC 광류 속도(비행). 1: 카메라로 잰 태그 상대 속도(위치 미분 + 평활)
  float heightSrc = 0;           // 0: 거리센서만. 1: 거리센서가 무효면 태그 크기로 잰 카메라~태그 거리를 높이로 사용
  float tagVelAlpha = 0.5f;      // 카메라 속도 지수평활 계수 (1 = 평활 없음). 3~6fps 미분 잡음 억제
};

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

  bool heightValid = false;
  uint32_t heightAtMs = 0;
  float heightCm = 0;

  // FC msp_override_channels 확인 결과. 스로틀 비트가 들어 있거나 아직 못 읽었으면 출력하지 않는다.
  bool maskKnown = true;
  bool maskOk = true;
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
  float targetSpeedCmS = 0;      // 프로파일이 정한 속도 크기 (가속도 제한 전)
  float vTargetRightCmS = 0;     // 가속도 제한 후 목표 속도
  float vTargetFwdCmS = 0;
  float rollUs = 0;              // 1500 기준 오프셋
  float pitchUs = 0;
  uint32_t arriveHeldMs = 0;
  bool neutral = false;          // deadband·도착·유실로 명령을 중립으로 수렴시키는 중
  bool overspeed = false;        // 실측 속도가 overspeedCmS 초과 → 목표 0 으로 제동 중
};

class ApproachController {
 public:
  ApproachParams p;

  // 거리 → 목표 속도 크기. 25cm 이상 10cm/s, 5~25cm 는 거리에 비례(10 × d/25), 5cm 이하 0.
  // 정지 반경에서 0 으로 수렴하는 램프(d−5 비례)를 쓰면 경계 바깥에서 멈춰 버려 영원히 들어오지 못한다.
  // 거리 비례면 5cm 경계에서 2cm/s 로 진입한 뒤 0 이 되고, 가속도 제한이 그 단차를 부드럽게 만든다.
  float targetSpeed(float distCm) const {
    if (distCm >= p.slowRadiusCm) return p.maxSpeedCmS;
    if (distCm <= p.stopRadiusCm) return 0.0f;
    if (p.slowRadiusCm <= 1e-3f) return 0.0f;
    return p.maxSpeedCmS * distCm / p.slowRadiusCm;
  }

  void reset() {
    vtR_ = vtF_ = 0;
    cmdR_ = cmdF_ = 0;
    iR_ = iF_ = 0;
    heldMs_ = 0;
    arrived_ = false;
  }

  ApproachOutput step(const ApproachInput &in, float dtS) {
    ApproachOutput o;
    if (dtS <= 0) dtS = 0.05f;
    if (dtS > 0.5f) dtS = 0.5f;

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
    else if (vMag > p.maxVelCmS) fault = "velocity";

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
      o.targetSpeedCmS = targetSpeed(o.distCm);
      if (o.distCm > 1e-3f) {
        desR = in.errRightCm / o.distCm * o.targetSpeedCmS;
        desF = in.errFwdCm / o.distCm * o.targetSpeedCmS;
      }
      // 도착 판정: 오차·속도가 허용 범위에 arriveHoldMs 동안 머물러야 한다
      if (o.distCm <= p.arriveTolCm && vMag <= p.arriveVelCmS) {
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
    if (tagFresh && p.deadbandNeutral != 0 && (arrived_ || o.distCm <= p.stopRadiusCm)) {
      neutralStep(o, dtS);
      fill(o, in);
      return o;
    }

    // 실측 속도가 상한을 넘으면 이 스텝의 목표를 0 으로 → 속도 루프가 제동한다
    if (vMag > p.overspeedCmS) {
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
    float rawR = clampf(p.velKpUsPerCmS * eR + iR_, -p.cmdMaxUs, p.cmdMaxUs);
    float rawF = clampf(p.velKpUsPerCmS * eF + iF_, -p.cmdMaxUs, p.cmdMaxUs);
    slewTo(cmdR_, rawR, dtS);
    slewTo(cmdF_, rawF, dtS);
    fill(o, in);
    return o;
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
    float maxStep = p.cmdSlewUsPerS * dtS;
    float d = clampf(target - cmd, -maxStep, maxStep);
    cmd = clampf(cmd + d, -p.cmdMaxUs, p.cmdMaxUs);
  }

  void fill(ApproachOutput &o, const ApproachInput &in) const {
    (void)in;
    o.vTargetRightCmS = vtR_;
    o.vTargetFwdCmS = vtF_;
    o.rollUs = p.rollSign * cmdR_;
    o.pitchUs = p.pitchSign * cmdF_;
    o.arriveHeldMs = heldMs_;
  }

  float vtR_ = 0, vtF_ = 0;    // 가속도 제한된 목표 속도
  float cmdR_ = 0, cmdF_ = 0;  // 변화율 제한된 명령
  float iR_ = 0, iF_ = 0;      // 적분 항
  uint32_t heldMs_ = 0;
  bool arrived_ = false;
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
  };
  if (strcmp(name, "tag_vel_alpha") == 0 && (v <= 0 || v > 1)) return false;
  if ((strcmp(name, "vel_src") == 0 || strcmp(name, "height_src") == 0) && v != 0 && v != 1) return false;
  for (const Entry &e : table) {
    if (strcmp(e.n, name) != 0) continue;
    if (e.f) *e.f = v;
    else if (e.u) *e.u = v < 0 ? 0u : (uint32_t)v;
    else if (e.b) *e.b = v < 0 ? 0 : (v > 255 ? 255 : (uint8_t)v);
    return true;
  }
  return false;
}

// 파라미터 전체를 JSON 객체 문자열로. out 은 900 바이트 이상.
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
                  "\"vel_src\":%.0f,\"height_src\":%.0f,\"tag_vel_alpha\":%.2f}",
                  p.maxSpeedCmS, p.slowRadiusCm, p.stopRadiusCm, p.maxAccelCmS2, p.velKpUsPerCmS,
                  p.velKiUsPerCm, p.velIMaxUs, p.cmdMaxUs, p.cmdSlewUsPerS, p.rollSign, p.pitchSign,
                  p.arriveTolCm, p.arriveVelCmS, (unsigned long)p.arriveHoldMs, (unsigned long)p.tagStaleMs,
                  (unsigned long)p.flowStaleMs, p.flowMinQuality, (unsigned long)p.heightStaleMs,
                  p.heightMinCm, p.heightMaxCm, p.maxVelCmS, p.flowSwapXY, p.flowFwdSign, p.flowRightSign,
                  p.attComp, p.attRollSign, p.attPitchSign, p.camOffsetRightCm, p.camOffsetFwdCm,
                  p.deadbandNeutral, p.lostNeutral, p.overspeedCmS, p.handoverSlewUsPerS, p.velSrc, p.heightSrc,
                  p.tagVelAlpha);
}
