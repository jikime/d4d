#pragma once
// 고도 유지·착륙 (-43): 거리센서 높이로 스로틀을 보드가 정한다. 하드웨어 의존 없음 (test/alt_hold_host_test.cpp).
//
// 사용자 요청 (20261011):
//   1) 수동 조종 중 지금 스로틀과 고도를 보드가 읽는다 → 수동 중 조종자 스로틀(MSP_RC)·거리센서 높이를 따라간다
//   2) CH6 AUTO 로 바뀌는 순간의 고도를 목표 고도로 → 그 순간의 조종자 스로틀이 기준(호버) 스로틀
//   3) 거리센서 피드백으로 목표 고도와의 오차를 계산 → 기준 + PID(오차, 수직 속도)
//   4) 스로틀 명령의 변화율과 출력 범위를 제한 → 기준 ± range_us, 절대 min_us~max_us, slew_us_s
//   5) 보드 통신이 끊기면 조종기로 돌아오는지 검증 → INAV 쪽 동작 (지상 시험 절차는 문서)
// 착륙(-45): 하강 속도 제어 — 목표 하강 속도(15cm/s, 40cm 아래 6cm/s)보다 느리면 스로틀을 계속 조금씩 내린다(적분).
//       -44 는 "목표 고도를 지금보다 10cm 아래까지만" 내려 스로틀이 호버보다 5µs 낮을 뿐이라 실제 기체가 안 내려왔다
//       (20261011 비행: 120~130cm 에서 100초 넘게 머묾). 거리센서가 바닥(touch_cm)을 touch_ms 동안 보이면 착지 →
//       스로틀을 최소로 내리고 disarmRequest. 거리센서가 안 보이면 내려가지 않는다 (조종자에게).
//
// 전제: FC msp_override_channels 에 스로틀(논리 채널 3, 비트 8)이 들어 있을 때만 FC 가 이 값을 쓴다.
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

enum class AltState : uint8_t { Manual, Hold, Landing, Touchdown, Fault };

inline const char *altStateName(AltState s) {
  switch (s) {
    case AltState::Manual: return "MANUAL";
    case AltState::Hold: return "HOLD";
    case AltState::Landing: return "LANDING";
    case AltState::Touchdown: return "TOUCHDOWN";
    default: return "FAULT";
  }
}

struct AltHoldParams {
  // 게인: PC 시뮬레이션에서 추력 민감도 1.5~5 cm/s²/µs · 거리센서 지연 0.1~0.25s 모두 안정인 값 (높은 게인은 흔들림 ±1m)
  float kpUsPerCm = 0.5f;        // 고도 오차 1cm 당 µs (+ = 낮으면 올림)
  float kdUsPerCmS = 0.6f;       // 수직 속도 1cm/s 당 µs (제동)
  float kiUsPerCmS = 0.2f;       // 오차 적분 (배터리 전압이 내려가 호버 스로틀이 오르는 것 보상)
  float abortErrCm = 60;         // 오차가 이보다 커지면 보정을 멈추고(기준 스로틀 유지) 경고 → 조종자가 넘겨받는다
  float iMaxUs = 60;             // 적분 한계
  float rangeUs = 120;           // 기준(호버) 스로틀 ± 이 범위 안에서만 움직인다
  float minUs = 1100, maxUs = 1800;  // 절대 한계 (착지 뒤 스로틀 내리기는 예외)
  float slewUsPerS = 150;        // 스로틀 변화율 한계
  uint32_t heightStaleMs = 400;  // 거리센서 값이 이보다 오래되면 무효 (FC 경유 10Hz)
  float heightMinCm = 2, heightMaxCm = 300;
  float targetMinCm = 30, targetMaxCm = 250;  // 유지 목표 범위 (AUTO 순간 고도를 이 안으로)
  // 착륙
  float landRateCmS = 15;        // 목표 고도를 내리는 속도
  float landSlowCmS = 6;         // land_slow_below 아래에서는 이 속도
  float landSlowBelowCm = 40;
  float landKvUs = 1.0f;         // 하강 속도 오차 1cm/s 당 µs
  float landKiUs = 1.5f;         // 하강 속도 오차 적분 (µs per cm/s·s) — 안 내려가면 계속 줄인다
  float touchCm = 8;             // 이 아래 높이가 touch_ms 동안 이어지면 착지 (바닥에서 3~5cm 로 읽힘)
  uint32_t touchMs = 1000;
  float cutSlewUsPerS = 400;     // 착지 뒤 스로틀을 최소로 내리는 속도
  float idleUs = 1000;           // 착지 뒤 스로틀
};

struct AltHoldInput {
  uint32_t nowMs = 0;
  bool autoOn = false;           // CH6 AUTO (FC 가 보드 프레임을 쓰는 중)
  bool pilotThrValid = false;    // 수동 중 조종자 스로틀 (MSP_RC, 덮어쓰기 전 값)
  float pilotThrUs = 1000;
  bool hValid = false;           // 거리센서 높이
  uint32_t hAtMs = 0;
  float hCm = 0;
  bool vzValid = false;          // 수직 속도 (높이 미분 평활, + = 올라감)
  float vzCmS = 0;
  bool landRequest = false;      // 착륙 (투어 착륙 태그 또는 명령)
};

struct AltHoldOutput {
  AltState state = AltState::Manual;
  const char *fault = nullptr;   // 무엇 때문에 고도 보정을 못 하나 (스로틀은 기준값 유지)
  bool known = false;            // thrUs 를 보낼 수 있다 (조종자 스로틀을 한 번이라도 읽음)
  float thrUs = 1000;            // 보낼 스로틀
  float baseUs = 0;              // 기준(AUTO 순간 조종자 스로틀)
  float targetCm = 0, errCm = 0;
  float pUs = 0, dUs = 0, iUs = 0;
  bool touchdown = false;
  bool disarmRequest = false;    // 착지 뒤 스로틀을 다 내림 → 시동 끄기 요청
};

class AltHold {
 public:
  AltHoldParams p;

  void reset() {
    st_ = AltState::Manual;
    known_ = false;
    thr_ = base_ = 1000;
    target_ = i_ = 0;
    touchSinceMs_ = 0;
    landing_ = false;
    aborted_ = false;
  }

  AltHoldOutput step(const AltHoldInput &in, float dtS) {
    AltHoldOutput o;
    if (dtS <= 0) dtS = 0.02f;
    bool hOk = in.hValid && (uint32_t)(in.nowMs - in.hAtMs) <= p.heightStaleMs && in.hCm >= p.heightMinCm &&
               in.hCm <= p.heightMaxCm;
    if (!in.autoOn) {
      // 1) 수동: 조종자 스로틀·고도를 따라간다. 보낼 값도 조종자 값(FC 는 수동이면 보드 프레임을 무시)
      st_ = AltState::Manual;
      landing_ = false;
      aborted_ = false;
      touchSinceMs_ = 0;
      i_ = 0;
      if (in.pilotThrValid) {
        thr_ = base_ = in.pilotThrUs;
        known_ = true;
      }
      if (hOk) target_ = in.hCm;
      fill(o, in, 0, 0);
      return o;
    }
    if (st_ == AltState::Manual) {
      // 2) AUTO 로 바뀌는 순간: 기준 = 마지막 조종자 스로틀 (튀지 않게), 목표 = 지금 고도
      st_ = known_ ? AltState::Hold : AltState::Fault;
      if (hOk) target_ = clampf(in.hCm, p.targetMinCm, p.targetMaxCm);
      i_ = 0;
    }
    if (!known_) {  // 조종자 스로틀을 한 번도 못 읽었다 → 보낼 값이 없다 (main 은 안전값 1000 을 보낸다)
      fill(o, in, 0, 0);
      o.fault = "no_pilot_throttle";
      return o;
    }
    if (in.landRequest && !landing_ && st_ != AltState::Touchdown) {
      landing_ = true;
      landI_ = 0;
      st_ = AltState::Landing;
      if (hOk) target_ = fminf(target_, in.hCm);
    }

    if (st_ == AltState::Touchdown) {
      // 착지: 스로틀을 최소로 내리고 시동 끄기 요청
      thr_ = fmaxf(p.idleUs, thr_ - p.cutSlewUsPerS * dtS);
      fill(o, in, 0, 0);
      o.touchdown = true;
      o.disarmRequest = thr_ <= p.idleUs + 1;
      return o;
    }

    const char *fault = nullptr;
    float pUs = 0, dUs = 0;
    if (hOk && fabsf(target_ - in.hCm) > p.abortErrCm) aborted_ = true;  // 흔들림·이상 → 보정 멈춤 (수동 전환까지)
    if (aborted_) {
      fault = "alt_error";
      st_ = AltState::Fault;
    } else if (hOk) {
      if (st_ == AltState::Landing) {
        // 착륙: 하강 속도 제어. 목표 하강 속도보다 느리면 스로틀을 줄이고(P), 계속 느리면 더 줄인다(적분)
        float vzT = -(in.hCm <= p.landSlowBelowCm ? p.landSlowCmS : p.landRateCmS);
        float ev = vzT - (in.vzValid ? in.vzCmS : 0.0f);  // − = 더 빨리 내려가야 함
        landI_ = clampf(landI_ + p.landKiUs * ev * dtS, -p.rangeUs, p.rangeUs);
        target_ = in.hCm;  // 화면용 (유지 목표가 아니다)
        if (in.hCm <= p.touchCm) {
          if (!touchSinceMs_) touchSinceMs_ = in.nowMs ? in.nowMs : 1;
          if ((uint32_t)(in.nowMs - touchSinceMs_) >= p.touchMs) {
            st_ = AltState::Touchdown;
            fill(o, in, 0, 0);
            o.touchdown = true;
            return o;
          }
        } else {
          touchSinceMs_ = 0;
        }
      }
      // 3) 오차·속도 → 스로틀 (착륙 중에는 하강 속도 오차 → P + 적분)
      float err = target_ - in.hCm;
      if (st_ == AltState::Landing) {
        float vzT = -(in.hCm <= p.landSlowBelowCm ? p.landSlowCmS : p.landRateCmS);
        pUs = p.landKvUs * (vzT - (in.vzValid ? in.vzCmS : 0.0f));
        dUs = landI_;
      } else {
        pUs = p.kpUsPerCm * err;
        dUs = in.vzValid ? -p.kdUsPerCmS * in.vzCmS : 0.0f;
      }
      float raw = base_ + pUs + dUs + i_;
      bool satHi = raw >= hiLimit(), satLo = raw <= loLimit();
      // 적분: 한계에 걸린 쪽으로는 더 쌓지 않는다 (와인드업 방지). 착륙 중에는 쌓지 않는다 (목표가 계속 움직임)
      if (st_ == AltState::Hold && !((satHi && err > 0) || (satLo && err < 0)))
        i_ = clampf(i_ + p.kiUsPerCmS * err * dtS, -p.iMaxUs, p.iMaxUs);
      if (st_ == AltState::Fault) st_ = landing_ ? AltState::Landing : AltState::Hold;
    } else {
      // 거리센서 무효 → 보정 없이 기준 + 적분 유지 (내려가지 않는다). 조종자가 넘겨받아야 할 수 있다
      fault = "height";
      st_ = AltState::Fault;
    }
    // 4) 범위·변화율 제한
    float want = clampf(base_ + pUs + dUs + i_, loLimit(), hiLimit());
    float step = p.slewUsPerS * dtS;
    thr_ += clampf(want - thr_, -step, step);
    fill(o, in, pUs, dUs);
    o.fault = fault;
    return o;
  }

  float targetCm() const { return target_; }
  // 착륙 취소 → 지금 높이에서 유지
  void cancelLanding(float hCm) {
    if (st_ != AltState::Landing) return;
    landing_ = false;
    touchSinceMs_ = 0;
    target_ = clampf(hCm, p.targetMinCm, p.targetMaxCm);
    st_ = AltState::Hold;
  }

  // 목표 고도 바꾸기 (명령). AUTO 유지 중에만 의미가 있다
  void setTarget(float cm) { target_ = clampf(cm, p.targetMinCm, p.targetMaxCm); }

 private:
  static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
  float loLimit() const { return fmaxf(p.minUs, base_ - p.rangeUs); }
  float hiLimit() const { return fminf(p.maxUs, base_ + p.rangeUs); }

  void fill(AltHoldOutput &o, const AltHoldInput &in, float pUs, float dUs) const {
    o.state = st_;
    o.known = known_;
    o.thrUs = thr_;
    o.baseUs = base_;
    o.targetCm = target_;
    o.errCm = target_ - in.hCm;
    o.pUs = pUs;
    o.dUs = dUs;
    o.iUs = i_;
  }

  AltState st_ = AltState::Manual;
  bool known_ = false, landing_ = false, aborted_ = false;
  float thr_ = 1000, base_ = 1000, target_ = 0, i_ = 0;
  float landI_ = 0;  // 착륙 하강 속도 적분 (µs)
  uint32_t touchSinceMs_ = 0;
};

inline bool altSetParam(AltHoldParams &p, const char *name, float v) {
  struct Entry { const char *n; float *f; uint32_t *u; float lo, hi; };
  Entry table[] = {
      {"kp", &p.kpUsPerCm, nullptr, 0, 10},
      {"kd", &p.kdUsPerCmS, nullptr, 0, 20},
      {"ki", &p.kiUsPerCmS, nullptr, 0, 5},
      {"i_max", &p.iMaxUs, nullptr, 0, 200},
      {"abort_err", &p.abortErrCm, nullptr, 20, 200},
      {"range_us", &p.rangeUs, nullptr, 0, 300},
      {"min_us", &p.minUs, nullptr, 1000, 1500},
      {"max_us", &p.maxUs, nullptr, 1500, 2000},
      {"slew_us_s", &p.slewUsPerS, nullptr, 10, 1000},
      {"land_rate", &p.landRateCmS, nullptr, 2, 50},
      {"land_slow", &p.landSlowCmS, nullptr, 1, 30},
      {"land_slow_below", &p.landSlowBelowCm, nullptr, 10, 150},
      {"land_kv", &p.landKvUs, nullptr, 0, 10},
      {"land_ki", &p.landKiUs, nullptr, 0, 10},
      {"touch_cm", &p.touchCm, nullptr, 3, 30},
      {"touch_ms", nullptr, &p.touchMs, 200, 5000},
      {"target_min", &p.targetMinCm, nullptr, 10, 200},
      {"target_max", &p.targetMaxCm, nullptr, 50, 300},
  };
  for (const Entry &e : table) {
    if (strcmp(e.n, name) != 0) continue;
    if (v < e.lo || v > e.hi) return false;
    if (e.f) *e.f = v;
    else *e.u = (uint32_t)v;
    return true;
  }
  return false;
}

inline int altParamsJson(const AltHoldParams &p, char *out, size_t n) {
  return snprintf(out, n,
                  "{\"kp\":%.2f,\"kd\":%.2f,\"ki\":%.2f,\"i_max\":%.0f,\"range_us\":%.0f,\"min_us\":%.0f,\"max_us\":%.0f,"
                  "\"slew_us_s\":%.0f,\"land_rate\":%.1f,\"land_slow\":%.1f,\"land_slow_below\":%.0f,\"touch_cm\":%.1f,"
                  "\"touch_ms\":%lu,\"target_min\":%.0f,\"target_max\":%.0f}",
                  p.kpUsPerCm, p.kdUsPerCmS, p.kiUsPerCmS, p.iMaxUs, p.rangeUs, p.minUs, p.maxUs, p.slewUsPerS,
                  p.landRateCmS, p.landSlowCmS, p.landSlowBelowCm, p.touchCm, (unsigned long)p.touchMs, p.targetMinCm,
                  p.targetMaxCm);
}
