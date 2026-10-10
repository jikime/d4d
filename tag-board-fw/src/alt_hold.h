#pragma once
// 고도 유지·착륙 (-43): 거리센서 높이로 스로틀을 보드가 정한다. 하드웨어 의존 없음 (test/alt_hold_host_test.cpp).
//
// 사용자 요청 (20261011):
//   1) 수동 조종 중 지금 스로틀과 고도를 보드가 읽는다 → 수동 중 조종자 스로틀(MSP_RC)·거리센서 높이를 따라간다
//   2) CH6 AUTO 로 바뀌는 순간의 고도를 목표 고도로 → 그 순간의 조종자 스로틀이 기준(호버) 스로틀
//   3) 거리센서 피드백으로 목표 고도와의 오차를 계산 → 기준 + PID(오차, 수직 속도)
//   4) 스로틀 명령의 변화율과 출력 범위를 제한 → 기준 ± range_us, 절대 min_us~max_us, slew_us_s
//   5) 보드 통신이 끊기면 조종기로 돌아오는지 검증 → INAV 쪽 동작 (지상 시험 절차는 문서)
// 착륙(-49): 조종자 수동 착륙 3번(20261011)을 그대로 따른다.
//       (1) 높이별 호버 스로틀 실측 120cm 1513 · 80cm 1494 · 40cm 1479 · 20cm 1456 → 착륙 시작 높이 대비 이만큼 미리 뺀다
//           (적분이 늦게 따라가 -48 은 2cm/s 로 느렸다). 40cm 위 0.45µs/cm, 아래 1.1µs/cm.
//       (2) 20cm 에서 조종자처럼 스로틀을 45µs 내리고 내려앉기를 기다린다(조종자: 1456 → ~1410, 0.2~1초에 바닥).
//           3초 안에 안 닿으면 초당 10µs 씩 더 내린다. (3) 바닥에 닿으면 바로 1000 (-48 그대로).
// 착지(-48): 바닥에 닿는 순간 스로틀 1000 (사용자 요청 "땅에 닿으면 정지상태이니 그냥 내려서 1000"). -47 비행: 마지막 10초 동안
//       센서가 1~4cm(바닥)를 8번 넘게 냈지만 "1초 연속" 조건 때문에 매번 다시 튀어 올랐다(10~26cm). → 착륙 중 25cm 아래에서
//       센서 값이 4cm 이하(또는 최소 거리 아래)인 표본이 1초 안에 2번이면 바로 착지 → 스로틀 즉시 1000, 수동 전환까지 유지.
// 착륙(-48): -47 비행에서 117→14cm 는 안정적으로 내려왔지만, 10cm 아래에서 거리센서가 최소 거리 아래 값(1.0cm)을 내자
//       "높이 무효"로 보고 착륙 스로틀을 버려 호버 쪽으로 올라가 10cm 에서 떠 있었다. 그리고 일반 범위(기준 ±120µs)가 착륙에도
//       걸려 1386µs 아래로 못 내려갔다. → 착륙 중 25cm 아래에서 센서가 최소 거리 아래면 "바닥에 거의 닿음"으로 보고 스로틀을
//       계속 천천히 내리며 착지 시간을 센다. 착륙 중 센서가 끊겨도 착륙 스로틀을 유지(올리지 않음). 착륙 중에는 착륙 범위만 적용.
// 착륙(-47): -46 비행에서 104→18cm 는 흔들림 없이 내려왔지만 (1) 스로틀 하한(호버 −50µs)이 지면 효과 구간(18cm)에서 막혀 멈췄고
//       (2) 적분이 느려 평균 1.4cm/s 였다 → 하한 −200µs, 적분 0.25 → 0.5. 태그는 50cm 아래에서 안 보이지만(너무 가까움)
//       하강은 거리센서로만 하므로 그대로 이어지고, 수평은 칼만 예측으로 그 자리를 지킨다.
// 착륙(-46): 5cm/s 하강 속도 제어 (사용자 요청 "천천히, 5cm/s"). 거리센서 값은 필터(튀는 값 무시)를 거친다.
//       -45 비행: 거리센서가 한 번 튀자(91→112→91cm) 상승으로 오인 → 스로틀 −46µs → 60cm/s 로 떨어졌다가 반동으로 191cm 까지
//       솟음. 그래서 착륙 중 스로틀은 호버 −50µs ~ +40µs 안으로만, 수직 속도는 필터 값으로.
// 착륙(-45): 하강 속도 제어 — 목표 하강 속도보다 느리면 스로틀을 계속 조금씩 내린다(적분).
//       -44 는 "목표 고도를 지금보다 10cm 아래까지만" 내려 스로틀이 호버보다 5µs 낮을 뿐이라 실제 기체가 안 내려왔다
//       (20261011 비행: 120~130cm 에서 100초 넘게 머묾). 거리센서가 바닥(touch_cm)을 touch_ms 동안 보이면 착지 →
//       스로틀을 최소로 내리고 disarmRequest. 거리센서가 안 보이면 내려가지 않는다 (조종자에게).
//
// 전제: FC msp_override_channels 에 스로틀(논리 채널 3, 비트 8)이 들어 있을 때만 FC 가 이 값을 쓴다.
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

enum class AltState : uint8_t { Manual, Hold, Landing, Touchdown, Fault, Ground, Takeoff };

inline const char *altStateName(AltState s) {
  switch (s) {
    case AltState::Manual: return "MANUAL";
    case AltState::Hold: return "HOLD";
    case AltState::Landing: return "LANDING";
    case AltState::Touchdown: return "TOUCHDOWN";
    case AltState::Ground: return "GROUND";
    case AltState::Takeoff: return "TAKEOFF";
    default: return "FAULT";
  }
}

// 이륙(-50): 조종자 수동 이륙 실측(20261011) — 스로틀을 약 150µs/s 로 올려 1455~1473 에서 떠오르고(지면 효과로 호버보다 낮음),
//       10→100cm 를 약 3.7초(24cm/s)에 오름. 바닥에서 CH6 AUTO + 시동 → 같은 방식으로 올리고, 뜨면 25cm/s 로 120cm 까지.
//       호버 추정 +60µs 까지 올려도 안 뜨면 중단(1000). 유지 중 배운 호버 스로틀을 다음 이륙에 쓴다.
struct AltHoldParams {
  // 게인: PC 시뮬레이션에서 추력 민감도 1.5~5 cm/s²/µs · 거리센서 지연 0.1~0.25s 모두 안정인 값 (높은 게인은 흔들림 ±1m)
  float kpUsPerCm = 0.5f;        // 고도 오차 1cm 당 µs (+ = 낮으면 올림)
  float kdUsPerCmS = 0.5f;       // 수직 속도 1cm/s 당 µs (제동). 필터 지연을 넣고 다시 맞춤 (-46)
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
  float landRateCmS = 10;        // 하강 속도 (cm/s, 40cm 위). 조종자 실측 12~24cm/s, 사용자 "천천히" 사이 (-50)
  float landSlowCmS = 5;         // land_slow_below 아래에서는 이 속도
  float landSlowBelowCm = 40;
  float landKvUs = 0.3f;         // 하강 속도 오차 1cm/s 당 µs
  float landKiUs = 0.5f;         // 하강 속도 오차 적분 (µs per cm/s·s) — 안 내려가면 더 줄인다 (-46 의 0.25 는 평균 1.4cm/s)
  float landDownUs = 200;        // 착륙 중 스로틀 하한 = 착륙 시작 때 호버 − 이 값 (-46 의 50 은 18cm 지면 효과에서 막힘)
  float landGeHighUsPerCm = 0.45f;  // 40cm 위: 높이 1cm 낮아질 때 호버 스로틀 감소 (조종자 실측)
  float landGeLowUsPerCm = 1.1f;     // 40cm 아래 (지면 효과)
  float landSettleCm = 20;       // 이 높이에서 마지막 단계: 스로틀을 settle_drop 만큼 내리고 내려앉기를 기다린다
  float landSettleDropUs = 45;   // 조종자 실측: 20cm 에서 약 45µs 내림
  uint32_t landSettleWaitMs = 3000;  // 이 안에 안 닿으면 더 내린다
  float landSettleRampUs = 10;   // 그 뒤 초당 더 내리는 스로틀
  float landCloseCm = 25;        // 이 아래에서 센서가 최소 거리 아래 값을 내면 "바닥에 거의 닿음"
  float landCloseRampUs = 15;    // 그때 스로틀을 내리는 속도 (µs/s)
  float landUpUs = 40;           // 착륙 중 스로틀 상한 = 호버 + 이 값 (제동만, 다시 솟지 않게)
  // 거리센서 필터 (알파-베타 + 튀는 값 무시)
  float spikeCm = 15;            // 예측과 이보다 다르면 그 값은 버린다 (연속 3번이면 받아들임)
  float filtAlpha = 0.7f, filtBeta = 0.3f;  // 추력 민감도 1~5 · 지연 0.1~0.25s · 튀는 값 5% 시뮬레이션으로 정함
  float touchCm = 8;             // 이 아래 높이가 touch_ms 동안 이어지면 착지 (바닥에서 3~5cm 로 읽힘)
  uint32_t touchMs = 1000;
  float groundCm = 4;            // 착륙 중 센서 값이 이 이하(또는 최소 거리 아래)면 바닥에 닿은 표본 (바닥에 놓으면 3cm)
  uint32_t groundWinMs = 1000;   // 바닥 표본이 이 시간 안에 2번이면 착지
  float idleUs = 1000;           // 착지 뒤 스로틀
  // 이륙
  float takeoffEnable = 1;       // 1: 바닥에서 AUTO + 시동이면 이륙
  float takeoffGroundCm = 10;    // AUTO 로 바뀔 때 이 아래면 바닥으로 본다
  float takeoffRampUs = 150;     // 떠오를 때까지 스로틀을 올리는 속도 (µs/s) — 호버 추정 −80µs 까지
  float takeoffRampSlowUs = 50;  // 그 뒤로는 천천히 (조종자: 1400 → 1455 는 약 0.5~1초)
  uint32_t takeoffFailWaitMs = 1500;  // 한계까지 올린 뒤 이만큼 기다려도 안 뜨면 중단
  float takeoffHoverUs = 1510;   // 처음 쓰는 호버 추정 (조종자 실측 60cm 위 중앙 1510~1512)
  float takeoffMaxOverUs = 60;   // 호버 추정 + 이만큼 올려도 안 뜨면 중단
  float liftoffCm = 7;           // 센서 값이 이보다 크면(바닥 3cm) 또는 8cm/s 넘게 오르면 떠올랐다
  float takeoffHeightCm = 120;   // 이륙 목표 고도 (조종자 비행 113~135cm)
  float takeoffClimbCmS = 25;    // 상승 속도 (조종자 24cm/s)
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
  bool armed = false;            // FC 시동 (이륙 판단)
};

struct AltHoldOutput {
  AltState state = AltState::Manual;
  const char *fault = nullptr;   // 무엇 때문에 고도 보정을 못 하나 (스로틀은 기준값 유지)
  bool known = false;            // thrUs 를 보낼 수 있다 (조종자 스로틀을 한 번이라도 읽음)
  float thrUs = 1000;            // 보낼 스로틀
  float baseUs = 0;              // 기준(AUTO 순간 조종자 스로틀)
  float targetCm = 0, errCm = 0;
  float pUs = 0, dUs = 0, iUs = 0;
  float hF = 0, vzF = 0;         // 필터한 높이·수직 속도
  uint32_t spikes = 0;           // 버린 거리센서 값 수
  bool touchdown = false;
  bool disarmRequest = false;    // 착지 뒤 스로틀을 다 내림 → 시동 끄기 요청
  bool takeoffFail = false;      // 이륙 중단 (호버 추정 +60µs 까지 올려도 안 뜸)
  float hoverUs = 0;             // 배운 호버 스로틀 (0 = 아직 모름)
  bool ready = false;            // 유지 중 (이륙·바닥 대기 아님) — 투어가 이때만 움직인다
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
    fInit_ = false;
  }

  AltHoldOutput step(const AltHoldInput &in, float dtS) {
    AltHoldOutput o;
    if (dtS <= 0) dtS = 0.02f;
    bool hOk = in.hValid && (uint32_t)(in.nowMs - in.hAtMs) <= p.heightStaleMs && in.hCm >= p.heightMinCm &&
               in.hCm <= p.heightMaxCm;
    // 바닥에 너무 가까움: 센서 값은 신선한데 최소 거리 아래 (MTF-01P 는 10cm 아래에서 1.0cm 같은 값을 낸다)
    bool tooClose = in.hValid && (uint32_t)(in.nowMs - in.hAtMs) <= p.heightStaleMs && in.hCm < p.heightMinCm;
    // 바닥 닿음: 착륙 중 25cm 아래에서 새 센서 값이 groundCm 이하(또는 최소 거리 아래) — 1초 안에 2번이면 바로 착지
    bool newSample = in.hValid && in.hAtMs != groundSampleAt_;
    if (newSample) groundSampleAt_ = in.hAtMs;
    if (newSample && landing_ && st_ != AltState::Touchdown && lastH_ <= p.landCloseCm &&
        in.hCm <= p.groundCm) {
      if (lastGroundMs_ && (uint32_t)(in.nowMs - lastGroundMs_) <= p.groundWinMs) groundHit_ = true;
      lastGroundMs_ = in.nowMs ? in.nowMs : 1;
    }
    if (hOk) filterHeight(in);
    else if (!tooClose) fInit_ = false;  // 너무 가까운 값은 필터를 깨지 않는다 (다시 정상 값이 오면 이어서)
    hOk = hOk && fInit_;
    const float h = hF_, vz = vF_;  // 제어에는 필터 값만 쓴다
    if (!in.autoOn) {
      // 1) 수동: 조종자 스로틀·고도를 따라간다. 보낼 값도 조종자 값(FC 는 수동이면 보드 프레임을 무시)
      st_ = AltState::Manual;
      landing_ = false;
      aborted_ = false;
      touchSinceMs_ = 0;
      groundHit_ = false;
      i_ = 0;
      if (in.pilotThrValid) {
        thr_ = base_ = in.pilotThrUs;
        known_ = true;
      }
      if (hOk) target_ = h;
      fill(o, in, 0, 0);
      return o;
    }
    // 바닥: 필터 높이가 낮거나, 센서가 최소 거리 아래 값(바닥에서 1~2cm 로 오락가락 — -50 업로드 뒤 실측)
    bool onGround = (hOk && h <= p.takeoffGroundCm) || tooClose;
    if (p.takeoffEnable != 0 && st_ != AltState::Manual && st_ != AltState::Ground && st_ != AltState::Takeoff &&
        !in.armed && onGround) {
      // AUTO 중 시동이 꺼진 채 바닥 (착지 뒤 INAV 가 시동을 끔 등) → 이륙 대기. 다시 시동을 걸면 다시 이륙
      st_ = AltState::Ground;
      landing_ = false;
      groundHit_ = false;
      thr_ = p.idleUs;
    }
    if (st_ == AltState::Manual) {
      // 2) AUTO 로 바뀌는 순간: 바닥이면 이륙 대기, 아니면 기준 = 마지막 조종자 스로틀 (튀지 않게), 목표 = 지금 고도
      i_ = 0;
      if (p.takeoffEnable != 0 && onGround) {
        st_ = AltState::Ground;
        thr_ = p.idleUs;
        takeoffFail_ = false;
      } else {
        st_ = known_ ? AltState::Hold : AltState::Fault;
        if (hOk) target_ = clampf(h, p.targetMinCm, p.targetMaxCm);
      }
    }
    if (st_ == AltState::Ground || (st_ == AltState::Takeoff && !lifted_)) {
      // 바닥 대기 → 시동이 걸리면 조종자처럼 스로틀을 천천히 올려 떠오르게 한다
      if (st_ == AltState::Ground) {
        thr_ = p.idleUs;
        if (in.armed && !takeoffFail_) {
          st_ = AltState::Takeoff;
          lifted_ = false;
          capAtMs_ = 0;
        }
      } else if (!in.armed) {
        st_ = AltState::Ground;
        thr_ = p.idleUs;
      } else {
        float guess = hoverKnown_ ? hoverLearned_ : p.takeoffHoverUs;
        float cap = guess + p.takeoffMaxOverUs;
        thr_ = fminf(cap, thr_ + (thr_ < guess - 80 ? p.takeoffRampUs : p.takeoffRampSlowUs) * dtS);
        if (thr_ >= cap && !capAtMs_) capAtMs_ = in.nowMs ? in.nowMs : 1;
        bool rawUp = in.hValid && (uint32_t)(in.nowMs - in.hAtMs) <= p.heightStaleMs && in.hCm > p.liftoffCm;
        if (rawUp || (hOk && vz > 8)) {
          // 떠올랐다: 호버 추정을 기준으로, 목표 고도를 지금부터 천천히 올린다
          lifted_ = true;
          base_ = guess;
          i_ = 0;
          target_ = h;
          known_ = true;
        } else if (capAtMs_ && (uint32_t)(in.nowMs - capAtMs_) > p.takeoffFailWaitMs) {
          takeoffFail_ = true;  // 안 뜸 (무게·배터리·센서 이상) → 바닥으로
          st_ = AltState::Ground;
          thr_ = p.idleUs;
        }
      }
      if (!lifted_ || st_ == AltState::Ground) {
        fill(o, in, 0, 0);
        o.takeoffFail = takeoffFail_;
        return o;
      }
    }
    if (st_ == AltState::Takeoff && lifted_) {
      // 상승: 목표를 takeoff_climb 으로 올려 takeoff_height 에서 유지로
      target_ = fminf(p.takeoffHeightCm, target_ + p.takeoffClimbCmS * dtS);
      if (target_ >= p.takeoffHeightCm && hOk && fabsf(h - target_) < 10) st_ = AltState::Hold;
    }
    if (!known_) {  // 조종자 스로틀을 한 번도 못 읽었다 → 보낼 값이 없다 (main 은 안전값 1000 을 보낸다)
      fill(o, in, 0, 0);
      o.fault = "no_pilot_throttle";
      return o;
    }
    if (in.landRequest && !landing_ && st_ != AltState::Touchdown) {
      landing_ = true;
      landI_ = 0;
      settle_ = false;
      landStartH_ = hOk ? h : 120;
      lastH_ = hOk ? h : 1000;
      lastGroundMs_ = 0;
      groundHit_ = false;
      hoverRef_ = base_ + i_;  // 착륙 시작 때 호버 (유지 중 적분까지)
      st_ = AltState::Landing;
      if (hOk) target_ = h;
    }

    if (groundHit_ && st_ != AltState::Touchdown) st_ = AltState::Touchdown;  // 바닥에 닿음 → 바로 스로틀 내림
    if (st_ == AltState::Touchdown) {
      // 착지: 스로틀을 바로 1000 으로 (사용자 요청: 시동 끄기 기능은 없으니 1000 으로). 수동 전환까지 유지
      thr_ = p.idleUs;
      fill(o, in, 0, 0);
      o.touchdown = true;
      o.disarmRequest = thr_ <= p.idleUs + 1;
      return o;
    }

    const char *fault = nullptr;
    float pUs = 0, dUs = 0;
    if (hOk && st_ != AltState::Landing && fabsf(target_ - h) > p.abortErrCm) aborted_ = true;  // 흔들림·이상 → 보정 멈춤
    if (aborted_) {
      fault = "alt_error";
      st_ = AltState::Fault;
    } else if (hOk) {
      if (st_ == AltState::Landing) {
        // 착륙: 하강 속도 제어. 목표 하강 속도보다 느리면 스로틀을 줄이고(P), 계속 느리면 더 줄인다(적분)
        float vzT = -(h <= p.landSlowBelowCm ? p.landSlowCmS : p.landRateCmS);
        float ev = vzT - vz;  // − = 더 빨리 내려가야 함
        landI_ = clampf(landI_ + p.landKiUs * ev * dtS, -p.landDownUs, p.landUpUs);
        target_ = h;  // 화면용 (유지 목표가 아니다)
        lastH_ = h;
        if (!settle_ && h <= p.landSettleCm) {  // 마지막 단계: 조종자처럼 한 번 내리고 기다린다
          settle_ = true;
          settleAtMs_ = in.nowMs ? in.nowMs : 1;
          settleThr_ = thr_ - p.landSettleDropUs;
        }
        if (touchStep(h <= p.touchCm, in.nowMs)) {
          fill(o, in, 0, 0);
          o.touchdown = true;
          return o;
        }
      }
      // 3) 오차·속도 → 스로틀 (착륙 중에는 하강 속도 오차 → P + 적분)
      float err = target_ - h;
      if (st_ == AltState::Landing) {
        float vzT = -(h <= p.landSlowBelowCm ? p.landSlowCmS : p.landRateCmS);
        pUs = p.landKvUs * (vzT - vz);
        dUs = landI_ + groundFf(h);  // 높이에 따른 호버 감소를 미리 뺀다
      } else {
        pUs = p.kpUsPerCm * err;
        dUs = -p.kdUsPerCmS * vz;
      }
      float raw = base_ + pUs + dUs + i_;
      bool satHi = raw >= hiLimit(), satLo = raw <= loLimit();
      // 적분: 한계에 걸린 쪽으로는 더 쌓지 않는다 (와인드업 방지). 착륙 중에는 쌓지 않는다 (목표가 계속 움직임)
      if ((st_ == AltState::Hold || st_ == AltState::Takeoff) && !((satHi && err > 0) || (satLo && err < 0)))
        i_ = clampf(i_ + p.kiUsPerCmS * err * dtS, -p.iMaxUs, p.iMaxUs);
      // 호버 학습: 유지 중 거의 멈춰 있으면 기준 + 적분이 곧 호버 스로틀 → 다음 이륙·착륙에 쓴다
      if (st_ == AltState::Hold && fabsf(vz) < 5 && fabsf(err) < 5) {
        float hov = base_ + i_;
        hoverLearned_ = hoverKnown_ ? hoverLearned_ + 0.01f * (hov - hoverLearned_) : hov;
        hoverKnown_ = true;
      }
      if (st_ == AltState::Fault) st_ = landing_ ? AltState::Landing : AltState::Hold;
    } else if (landing_ && tooClose && lastH_ <= p.landCloseCm) {
      // 착륙 중 바닥에 거의 닿음 (센서 최소 거리 아래): 스로틀을 계속 천천히 내리고 착지 시간을 센다
      st_ = AltState::Landing;
      landI_ = fmaxf(landI_ - p.landCloseRampUs * dtS, -p.landDownUs);
      target_ = 0;
      dUs = landI_ + groundFf(0);
      if (settle_) settleThr_ -= p.landCloseRampUs * dtS;
      if (touchStep(true, in.nowMs)) {
        fill(o, in, 0, 0);
        o.touchdown = true;
        return o;
      }
    } else {
      // 거리센서 무효 → 보정 없이 유지. 착륙 중이면 착륙 스로틀을 그대로 (올리지 않는다). 조종자가 넘겨받아야 할 수 있다
      fault = "height";
      st_ = AltState::Fault;
      if (landing_) dUs = landI_;
    }
    // 4) 범위·변화율 제한. 착륙 중에는 호버 −landDown ~ +landUp 안으로만 (다시 솟지 않게)
    float want = base_ + pUs + dUs + i_;
    if (landing_ && settle_ && st_ == AltState::Landing) {
      // 마지막 단계: 내린 스로틀로 기다리고, settle_wait 지나도 안 닿으면 천천히 더 내린다
      if ((uint32_t)(in.nowMs - settleAtMs_) > p.landSettleWaitMs) settleThr_ -= p.landSettleRampUs * dtS;
      want = settleThr_;
    }
    if (landing_)  // 착륙 중에는 착륙 범위만 (일반 범위 기준 ±120µs 는 지면 효과 구간에서 막는다)
      want = clampf(want, fmaxf(p.minUs, hoverRef_ - p.landDownUs), fminf(p.maxUs, hoverRef_ + p.landUpUs));
    else
      want = clampf(want, loLimit(), hiLimit());
    float step = p.slewUsPerS * dtS;
    thr_ += clampf(want - thr_, -step, step);
    fill(o, in, pUs, dUs);
    o.fault = fault;
    return o;
  }

  float targetCm() const { return target_; }

  // 조종자 실측 높이별 호버 스로틀 모델 (120cm 기준 0, 낮을수록 −). 착륙 시작 높이 대비 차이를 미리 더한다
  float hoverModel(float hCm) const {
    float x = clampf(hCm, 0, 150);
    if (x >= 40) return -p.landGeHighUsPerCm * (120 - x);
    return -p.landGeHighUsPerCm * 80 - p.landGeLowUsPerCm * (40 - x);
  }
  float groundFf(float hCm) const { return hoverModel(hCm) - hoverModel(landStartH_); }

  // 착지 판정: near 가 touch_ms 동안 이어지면 Touchdown 으로. 이어지지 않으면 처음부터
  bool touchStep(bool near, uint32_t now) {
    if (!near) {
      touchSinceMs_ = 0;
      return false;
    }
    if (!touchSinceMs_) touchSinceMs_ = now ? now : 1;
    if ((uint32_t)(now - touchSinceMs_) < p.touchMs) return false;
    st_ = AltState::Touchdown;
    return true;
  }
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
    o.errCm = target_ - hF_;
    o.pUs = pUs;
    o.dUs = dUs;
    o.iUs = i_;
    o.hF = hF_;
    o.vzF = vF_;
    o.hoverUs = hoverKnown_ ? hoverLearned_ : 0;
    o.ready = st_ == AltState::Hold || st_ == AltState::Landing || st_ == AltState::Fault;
    o.spikes = spikes_;
    (void)in;
  }

  // 거리센서 새 값이 오면 알파-베타 필터. 예측에서 spike_cm 넘게 벗어난 값은 버린다 (연속 3번째는 받아들임 — 실제 변화)
  void filterHeight(const AltHoldInput &in) {
    if (fInit_ && in.hAtMs == fAtMs_) return;
    float dt = fInit_ ? (float)(uint32_t)(in.hAtMs - fAtMs_) / 1000.0f : 0.0f;
    if (!fInit_ || dt <= 0 || dt > 0.6f) {
      hF_ = in.hCm;
      vF_ = 0;
      fInit_ = true;
      fAtMs_ = in.hAtMs;
      outliers_ = 0;
      return;
    }
    fAtMs_ = in.hAtMs;
    float pred = hF_ + vF_ * dt;
    float r = in.hCm - pred;
    if (fabsf(r) > p.spikeCm + fabsf(vF_) * dt && outliers_ < 2) {
      outliers_++;
      spikes_++;
      hF_ = pred;
      return;
    }
    outliers_ = 0;
    hF_ = pred + p.filtAlpha * r;
    vF_ += p.filtBeta * r / dt;
  }

  AltState st_ = AltState::Manual;
  bool known_ = false, landing_ = false, aborted_ = false;
  float thr_ = 1000, base_ = 1000, target_ = 0, i_ = 0;
  float landI_ = 0;  // 착륙 하강 속도 적분 (µs)
  float hoverRef_ = 1500;  // 착륙 시작 때 호버 스로틀
  float lastH_ = 1000;     // 마지막으로 믿을 수 있던 필터 높이
  bool lifted_ = false, takeoffFail_ = false;  // 이륙: 떠올랐나, 중단했나
  uint32_t capAtMs_ = 0;  // 이륙 스로틀이 한계에 닿은 시각
  float hoverLearned_ = 1500;
  bool hoverKnown_ = false;
  float landStartH_ = 120; // 착륙 시작 높이 (호버 모델 기준점)
  bool settle_ = false;    // 착륙 마지막 단계 (20cm 아래)
  uint32_t settleAtMs_ = 0;
  float settleThr_ = 1500;
  uint32_t groundSampleAt_ = 0, lastGroundMs_ = 0;  // 바닥 표본 판정용
  bool groundHit_ = false;  // 바닥에 닿음 (착지)
  bool fInit_ = false;
  float hF_ = 0, vF_ = 0;
  uint32_t fAtMs_ = 0, spikes_ = 0;
  uint8_t outliers_ = 0;
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
      {"land_down_us", &p.landDownUs, nullptr, 10, 150},
      {"land_up_us", &p.landUpUs, nullptr, 0, 150},
      {"spike_cm", &p.spikeCm, nullptr, 5, 100},
      {"touch_cm", &p.touchCm, nullptr, 3, 30},
      {"ground_cm", &p.groundCm, nullptr, 1, 10},
      {"takeoff", &p.takeoffEnable, nullptr, 0, 1},
      {"takeoff_height", &p.takeoffHeightCm, nullptr, 40, 250},
      {"takeoff_climb", &p.takeoffClimbCmS, nullptr, 5, 60},
      {"takeoff_hover", &p.takeoffHoverUs, nullptr, 1300, 1700},
      {"takeoff_ramp", &p.takeoffRampUs, nullptr, 30, 400},
      {"land_settle_cm", &p.landSettleCm, nullptr, 8, 50},
      {"land_settle_drop", &p.landSettleDropUs, nullptr, 0, 150},
      {"land_ge_high", &p.landGeHighUsPerCm, nullptr, 0, 3},
      {"land_ge_low", &p.landGeLowUsPerCm, nullptr, 0, 5},
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
