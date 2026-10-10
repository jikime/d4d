// PC에서 고도 유지·착륙(src/alt_hold.h)을 시험한다
//   clang++ -std=c++17 -O1 -I src test/alt_hold_host_test.cpp -o /tmp/alt_test && /tmp/alt_test
// 기체(수직): 가속도 = k·(모터 스로틀 − 실제 호버) − 0.5·vz. k(추력 민감도) 1.5~5 cm/s² per µs 로 바꿔 본다.
// 모터 지연 60ms, 거리센서 10Hz·지연 0.1~0.25s·잡음 ±1cm, 바닥에서 3cm 로 읽힘.
// 수동 2초(조종자가 호버 유지) → AUTO. 실제 호버 스로틀은 AUTO 뒤 20초 동안 sag µs 오른다(배터리 전압 감소).
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include "alt_hold.h"

static int fails = 0;
#define CHECK(c)                                                  \
  do {                                                            \
    if (!(c)) {                                                   \
      std::fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #c); \
      fails++;                                                    \
    }                                                             \
  } while (0)

struct R {
  float maxErr = 0, finalErr = 0, touchVz = 0, maxThr = 0, minThr = 3000, maxSlew = 0;
  float landMaxDown = 0, landClimb = 0;  // 착륙 중 가장 빠른 하강 속도, 가장 낮았던 곳에서 다시 오른 높이
  bool touch = false, disarm = false, fault = false;
  float tTouch = -1;
};

static R run(float kPlant, float lagS, float sag, float z0, float tLand, float simS, float dropFrom = -1,
             float kick = 0, float landSag = 0, float spikeProb = 0, float groundUs = 0, bool closeBad = false) {
  AltHold a;
  const float dt = 0.02f, pilot = 1500;
  float z = z0, vz = 0, thrAct = pilot, hover = pilot;
  bool landed = false;
  std::vector<float> hist;
  uint32_t seed = 7;
  auto rnd = [&]() { seed = seed * 1103515245u + 12345u; return ((seed >> 8) & 0xffff) / 65535.0f - 0.5f; };
  float hMeas = z0 + 3, vzMeas = 0, hPrev = z0 + 3, last = pilot;
  uint32_t hAt = 0;
  R r;
  for (int i = 0; i * dt < simS; i++) {
    float t = i * dt;
    uint32_t now = (uint32_t)(t * 1000 + .5f);
    bool autoOn = t >= 2;
    if (autoOn) hover = pilot + sag * fminf(1.0f, (t - 2) / 20.0f);
    if (tLand >= 0 && t >= tLand) hover += landSag;  // 착륙 시작 뒤 실제 호버가 더 높음 (20261011 비행: 안 내려옴)
    hist.push_back(z);
    if (i % 5 == 0) {
      size_t lag = (size_t)(lagS / dt);
      float zl = hist.size() > lag ? hist[hist.size() - 1 - lag] : z;
      float m = fmaxf(3.0f, zl + 3.0f) + 2.0f * rnd();
      if (closeBad && zl < 9.0f && rnd() > -0.2f) m = 1.0f;  // MTF-01P: 10cm 아래에서 최소 거리 아래 값 (-47 비행)
      if (spikeProb > 0 && rnd() + 0.5f < spikeProb) m += (rnd() > 0 ? 1 : -1) * (20.0f + 20.0f * (rnd() + 0.5f));  // 튀는 값
      vzMeas += 0.5f * ((m - hPrev) / 0.1f - vzMeas);
      hPrev = m;
      hMeas = m;
      hAt = now;
    }
    AltHoldInput in;
    in.nowMs = now;
    in.autoOn = autoOn;
    in.pilotThrValid = !autoOn;
    in.pilotThrUs = pilot;
    in.hValid = !(dropFrom >= 0 && t >= dropFrom && t < dropFrom + 2);
    in.hAtMs = hAt;
    in.hCm = hMeas;
    in.vzValid = in.hValid;
    in.vzCmS = vzMeas;
    in.landRequest = tLand >= 0 && t >= tLand;
    in.armed = true;  // 비행 중 (착지 판정 전에 "시동 꺼짐 + 바닥 → 이륙 대기"로 가지 않게)
    AltHoldOutput o = a.step(in, dt);
    float cmd = autoOn ? o.thrUs : hover;
    if (autoOn) {
      r.maxThr = fmaxf(r.maxThr, cmd);
      r.minThr = fminf(r.minThr, cmd);
      r.maxSlew = fmaxf(r.maxSlew, fabsf(cmd - last) / dt);
    }
    last = cmd;
    if (o.touchdown && !r.touch) {
      r.touch = true;
      r.tTouch = t;
    }
    if (o.disarmRequest) r.disarm = true;
    if (o.fault && std::string(o.fault) == "alt_error") r.fault = true;
    thrAct += (cmd - thrAct) * (dt / 0.06f);
    // 지면 효과: 30cm 아래에서 필요한 호버 스로틀이 줄어든다 (-46 비행: 100cm 1506 → 20cm 에서 1459 로도 안 내려감)
    float hov = hover - groundUs * fmaxf(0.0f, 1.0f - z / 30.0f);
    if (groundUs < 0) {  // 조종자 실측 곡선 (120cm 1513 · 80cm 1494 · 40cm 1479 · 20cm 1456) — 시작 높이 기준
      auto m = [](float x) { x = fminf(150.0f, fmaxf(0.0f, x)); return x >= 40 ? -0.45f * (120 - x) : -36.0f - 1.1f * (40 - x); };
      hov = hover + (m(z) - m(z0)) * (-groundUs);  // -groundUs = 실측 대비 배율 (1 = 실측 그대로)
    }
    float acc = landed ? 0 : kPlant * (thrAct - hov) - 0.5f * vz;
    if (kick != 0 && t >= 10 && t < 10.5f) acc += kick;  // 큰 외란 (돌풍·충돌)
    if (landed && thrAct > hover + 5) landed = false;
    vz += acc * dt;
    z += vz * dt;
    if (z <= 0) {
      if (!landed) r.touchVz = vz;
      z = 0;
      vz = 0;
      landed = true;
    }
    if (autoOn && !in.landRequest && t > 4) r.maxErr = fmaxf(r.maxErr, fabsf(z + 3 - o.targetCm));
    static float zMin = 1e9f;
    if (i == 0) zMin = 1e9f;
    if (in.landRequest && !landed && !r.touch) {
      r.landMaxDown = fmaxf(r.landMaxDown, -vz);
      zMin = fminf(zMin, z);
      r.landClimb = fmaxf(r.landClimb, z - zMin);
    }
    r.finalErr = z + 3 - o.targetCm;
  }
  return r;
}

static void testManualTracking() {
  // 1)·2) 수동 중 조종자 스로틀·고도를 읽고, AUTO 순간 그 값으로 (튀지 않음)
  AltHold a;
  AltHoldInput in;
  in.pilotThrValid = true;
  in.pilotThrUs = 1480;
  in.hValid = true;
  in.hCm = 90;
  for (uint32_t t = 0; t < 1000; t += 20) {
    in.nowMs = in.hAtMs = t;
    a.step(in, 0.02f);
  }
  in.autoOn = true;
  in.pilotThrValid = false;
  in.nowMs = in.hAtMs = 1020;
  AltHoldOutput o = a.step(in, 0.02f);
  CHECK(o.state == AltState::Hold && std::fabs(o.thrUs - 1480) < 2 && std::fabs(o.targetCm - 90) < 0.1f);
  // 조종자 스로틀을 한 번도 못 읽었으면 AUTO 여도 보낼 값이 없다 (main 은 1000 을 보내지 않고 기준값이 없음을 알린다)
  AltHold b;
  AltHoldInput j;
  j.autoOn = true;
  j.nowMs = 100;
  AltHoldOutput q = b.step(j, 0.02f);
  CHECK(!q.known && q.fault != nullptr);
}

static void testHoldRobust() {
  // 3)·4) 추력 민감도·센서 지연을 바꿔도 안정, 범위·변화율 제한
  for (float k : {1.5f, 3.0f, 5.0f})
    for (float lag : {0.1f, 0.25f}) {
      R h = run(k, lag, 40, 100, -1, 40);
      std::printf("hold k%.1f lag%.2f: 최대 오차 %.1fcm, 끝 %.1fcm, 스로틀 %.0f~%.0f, 변화율 %.0fus/s\n", k, lag, h.maxErr,
                  h.finalErr, h.minThr, h.maxThr, h.maxSlew);
      CHECK(h.maxErr < 15);
      CHECK(std::fabs(h.finalErr) < 5);
      CHECK(h.maxSlew <= 151);
      CHECK(h.maxThr <= 1500 + 120 + 0.5f && h.minThr >= 1500 - 120 - 0.5f);
      CHECK(!h.fault);
    }
  // 거리센서 2초 끊김 → 그동안 보정 없이 유지, 돌아오면 다시
  R d = run(3.0f, 0.1f, 30, 100, -1, 30, 8);
  std::printf("hold 센서 끊김: 최대 오차 %.1fcm, 끝 %.1fcm\n", d.maxErr, d.finalErr);
  CHECK(std::fabs(d.finalErr) < 5);
  // 큰 외란으로 오차 60cm 넘음 → 보정 멈춤(경고), 조종자에게
  R f = run(3.0f, 0.1f, 0, 100, -1, 20, -1, -400);
  std::printf("hold 큰 외란: 경고 %d\n", f.fault);
  CHECK(f.fault);
}

static void testAbortPersist() {
  // -54: 거리센서 오값(-53 비행 124→102→176cm)이 잠깐이면 보정을 멈추지 않고, 1.5초 넘게 이어질 때만 멈춘다.
  // 오차가 20cm 안으로 돌아오면 다시 보정, 착륙 요청은 멈춤보다 우선
  AltHold a;
  AltHoldInput in;
  in.armed = true;
  in.pilotThrValid = true;
  in.pilotThrUs = 1500;
  in.hValid = in.vzValid = true;
  uint32_t t = 0;
  auto feed = [&](float h, uint32_t ms, bool autoOn, bool land = false) {
    bool fault = false;
    AltHoldOutput o;
    for (uint32_t e = 0; e < ms; e += 20, t += 20) {
      in.nowMs = t;
      if (t % 100 == 0) in.hAtMs = t;
      in.hCm = h;
      in.autoOn = autoOn;
      in.pilotThrValid = !autoOn;
      in.landRequest = land;
      o = a.step(in, 0.02f);
      fault |= o.fault && std::string(o.fault) == "alt_error";
    }
    return std::make_pair(fault, o);
  };
  feed(120, 1000, false);
  CHECK(!feed(120, 2000, true).first);
  bool shortBad = feed(200, 1000, true).first;   // 1초 오값
  bool backOk = !feed(120, 2000, true).first;
  bool longBad = feed(200, 3000, true).first;    // 3초 이어짐
  auto rec = feed(120, 3000, true);
  bool recovered = !rec.second.fault;
  feed(200, 3000, true);
  auto land = feed(200, 200, true, true);
  std::printf("고도 오류 지속: 1초 오값 경고 %d, 3초 경고 %d, 돌아온 뒤 정상 %d, 착륙 우선 %s\n", shortBad, longBad, recovered,
              altStateName(land.second.state));
  CHECK(!shortBad && backOk);
  CHECK(longBad);
  CHECK(recovered);
  CHECK(land.second.state == AltState::Landing && !land.second.fault);
}

// -49 부터 착륙 시험은 조종자 실측 높이별 호버 곡선(groundUs = −1 → ×1.0)을 기체 모델로 쓴다. 착륙 제어가 그 곡선을
// 미리 빼므로, 지면 효과가 없는 모델에서는 일부러 더 빨리 내려간다 (실제 기체와 다름).
// 마지막 20cm 는 조종자처럼 한 번에 내려앉는다 → 이 구간 하강 속도는 조종자 실측(10cm→바닥 0.2초 ≈ 50cm/s)과 비슷하게 허용
static void testLandSlowSpikes() {
  // 튀는 값 5% + 추력 민감도 + 착륙 시작 순간 실제 호버 ±8µs 변화
  for (float k : {1.0f, 2.5f, 4.0f})
    for (float lag : {0.1f, 0.25f})
      for (float ls : {-8.0f, 0.0f, 8.0f}) {
        R l = run(k, lag, 10, 115, 10, 90, -1, 0, ls, 0.05f, -1);
        std::printf("land5 k%.1f lag%.2f 호버%+3.0f: 착지 %d (%.1f초), 닿는 속도 %.1f, 최대 하강 %.1fcm/s, 다시 오름 %.1fcm\n",
                    k, lag, ls, l.touch, l.tTouch, -l.touchVz, l.landMaxDown, l.landClimb);
        CHECK(l.touch && l.disarm);
        CHECK(-l.touchVz < 70);
        CHECK(l.tTouch < 75);
        CHECK(l.landMaxDown < 60);
        CHECK(l.landClimb < 12);
      }
}

static void testLandCloseSensor() {
  // -48: 10cm 아래에서 거리센서가 1.0cm(최소 거리 아래)를 70% 확률로 냄 → 그래도 착지·스로틀 1000
  for (float k : {1.0f, 2.0f, 3.0f})
    for (float scale : {0.6f, 1.0f, 1.5f}) {
      R l = run(k, 0.15f, 10, 105, 10, 160, -1, 0, 0, 0.05f, -scale, true);
      std::printf("land 센서 근접 k%.1f ×%.1f: 착지 %d (%.0f초), 최대 하강 %.1fcm/s, 다시 오름 %.1fcm\n", k, scale, l.touch,
                  l.tTouch - 10, l.landMaxDown, l.landClimb);
      CHECK(l.touch && l.disarm);
      CHECK(l.landClimb < 8);
      CHECK(l.tTouch - 10 < 100);
    }
}

static void testGroundContactCut() {
  // -48: 25cm 아래에서 센서 값이 바닥(3cm)으로 1초 안에 2번 → 즉시 착지, 스로틀 1000
  AltHold a;
  AltHoldInput in;
  in.pilotThrValid = true;
  in.pilotThrUs = 1500;
  in.hValid = true;
  in.hCm = 100;
  uint32_t t = 0;
  for (; t < 1000; t += 20) {
    in.nowMs = t;
    in.hAtMs = (t / 100) * 100;
    a.step(in, 0.02f);
  }
  in.autoOn = true;
  in.pilotThrValid = false;
  in.landRequest = true;
  in.armed = true;
  // 20cm 까지 내려온 뒤 바닥 3cm → 12cm(튀어 오름) → 3cm
  float seq[] = {40, 30, 22, 18, 15, 3, 12, 14, 3};
  AltHoldOutput o;
  for (float h : seq) {
    for (int k = 0; k < 5; k++) {
      t += 20;
      in.nowMs = t;
      in.hAtMs = (t / 100) * 100;
      in.hCm = h;
      o = a.step(in, 0.02f);
    }
  }
  t += 20;
  in.nowMs = t;
  o = a.step(in, 0.02f);
  std::printf("바닥 닿음: 상태 %s 스로틀 %.0f\n", altStateName(o.state), o.thrUs);
  CHECK(o.state == AltState::Touchdown && o.thrUs <= 1000.5f && o.disarmRequest);
  // 높은 곳에서 한 번 튄 값(3cm)은 착지가 아니다
  AltHold b;
  in = AltHoldInput();
  in.pilotThrValid = true;
  in.pilotThrUs = 1500;
  in.hValid = true;
  in.hCm = 100;
  for (t = 0; t < 1000; t += 20) {
    in.nowMs = t;
    in.hAtMs = (t / 100) * 100;
    b.step(in, 0.02f);
  }
  in.autoOn = true;
  in.pilotThrValid = false;
  in.landRequest = true;
  in.armed = true;
  float seq2[] = {90, 3, 89, 3, 88};
  for (float h : seq2)
    for (int k = 0; k < 5; k++) {
      t += 20;
      in.nowMs = t;
      in.hAtMs = (t / 100) * 100;
      in.hCm = h;
      o = b.step(in, 0.02f);
    }
  CHECK(o.state != AltState::Touchdown);
}

static void testLandMeasuredGround() {
  // -49: 조종자 실측 높이별 호버(0.6~1.5배) + 튀는 값 + 추력 민감도 → 착지, 하강 속도·다시 오름 확인
  for (float k : {1.0f, 2.0f, 3.0f})
    for (float scale : {0.6f, 1.0f, 1.5f}) {
      R l = run(k, 0.15f, 10, 120, 10, 160, -1, 0, 0, 0.05f, -scale);
      std::printf("land 실측지면 k%.1f ×%.1f: 착지 %d (%.0f초), 평균 %.1fcm/s, 최대 하강 %.1fcm/s, 다시 오름 %.1fcm\n", k, scale,
                  l.touch, l.tTouch - 10, 117.0f / (l.tTouch - 10), l.landMaxDown, l.landClimb);
      CHECK(l.touch && l.disarm);
      CHECK(l.landMaxDown < 60);
      CHECK(l.landClimb < 8);
      CHECK(l.tTouch - 10 < 60);
    }
}

// 이륙 시뮬레이션: 바닥(z 0)에서 시작. 실측 높이별 호버(120cm 에서 hover120, 바닥 가까이 낮음 — 지면 효과)
struct TO { bool lift = false, hold = false, fail = false; float tLift = -1, t100 = -1, maxZ = 0, finalErr = 0; };
static TO takeoffSim(float k, float hover120, bool arm, float simS = 30) {
  AltHold a;
  const float dt = 0.02f;
  float z = 0, vz = 0, thrAct = 1000;
  auto m = [](float x) { x = fminf(150.0f, fmaxf(0.0f, x)); return x >= 40 ? -0.45f * (120 - x) : -36.0f - 1.1f * (40 - x); };
  float hMeas = 3, vzm = 0, hPrev = 3;
  uint32_t hAt = 0;
  std::vector<float> hist;
  TO r;
  bool onGround = true;
  for (int i = 0; i * dt < simS; i++) {
    float t = i * dt;
    uint32_t now = (uint32_t)(t * 1000 + .5f);
    hist.push_back(z);
    if (i % 5 == 0) {
      float zl = hist.size() > 5 ? hist[hist.size() - 6] : z;
      hMeas = fmaxf(3.0f, zl + 3.0f);
      vzm += 0.5f * ((hMeas - hPrev) / 0.1f - vzm);
      hPrev = hMeas;
      hAt = now;
    }
    AltHoldInput in;
    in.nowMs = now;
    in.autoOn = t >= 1;
    in.pilotThrValid = t < 1;
    in.pilotThrUs = 1000;
    in.hValid = true;
    in.hAtMs = hAt;
    in.hCm = hMeas;
    in.armed = arm && t >= 2;
    AltHoldOutput o = a.step(in, dt);
    float cmd = in.autoOn ? o.thrUs : 1000;
    if (o.takeoffFail) r.fail = true;
    if (o.state == AltState::Hold) r.hold = true;
    thrAct += (cmd - thrAct) * (dt / 0.06f);
    float hov = hover120 + m(z) - m(120);
    float acc = k * (thrAct - hov) - 0.5f * vz;
    if (onGround && acc <= 0) acc = 0, vz = 0;
    else onGround = false;
    vz += acc * dt;
    z += vz * dt;
    if (z < 0) { z = 0; vz = 0; onGround = true; }
    if (!r.lift && z > 10) { r.lift = true; r.tLift = t; }
    if (r.t100 < 0 && z >= 100) r.t100 = t;
    r.maxZ = fmaxf(r.maxZ, z);
    r.finalErr = z + 3 - 120;
  }
  return r;
}

static void testTakeoff() {
  // -50: 바닥에서 AUTO → 시동(2초) → 조종자처럼 스로틀을 올려 떠오르고 120cm 까지 → 유지
  for (float k : {1.0f, 2.0f, 3.0f})
    for (float hv : {1490.0f, 1512.0f, 1540.0f}) {  // 실제 호버가 추정(1510)보다 낮거나 높을 때
      TO r = takeoffSim(k, hv, true);
      std::printf("이륙 k%.1f 호버%.0f: 떠오름 %.1f초, 100cm %.1f초, 최고 %.0fcm, 끝 오차 %+.1fcm, 유지 %d\n", k, hv,
                  r.tLift - 2, r.t100 - 2, r.maxZ, r.finalErr, r.hold);
      CHECK(r.lift && r.hold && !r.fail);
      CHECK(r.tLift - 2 < 6);
      CHECK(r.t100 > 0 && r.t100 - 2 < 15);
      CHECK(r.maxZ < 150);
      CHECK(std::fabs(r.finalErr) < 8);
    }
  // 바닥 센서가 1.0cm(최소 거리 아래)로 읽혀도 바닥으로 보고 이륙 (-51: 바닥에서 1~2cm 오락가락)
  {
    AltHold a;
    AltHoldInput in;
    in.pilotThrValid = true;
    in.pilotThrUs = 1000;
    in.hValid = true;
    in.hCm = 1.0f;
    for (uint32_t t = 0; t < 500; t += 20) {
      in.nowMs = in.hAtMs = t;
      a.step(in, 0.02f);
    }
    in.autoOn = true;
    in.pilotThrValid = false;
    in.nowMs = in.hAtMs = 520;
    AltHoldOutput o = a.step(in, 0.02f);
    CHECK(o.state == AltState::Ground && o.thrUs <= 1000.5f);
    in.armed = true;
    for (uint32_t t = 540; t < 1500; t += 20) {
      in.nowMs = in.hAtMs = t;
      o = a.step(in, 0.02f);
    }
    CHECK(o.state == AltState::Takeoff && o.thrUs > 1100);
    // 착지 뒤 시동 꺼짐(바닥) → 이륙 대기로 돌아오고, 다시 시동이면 다시 이륙
    in.armed = false;
    in.nowMs = in.hAtMs = 1520;
    o = a.step(in, 0.02f);
    CHECK(o.state == AltState::Ground && o.thrUs <= 1000.5f);
  }
  // 시동이 안 걸리면 이륙하지 않는다
  TO n = takeoffSim(2.0f, 1512, false);
  CHECK(!n.lift);
  // 너무 무거우면(호버 1650) 호버 추정 +60µs 까지 올려도 안 떠 → 중단, 바닥
  TO f = takeoffSim(2.0f, 1650, true);
  std::printf("이륙 무거움: 떠오름 %d, 중단 %d\n", f.lift, f.fail);
  CHECK(!f.lift && f.fail);
}

static void testLand() {
  for (float k : {1.5f, 3.0f, 5.0f})
    for (float lag : {0.1f, 0.25f}) {
      R l = run(k, lag, 20, 110, 10, 80, -1, 0, 0, 0, -1);
      std::printf("land k%.1f lag%.2f: 착지 %d (%.1f초), 닿는 속도 %.1fcm/s, 시동 끄기 요청 %d\n", k, lag, l.touch, l.tTouch,
                  -l.touchVz, l.disarm);
      CHECK(l.touch && l.disarm);
      CHECK(-l.touchVz < 80);
      CHECK(l.tTouch > 15 && l.tTouch < 60);
    }
  // 추력 둔감 + 착륙 시작 뒤 실제 호버가 15µs 높음 → 적분으로 결국 내려온다
  for (float k : {0.7f, 1.0f, 3.0f}) {
    R l = run(k, 0.15f, 10, 125, 10, 120, -1, 0, 15, 0, -1);
    std::printf("land 둔감 k%.1f 호버+15: 착지 %d (%.1f초), 닿는 속도 %.1fcm/s\n", k, l.touch, l.tTouch, -l.touchVz);
    CHECK(l.touch && l.disarm);
    CHECK(l.tTouch < 90);
  }
}

int main() {
  testManualTracking();
  testHoldRobust();
  testAbortPersist();
  testLand();
  testLandSlowSpikes();
  testLandCloseSensor();
  testGroundContactCut();
  testLandMeasuredGround();
  testTakeoff();
  std::fprintf(stderr, fails ? "고도 테스트 실패 %d건\n" : "고도 테스트 통과\n", fails);
  return fails ? 1 : 0;
}
