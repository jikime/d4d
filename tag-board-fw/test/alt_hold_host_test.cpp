// PC에서 고도 유지·착륙(src/alt_hold.h)을 시험한다
//   clang++ -std=c++17 -O1 -I src test/alt_hold_host_test.cpp -o /tmp/alt_test && /tmp/alt_test
// 기체(수직): 가속도 = k·(모터 스로틀 − 실제 호버) − 0.5·vz. k(추력 민감도) 1.5~5 cm/s² per µs 로 바꿔 본다.
// 모터 지연 60ms, 거리센서 10Hz·지연 0.1~0.25s·잡음 ±1cm, 바닥에서 3cm 로 읽힘.
// 수동 2초(조종자가 호버 유지) → AUTO. 실제 호버 스로틀은 AUTO 뒤 20초 동안 sag µs 오른다(배터리 전압 감소).
#include <cmath>
#include <cstdio>
#include <string>
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

static void testLandSlowSpikes() {
  // -46: 5cm/s 착륙 + 거리센서 튀는 값(5%, ±20~40cm) + 추력 민감도·호버 차이. 다시 솟지 않고 천천히
  for (float k : {1.0f, 2.5f, 4.0f, 5.0f})
    for (float lag : {0.1f, 0.25f})
      for (float ls : {-8.0f, 0.0f, 8.0f}) {  // 착륙 시작 순간 실제 호버가 ±8µs 바뀌는 나쁜 경우까지
        R l = run(k, lag, 10, 115, 10, 90, -1, 0, ls, 0.05f);
        std::printf("land5 k%.1f lag%.2f 호버%+3.0f: 착지 %d (%.1f초), 닿는 속도 %.1f, 최대 하강 %.1fcm/s, 다시 오름 %.1fcm\n",
                    k, lag, ls, l.touch, l.tTouch, -l.touchVz, l.landMaxDown, l.landClimb);
        CHECK(l.touch && l.disarm);
        CHECK(-l.touchVz < 50);  // 닿은 뒤 스로틀 1000 → 1~2cm 떨어지는 속도 포함 (50cm/s = 1.3cm 낙하)
        CHECK(l.tTouch < 75);
        if (ls == 0) {  // 호버가 그대로면: 5cm/s 근처로 내려오고 다시 오르지 않는다
          CHECK(l.landMaxDown < 15);
          CHECK(l.landClimb < 2);
        } else {
          CHECK(l.landMaxDown < 36);
          CHECK(l.landClimb < 18);
        }
      }
}

static void testLandGroundEffect() {
  // -47: 지면 효과(30cm 아래에서 필요한 스로틀이 최대 50~70µs 줄어듦, -46 비행에서 18cm 에 멈춘 원인)에서도 착지
  for (float k : {1.0f, 2.0f, 3.0f})
    for (float gu : {30.0f, 50.0f, 70.0f}) {
      R l = run(k, 0.15f, 10, 105, 10, 160, -1, 0, 0, 0.05f, gu);
      std::printf("land 지면효과 k%.1f GE%.0f: 착지 %d (%.0f초), 최대 하강 %.1fcm/s, 다시 오름 %.1fcm\n", k, gu, l.touch,
                  l.tTouch - 10, l.landMaxDown, l.landClimb);
      CHECK(l.touch && l.disarm);
      CHECK(l.landMaxDown < 20);
      CHECK(l.landClimb < 8);
      CHECK(l.tTouch - 10 < 90);
    }
}

static void testLandCloseSensor() {
  // -48: 10cm 아래에서 거리센서가 1.0cm(최소 거리 아래)를 70% 확률로 냄 + 지면 효과 → 그래도 착지·시동 끄기 요청
  for (float k : {1.0f, 2.0f, 3.0f})
    for (float gu : {0.0f, 50.0f}) {
      R l = run(k, 0.15f, 10, 105, 10, 160, -1, 0, 0, 0.05f, gu, true);
      std::printf("land 센서 근접 k%.1f GE%.0f: 착지 %d (%.0f초), 최대 하강 %.1fcm/s, 다시 오름 %.1fcm\n", k, gu, l.touch,
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

static void testLand() {
  for (float k : {1.5f, 3.0f, 5.0f})
    for (float lag : {0.1f, 0.25f}) {
      R l = run(k, lag, 20, 110, 10, 60);
      std::printf("land k%.1f lag%.2f: 착지 %d (%.1f초), 닿는 속도 %.1fcm/s, 시동 끄기 요청 %d\n", k, lag, l.touch, l.tTouch,
                  -l.touchVz, l.disarm);
      CHECK(l.touch && l.disarm);
      CHECK(-l.touchVz < 50);  // 닿은 뒤 스로틀 1000 → 1~2cm 낙하 포함
      CHECK(l.tTouch > 20 && l.tTouch < 50);
    }
  // 실제 비행 재현: 추력 둔감(k 1.0) + 착륙 시작 뒤 실제 호버가 기준보다 15µs 높음 → 하강 속도 적분으로 결국 내려온다
  for (float k : {0.7f, 1.0f, 3.0f}) {
    R l = run(k, 0.15f, 10, 125, 10, 90, -1, 0, 15);
    std::printf("land 둔감 k%.1f 호버+15: 착지 %d (%.1f초), 닿는 속도 %.1fcm/s\n", k, l.touch, l.tTouch, -l.touchVz);
    CHECK(l.touch && l.disarm);
    CHECK(-l.touchVz < 20);
    CHECK(l.tTouch < 60);
  }
}

int main() {
  testManualTracking();
  testHoldRobust();
  testLand();
  testLandSlowSpikes();
  testLandGroundEffect();
  testLandCloseSensor();
  testGroundContactCut();
  std::fprintf(stderr, fails ? "고도 테스트 실패 %d건\n" : "고도 테스트 통과\n", fails);
  return fails ? 1 : 0;
}
