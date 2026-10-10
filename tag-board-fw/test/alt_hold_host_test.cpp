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
  bool touch = false, disarm = false, fault = false;
  float tTouch = -1;
};

static R run(float kPlant, float lagS, float sag, float z0, float tLand, float simS, float dropFrom = -1,
             float kick = 0) {
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
    hist.push_back(z);
    if (i % 5 == 0) {
      size_t lag = (size_t)(lagS / dt);
      float zl = hist.size() > lag ? hist[hist.size() - 1 - lag] : z;
      float m = fmaxf(3.0f, zl + 3.0f) + 2.0f * rnd();
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
    float acc = landed ? 0 : kPlant * (thrAct - hover) - 0.5f * vz;
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

static void testLand() {
  for (float k : {1.5f, 3.0f, 5.0f})
    for (float lag : {0.1f, 0.25f}) {
      R l = run(k, lag, 20, 110, 10, 60);
      std::printf("land k%.1f lag%.2f: 착지 %d (%.1f초), 닿는 속도 %.1fcm/s, 시동 끄기 요청 %d\n", k, lag, l.touch, l.tTouch,
                  -l.touchVz, l.disarm);
      CHECK(l.touch && l.disarm);
      CHECK(-l.touchVz < 15);
      CHECK(l.tTouch > 10 && l.tTouch < 40);
    }
}

int main() {
  testManualTracking();
  testHoldRobust();
  testLand();
  std::fprintf(stderr, fails ? "고도 테스트 실패 %d건\n" : "고도 테스트 통과\n", fails);
  return fails ? 1 : 0;
}
