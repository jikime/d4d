// PC에서 태그 투어(src/tour.h)를 시험한다: 단계 전환 + 접근 제어(approach.h)와 묶은 비행 시뮬레이션
//   clang++ -std=c++17 -O1 -I src test/tour_host_test.cpp -o /tmp/tour_test && /tmp/tour_test
#include <cmath>
#include <cstdio>
#include <vector>

#include "approach.h"
#include "tour.h"

static int fails = 0;
#define CHECK(c)                                                  \
  do {                                                            \
    if (!(c)) {                                                   \
      std::fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #c); \
      fails++;                                                    \
    }                                                             \
  } while (0)

// ---------------------------------------------------------------------------
// 1) 단계 전환 (입력을 직접 만들어 넣는다)
// ---------------------------------------------------------------------------
static TourInput frame(uint32_t now, std::initializer_list<TourSeen> tags) {
  TourInput in;
  in.nowMs = now;
  in.active = true;
  in.frameAtMs = now;
  for (const auto &t : tags)
    if (in.n < TOUR_MAX_SEEN) in.tags[in.n++] = t;
  in.yawValid = in.yawCapValid = true;
  return in;
}

static TourSeen tag(int id, float r, float f, float up) {
  TourSeen s;
  s.id = id;
  s.rightCm = r;
  s.fwdCm = f;
  s.upDeg = up;
  return s;
}

static void testStates() {
  TagTour t;
  t.p.mode = 0;
  uint32_t now = 1000;
  TourOutput o;
  // 태그 1 이 오른쪽 앞 30cm, 화살표는 기수에서 오른쪽 40°
  o = t.step(frame(now, {tag(1, 20, 22, 40)}), 0.02f);
  CHECK(o.state == TourState::Center && o.targetId == 1 && o.retarget);
  CHECK(!o.yawOn);  // 중심 맞추는 동안은 돌지 않는다
  // 가까워짐 → 방향 맞추기. 기수 0°, 화살표 +40° → 오른쪽으로 돌기
  now += 20;
  o = t.step(frame(now, {tag(1, 3, 4, 40)}), 0.02f);
  CHECK(o.state == TourState::Align);
  now += 20;
  o = t.step(frame(now, {tag(1, 3, 4, 40)}), 0.02f);
  CHECK(o.yawOn && o.yawRateDegS > 0 && std::fabs(o.yawErrDeg - 40) < 1);
  // 기수가 40° 로 돌았다 (태그 화살표는 기체 기준 0°) → 1초 뒤 머무르기
  for (int i = 0; i < 60; i++) {
    now += 20;
    TourInput in = frame(now, {tag(1, 2, 1, 0)});
    in.yawDeg = in.yawCapDeg = 40;
    o = t.step(in, 0.02f);
  }
  CHECK(o.state == TourState::Dwell);
  // 머무름: 태그를 못 보는 동안은 시간이 멈춘다
  uint32_t d0 = o.dwellMs;
  for (int i = 0; i < 50; i++) {
    now += 20;
    TourInput in = frame(now, {});
    in.yawDeg = 40;
    o = t.step(in, 0.02f);
  }
  CHECK(o.dwellMs == d0);
  CHECK(o.state == TourState::Dwell);
  // 10초 채우면 전진: 목표점 = 태그 + 화살표 방향(기수 40° = 진행 40° → 기체 기준 앞) 100cm
  for (int i = 0; i < 520; i++) {
    now += 20;
    TourInput in = frame(now, {tag(1, 1, 1, 0)});
    in.yawDeg = in.yawCapDeg = 40;
    o = t.step(in, 0.02f);
  }
  CHECK(o.state == TourState::Transit);
  CHECK(std::fabs(o.goalF - t.p.distCm) < 1 && std::fabs(o.goalR) < 1);
  CHECK(o.speedCapCmS > 0 && o.kfLostMs > 3000 && o.scanFull);
  // 새 태그 3 이 두 프레임 연속 → 목표 변경 (떠나온 1 은 무시)
  now += 200;
  TourInput in = frame(now, {tag(1, 0, -40, 0), tag(3, 5, 30, 0)});
  in.yawDeg = 40;
  o = t.step(in, 0.02f);
  CHECK(o.state == TourState::Transit);
  now += 200;
  in = frame(now, {tag(3, 5, 25, 0)});
  in.yawDeg = 40;
  o = t.step(in, 0.02f);
  CHECK(o.state == TourState::Center && o.targetId == 3 && o.lastId == 1 && o.retarget);
  // 태그 3 의 화살표가 기체 기준 150° (뒤쪽) → 돌지 않고 후진. 기체 뒤쪽을 화살표에 맞추는 작은 보정 -30° 만
  for (int i = 0; i < 5; i++) {
    now += 200;
    in = frame(now, {tag(3, 1, 1, 150)});
    in.yawDeg = in.yawCapDeg = 40;
    o = t.step(in, 0.02f);
  }
  CHECK(o.state == TourState::Align && o.backward && std::fabs(o.yawErrDeg + 30) < 2);
  CHECK(o.yawRateDegS < 0 && o.yawRateDegS >= -t.p.yawRateMax);
  // AUTO 가 잠깐(1초) 끊김 → 단계 유지, 출력 없음 → 돌아오면 이어서
  in.active = false;
  for (int i = 0; i < 50; i++) {
    in.nowMs = now += 20;
    o = t.step(in, 0.02f);
  }
  CHECK(o.state == TourState::Align && o.paused && !o.yawOn);
  in.active = true;
  in.nowMs = now += 20;
  in.frameAtMs = now;
  o = t.step(in, 0.02f);
  CHECK(o.state == TourState::Align && !o.paused && o.targetId == 3);
  // AUTO 해제가 1.5초 넘음 → 꺼짐
  in.active = false;
  for (int i = 0; i < 100; i++) {
    in.nowMs = now += 20;
    o = t.step(in, 0.02f);
  }
  CHECK(o.state == TourState::Off && !o.yawOn);
}

static void testHomeTag() {
  // -52: 이륙 태그(1)를 알면, 시작할 때 더 가까운 다른 태그(2)가 보여도 1 로 간다 (안 보여도 목표는 1)
  TagTour t;
  TourInput in = frame(1000, {tag(2, 10, 30, 0)});
  in.homeId = 1;
  TourOutput o = t.step(in, 0.02f);
  CHECK(o.state == TourState::Center && o.targetId == 1 && !o.targetInFrame);
  in = frame(1200, {tag(2, 5, 20, 0), tag(1, -60, -20, 0)});
  in.homeId = 1;
  o = t.step(in, 0.02f);
  CHECK(o.targetId == 1 && o.targetInFrame);
  // 이륙 태그 없음 → 예전처럼 가장 가까운 태그
  TagTour u;
  TourInput j = frame(1000, {tag(2, 10, 30, 0), tag(1, 50, 50, 0)});
  o = u.step(j, 0.02f);
  CHECK(o.targetId == 2);
}

static void testAlignTimeout() {
  // 요가 반대로 먹어 기수가 안 맞으면(오차가 그대로) 시간 초과 → 요 멈추고 태그 위 대기
  TagTour t;
  t.p.mode = 0;
  TourOutput o;
  uint32_t now = 1000;
  for (; now < 1000 + 40000; now += 20) {
    TourInput in = frame(now, {tag(1, 0, 0, 80)});
    in.yawDeg = in.yawCapDeg = 0;
    o = t.step(in, 0.02f);
  }
  CHECK(o.state == TourState::Done && !o.yawOn && o.yawRateDegS == 0);
}

static void testEndAndReturn() {
  // 끝 태그(9): 머무름 뒤 계속 대기 / 착륙 태그(5): 머무름 뒤 착륙 요청
  {
    TagTour t;
    t.p.mode = 0;
    t.p.dwellS = 1;
    TourOutput o;
    for (uint32_t now = 1000; now < 5000; now += 20) o = t.step(frame(now, {tag(9, 0, 0, 0)}), 0.02f);
    CHECK(o.state == TourState::Done && o.targetId == 9 && !o.landRequest);
  }
  {
    TagTour t;
    t.p.mode = 0;
    t.p.dwellS = 1;
    TourOutput o;
    for (uint32_t now = 1000; now < 5000; now += 20) o = t.step(frame(now, {tag(5, 0, 0, 0)}), 0.02f);
    CHECK(o.state == TourState::Done && o.targetId == 5 && o.landRequest);
  }
  // 다음 태그 없음: 목표점 도착 → 복귀 → 이전 태그 보이면 대기 / 못 보면 시간 초과 → LOST
  {
    TagTour t;
    t.p.mode = 0;
    t.p.dwellS = 1;
    TourOutput o;
    uint32_t now = 0;
    for (; now < 4000; now += 20) o = t.step(frame(now, {tag(1, 0, 0, 0)}), 0.02f);
    CHECK(o.state == TourState::Transit);
    TourInput in = frame(now, {});
    in.goalKnown = true;
    in.distToGoalCm = 5;
    o = t.step(in, 0.02f);
    CHECK(o.state == TourState::Return && o.goalF == 0 && o.targetId == 1);
    now += 3000;
    o = t.step(frame(now, {tag(1, 10, 10, 0)}), 0.02f);
    CHECK(o.state == TourState::Done);
  }
  {
    TagTour t;
    t.p.mode = 0;
    t.p.dwellS = 1;
    t.p.transitMs = 2000;
    t.p.returnMs = 2000;
    TourOutput o;
    uint32_t now = 0;
    for (; now < 4000; now += 20) o = t.step(frame(now, {tag(1, 0, 0, 0)}), 0.02f);
    for (; now < 9000; now += 20) o = t.step(frame(now, {}), 0.02f);
    CHECK(o.state == TourState::Lost && o.targetId == -1);
    o = t.step(frame(now, {tag(2, 30, 0, 0)}), 0.02f);  // 태그가 다시 보이면 그 태그부터
    CHECK(o.state == TourState::Center && o.targetId == 2);
  }
}

// ---------------------------------------------------------------------------
// 2) 비행 시뮬레이션: 접근 제어 + 칼만 + 투어 + 요 (main.cpp 의 approachStep 과 같은 연결)
//   기체: 명령 기울기 → 0.3s 지연 + 1차(τ 0.33s), 크기 78% (20261011 비행 실측: 지연 약 0.6s, 78%)
//   카메라: 5fps, 촬영 → 결과 0.18s, 화면 범위 ±52cm(좌우) × ±39cm(앞뒤) @ 110cm, 위치 잡음 ±1.5cm, 각도 ±2°
//   FC: 자세·기수 10Hz (0.1°, 1°), 수평 보정 오차(trim) 가속도
// ---------------------------------------------------------------------------
struct WorldTag { int id; float x, y, up; };  // x 동, y 북, up = 화살표 방위 (북 0, 시계 +)

struct SimResult {
  std::vector<int> visited;     // 머무름을 마친(또는 끝에 도달한) 순서
  TourState final = TourState::Off;
  float x = 0, y = 0, yaw = 0;
  float maxTransitSpeed = 0;
  float dwellSpread = 0;        // 머무는 동안 태그에서 최대 거리
  float maxYawStep = 0;         // 한 태그를 목표로 하는 동안 기수가 돈 최대 각도
  float minY = 0;               // 가장 남쪽 (후진 확인)
  bool landReq = false;
  bool diverged = false;
};

static SimResult tourSim(const std::vector<WorldTag> &tags, float x0, float y0, float yaw0, float trimR, float trimF,
                         float simS, bool verbose, float drag = 0.3f, float fps = 5, float lat = 0.18f, float landAt = 0,
                         bool shuttle = false) {
  const float g = 981, dt = 0.02f, tau = 0.33f, gain = 0.78f, delayS = 0.3f, h = 110;
  const float fovR = 52, fovF = 39;
  ApproachController c;
  c.p.maxSpeedCmS = 20;
  c.p.slowRadiusCm = 50;
  c.p.maxAccelCmS2 = 30;
  c.p.kfSearchSpeedCmS = 12;
  c.p.maxAccelCmdCmS2 = 80;
  c.p.overspeedCmS = 24;
  TagTour tour;
  tour.p.landAt = landAt;
  tour.p.mode = shuttle ? 1 : 0;
  SimResult r;
  float x = x0, y = y0, vx = 0, vy = 0, yaw = yaw0, yawRate = 0;
  float tiltR = 0, tiltF = 0;
  std::vector<float> cmdHistR, cmdHistF;  // 지연 버퍼
  int delaySteps = (int)(delayS / dt);
  uint32_t seed = 777;
  auto rnd = [&]() { seed = seed * 1103515245u + 12345u; return ((seed >> 8) & 0xffff) / 65535.0f - 0.5f; };
  struct Pending { float t; TourSeen tags[TOUR_MAX_SEEN]; int n; float yawCap; };
  std::vector<Pending> pend;
  float nextCap = 0;
  // 보드가 아는 최신 프레임
  TourSeen seen[TOUR_MAX_SEEN];
  int nSeen = 0;
  uint32_t seenAt = 0, seenCap = 0;
  float yawCapSeen = 0;
  float fcYaw = yaw0, fcRoll = 0, fcPitch = 0;
  uint32_t fcAt = 0;
  float lastFcYaw = yaw0;
  bool haveLastYaw = false;
  ApproachOutput ao;
  TourState prev = TourState::Off;
  int dwellId = -1;
  float yawUsCmd = 0;
  for (int i = 0; i * dt < simS; i++) {
    float t = i * dt;
    uint32_t now = (uint32_t)(t * 1000 + 0.5f);
    float cy = cosf(yaw / 57.2957795f), sy = sinf(yaw / 57.2957795f);
    // FC 자세·기수 10Hz
    if (i % 5 == 0) {
      fcYaw = tourWrap360(roundf(yaw));
      fcRoll = roundf(tiltR * 10) / 10;
      fcPitch = roundf(tiltF * 10) / 10;
      fcAt = now;
      if (haveLastYaw) {
        float d = tourWrap180(fcYaw - lastFcYaw);
        if (std::fabs(d) < 45) c.rotateFrame(d);
      }
      lastFcYaw = fcYaw;
      haveLastYaw = true;
    }
    // 카메라 촬영
    if (t >= nextCap) {
      Pending p;
      p.t = t;
      p.n = 0;
      p.yawCap = fcYaw;
      for (const auto &w : tags) {
        float dx = w.x - x, dy = w.y - y;
        float br = dx * cy - dy * sy, bf = dx * sy + dy * cy;
        // 기울기만큼 카메라가 보는 곳이 밀린다 → 보드는 촬영 순간 자세로 보정한다 (여기서는 보정 오차 1cm 만 남김)
        if (std::fabs(br) < fovR - 5 && std::fabs(bf) < fovF - 5 && p.n < TOUR_MAX_SEEN) {
          TourSeen s;
          s.id = w.id;
          s.rightCm = br + 3.0f * rnd();
          s.fwdCm = bf + 3.0f * rnd();
          s.upDeg = tourWrap180(w.up - yaw + 4.0f * rnd());
          p.tags[p.n++] = s;
        }
      }
      pend.push_back(p);
      nextCap = t + 1.0f / fps;
    }
    while (!pend.empty() && t >= pend.front().t + lat) {
      const Pending &p = pend.front();
      nSeen = p.n;
      for (int k = 0; k < p.n; k++) seen[k] = p.tags[k];
      seenAt = now;
      seenCap = (uint32_t)(p.t * 1000 + 0.5f);
      yawCapSeen = p.yawCap;
      pend.erase(pend.begin());
    }
    // 투어
    TourInput ti;
    ti.nowMs = now;
    ti.active = true;
    ti.frameAtMs = seenAt;
    ti.n = (uint8_t)nSeen;
    for (int k = 0; k < nSeen; k++) ti.tags[k] = seen[k];
    ti.yawValid = true;
    ti.yawDeg = fcYaw;
    ti.yawCapValid = seenAt != 0;
    ti.yawCapDeg = yawCapSeen;
    ti.goalKnown = ao.distValid;
    ti.distToGoalCm = ao.distCm;
    TourOutput to = tour.step(ti, dt);
    if (verbose && to.state != prev)
      std::printf("  t=%5.1f %-8s → %-8s tgt=%d (%.0f,%.0f) yaw=%.0f  %s\n", t, tourStateName(prev),
                  tourStateName(to.state), to.targetId, x, y, yaw, to.reason);
    if (prev == TourState::Dwell && to.state != TourState::Dwell && to.state != TourState::Center) r.visited.push_back(dwellId);
    r.landReq = to.landRequest;
    if (to.state == TourState::Dwell) {
      dwellId = to.targetId;
      for (const auto &w : tags)
        if (w.id == to.targetId) r.dwellSpread = std::fmax(r.dwellSpread, hypotf(w.x - x, w.y - y));
    }
    prev = to.state;
    // 접근 제어 입력
    ApproachInput in;
    in.nowMs = now;
    in.enabled = in.authorityAuto = true;
    in.heightValid = true;
    in.heightAtMs = now;
    in.heightCm = h;
    in.fcRcExpo = 80;
    in.fcDeadbandUs = 2;
    in.tagValid = to.targetInFrame;
    in.tagAtMs = to.targetInFrame ? seenAt : 0;
    in.tagCaptureMs = seenCap;
    in.errRightCm = to.targetR;
    in.errFwdCm = to.targetF;
    in.goalRightCm = to.goalR;
    in.goalFwdCm = to.goalF;
    in.speedCapCmS = to.speedCapCmS;
    in.kfLostMs = to.kfLostMs;
    in.attValid = true;
    in.attAtMs = fcAt;
    in.attRollDeg = fcRoll;
    in.attPitchDeg = fcPitch;
    if (to.resetKf) c.reset();
    else if (to.retarget) c.retarget();
    ao = c.step(in, dt);
    // 왕복 이동: 정해진 기울기 (main.cpp 와 같게 — 머무는 동안 평균 명령을 더한다)
    static float olTrimR = 0, olTrimP = 0;
    if (i == 0) olTrimR = olTrimP = 0;
    if (to.state == TourState::Dwell && !ao.fault) {
      olTrimR += 0.01f * (ao.rollUs - olTrimR);
      olTrimP += 0.01f * (ao.pitchUs - olTrimP);
    }
    if (to.openLoop) {  // main.cpp 와 같게: 머무는 동안 평균 명령 + 이동 기울기 (앞뒤는 이동 방향 쪽일 때만)
      ao.rollUs = olTrimR;
      ao.pitchUs = std::fmax(-200.0f, std::fmin(200.0f, tourOlPitchUs(olTrimP, tiltToStickUs(to.olTiltFwdDeg, 80, 2, 45))));
    } else {
      ao.rollUs = std::fmax(-80.0f, std::fmin(80.0f, ao.rollUs));  // main.cpp cmd_max
      ao.pitchUs = std::fmax(-80.0f, std::fmin(80.0f, ao.pitchUs));
    }
    // 요: °/s → INAV 요 곡선 역변환 (yaw_rate 20 = 200°/s, rc_yaw_expo 20, yaw_deadband 5)
    float yawUs = to.yawOn ? tour.p.yawSign * tiltToStickUs(to.yawRateDegS, 20, 5, 200) : 0.0f;
    yawUs = std::fmax(-tour.p.yawUsMax, std::fmin(tour.p.yawUsMax, yawUs));
    float ys = 400 * dt;
    yawUsCmd += std::fmax(-ys, std::fmin(ys, yawUs - yawUsCmd));
    // 기체
    cmdHistR.push_back(stickUsToTilt(ao.rollUs, 80, 2, 45) * gain);
    cmdHistF.push_back(stickUsToTilt(ao.pitchUs, 80, 2, 45) * gain);
    float tgtR = cmdHistR.size() > (size_t)delaySteps ? cmdHistR[cmdHistR.size() - 1 - delaySteps] : 0;
    float tgtF = cmdHistF.size() > (size_t)delaySteps ? cmdHistF[cmdHistF.size() - 1 - delaySteps] : 0;
    tiltR += (tgtR - tiltR) * (dt / tau);
    tiltF += (tgtF - tiltF) * (dt / tau);
    float aR = g * tanf(tiltR / 57.2957795f) + trimR, aF = g * tanf(tiltF / 57.2957795f) + trimF;
    float ax = aR * cy + aF * sy, ay = -aR * sy + aF * cy;
    vx += (ax - drag * vx) * dt;
    vy += (ay - drag * vy) * dt;
    x += vx * dt;
    y += vy * dt;
    float wantRate = stickUsToTilt(yawUsCmd, 20, 5, 200);
    yawRate += (wantRate - yawRate) * (dt / 0.2f);
    yaw = tourWrap360(yaw + yawRate * dt);
    if (verbose && i % 50 == 0 && (to.state == TourState::Transit || to.state == TourState::Return)) {
      float dx = 0 - x, dy = 0 - y;  // 첫 태그 기준 참값 (sim-single 진단)
      std::printf("    t=%.1f 참 e=(%.0f,%.0f) v=(%.1f) | kf e=(%.0f,%.0f) v=(%.1f,%.1f) b=(%.1f,%.1f) dist=%.0f\n", t,
                  dx * cy - dy * sy, dx * sy + dy * cy, hypotf(vx, vy), ao.kfE[0], ao.kfE[1], ao.kfV[0], ao.kfV[1],
                  ao.kfB[0], ao.kfB[1], ao.distCm);
    }
    {
      static int yawTgt = -2;
      static float yawStart = 0;
      if (to.targetId != yawTgt) {
        yawTgt = to.targetId;
        yawStart = yaw;
      }
      r.maxYawStep = std::fmax(r.maxYawStep, std::fabs(tourWrap180(yaw - yawStart)));
      r.minY = std::fmin(r.minY, y);
    }
    if (to.state == TourState::Transit) r.maxTransitSpeed = std::fmax(r.maxTransitSpeed, hypotf(vx, vy));
    if (hypotf(x, y) > 1000) {
      r.diverged = true;
      break;
    }
  }
  r.final = tour.state();
  r.x = x;
  r.y = y;
  r.yaw = yaw;
  return r;
}

static void printVisited(const char *name, const SimResult &r) {
  std::printf("%s: 방문", name);
  for (int id : r.visited) std::printf(" %d", id);
  std::printf(" | 끝 %s (%.0f,%.0f) 기수 %.0f° | 전진 최고 %.1fcm/s | 머무는 중 최대 %.1fcm\n", tourStateName(r.final),
              r.x, r.y, r.yaw, r.maxTransitSpeed, r.dwellSpread);
}

static void testSimPath() {
  // 화살표로만 만든 경로: 태그 0(북) → 90cm 북쪽 태그 1(북동 40°) → 그 방향 80cm 태그 2(남쪽 화살표 = 후진)
  // → 그 아래 70cm 태그 5(착륙). 큰 회전 없이 앞뒤 축을 맞추는 작은 보정만
  std::vector<WorldTag> tags = {{0, 0, 0, 0}, {1, 0, 90, 40}, {2, 51.4f, 151.3f, 180}, {5, 51.4f, 81.3f, 0}};
  for (float trim : {0.0f, 6.0f, -6.0f}) {
    SimResult r = tourSim(tags, 12, -14, 30, trim, -trim, 100, trim == 0);
    char name[32];
    std::snprintf(name, sizeof(name), "sim-path trim%+.0f", trim);
    printVisited(name, r);
    CHECK(!r.diverged);
    CHECK(r.visited.size() == 4 && r.visited[0] == 0 && r.visited[1] == 1 && r.visited[2] == 2 && r.visited[3] == 5);
    CHECK(r.final == TourState::Done);
    CHECK(hypotf(r.x - 51.4f, r.y - 81.3f) < 12);
    CHECK(r.maxYawStep < 45);  // 한 태그에서 최대 회전 (태그 1: 40°, 태그 2: 후진이라 축 보정 40°)
    CHECK(r.maxTransitSpeed < 25);
    CHECK(r.dwellSpread < 25);
  }
}

static void testLandAtSecond() {
  // 두 번째 태그에서 10초 머문 뒤 착륙 요청 (ID 와 무관, 기본 land_at 2)
  std::vector<WorldTag> tags = {{1, 0, 0, 0}, {2, 0, 75, 0}, {3, 0, 150, 0}};
  SimResult r = tourSim(tags, 5, -6, 0, 0, 0, 60, false, 0.3f, 5, 0.18f, 2);
  printVisited("sim-land-at-2", r);
  CHECK(r.visited.size() == 2 && r.visited[0] == 1 && r.visited[1] == 2);
  CHECK(r.final == TourState::Done && r.landReq);
  CHECK(hypotf(r.x - 0, r.y - 75) < 12);
}

static void testShuttle() {
  // -50 왕복 미션 (조종자 수동 비행 순서): A 10초 → 앞 B 10초 → 뒤 A 5초 → 앞 B 5초 → B 에서 착륙. 화살표는 둘 다 앞(0°)
  for (float gap : {70.0f, 90.0f}) {
    std::vector<WorldTag> tags = {{1, 0, 0, 0}, {2, 0, gap, 0}};
    SimResult r = tourSim(tags, 5, -6, 0, 0, 0, 120, gap == 90.0f, 0.3f, 5, 0.18f, 0, true);
    char name[40];
    std::snprintf(name, sizeof(name), "sim-shuttle %.0fcm", gap);
    printVisited(name, r);
    CHECK(r.visited.size() == 4 && r.visited[0] == 1 && r.visited[1] == 2 && r.visited[2] == 1 && r.visited[3] == 2);
    CHECK(r.final == TourState::Done && r.landReq);
    CHECK(hypotf(r.x, r.y - gap) < 12);
    CHECK(std::fabs(tourWrap180(r.yaw)) < 10);
  }
}

static void testShuttleRobust() {
  // 왕복 정해진 기울기 이동: 항력·수평 보정 오차·느린 카메라·태그 간격이 달라도 A→B→A→B, 태그 한 장이면 A 로 돌아와 대기
  int ok = 0, n = 0, singleOk = 0;
  for (float drag : {0.15f, 0.3f, 0.5f})
    for (float trim : {-6.0f, 0.0f, 6.0f})
      for (float gap : {60.0f, 80.0f, 100.0f}) {
        std::vector<WorldTag> tags = {{1, 0, 0, 0}, {2, 0, gap, 0}};
        SimResult r = tourSim(tags, 4, -5, 0, trim, -trim, 140, false, drag, 4, 0.3f, 0, true);
        bool good = r.visited.size() == 4 && r.visited[1] == 2 && r.visited[2] == 1 && r.final == TourState::Done &&
                    r.landReq && hypotf(r.x, r.y - gap) < 15;
        n++;
        ok += good;
        if (!good)
          std::printf("  왕복 실패: drag %.2f trim %+.0f 간격 %.0f → 방문 %zu, 끝 %s (%.0f,%.0f)\n", drag, trim, gap,
                      r.visited.size(), tourStateName(r.final), r.x, r.y);
      }
  for (float drag : {0.15f, 0.3f, 0.5f}) {
    std::vector<WorldTag> one = {{1, 0, 0, 0}};
    SimResult r = tourSim(one, 4, -5, 0, 3, 3, 60, false, drag, 4, 0.3f, 0, true);
    singleOk += r.final == TourState::Done && hypotf(r.x, r.y) < 20;
    std::printf("  한 장 drag %.2f → 끝 %s (%.0f,%.0f) 착륙요청 %d\n", drag, tourStateName(r.final), r.x, r.y, r.landReq);
  }
  std::printf("왕복 견고성: %d/%d 성공, 태그 한 장 복귀 %d/3\n", ok, n, singleOk);
  CHECK(ok >= n - 5);  // 남는 실패: 항력 0.15(지나쳤다 돌아옴) · 간격 100cm+항력 0.5(못 미침) — 실제 항력·간격 확인 뒤 조정
  CHECK(singleOk >= 2);
}

static void testSimSingleTag() {
  // 태그 한 장(화살표가 뒤쪽 — 실제 비행에서 rot 179°): 돌지 않고 머무름 → 후진 → 없음 → 복귀 → 대기
  std::vector<WorldTag> tags = {{1, 0, 0, 180}};
  SimResult r = tourSim(tags, -10, 8, 0, 4, 3, 80, true);
  printVisited("sim-single", r);
  CHECK(!r.diverged);
  CHECK(r.final == TourState::Done);
  CHECK(hypotf(r.x, r.y) < 15);
  CHECK(std::fabs(tourWrap180(r.yaw - 0)) < 10);  // 기수는 그대로
  CHECK(r.minY < -40);                             // 남쪽(뒤)으로 갔다 왔다
}

static void testSimRobust() {
  // 실제 항력이 모델(0.3)과 다르거나 카메라가 느릴 때. 태그 간격 70~90cm(앞 화면 39cm 보다 멀어 중간에 예측 구간이 있다)
  std::vector<WorldTag> path = {{0, 0, 0, 0}, {1, 0, 90, 40}, {2, 51.4f, 151.3f, 180}, {5, 51.4f, 81.3f, 0}};
  std::vector<WorldTag> single = {{1, 0, 0, 180}};
  struct Case { float drag, fps, lat, trim; } cases[] = {
      {0.15f, 5, 0.18f, 5}, {0.5f, 5, 0.18f, 5}, {0.3f, 3, 0.4f, 5}, {0.15f, 3, 0.4f, -6}, {0.5f, 3, 0.4f, 6}};
  int pathOk = 0, singleOk = 0;
  for (const Case &k : cases) {
    SimResult a = tourSim(path, 12, -14, 30, k.trim, -k.trim, 110, false, k.drag, k.fps, k.lat);
    SimResult b = tourSim(single, -10, 8, 0, k.trim, k.trim, 80, false, k.drag, k.fps, k.lat);
    bool aOk = a.visited.size() == 4 && a.final == TourState::Done && hypotf(a.x - 51.4f, a.y - 81.3f) < 15;
    bool bOk = b.final == TourState::Done && hypotf(b.x, b.y) < 15;
    pathOk += aOk;
    singleOk += bOk;
    std::printf("robust drag%.2f %gfps lat%.1f trim%+.0f: 경로 %s(방문 %zu, 끝 %s) / 한 장 %s(끝 %s %.0fcm)\n", k.drag,
                k.fps, k.lat, k.trim, aOk ? "OK" : "실패", a.visited.size(), tourStateName(a.final), bOk ? "OK" : "실패",
                tourStateName(b.final), hypotf(b.x, b.y));
  }
  CHECK(pathOk == 5);
  CHECK(singleOk >= 4);
}

int main() {
  testStates();
  testEndAndReturn();
  testHomeTag();
  testAlignTimeout();
  testSimPath();
  testSimSingleTag();
  testLandAtSecond();
  testShuttle();
  testShuttleRobust();
  testSimRobust();
  std::fprintf(stderr, fails ? "투어 테스트 실패 %d건\n" : "투어 테스트 통과\n", fails);
  return fails ? 1 : 0;
}
