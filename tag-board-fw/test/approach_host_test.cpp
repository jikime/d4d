// 접근 제어기를 PC에서 시험한다.
//   uv run --with ziglang python -m ziglang c++ -std=c++17 -I src test/approach_host_test.cpp -o approach_test
//   (또는 g++ / clang++ 동일 옵션)
#include <cmath>
#include <cstdio>
#include <initializer_list>

#include "approach.h"

static int fails = 0;
#define CHECK(c)                                                       \
  do {                                                                 \
    if (!(c)) {                                                        \
      std::fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #c); \
      fails++;                                                         \
    }                                                                  \
  } while (0)
#define NEAR(a, b, tol) CHECK(std::fabs((a) - (b)) <= (tol))

static ApproachInput healthy(uint32_t now) {
  ApproachInput in;
  in.nowMs = now;
  in.enabled = true;
  in.authorityAuto = true;
  in.tagValid = true;
  in.tagAtMs = now;
  in.velValid = true;
  in.velAtMs = now;
  in.flowQuality = 60;
  in.heightValid = true;
  in.heightAtMs = now;
  in.heightCm = 50;
  return in;
}

static void testProfile() {
  ApproachController c;
  c.p.velSrc = 0;  // 이 시험은 광류 속도 기준 (-39 부터 기본은 칼만)
  c.p.outTilt = 0;  // 이 시험은 µs 비례 출력 기준
  NEAR(c.targetSpeed(100), 10.0f, 1e-4f);
  NEAR(c.targetSpeed(25), 10.0f, 1e-4f);
  NEAR(c.targetSpeed(15), 6.0f, 1e-4f);   // 10 × 15/25
  NEAR(c.targetSpeed(5.01f), 2.004f, 1e-3f);  // 경계 직전 2cm/s 로 진입
  NEAR(c.targetSpeed(5), 0.0f, 1e-4f);
  NEAR(c.targetSpeed(0), 0.0f, 1e-4f);
  c.p.slowRadiusCm = 40;
  c.p.stopRadiusCm = 10;
  NEAR(c.targetSpeed(20), 5.0f, 1e-4f);   // 설정값 변경 반영: 10 × 20/40
}

static void testAccelLimit() {
  ApproachController c;
  c.p.velSrc = 0;  // 이 시험은 광류 속도 기준 (-39 부터 기본은 칼만)
  c.p.outTilt = 0;  // 이 시험은 µs 비례 출력 기준
  ApproachInput in = healthy(0);
  in.errFwdCm = 100;  // 멀리 앞
  ApproachOutput o = c.step(in, 0.05f);
  // 첫 스텝: 목표 속도는 10 이지만 가속도 10cm/s² × 0.05s = 0.5cm/s 만 허용
  NEAR(o.targetSpeedCmS, 10.0f, 1e-4f);
  NEAR(o.vTargetFwdCmS, 0.5f, 1e-4f);
  NEAR(o.vTargetRightCmS, 0.0f, 1e-4f);
  // 1초 뒤 ≈ 10cm/s 에 도달 (20스텝 × 0.5)
  for (int i = 0; i < 19; i++) o = c.step(in, 0.05f);
  NEAR(o.vTargetFwdCmS, 10.0f, 1e-3f);
  for (int i = 0; i < 20; i++) o = c.step(in, 0.05f);
  NEAR(o.vTargetFwdCmS, 10.0f, 1e-3f);  // 상한 유지
  CHECK(o.state == ApproachState::Approach);
}

static void testSlewAndSign() {
  ApproachController c;
  c.p.velSrc = 0;  // 이 시험은 광류 속도 기준 (-39 부터 기본은 칼만)
  c.p.outTilt = 0;  // 이 시험은 µs 비례 출력 기준
  c.p.maxAccelCmS2 = 1000;  // 가속도 제한을 사실상 끄고 명령 변화율만 본다
  c.p.cmdSlewUsPerS = 100;
  c.p.velKpUsPerCmS = 2;
  ApproachInput in = healthy(0);
  in.errRightCm = 100;
  ApproachOutput o = c.step(in, 0.05f);
  // 목표 10cm/s, 측정 0 → raw 20µs, 변화율 100µs/s × 0.05 = 5µs
  NEAR(o.rollUs, 5.0f, 1e-4f);
  NEAR(o.pitchUs, 0.0f, 1e-4f);
  for (int i = 0; i < 10; i++) o = c.step(in, 0.05f);
  NEAR(o.rollUs, 20.0f, 1e-3f);  // raw 에 수렴하고 넘지 않는다
  c.p.rollSign = -1;
  o = c.step(in, 0.05f);
  NEAR(o.rollUs, -20.0f, 1e-3f);  // 부호 파라미터
}

static void testCmdMax() {
  ApproachController c;
  c.p.velSrc = 0;  // 이 시험은 광류 속도 기준 (-39 부터 기본은 칼만)
  c.p.outTilt = 0;  // 이 시험은 µs 비례 출력 기준
  c.p.maxAccelCmS2 = 1000;
  c.p.cmdSlewUsPerS = 10000;
  c.p.velKpUsPerCmS = 50;  // 과한 게인
  c.p.cmdMaxUs = 80;
  ApproachInput in = healthy(0);
  in.errFwdCm = 100;
  ApproachOutput o = c.step(in, 0.05f);
  NEAR(o.pitchUs, 80.0f, 1e-3f);
}

static void testFaults() {
  struct Case { const char *name; void (*mut)(ApproachInput &); const char *fault; };
  Case cases[] = {
    {"manual", [](ApproachInput &i) { i.authorityAuto = false; }, "manual_authority"},
    {"kill", [](ApproachInput &i) { i.kill = true; }, "kill"},
    {"height_low", [](ApproachInput &i) { i.heightCm = 10; }, "height"},
    {"height_high", [](ApproachInput &i) { i.heightCm = 350; }, "height"},
    {"height_stale", [](ApproachInput &i) { i.heightAtMs = i.nowMs - 1000; }, "height"},
    {"flow_quality", [](ApproachInput &i) { i.flowQuality = 10; }, "flow"},
    {"flow_stale", [](ApproachInput &i) { i.velAtMs = i.nowMs - 1000; }, "flow"},
    {"flow_invalid", [](ApproachInput &i) { i.velValid = false; }, "flow"},
    {"velocity", [](ApproachInput &i) { i.velFwdCmS = 90; }, "velocity"},
    {"mask_unknown", [](ApproachInput &i) { i.maskKnown = false; }, "mask_unknown"},
    {"mask_throttle", [](ApproachInput &i) { i.maskOk = false; }, "mask"},
  };
  for (const Case &k : cases) {
    ApproachController c;
    c.p.velSrc = 0;  // 이 시험은 광류 속도 기준 (-39 부터 기본은 칼만)
  c.p.outTilt = 0;  // 이 시험은 µs 비례 출력 기준
    c.p.maxAccelCmS2 = 1000;
    c.p.cmdSlewUsPerS = 10000;
    // 먼저 정상 접근으로 명령을 만들어 둔다
    ApproachInput in = healthy(1000);
    in.errFwdCm = 100;
    ApproachOutput o = c.step(in, 0.05f);
    CHECK(o.pitchUs > 10);
    // 고장 주입 → FAULT, 목표 속도 0, 명령이 변화율 한계로 0 으로 돌아간다
    c.p.cmdSlewUsPerS = 100;
    k.mut(in);
    o = c.step(in, 0.05f);
    if (o.state != ApproachState::Fault || !o.fault || strcmp(o.fault, k.fault) != 0) {
      std::fprintf(stderr, "FAIL fault case %s: state=%s fault=%s\n", k.name, approachStateName(o.state),
                   o.fault ? o.fault : "null");
      fails++;
    }
    NEAR(o.vTargetFwdCmS, 0.0f, 1e-4f);
    float before = o.pitchUs;
    for (int i = 0; i < 10; i++) o = c.step(in, 0.05f);
    CHECK(o.pitchUs < before);
    for (int i = 0; i < 40; i++) o = c.step(in, 0.05f);
    NEAR(o.pitchUs, 0.0f, 1e-3f);
  }
}

static void testTagLostBrakes() {
  ApproachController c;
  c.p.velSrc = 0;  // 이 시험은 광류 속도 기준 (-39 부터 기본은 칼만)
  c.p.outTilt = 0;  // 이 시험은 µs 비례 출력 기준
  c.p.lostNeutral = 0;  // 광류 제동 방식 (lost_neutral 0)
  ApproachInput in = healthy(0);
  in.errFwdCm = 100;
  ApproachOutput o;
  for (int i = 0; i < 40; i++) {
    in.nowMs += 50;
    in.tagAtMs = in.nowMs;
    in.velAtMs = in.heightAtMs = in.nowMs;
    o = c.step(in, 0.05f);
  }
  NEAR(o.vTargetFwdCmS, 10.0f, 1e-3f);
  // 태그를 1초 동안 못 봄 → HOLD. 기체가 아직 10cm/s 로 움직인다고 하면 반대 방향 명령(제동)이 나와야 한다
  in.velFwdCmS = 10;
  for (int i = 0; i < 20; i++) {
    in.nowMs += 50;
    in.velAtMs = in.heightAtMs = in.nowMs;  // tagAtMs 는 그대로 → stale
    o = c.step(in, 0.05f);
  }
  CHECK(o.state == ApproachState::Hold);
  CHECK(o.fault && strcmp(o.fault, "tag_lost") == 0);
  CHECK(o.vTargetFwdCmS < 10.0f);  // 목표 속도가 0 으로 내려가는 중
  for (int i = 0; i < 40; i++) {
    in.nowMs += 50;
    in.velAtMs = in.heightAtMs = in.nowMs;
    o = c.step(in, 0.05f);
  }
  NEAR(o.vTargetFwdCmS, 0.0f, 1e-3f);
  CHECK(o.pitchUs < -10.0f);  // 측정 속도 +10 → 오차 -10 → 제동 명령
}

// 엔지니어 시험 사양(기본값): 태그 유실 → 즉시 중립 복귀, 광류 제동 없음. 명령은 변화율 한계로 0 에 수렴
static void testTagLostNeutral() {
  ApproachController c;
  c.p.velSrc = 0;  // 이 시험은 광류 속도 기준 (-39 부터 기본은 칼만)
  c.p.outTilt = 0;  // 이 시험은 µs 비례 출력 기준
  ApproachInput in = healthy(0);
  in.errFwdCm = 100;
  ApproachOutput o;
  for (int i = 0; i < 40; i++) {
    in.nowMs += 50;
    in.tagAtMs = in.velAtMs = in.heightAtMs = in.nowMs;
    o = c.step(in, 0.05f);
  }
  CHECK(o.pitchUs > 5.0f);
  in.velFwdCmS = 10;  // 아직 10cm/s 로 움직이는 중이어도
  for (int i = 0; i < 20; i++) {
    in.nowMs += 50;
    in.velAtMs = in.heightAtMs = in.nowMs;  // tagAtMs 그대로 → stale
    o = c.step(in, 0.05f);
  }
  CHECK(o.state == ApproachState::Hold);
  CHECK(o.fault && strcmp(o.fault, "tag_lost") == 0);
  CHECK(o.neutral);
  NEAR(o.vTargetFwdCmS, 0.0f, 1e-4f);
  CHECK(o.pitchUs >= 0.0f);  // 제동(음수) 명령을 만들지 않는다
  for (int i = 0; i < 40; i++) {
    in.nowMs += 50;
    in.velAtMs = in.heightAtMs = in.nowMs;
    o = c.step(in, 0.05f);
  }
  NEAR(o.pitchUs, 0.0f, 1e-3f);
  NEAR(o.rollUs, 0.0f, 1e-3f);
}

// deadband(정지 반경) 안에 들어오면 롤·피치를 중립으로. 밖으로 나가면 다시 접근
static void testDeadbandNeutral() {
  ApproachController c;
  c.p.velSrc = 0;  // 이 시험은 광류 속도 기준 (-39 부터 기본은 칼만)
  c.p.outTilt = 0;  // 이 시험은 µs 비례 출력 기준
  c.p.cmdSlewUsPerS = 10000;
  ApproachInput in = healthy(0);
  in.errFwdCm = 4;    // 5cm 안
  in.velFwdCmS = 6;   // 아직 움직이는 중 → 속도 유지 모드라면 제동 명령이 나왔을 것
  ApproachOutput o = c.step(in, 0.05f);
  CHECK(o.state == ApproachState::Arriving);
  CHECK(o.neutral);
  NEAR(o.pitchUs, 0.0f, 1e-4f);
  NEAR(o.rollUs, 0.0f, 1e-4f);
  // 비교: deadband_neutral 0 이면 속도 0 유지를 위해 제동한다
  ApproachController c2;
  c2.p.velSrc = 0;  // 광류 기준 시험
  c2.p.deadbandNeutral = 0;
  c2.p.cmdSlewUsPerS = 10000;
  o = c2.step(in, 0.05f);
  CHECK(!o.neutral);
  CHECK(o.pitchUs < -5.0f);
  // 다시 밖으로 밀려나면 접근 재개
  in.errFwdCm = 15;
  in.velFwdCmS = 0;
  o = c.step(in, 0.05f);
  CHECK(!o.neutral);
  CHECK(o.targetSpeedCmS > 0);
}

// 실측 수평 속도가 overspeed 를 넘으면 목표 속도를 0 으로 (0.10 m/s 상한 보강)
static void testOverspeed() {
  ApproachController c;
  c.p.velSrc = 0;  // 이 시험은 광류 속도 기준 (-39 부터 기본은 칼만)
  c.p.outTilt = 0;  // 이 시험은 µs 비례 출력 기준
  c.p.maxAccelCmS2 = 1000;
  ApproachInput in = healthy(0);
  in.errFwdCm = 100;
  in.velFwdCmS = 15;  // > 12
  ApproachOutput o = c.step(in, 0.05f);
  CHECK(o.overspeed);
  NEAR(o.vTargetFwdCmS, 0.0f, 1e-4f);
  CHECK(o.pitchUs < 0);  // 제동
  in.velFwdCmS = 8;
  o = c.step(in, 0.05f);
  CHECK(!o.overspeed);
  CHECK(o.vTargetFwdCmS > 0);
}

static void testArrival() {
  ApproachController c;
  c.p.velSrc = 0;  // 이 시험은 광류 속도 기준 (-39 부터 기본은 칼만)
  c.p.outTilt = 0;  // 이 시험은 µs 비례 출력 기준
  ApproachInput in = healthy(0);
  in.errFwdCm = 3;  // 허용 범위 안
  in.velFwdCmS = 0;
  ApproachOutput o = c.step(in, 0.05f);
  CHECK(o.state == ApproachState::Arriving);
  NEAR(o.targetSpeedCmS, 0.0f, 1e-4f);  // 5cm 안 → 목표 0
  // 2초 유지 → ARRIVED
  for (int i = 0; i < 40; i++) {
    in.nowMs += 50;
    in.tagAtMs = in.velAtMs = in.heightAtMs = in.nowMs;
    o = c.step(in, 0.05f);
  }
  CHECK(o.state == ApproachState::Arrived);
  // 유지 중 속도가 허용치를 넘으면 유지 시간이 리셋된다
  ApproachController c2;
  c2.p.velSrc = 0;  // 광류 기준 시험
  ApproachInput in2 = healthy(0);
  in2.errFwdCm = 3;
  in2.velFwdCmS = 6;  // > 3cm/s
  for (int i = 0; i < 60; i++) {
    in2.nowMs += 50;
    in2.tagAtMs = in2.velAtMs = in2.heightAtMs = in2.nowMs;
    o = c2.step(in2, 0.05f);
  }
  CHECK(o.state == ApproachState::Arriving);
  CHECK(o.arriveHeldMs == 0);
  // 도착 뒤 10cm 밀려나면 다시 접근
  in.errFwdCm = 10;
  in.nowMs += 50;
  in.tagAtMs = in.velAtMs = in.heightAtMs = in.nowMs;
  o = c.step(in, 0.05f);
  CHECK(o.state == ApproachState::Arriving);
  CHECK(o.targetSpeedCmS > 0);
}

// 1차원 시뮬레이션: ANGLE 모드 근사 — 명령 µs → 가속도 (k_acc cm/s² per µs), 1차 지연 0.3s.
// 100cm 떨어진 목표로 접근해 오버슈트 없이 5cm 안에서 멈추는지 본다.
static void testSim1D() {
  ApproachController c;
  c.p.velSrc = 0;  // 이 시험은 광류 속도 기준 (-39 부터 기본은 칼만)
  c.p.outTilt = 0;  // 이 시험은 µs 비례 출력 기준
  c.p.velKpUsPerCmS = 2.0f;
  c.p.deadbandNeutral = 0;  // 마찰 없는 1D 모델에서는 속도 0 유지 방식으로 정지 특성을 본다
  const float kAcc = 1.0f;      // 1µs ≈ 0.06° ≈ 1 cm/s² (ANGLE 30°/500µs 가정)
  const float tau = 0.3f;
  float pos = 0, vel = 0, acc = 0;  // 기체 위치·속도(앞 방향)
  const float target = 100;
  float dt = 0.05f;
  ApproachInput in = healthy(0);
  float maxVel = 0, overshoot = 0;
  ApproachOutput o;
  for (int i = 0; i < 2400; i++) {  // 120초
    in.nowMs += 50;
    in.tagAtMs = in.velAtMs = in.heightAtMs = in.nowMs;
    in.errFwdCm = target - pos;
    in.velFwdCmS = vel;
    o = c.step(in, dt);
    float accCmd = o.pitchUs * kAcc;
    acc += (accCmd - acc) * (dt / tau);
    vel += acc * dt;
    pos += vel * dt;
    if (vel > maxVel) maxVel = vel;
    if (pos - target > overshoot) overshoot = pos - target;
  }
  std::printf("sim1d: pos=%.1f vel=%.2f maxVel=%.2f overshoot=%.2f state=%s\n", pos, vel, maxVel, overshoot,
              approachStateName(o.state));
  CHECK(maxVel <= 11.0f);                         // 10cm/s 상한 (약간의 추종 오차 허용)
  CHECK(overshoot <= 2.0f);                        // 목표를 2cm 넘게 지나치지 않음
  CHECK(std::fabs(target - pos) <= 5.0f);          // 정지 반경 안
  CHECK(o.state == ApproachState::Arrived);
}

static void testParams() {
  ApproachParams p;
  CHECK(approachSetParam(p, "max_speed", 7));
  NEAR(p.maxSpeedCmS, 7.0f, 1e-6f);
  CHECK(approachSetParam(p, "arrive_hold_ms", 1500));
  CHECK(p.arriveHoldMs == 1500);
  CHECK(approachSetParam(p, "flow_min_q", 300));
  CHECK(p.flowMinQuality == 255);
  CHECK(!approachSetParam(p, "nope", 1));
  char buf[1300];
  int n = approachParamsJson(p, buf, sizeof(buf));
  CHECK(n > 0 && n < (int)sizeof(buf));
  CHECK(strstr(buf, "\"max_speed\":7.0") != nullptr);
  CHECK(approachSetParam(p, "deadband_neutral", 0));
  NEAR(p.deadbandNeutral, 0.0f, 1e-6f);
  CHECK(approachSetParam(p, "handover_slew", 50));
  NEAR(p.handoverSlewUsPerS, 50.0f, 1e-6f);
  CHECK(strstr(buf, "\"lost_neutral\":1") != nullptr);
}

// 지상 시험 (vel_src 1): 기체는 서 있고 태그를 손으로 움직인다. 속도 = 카메라로 잰 상대 속도
static void testBenchTagVelocity() {
  // 카메라 속도는 광류 품질과 무관하고, 신선도는 tag_stale_ms(600) 로 본다
  {
    ApproachController ac;
    ac.p.velSrc = 0;  // -39 이전 기준 시험 (기본은 칼만)
    ac.p.outTilt = 0;  // 이 시험은 µs 비례 출력 기준
    ApproachInput in = healthy(1000);
    in.flowQuality = 0;
    in.velFromTag = true;
    in.velAtMs = 1000 - 400;  // 광류 기준(300ms)이면 끊김, 카메라 기준(600ms)이면 유효
    in.tagAtMs = 1000 - 400;
    in.errFwdCm = 40;
    ApproachOutput o = ac.step(in, 0.05f);
    CHECK(o.fault == nullptr);
    in.velFromTag = false;  // 같은 값을 광류로 보면 품질·신선도 미달
    o = ac.step(in, 0.05f);
    CHECK(o.fault && strcmp(o.fault, "flow") == 0);
  }
  // 태그를 앞 40cm 에 가만히 들고 있음 → 상대 속도 0 → 목표 10cm/s × kp 2 = +20µs 로 수렴
  ApproachController ac;
  ac.p.velSrc = 0;  // -39 이전 기준 시험 (기본은 칼만)
    ac.p.outTilt = 0;  // 이 시험은 µs 비례 출력 기준
  uint32_t t = 0;
  ApproachOutput o;
  for (int i = 0; i < 60; i++) {  // 3초 (가속 1초 + 변화율 제한)
    t += 50;
    ApproachInput in = healthy(t);
    in.velFromTag = true;
    in.flowQuality = 0;
    in.errFwdCm = 40;
    in.velFwdCmS = 0;
    o = ac.step(in, 0.05f);
  }
  NEAR(o.targetSpeedCmS, 10.0f, 1e-3f);
  NEAR(o.pitchUs, 20.0f, 0.5f);
  NEAR(o.rollUs, 0.0f, 1e-3f);
  // 태그를 20cm/s 로 가운데 쪽으로 가져옴 = 기체가 태그 쪽으로 20cm/s 로 다가가는 것과 같다 → 과속 → 제동(음수)
  for (int i = 0; i < 20; i++) {
    t += 50;
    ApproachInput in = healthy(t);
    in.velFromTag = true;
    in.errFwdCm = 30;
    in.velFwdCmS = 20;
    o = ac.step(in, 0.05f);
  }
  CHECK(o.overspeed);
  CHECK(o.pitchUs < -20.0f);
  // 파라미터 범위
  ApproachParams p;
  CHECK(approachSetParam(p, "vel_src", 2) && p.velSrc == 2);  // 2 = 칼만 (-39~)
  CHECK(!approachSetParam(p, "vel_src", 3));
  CHECK(!approachSetParam(p, "tag_vel_alpha", 0));
  CHECK(approachSetParam(p, "vel_src", 1));
  NEAR(p.velSrc, 1.0f, 1e-6f);
}

// INAV 스틱 곡선 (rc_curves.c·maths.c) 정방향·역방향
static void testInavCurve() {
  // expo 80, deadband 2: 10µs ≈ 0.145° (작은 변위는 0.2배)
  NEAR(stickUsToTilt(10, 80, 2, 45), 0.1447f, 0.002f);
  NEAR(stickUsToTilt(500, 80, 2, 45), 45.0f, 0.01f);
  NEAR(stickUsToTilt(1.5f, 80, 2, 45), 0.0f, 1e-6f);  // 데드밴드 안
  const float uss[] = {3.0f, 20.0f, 64.0f, 130.0f, 400.0f};
  for (float us : uss) {
    NEAR(tiltToStickUs(stickUsToTilt(us, 80, 2, 45), 80, 2, 45), us, 0.05f);
    if (us > 5) NEAR(tiltToStickUs(stickUsToTilt(-us, 70, 5, 30), 70, 5, 30), -us, 0.05f);  // 데드밴드 5 밖만
  }
  NEAR(tiltToStickUs(4.5f, 0, 2, 45), 2.0f + 50.0f * 498.0f / 500.0f, 0.01f);  // expo 0 이면 선형
  CHECK(tiltToStickUs(0, 80, 2, 45) == 0);
}

// 기울기 모드: 기체가 서 있고 목표가 40cm 앞 → 가속도 1.5×10 = 15cm/s² → 0.88° → expo 80 에서 약 49µs
static void testTiltOutput() {
  ApproachController c;  // 기본값 out_tilt 1
  c.p.velSrc = 0;  // 광류 기준 시험
  uint32_t t = 0;
  ApproachOutput o;
  for (int i = 0; i < 60; i++) {
    t += 50;
    ApproachInput in = healthy(t);
    in.errFwdCm = 40;
    in.fcRcExpo = 80;
    in.fcDeadbandUs = 2;
    o = c.step(in, 0.05f);
  }
  float want = tiltToStickUs(atanf(15.0f / 981.0f) * 57.2957795f, 80, 2, 45);
  NEAR(o.accelFwdCmS2, 15.0f, 0.01f);
  NEAR(o.pitchUs, want, 0.5f);
  CHECK(o.pitchUs > 40.0f && o.pitchUs < 60.0f);  // µs 비례(20µs)의 약 2.5배
  // FC expo 가 0 이면 같은 기울기에 필요한 µs 가 훨씬 작다 (FC 값을 따라간다)
  for (int i = 0; i < 20; i++) {
    t += 50;
    ApproachInput in = healthy(t);
    in.errFwdCm = 40;
    in.fcRcExpo = 0;
    in.fcDeadbandUs = 2;
    o = c.step(in, 0.05f);
  }
  NEAR(o.pitchUs, tiltToStickUs(atanf(15.0f / 981.0f) * 57.2957795f, 0, 2, 45), 0.5f);
  // 가속도 상한 50cm/s² → 약 2.9° → expo 80 에서 약 130µs
  ApproachController c2;
  c2.p.velSrc = 0;  // 광류 기준 시험
  for (int i = 0; i < 40; i++) {
    t += 50;
    ApproachInput in = healthy(t);
    in.errFwdCm = 40;
    in.velFwdCmS = -40;  // 반대로 밀려나는 중 → 큰 가속 필요
    in.fcRcExpo = 80;
    in.fcDeadbandUs = 2;
    o = c2.step(in, 0.05f);
  }
  NEAR(o.accelFwdCmS2, 50.0f, 0.01f);
  NEAR(o.pitchUs, tiltToStickUs(atanf(50.0f / 981.0f) * 57.2957795f, 80, 2, 45), 1.0f);
}

// 실제에 가까운 1차원 모델: µs → INAV 곡선(expo 80) → 목표 기울기 → 자세 1차 지연 0.33s → a = g·tanθ
// 카메라 위치는 4fps·지연 0.3s, 광류 속도는 10Hz·지연 0.1s
static void testSim1DTilt() {
  ApproachController c;
  c.p.velSrc = 0;  // 이 시험은 광류 속도 기준 (-39 부터 기본은 칼만)
  c.p.deadbandNeutral = 0;
  const float g = 981.0f, tauAtt = 0.33f, dt = 0.05f;
  float pos = 0, vel = 0, tilt = 0;
  const float target = 100;
  const int camDelay = 6, flowDelay = 2;  // 스텝 수 (0.3s, 0.1s)
  float posHist[64] = {0}, velHist[64] = {0};
  float camErr = target, flowVel = 0;
  ApproachInput in = healthy(0);
  in.fcRcExpo = 80;
  in.fcDeadbandUs = 2;
  float maxVel = 0, overshoot = 0;
  uint32_t camAt = 0;
  ApproachOutput o;
  for (int i = 0; i < 2400; i++) {
    in.nowMs += 50;
    posHist[i % 64] = pos;
    velHist[i % 64] = vel;
    if (i % 5 == 0 && i >= camDelay) {  // 4fps
      camErr = target - posHist[(i - camDelay) % 64];
      camAt = in.nowMs;
    }
    if (i % 2 == 0 && i >= flowDelay) flowVel = velHist[(i - flowDelay) % 64];  // 10Hz
    in.tagAtMs = camAt;
    in.velAtMs = in.heightAtMs = in.nowMs;
    in.errFwdCm = camErr;
    in.velFwdCmS = flowVel;
    o = c.step(in, dt);
    float tiltCmd = stickUsToTilt(o.pitchUs, 80, 2, 45);
    tilt += (tiltCmd - tilt) * (dt / tauAtt);
    vel += g * tanf(tilt / 57.2957795f) * dt;
    pos += vel * dt;
    if (vel > maxVel) maxVel = vel;
    if (pos - target > overshoot) overshoot = pos - target;
  }
  std::printf("sim1d-tilt: pos=%.1f vel=%.2f maxVel=%.2f overshoot=%.2f state=%s\n", pos, vel, maxVel, overshoot,
              approachStateName(o.state));
  CHECK(maxVel <= 11.5f);
  CHECK(overshoot <= 2.0f);
  CHECK(std::fabs(target - pos) <= 5.0f);
}


// ---- vel_src 2: 칼만 (tag_kf.h) ----
static void testKfFilter() {
  // 일정 속도 8cm/s 로 태그에서 멀어지는 중(e 증가), 5fps·지연 0.3s 측정 → 속도·현재 위치를 맞춘다
  TagKF1 k;
  float eTrue = 20, v = -8;  // 기체가 −8cm/s (태그 반대로) → e 는 +8cm/s 로 증가
  const float dt = 0.02f;
  for (int i = 0; i < 300; i++) {
    float t = i * dt;
    eTrue += -v * dt;
    k.predict(dt, 0, 25, 3);
    if (i % 10 == 0) k.update(eTrue - (-v) * 0.3f, 0.3f, 0, 1.0f, 5);  // 0.3초 전 위치
    (void)t;
  }
  NEAR(k.v, -8.0f, 1.0f);
  NEAR(k.e, eTrue, 2.0f);
  // 터무니없는 측정(다른 태그)은 버린다
  float before = k.e;
  CHECK(!k.update(before + 200, 0.3f, 0, 1.0f, 5));
  NEAR(k.e, before, 1e-3f);
}

// 1차원 기체 + 카메라(지연·잡음·화면 범위) + 바람 + 조종자 수동 구간 → AUTO
struct KfSimResult { float finalErr, maxDev; bool reacquired; bool diverged; };
static KfSimResult kfSim(float wind, float e0, float tManual, float fps, float lat, float fov, float v0Auto) {
  ApproachController c;  // 기본 vel_src 2
  const float g = 981, dt = 0.02f, tau = 0.33f;
  float pos = -e0, vel = 0, tilt = 0, pilotUs = 0, base = 0;
  bool wasAuto = false, lost = false;
  KfSimResult r = {0, 0, false, false};
  float pendT[64], pendE[64];
  int ph = 0, pt = 0;
  float nextCap = 0;
  uint32_t seed = 12345;
  auto rnd = [&]() { seed = seed * 1103515245u + 12345u; return ((seed >> 8) & 0xffff) / 65535.0f - 0.5f; };
  ApproachInput in = healthy(0);
  in.velValid = false;
  in.heightCm = 110;
  in.fcRcExpo = 80;
  in.fcDeadbandUs = 2;
  ApproachOutput o;
  for (int i = 0; i * dt < 40; i++) {
    float t = i * dt;
    uint32_t now = (uint32_t)(t * 1000);
    bool autoOn = t >= tManual;
    if (!autoOn) {
      float want = -atanf(wind / g) * 57.2957795f + 0.03f * (-e0 - pos) - 0.15f * vel;
      pilotUs = tiltToStickUs(want, 80, 2, 45);
    } else if (!wasAuto) {
      base = pilotUs;
      wasAuto = true;
      vel += v0Auto;
    }
    if (autoOn) base += fmaxf(-2.0f, fminf(2.0f, -base));  // 핸드오버 100us/s
    in.authorityAuto = autoOn;
    in.basePitchUs = autoOn ? base : 0;
    in.pilotValid = !autoOn;  // 수동 중에는 FC 가 알려 주는 조종자 스틱
    in.pilotPitchUs = pilotUs;
    if (t >= nextCap) {
      if (fabsf(pos) < fov && (ph + 1) % 64 != pt) {
        pendT[ph] = t;
        pendE[ph] = -pos + 3.0f * rnd();
        ph = (ph + 1) % 64;
      }
      nextCap = t + 1.0f / fps;
    }
    while (pt != ph && t >= pendT[pt] + lat) {
      in.tagValid = true;
      in.errFwdCm = pendE[pt];
      in.errRightCm = 0;
      in.tagCaptureMs = (uint32_t)(pendT[pt] * 1000);
      in.tagAtMs = now;
      pt = (pt + 1) % 64;
    }
    in.nowMs = in.heightAtMs = now;
    o = c.step(in, dt);
    float stick = autoOn ? base + o.pitchUs : pilotUs;
    tilt += (stickUsToTilt(stick, 80, 2, 45) - tilt) * (dt / tau);
    vel += (g * tanf(tilt / 57.2957795f) + wind) * dt;
    pos += vel * dt;
    if (autoOn) r.maxDev = fmaxf(r.maxDev, fabsf(pos));
    if (autoOn && fabsf(pos) >= fov) lost = true;
    if (lost && fabsf(pos) < fov * 0.5f) r.reacquired = true;
    if (fabsf(pos) > 300) {
      r.diverged = true;
      break;
    }
  }
  r.finalErr = -pos;
  return r;
}

static void testKfClosedLoop() {
  // 수평 보정 오차·바람 ±8cm/s² (약 0.5°), 4fps, 지연 0.3s: 조종자가 태그 위에서 4초 버틴 뒤 AUTO → 태그 위로
  for (float wind : {-8.0f, 0.0f, 8.0f}) {
    KfSimResult r = kfSim(wind, 25, 4, 4, 0.3f, 50, 0);
    std::printf("kf-sim wind%+.0f: final=%.1f maxDev=%.1f\n", wind, r.finalErr, r.maxDev);
    CHECK(!r.diverged);
    CHECK(std::fabs(r.finalErr) <= 6.0f);
    CHECK(r.maxDev <= 50.0f);
  }
}

static void testKfReacquire() {
  // AUTO 직후 태그 반대쪽으로 15cm/s 로 밀려 화면 밖(50cm)으로 나감 → 예측으로 되돌아와 다시 잡는다
  KfSimResult r = kfSim(-6, 40, 4, 4, 0.3f, 50, -15);
  std::printf("kf-reacquire: final=%.1f maxDev=%.1f reacquired=%d\n", r.finalErr, r.maxDev, r.reacquired);
  CHECK(!r.diverged);
  CHECK(r.reacquired);
  CHECK(std::fabs(r.finalErr) <= 6.0f);
}

static void testKfLostTimeout() {
  // 태그를 kf_lost_ms 넘게 못 보면 예측을 멈추고 중립 (조종자에게)
  ApproachController c;
  ApproachInput in = healthy(0);
  in.velValid = false;
  in.errRightCm = 20;
  in.errFwdCm = 0;
  in.tagCaptureMs = 0;
  ApproachOutput o = c.step(in, 0.02f);  // 첫 측정
  for (uint32_t t = 20; t <= 1000; t += 20) {
    in.nowMs = in.heightAtMs = t;
    in.tagAtMs = t;  // 계속 보임
    o = c.step(in, 0.02f);
  }
  CHECK(o.kfOn && !o.predicting);
  for (uint32_t t = 1020; t <= 1000 + 2500; t += 20) {  // 2.5초 못 봄 → 예측
    in.nowMs = in.heightAtMs = t;
    o = c.step(in, 0.02f);
  }
  CHECK(o.predicting);
  CHECK(o.fault == nullptr);
  for (uint32_t t = 3520; t <= 1000 + 4000; t += 20) {  // 3초 넘음 → 중립
    in.nowMs = in.heightAtMs = t;
    o = c.step(in, 0.02f);
  }
  CHECK(!o.kfOn);
  CHECK(o.fault != nullptr);
}

int main() {
  testKfFilter();
  testKfClosedLoop();
  testKfReacquire();
  testKfLostTimeout();
  testProfile();
  testBenchTagVelocity();
  testInavCurve();
  testTiltOutput();
  testSim1DTilt();
  testAccelLimit();
  testSlewAndSign();
  testCmdMax();
  testFaults();
  testTagLostBrakes();
  testTagLostNeutral();
  testDeadbandNeutral();
  testOverspeed();
  testArrival();
  testSim1D();
  testParams();
  if (fails) {
    std::fprintf(stderr, "%d failures\n", fails);
    return 1;
  }
  std::puts("접근 제어 테스트 통과");
  return 0;
}
