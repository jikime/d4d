#pragma once
// 태그 상대 위치·기체 속도·외란 가속도 추정 — 축(오른쪽·앞)마다 3상태 칼만 필터. 하드웨어 의존 없음
// (test/approach_host_test.cpp 에서 PC로 시험).
//
// 광류 없이 카메라 하나로 태그 쪽으로 가려면 "기체가 지금 얼마나 빨리 움직이는지"가 필요하다. 카메라 위치를 미분하면
// 3~5fps·지연 0.3s 에서 잡음과 지연이 커서 제어가 흔들린다(20261011 시뮬레이션: 3fps·지연 0.5s 에서 지나침 15cm 이상).
// 정밀 착륙 구현들과 같은 구조로 푼다:
//   PX4 Vision Target Estimator — 상태 [상대 위치, 기체 속도, 바이어스], 가속도를 입력으로 예측, 지연 측정은 이력으로 보정
//   ArduPilot AC_PrecLand        — 칼만 추정 + 카메라 지연(PLND_LAG) 보정, 표적을 놓치면 예측으로 잠시 유지
//   Borowczyk et al. 2017        — AprilTag + 칼만, 비동기 갱신, 태그를 놓치면 예측만으로 진행, 상대 위치 PD
//
// 상태 x = [e, v, b]
//   e: 태그(목표)의 기체 기준 위치 (cm, + = 오른쪽/앞)    de/dt = −v
//   v: 기체 속도 (cm/s, + = 오른쪽/앞)                     dv/dt = u + b − kd·v
//   b: 모델에 없는 가속도 — 바람·트림·자세 오차 (cm/s²)    db/dt = 잡음 (천천히 변함)
//   u: 보드가 명령한 기울기로 낸 가속도 g·tan(θ) (FC 가 보드 값을 쓸 때만. 조종기 수동이면 모름 → 0, 잡음 크게)
//   kd: 공기 저항(로터 항력) 1/s — 속도에 비례해 줄어든다. 태그 사이를 예측만으로 갈 때(투어) 이게 없으면 속도를
//       크게 잡아 실제보다 멀리 갔다고 믿는다 (시뮬레이션: 100cm 라고 믿을 때 실제 67cm)
// 측정: 카메라가 L 초 전에 찍은 사진의 태그 위치 z = e(t−L) ≈ e + v·L − ½(u+b−kd·v)·L²
#include <math.h>

struct TagKF1 {
  float e = 0, v = 0, b = 0;
  float drag = 0;  // kd (1/s)
  float P[3][3] = {};
  bool init = false;
  int rejects = 0;  // 연속으로 버린 측정 수 (많으면 다시 시작)

  void reset() {
    init = false;
    e = v = b = 0;
    rejects = 0;
  }

  void start(float z, float sigmaE) {
    e = z;
    v = 0;
    b = 0;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) P[i][j] = 0;
    P[0][0] = sigmaE * sigmaE;
    P[1][1] = 20.0f * 20.0f;  // 처음 속도는 모름 (±20cm/s)
    P[2][2] = 20.0f * 20.0f;  // 외란 가속도 ±20cm/s²
    init = true;
    rejects = 0;
  }

  // 목표 태그가 바뀜 (투어: 다음 태그) — 위치만 새 측정으로, 속도·외란은 그대로 (기체 운동은 연속이다)
  void rebase(float z, float L, float u, float sigmaE) {
    if (!init) {
      start(z, sigmaE);
      return;
    }
    e = z - v * L + 0.5f * (u + b - drag * v) * L * L;
    P[0][0] = sigmaE * sigmaE + P[1][1] * L * L;
    P[0][1] = P[1][0] = P[0][2] = P[2][0] = 0;
    rejects = 0;
  }

  // dt 초 앞으로. u: 아는 가속도(cm/s²), qa: 모르는 가속도 잡음(cm/s², 표준편차), qb: b 의 변화 (cm/s² /√s)
  void predict(float dt, float u, float qa, float qb) {
    if (!init || dt <= 0) return;
    float a = u + b - drag * v;
    e += -v * dt - 0.5f * a * dt * dt;
    v += a * dt;
    // P = F P Fᵀ + Q,  F = [[1, −dt + ½kd·dt², −½dt²], [0, 1 − kd·dt, dt], [0, 0, 1]]
    float F[3][3] = {{1, -dt + 0.5f * drag * dt * dt, -0.5f * dt * dt}, {0, 1 - drag * dt, dt}, {0, 0, 1}};
    float T[3][3], N[3][3];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) {
        T[i][j] = 0;
        for (int k = 0; k < 3; k++) T[i][j] += F[i][k] * P[k][j];
      }
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) {
        N[i][j] = 0;
        for (int k = 0; k < 3; k++) N[i][j] += T[i][k] * F[j][k];
      }
    // 모르는 가속도는 [−½dt², dt, 0] 방향으로 들어간다
    float g[3] = {-0.5f * dt * dt, dt, 0};
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) P[i][j] = N[i][j] + qa * qa * g[i] * g[j];
    P[2][2] += qb * qb * dt;
  }

  // 측정 z (L 초 전 위치), 측정 잡음 r (cm 표준편차). 혁신이 gate·σ 를 넘으면 버리고 false.
  bool update(float z, float L, float u, float r, float gate) {
    if (!init) {
      start(z, r);
      return true;
    }
    float H[3] = {1, L + 0.5f * drag * L * L, -0.5f * L * L};
    float zHat = e + v * L - 0.5f * (u + b - drag * v) * L * L;
    float y = z - zHat;
    float PH[3];
    for (int i = 0; i < 3; i++) PH[i] = P[i][0] * H[0] + P[i][1] * H[1] + P[i][2] * H[2];
    float S = H[0] * PH[0] + H[1] * PH[1] + H[2] * PH[2] + r * r;
    if (S <= 0) return false;
    if (y * y > gate * gate * S) {
      if (++rejects >= 3) start(z, r);  // 계속 어긋나면(다른 태그·큰 외란) 측정으로 다시 시작
      return false;
    }
    rejects = 0;
    float K[3] = {PH[0] / S, PH[1] / S, PH[2] / S};
    e += K[0] * y;
    v += K[1] * y;
    b += K[2] * y;
    // P = (I − K H) P
    float HP[3];
    for (int j = 0; j < 3; j++) HP[j] = H[0] * P[0][j] + H[1] * P[1][j] + H[2] * P[2][j];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) P[i][j] -= K[i] * HP[j];
    // 대칭 유지
    for (int i = 0; i < 3; i++)
      for (int j = i + 1; j < 3; j++) P[i][j] = P[j][i] = 0.5f * (P[i][j] + P[j][i]);
    return true;
  }
};
