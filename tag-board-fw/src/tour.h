#pragma once
// 태그 투어 (-41~): 태그 위로 → 머무름 → 태그 화살표 방향으로 이동(앞이면 전진, 뒤면 후진) → 다음 태그 → 반복.
//
// AprilTag 한 장이 주는 정보 세 가지를 쓴다.
//   ID   — 무엇을 할지 (TOUR_ACTIONS: 화살표 따라 이동·착륙·끝)
//   방향 — 어디로 갈지 (인쇄물 위쪽 = 진행 방향. 바닥에 놓는 방향으로 경로를 만든다)
//   위치 — 어디에 있는지 (approach.h 가 그 위로 데려간다)
//
// 기수(요)는 크게 돌리지 않는다 (-43, 사용자 요청): 화살표가 앞쪽 반이면 그 방향, 뒤쪽 반이면 반대 방향에 기체 앞뒤 축을
// 맞추는 작은 보정만 한다(최대 90°, 8°/s 이하). 이동은 기체가 옆·뒤로도 갈 수 있으니 회전 없이 화살표 방향으로 한다.
// 하드웨어 의존 없음 (test/tour_host_test.cpp 에서 PC로 시험).
//
// 각도는 모두 위에서 볼 때 시계방향 +. FC 기수 방향(MSP_ATTITUDE yaw, 0~359)도 시계방향 +.
// 기체 기준 벡터는 (오른쪽, 앞) cm.
//
// 태그 사이를 갈 때는 광류를 쓸 수 없어(품질 21~42) 칼만 예측(FC 자세 → 가속도 적분)으로 간다. 오래 갈수록 오차가
// 커지므로 거리(dist)·시간(transit_ms)에 한도를 두고, 다음 태그를 못 찾으면 떠난 태그로 되돌아가 머문다.
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

enum class TourState : uint8_t { Off, Acquire, Center, Align, Dwell, Transit, Return, Done, Lost };

inline const char *tourStateName(TourState s) {
  switch (s) {
    case TourState::Off: return "OFF";
    case TourState::Acquire: return "ACQUIRE";
    case TourState::Center: return "CENTER";
    case TourState::Align: return "ALIGN";
    case TourState::Dwell: return "DWELL";
    case TourState::Transit: return "TRANSIT";
    case TourState::Return: return "RETURN";
    case TourState::Done: return "DONE";
    default: return "LOST";
  }
}

inline const char *tourStateKo(TourState s) {
  switch (s) {
    case TourState::Off: return "꺼짐";
    case TourState::Acquire: return "태그 찾는 중";
    case TourState::Center: return "태그 위로 이동";
    case TourState::Align: return "기수 방향 맞추는 중";
    case TourState::Dwell: return "태그 위에 머무는 중";
    case TourState::Transit: return "다음 태그로 전진";
    case TourState::Return: return "다음 태그 없음 → 이전 태그로 복귀";
    case TourState::Done: return "끝 — 태그 위에서 대기";
    default: return "태그 놓침 — 조종자 확인";
  }
}

// ---- 태그 ID → 행동 ----
enum TourAct : uint8_t { TOUR_ACT_NEXT = 0, TOUR_ACT_END = 1, TOUR_ACT_LAND = 2 };
struct TourActionDef {
  int id;
  uint8_t act;
  int16_t turnDeg;  // 진행 방향 = 태그 화살표 + turnDeg (시계 +)
  uint16_t dwellS;  // 0 = 기본 머무름 (dwell_s)
  const char *code;
  const char *ko;
};
// -43: 꺾기(2·3·4)는 없앴다 — 방향은 태그를 놓는 방향으로 정한다 (사용자: 큰 회전 대신 화살표 방향으로 이동)
static const TourActionDef TOUR_ACTIONS[] = {
    {5, TOUR_ACT_LAND, 0, 0, "LAND", "착륙: 거리센서로 천천히 내려앉고 모터 끔"},
    {9, TOUR_ACT_END, 0, 0, "END", "끝: 이 태그 위에서 계속 머무름 (착륙은 조종자)"},
};
static const TourActionDef TOUR_ACTION_DEFAULT = {-1, TOUR_ACT_NEXT, 0, 0, "FOLLOW", "화살표 방향으로 이동 (앞이면 전진, 뒤면 후진)"};

// 왕복 미션 (-50, mode 1): 조종자 수동 비행(20261011)과 같은 순서. 태그 화살표와 관계없이 처음 태그에서의 기수 방향 기준
// 앞(+1)·뒤(−1). 머무름 시간은 단계마다. next 0 = 이 태그에서 착륙
struct TourLeg {
  uint16_t dwellS;
  int8_t next;
};
// -53 사용자: "태그를 인식하고 조금 후 바로 앞으로 → 뒤로 와서 태그에 머물다가 → 다시 앞으로 → 잠시 태그 확인 후 천천히 착륙"
static const TourLeg TOUR_SHUTTLE[] = {{3, +1}, {3, -1}, {5, +1}, {3, 0}};
static const int TOUR_SHUTTLE_N = sizeof(TOUR_SHUTTLE) / sizeof(TOUR_SHUTTLE[0]);

inline const TourActionDef &tourActionFor(int id) {
  for (const auto &a : TOUR_ACTIONS)
    if (a.id == id) return a;
  return TOUR_ACTION_DEFAULT;
}

inline float tourWrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a <= -180.0f) a += 360.0f;
  return a;
}
inline float tourWrap360(float a) {
  while (a >= 360.0f) a -= 360.0f;
  while (a < 0.0f) a += 360.0f;
  return a;
}

struct TourParams {
  float dwellS = 10;              // 태그 위에 머무는 시간 (태그가 보이고 dwell_radius 안일 때만 센다)
  float alignRadiusCm = 15;       // 태그 중심에서 이 안이면 기수 방향 맞추기 시작
  float alignExitCm = 35;         // 맞추는 중·머무는 중 이보다 밀리면 다시 중심부터
  float dwellRadiusCm = 25;
  float yawTolDeg = 5;            // 기수 방향 허용 오차
  uint32_t yawHoldMs = 1000;      // 허용 안에 이만큼 있으면 맞춤 완료
  // 이 안에 못 맞추면 요를 멈추고 태그 위에서 대기 (요 방향이 반대면 180° 어긋난 채 멈춘다 → yaw_sign 확인)
  uint32_t alignTimeoutMs = 25000;
  float yawKp = 0.5f;             // 방향 오차 1° 당 회전 속도 (°/s). 10° 어긋나면 5°/s
  float yawRateMax = 8;           // 회전 속도 상한 (°/s). -42 의 20°/s 는 "확 돈다"는 의견 → 천천히
  float yawDeadDeg = 2.0f;        // 이 안이면 돌지 않는다 (흔들림 방지)
  float reverse = 1;              // 1: 화살표가 뒤쪽 반이면 기체를 돌리지 않고 후진. 0: 항상 화살표 쪽으로 기수를 돌림
  float landAt = 2;               // (mode 0) N 번째 태그에서 머무름이 끝나면 착륙 (ID 와 무관). 0 = ID 5 에서만
  float mode = 1;                 // 0: 태그 화살표 따라가기, 1: 왕복 미션 (TOUR_SHUTTLE: A 10초 → 앞 B 10초 → 뒤 A 5초 → 앞 B 5초 → 착륙)
  float distCm = 80;              // 다음 태그를 찾아가는 거리. 앞쪽 화면(120cm 에서 약 40cm)까지 더해 약 120cm 간격까지 찾는다
                                  // (조종자 비행: 태그 간격 약 70~90cm 로 추정. 100cm 는 예측만으로 가는 구간이 길어 복귀가 불안)
  // 왕복 이동 (-53): 조종자처럼 정해진 기울기로 (기울기 1~4°, 1.5~4초). -52 비행: 칼만 예측 이동은 "이미 움직인다"고 잘못
  // 잡아 기울기 명령이 평균 +0.05° → 거의 안 움직였다. 가속 → 유지 → (다음 태그 보이면) 반대로 제동 → 태그 위 제어
  // -54: -53 비행에서 뒤로 갈 때 유지 기울기 −0.6° 가 수평 유지 보정(+34µs ≈ +0.6°)과 상쇄돼 출력 1499 → 5초에 45cm.
  // 보정은 이동 방향 쪽일 때만 더하고(tourOlPitchUs), 유지 0.8°·제동 2.5° 로. 가속 2.0°·유지 1.0° 는 PC 시험에서
  // 항력이 작으면 B 를 지나쳐 방문이 늘어 1.5°·0.8° (이동 최고 26~29cm/s, 조종자 25~40cm/s)
  float olAccelDeg = 1.5f;        // 가속 기울기
  uint32_t olAccelMs = 800;
  float olCruiseDeg = 0.8f;       // 유지 기울기
  uint32_t olMaxMs = 5000;        // 이 안에 다음 태그가 안 보이면 제동 뒤 되돌아감 (되돌아가기 한도는 2배)
  float olBrakeDeg = 2.5f;        // 다음 태그를 보면 반대로 이만큼
  uint32_t olBrakeMs = 600;
  uint32_t stuckLandMs = 10000;   // 왕복: 다음 태그를 못 찾아 대기(DONE)한 지 이만큼 지나면 그 태그에서 착륙
  float speedCmS = 22;            // 태그 사이 이동 속도 상한 (조종자 실측 25~40cm/s. 25 는 예측 복귀 시험에서 불안 → 22, -50)
  uint32_t transitMs = 20000;     // 이동 시간 한도 → 넘으면 복귀 (-42 비행: 12초에 45cm 만 가고 시간 초과)
  uint32_t returnMs = 20000;      // 복귀 시간 한도 → 넘으면 LOST (보드는 중립, 조종자)
  float goalTolCm = 10;           // 목표점 도착 판정
  uint8_t newTagFrames = 2;       // 새 태그를 연속 이만큼 봐야 인정 (오인식 방지)
  float newTagConeDeg = 45;       // 다음 태그는 진행 방향 ± 이 각도 안에 있어야 인정 (옆·뒤 태그로 새지 않게)
  float headingAlpha = 0.3f;      // 진행 방향 측정 평활 (태그 각도 잡음 ±1~2°)
  uint32_t frameStaleMs = 600;    // 이보다 오래된 인식 프레임은 "안 보임"
  uint32_t homeWaitMs = 10000;    // 처음 시작할 때 이륙 태그(home)로 돌아가기를 기다리는 시간. 넘으면 가장 가까운 태그
  uint32_t targetLostMs = 3000;   // 머무는 중 목표 태그를 이만큼 못 보면 다시 찾기
  // AUTO·FC 응답이 잠깐 끊겨도 이 안에 돌아오면 단계를 이어 간다 (20261011 비행: FC 응답이 한순간 끊겨 처음부터 다시 함)
  uint32_t pauseMs = 1500;
  // FC 요 곡선을 못 읽었을 때 쓰는 값 (INAV 기본: yaw_rate 20 = 200°/s, rc_yaw_expo 20, yaw_deadband 5)
  float yawSign = 1;              // + 명령 = 기수 오른쪽(시계). 반대로 돌면 -1
  float yawUsMax = 120;           // 요 스틱 오프셋 상한 (±µs)
};

// 한 프레임에서 본 태그 (기하 계산이 된 미션 태그만)
struct TourSeen {
  int id = -1;
  float rightCm = 0, fwdCm = 0;  // 기체 기준 태그 위치
  float upDeg = 0;               // 태그 화살표(인쇄물 위쪽)의 기체 기준 방향 (시계 +, 0 = 기수 방향)
};

static const int TOUR_MAX_SEEN = 4;

struct TourInput {
  uint32_t nowMs = 0;
  bool active = false;            // tour on + approach on/shadow + CH6 AUTO + 킬 아님
  uint32_t frameAtMs = 0;         // 마지막 인식 프레임 시각 (태그가 없어도 갱신)
  uint8_t n = 0;
  TourSeen tags[TOUR_MAX_SEEN];
  bool yawValid = false;          // FC 기수 방향 (지금)
  float yawDeg = 0;
  bool yawCapValid = false;       // 그 프레임을 찍은 순간의 FC 기수 방향
  float yawCapDeg = 0;
  bool goalKnown = false;         // approach 가 직전 step 에 목표점 거리를 계산했다
  int homeId = -1;                // 이륙 때 처음 본 태그 (-51 비행: 이륙 중 40~90cm 밀려 가장 가까운 다른 태그를 A 로 잡음)
  float distToGoalCm = 0;
};

struct TourOutput {
  TourState state = TourState::Off;
  const char *reason = "";
  bool paused = false;            // AUTO 가 잠깐 끊김 (pause_ms 안) — 출력 없이 단계 유지
  int targetId = -1;              // 따라갈 태그
  int lastId = -1;                // 떠나온 태그
  bool targetInFrame = false;     // 최근 프레임에 목표 태그가 있다 → targetR/F 를 측정으로 쓴다
  float targetR = 0, targetF = 0;
  float goalR = 0, goalF = 0;     // 목표점 = 목표 태그 + 이 오프셋 (기체 기준)
  bool retarget = false;          // 이번 step 에 목표 태그가 바뀜 → 칼만 위치를 새 태그로 (속도는 유지)
  bool resetKf = false;           // 정해진 기울기 이동 뒤 새 태그 → 칼만을 처음부터 (예측 속도는 믿을 수 없다)
  bool openLoop = false;          // 정해진 기울기로 이동·제동 중 (접근 제어 대신)
  float olTiltFwdDeg = 0;         // 그 앞 기울기 (+ = 앞으로)
  bool yawOn = false;             // 요를 보드가 맡는다
  float yawRateDegS = 0;          // + = 시계(오른쪽)
  float yawErrDeg = 0;
  bool travelKnown = false;
  float travelDeg = 0;            // 진행 방향 = 태그 화살표 (FC 기수 기준 절대각, 모르면 상대각)
  bool backward = false;          // 화살표가 뒤쪽 → 기수는 반대 방향에 맞추고 후진
  bool landRequest = false;       // 착륙 태그에서 머무름이 끝남 → 착륙 제어가 받아서 내린다 (스로틀은 main.cpp)
  uint32_t visited = 0;           // 머무름을 마친 태그 수 (이번 AUTO)
  uint32_t dwellMs = 0, dwellNeedMs = 0;
  uint32_t elapsedMs = 0;         // 지금 단계에 들어온 뒤 시간
  uint32_t kfLostMs = 0;          // 칼만 예측 허용 시간 (0 = approach 기본)
  float speedCapCmS = 0;          // 0 = approach 기본
  int focusId = -1;               // 카메라 ROI 를 이 태그에 (-1 = 가장 크게 보이는 태그)
  bool scanFull = false;          // 새 태그 찾기 → 전체 화면 인식을 섞는다
  const TourActionDef *act = &TOUR_ACTION_DEFAULT;
};

class TagTour {
 public:
  TourParams p;

  void reset() {
    st_ = TourState::Off;
    reason_ = "";
    targetId_ = lastId_ = -1;
    act_ = &TOUR_ACTION_DEFAULT;
    travelAbs_ = travelRel_ = false;
    travelDeg_ = relDeg_ = departRelDeg_ = 0;
    backward_ = backwardKnown_ = false;
    yawOkMs_ = dwellMs_ = 0;
    yawFault_ = false;
    inactiveSinceMs_ = 0;
    visited_ = 0;
    landHere_ = false;
    legSign_ = 1;
    missionHeadingKnown_ = false;
    seekingHome_ = false;
    brakeUntilMs_ = 0;
    candId_ = -1;
    candFrames_ = 0;
    lastFrameAt_ = 0;
  }

  TourState state() const { return st_; }

  TourOutput step(const TourInput &in, float dtS) {
    TourOutput o;
    uint32_t dtMs = (uint32_t)(dtS * 1000.0f + 0.5f);
    if (!in.active) {
      // AUTO 를 pause_ms 넘게 벗어나면 처음부터 (다시 AUTO 가 되면 가장 가까운 태그부터). 그 안이면 단계 유지
      if (st_ != TourState::Off) {
        if (!inactiveSinceMs_) inactiveSinceMs_ = in.nowMs ? in.nowMs : 1;
        if ((uint32_t)(in.nowMs - inactiveSinceMs_) <= p.pauseMs) {
          fill(o, in, nullptr);
          o.paused = true;
          o.yawOn = false;
          o.yawRateDegS = 0;
          return o;
        }
        int last = targetId_ >= 0 ? targetId_ : lastId_;
        reset();
        lastId_ = last;
      }
      fill(o, in, nullptr);
      return o;
    }
    inactiveSinceMs_ = 0;
    if (st_ == TourState::Off) {
      enter(TourState::Acquire, in.nowMs, "AUTO: 이륙 태그(없으면 가장 가까운 태그)로");
      startMs_ = in.nowMs;
    }
    bool frameFresh = in.frameAtMs && (uint32_t)(in.nowMs - in.frameAtMs) <= p.frameStaleMs;
    bool newFrame = in.frameAtMs && in.frameAtMs != lastFrameAt_;
    if (newFrame) lastFrameAt_ = in.frameAtMs;
    const TourSeen *tgt = frameFresh ? find(in, targetId_) : nullptr;
    if (tgt) {
      targetSeenMs_ = in.nowMs;
      seekingHome_ = false;
    }
    uint32_t elapsed = in.nowMs - enteredMs_;

    switch (st_) {
      case TourState::Acquire:
      case TourState::Lost: {
        // 미션 시작 전(아직 머문 태그 없음)이고 이륙 태그를 알면, 안 보여도 그 태그를 목표로 (칼만 예측으로 돌아간다)
        if (visited_ == 0 && in.homeId >= 0 && (uint32_t)(in.nowMs - startMs_) <= p.homeWaitMs) {
          const TourSeen *hm = frameFresh ? find(in, in.homeId) : nullptr;
          setTarget(in.homeId, o, in.nowMs);
          tgt = hm;
          seekingHome_ = !hm;
          enter(TourState::Center, in.nowMs, hm ? "이륙 태그 발견: 중심으로" : "이륙 태그로 돌아가는 중");
          break;
        }
        const TourSeen *n = frameFresh ? nearest(in, -1) : nullptr;
        if (n) {
          setTarget(n->id, o, in.nowMs);
          tgt = n;
          enter(TourState::Center, in.nowMs, "태그 발견: 중심으로");
        }
        break;
      }
      case TourState::Center:
        if (tgt && dist(tgt) <= p.alignRadiusCm) {
          yawOkMs_ = 0;
          enter(TourState::Align, in.nowMs, "중심 도착: 화살표 방향으로 돌기");
        }
        break;
      case TourState::Align: {
        updateTravel(in, tgt, newFrame);
        if (tgt && dist(tgt) > p.alignExitCm) {
          enter(TourState::Center, in.nowMs, "중심에서 밀림: 다시 중심으로");
          break;
        }
        if (elapsed > p.alignTimeoutMs) {
          yawFault_ = true;
          enter(TourState::Done, in.nowMs, "기수 맞추기 시간 초과: 요 멈추고 대기 — yaw_sign·FC 기수 확인");
          break;
        }
        float err = yawError(in, tgt);
        if ((travelAbs_ || travelRel_) && fabsf(err) <= p.yawTolDeg) {
          yawOkMs_ += dtMs;
          if (yawOkMs_ >= p.yawHoldMs) {
            dwellMs_ = 0;
            if (!missionHeadingKnown_ && in.yawValid) {  // 왕복 미션의 앞 방향 = 처음 태그에서 맞춘 기수
              missionHeading_ = in.yawDeg;
              missionHeadingKnown_ = true;
            }
            enter(TourState::Dwell, in.nowMs, "방향 맞춤 완료: 머무르기");
          }
        } else {
          yawOkMs_ = 0;
        }
        break;
      }
      case TourState::Dwell:
        updateTravel(in, tgt, newFrame);
        if (tgt && dist(tgt) > p.alignExitCm) {
          enter(TourState::Center, in.nowMs, "머무는 중 밀림: 다시 중심으로");
          break;
        }
        if (tgt && dist(tgt) <= p.dwellRadiusCm) dwellMs_ += dtMs;  // 태그가 보이고 가까울 때만 센다
        if (dwellMs_ >= dwellNeedMs()) {
          visited_++;  // 이 태그에서 머무름을 마쳤다
          if (p.mode == 1) {
            // 왕복 미션: 다음 단계 방향 (앞/뒤) 또는 착륙
            int8_t next = visited_ <= (uint32_t)TOUR_SHUTTLE_N ? TOUR_SHUTTLE[visited_ - 1].next : 0;
            if (next == 0) {
              landHere_ = true;
              enter(TourState::Done, in.nowMs, "미션 마지막 태그: 머무름 끝 → 착륙 요청");
            } else {
              legSign_ = next;
              lastId_ = targetId_;
              candId_ = -1;
              candFrames_ = 0;
              enter(TourState::Transit, in.nowMs, next > 0 ? "머무름 끝: 앞으로 이동" : "머무름 끝: 뒤로 이동");
            }
            break;
          }
          if (act_->act == TOUR_ACT_END) {
            enter(TourState::Done, in.nowMs, "끝 태그: 계속 머무름");
          } else if (act_->act == TOUR_ACT_LAND) {
            enter(TourState::Done, in.nowMs, "착륙 태그: 머무름 끝 → 착륙 요청");
          } else if (p.landAt >= 1 && visited_ >= (uint32_t)p.landAt) {
            landHere_ = true;
            enter(TourState::Done, in.nowMs, "정한 순서의 태그: 머무름 끝 → 착륙 요청");
          } else {
            lastId_ = targetId_;
            departRelDeg_ = relDeg_;
            candId_ = -1;
            candFrames_ = 0;
            enter(TourState::Transit, in.nowMs, "머무름 끝: 화살표 방향으로 전진");
          }
        }
        break;
      case TourState::Transit:
      case TourState::Return: {
        const TourSeen *nw = newTag(in, newFrame, frameFresh, travelRelNow(in));
        if (nw) {
          if (p.mode == 1) {  // 정해진 기울기로 오다가 새 태그 → 반대로 제동하고, 칼만은 처음부터
            brakeUntilMs_ = in.nowMs + p.olBrakeMs;
            brakeSign_ = st_ == TourState::Transit ? legSign_ : (int8_t)-legSign_;
            o.resetKf = true;
          }
          setTarget(nw->id, o, in.nowMs);
          tgt = nw;
          enter(TourState::Center, in.nowMs, "다음 태그 발견: 중심으로");
          break;
        }
        if (p.mode == 1) {
          // 왕복: 시간으로 끝낸다 (칼만 거리는 믿을 수 없다). 이동 시간 초과 → 제동하고 반대로 되돌아감
          if (st_ == TourState::Transit && elapsed > p.olMaxMs) {
            brakeUntilMs_ = in.nowMs + p.olBrakeMs;
            brakeSign_ = legSign_;
            enter(TourState::Return, in.nowMs + p.olBrakeMs, "다음 태그 없음: 제동 뒤 되돌아감");
          } else if (st_ == TourState::Return && tgt && in.nowMs >= brakeUntilMs_) {
            brakeUntilMs_ = in.nowMs + p.olBrakeMs;
            brakeSign_ = (int8_t)-legSign_;
            o.resetKf = true;
            enter(TourState::Done, in.nowMs, "다음 태그를 찾지 못해 이전 태그 위에서 대기");
          } else if (st_ == TourState::Return && (int32_t)(in.nowMs - enteredMs_) > (int32_t)(2 * p.olMaxMs)) {
            // 되돌아갈 때는 나가던 속도부터 죽여야 해서 나간 시간보다 오래 걸린다 (-54 시험: 6초로는 A 앞 60~115cm 에서 끊김)
            targetId_ = -1;
            enter(TourState::Lost, in.nowMs, "되돌아가기 시간 초과: 보드 중립, 조종자 확인");
          }
          break;
        }
        if (st_ == TourState::Transit) {
          bool reached = in.goalKnown && in.distToGoalCm <= p.goalTolCm;
          if (reached || elapsed > p.transitMs)
            enter(TourState::Return, in.nowMs, reached ? "목표 거리까지 갔지만 태그 없음: 복귀" : "전진 시간 초과: 복귀");
        } else if (tgt) {
          enter(TourState::Done, in.nowMs, "다음 태그를 찾지 못해 이전 태그 위에서 대기");
        } else if (elapsed > p.returnMs) {
          targetId_ = -1;
          enter(TourState::Lost, in.nowMs, "복귀 시간 초과: 보드 중립, 조종자 확인");
        }
        break;
      }
      case TourState::Done:
        updateTravel(in, tgt, newFrame);
        // 왕복 미션이 다음 태그를 못 찾아 멈췄으면 stuck_land 뒤 지금 태그에서 착륙 (사용자: 끝은 항상 착륙)
        if (p.mode == 1 && !landHere_ && elapsed > p.stuckLandMs) {
          landHere_ = true;
          reason_ = "다음 태그를 못 찾음: 이 태그에서 착륙";
        }
        break;
      case TourState::Off:
        break;
    }
    // 머무는 단계에서 목표 태그를 오래 못 보면 (칼만 예측도 끝남) 다시 찾는다
    // 착륙 태그 위에서 내려가는 중이면 태그가 안 보여도(너무 가까움) 그대로 — 칼만 예측으로 그 자리, 하강은 계속
    bool landing = st_ == TourState::Done && (act_->act == TOUR_ACT_LAND || landHere_);
    bool holding = (st_ == TourState::Center || st_ == TourState::Align || st_ == TourState::Dwell ||
                    st_ == TourState::Done) && !landing;
    if (holding && targetId_ >= 0 && (uint32_t)(in.nowMs - targetSeenMs_) > p.targetLostMs) {
      targetId_ = -1;
      tgt = nullptr;
      enter(TourState::Acquire, in.nowMs, "목표 태그를 놓침: 다시 찾기");
    }
    fill(o, in, tgt);
    return o;
  }

 private:
  static float dist(const TourSeen *t) { return hypotf(t->rightCm, t->fwdCm); }

  static const TourSeen *find(const TourInput &in, int id) {
    if (id < 0) return nullptr;
    for (int i = 0; i < in.n && i < TOUR_MAX_SEEN; i++)
      if (in.tags[i].id == id) return &in.tags[i];
    return nullptr;
  }

  // except 가 아닌 태그 중 가장 가까운 것
  static const TourSeen *nearest(const TourInput &in, int except) {
    const TourSeen *b = nullptr;
    for (int i = 0; i < in.n && i < TOUR_MAX_SEEN; i++) {
      if (in.tags[i].id < 0 || in.tags[i].id == except) continue;
      if (!b || dist(&in.tags[i]) < dist(b)) b = &in.tags[i];
    }
    return b;
  }

  // 지금 기체 기준 진행 방향 (시계 +, 0 = 기수). FC 기수를 알면 그것으로, 모르면 떠날 때 잰 상대각
  float travelRelNow(const TourInput &in) const {
    if (p.mode == 1) {  // 왕복: 미션 기수 기준 앞(0°)·뒤(180°)
      float leg = legSign_ < 0 ? 180.0f : 0.0f;
      return (missionHeadingKnown_ && in.yawValid) ? tourWrap180(missionHeading_ + leg - in.yawDeg) : leg;
    }
    return (travelAbs_ && in.yawValid) ? tourWrap180(travelDeg_ - in.yawDeg) : departRelDeg_;
  }

  // 떠나온 태그가 아니고 진행 방향 ± new_tag_cone 안에 있는 가장 가까운 태그를, 새 프레임마다 확인해
  // 연속 newTagFrames 번 보이면 인정
  const TourSeen *newTag(const TourInput &in, bool newFrame, bool frameFresh, float travelRel) {
    if (!newFrame) return (candFrames_ >= p.newTagFrames && frameFresh) ? find(in, candId_) : nullptr;
    const TourSeen *n = nullptr;
    for (int i = 0; i < in.n && i < TOUR_MAX_SEEN; i++) {
      const TourSeen &c = in.tags[i];
      if (c.id < 0 || c.id == lastId_) continue;
      float bearing = atan2f(c.rightCm, c.fwdCm) * 180.0f / (float)M_PI;
      if (fabsf(tourWrap180(bearing - travelRel)) > p.newTagConeDeg) continue;
      if (!n || dist(&c) < dist(n)) n = &c;
    }
    if (!n) {
      candId_ = -1;
      candFrames_ = 0;
      return nullptr;
    }
    if (n->id == candId_) {
      if (candFrames_ < 255) candFrames_++;
    } else {
      candId_ = n->id;
      candFrames_ = 1;
    }
    return candFrames_ >= p.newTagFrames ? n : nullptr;
  }

  void setTarget(int id, TourOutput &o, uint32_t now) {
    if (id != targetId_) o.retarget = true;
    targetId_ = id;
    targetSeenMs_ = now;
    act_ = &tourActionFor(id);
    travelAbs_ = travelRel_ = false;
    backward_ = backwardKnown_ = false;
    dwellMs_ = yawOkMs_ = 0;
  }

  void enter(TourState s, uint32_t now, const char *why) {
    st_ = s;
    enteredMs_ = now;
    reason_ = why;
  }

  uint32_t dwellNeedMs() const {
    if (p.mode == 1 && visited_ < (uint32_t)TOUR_SHUTTLE_N) return TOUR_SHUTTLE[visited_].dwellS * 1000u;
    return (uint32_t)((act_->dwellS ? act_->dwellS : p.dwellS) * 1000.0f);
  }

  // 새 프레임에 목표 태그가 있으면 진행 방향 갱신: 절대각 = 촬영 순간 FC 기수 + 태그 화살표 + 꺾기
  void updateTravel(const TourInput &in, const TourSeen *tgt, bool newFrame) {
    if (!newFrame || !tgt) return;
    relDeg_ = tourWrap180(tgt->upDeg + act_->turnDeg);
    travelRel_ = true;
    relAtMs_ = in.nowMs;
    if (!backwardKnown_) {  // 태그마다 처음 본 각도로 앞/뒤를 정하고 유지 (90° 근처에서 오락가락 방지)
      backward_ = p.reverse != 0 && fabsf(relDeg_) > 90.0f;
      backwardKnown_ = true;
    }
    if (!in.yawCapValid) return;
    float meas = tourWrap360(in.yawCapDeg + relDeg_);
    if (!travelAbs_) travelDeg_ = meas;
    else travelDeg_ = tourWrap360(travelDeg_ + p.headingAlpha * tourWrap180(meas - travelDeg_));
    travelAbs_ = true;
  }

  // 기수를 진행 방향으로 돌릴 오차 (+ = 오른쪽으로 돌아야 함). FC 기수를 알면 그것으로(빠르고 지연 없음),
  // 모르면 카메라가 잰 상대각으로 (지연이 있어 회전 속도를 절반으로)
  float yawError(const TourInput &in, const TourSeen *tgt) const {
    if (p.mode == 1 && missionHeadingKnown_ && in.yawValid) return tourWrap180(missionHeading_ - in.yawDeg);
    float flip = backward_ ? 180.0f : 0.0f;  // 후진: 기체 뒤쪽을 화살표에 맞춘다
    if (travelAbs_ && in.yawValid) return tourWrap180(travelDeg_ + flip - in.yawDeg);
    if (travelRel_ && tgt && (uint32_t)(in.nowMs - relAtMs_) <= p.frameStaleMs) return tourWrap180(relDeg_ + flip);
    return 0;
  }

  void fill(TourOutput &o, const TourInput &in, const TourSeen *tgt) {
    o.state = st_;
    o.reason = reason_;
    o.targetId = targetId_;
    o.lastId = lastId_;
    o.act = act_;
    o.elapsedMs = st_ == TourState::Off ? 0 : in.nowMs - enteredMs_;
    o.dwellMs = dwellMs_;
    o.dwellNeedMs = dwellNeedMs();
    o.travelKnown = travelAbs_ || travelRel_;
    o.travelDeg = travelAbs_ ? travelDeg_ : relDeg_;
    o.backward = backward_;
    o.landRequest = st_ == TourState::Done && (act_->act == TOUR_ACT_LAND || landHere_);
    o.visited = visited_;
    if (st_ == TourState::Off) return;
    if (tgt) {
      o.targetInFrame = true;
      o.targetR = tgt->rightCm;
      o.targetF = tgt->fwdCm;
    }
    o.focusId = targetId_;
    bool moving = st_ == TourState::Transit || st_ == TourState::Return;
    o.scanFull = moving || seekingHome_;
    if (p.mode == 1) {
      // 정해진 기울기: 제동 중이면 반대로, 이동 중이면 가속 → 유지 (복귀는 반대 방향)
      if (in.nowMs < brakeUntilMs_) {
        o.openLoop = true;
        o.olTiltFwdDeg = -brakeSign_ * p.olBrakeDeg;
      } else if (moving) {
        int8_t sgn = st_ == TourState::Transit ? legSign_ : (int8_t)-legSign_;
        uint32_t t = (uint32_t)(in.nowMs - enteredMs_);
        o.openLoop = true;
        o.olTiltFwdDeg = sgn * (t < p.olAccelMs ? p.olAccelDeg : p.olCruiseDeg);
      }
    }
    if (seekingHome_) {  // 이륙 태그가 안 보이는 동안 칼만 예측으로 그쪽으로 (기본 3초보다 길게)
      o.kfLostMs = p.homeWaitMs + 2000;
      o.speedCapCmS = 15;
    }
    if (moving) {
      o.speedCapCmS = p.speedCmS;
      o.kfLostMs = p.transitMs + p.returnMs + 2000;  // 태그 사이는 칼만 예측으로 간다
    }
    if (st_ == TourState::Transit) {
      // 목표점 = 떠나온 태그 + 진행 방향으로 dist. 기체 기준 진행 방향 = 진행 방향 − 지금 기수
      float rel = travelRelNow(in);
      o.goalR = p.distCm * sinf(rel * (float)M_PI / 180.0f);
      o.goalF = p.distCm * cosf(rel * (float)M_PI / 180.0f);
    }
    // 요: 방향을 맞춘 뒤로는 계속 진행 방향을 유지한다 (전진·복귀 중에도)
    bool yawPhase = st_ == TourState::Align || st_ == TourState::Dwell || moving || st_ == TourState::Done;
    if (yawPhase && !yawFault_ && (travelAbs_ || travelRel_)) {
      bool abs = travelAbs_ && in.yawValid;
      float err = yawError(in, tgt);
      float kp = abs ? p.yawKp : 0.5f * p.yawKp;
      float rate = fabsf(err) <= p.yawDeadDeg ? 0.0f : kp * err;
      if (rate > p.yawRateMax) rate = p.yawRateMax;
      if (rate < -p.yawRateMax) rate = -p.yawRateMax;
      o.yawOn = abs || (travelRel_ && !moving);  // 상대각만 알면 태그가 보일 때만 돈다
      o.yawErrDeg = err;
      o.yawRateDegS = o.yawOn ? rate : 0.0f;
    }
  }

  TourState st_ = TourState::Off;
  const char *reason_ = "";
  int targetId_ = -1, lastId_ = -1;
  const TourActionDef *act_ = &TOUR_ACTION_DEFAULT;
  uint32_t enteredMs_ = 0, targetSeenMs_ = 0, relAtMs_ = 0, lastFrameAt_ = 0, startMs_ = 0;
  bool travelAbs_ = false, travelRel_ = false;
  float travelDeg_ = 0, relDeg_ = 0, departRelDeg_ = 0;
  bool backward_ = false, backwardKnown_ = false;
  uint32_t yawOkMs_ = 0, dwellMs_ = 0;
  bool yawFault_ = false;  // 기수 맞추기 시간 초과 → 요 출력 안 함
  uint32_t inactiveSinceMs_ = 0;  // AUTO 가 끊긴 시각 (0 = 이어지는 중)
  uint32_t visited_ = 0;          // 머무름을 마친 태그 수
  bool landHere_ = false;         // land_at 번째 태그 → 이 태그 위에서 착륙
  int8_t legSign_ = 1;            // 왕복: 지금 이동 방향 (+1 앞, −1 뒤)
  bool seekingHome_ = false;      // 이륙 태그를 아직 못 봤고 예측으로 찾아가는 중
  uint32_t brakeUntilMs_ = 0;     // 이 시각까지 반대로 제동
  int8_t brakeSign_ = 1;          // 제동 전 움직이던 방향
  bool missionHeadingKnown_ = false;
  float missionHeading_ = 0;      // 왕복: 처음 태그에서 맞춘 기수 (앞 방향)
  int candId_ = -1;
  uint8_t candFrames_ = 0;
};

// 정해진 기울기 이동의 앞뒤 스틱 = 이동 기울기 + 머무는 동안 평균 명령(trim). trim 이 이동 방향과 반대면 더하지 않는다.
// -53 비행: trim +34µs(+0.6°) 가 뒤로 −0.6° 를 지워 스틱 1499 → 5초에 45cm(9cm/s). 스틱 가운데에서 9cm/s 면 실제
// 치우침은 0.6° 보다 훨씬 작다 — trim 에는 태그 중심에서 비켜 선 만큼의 제어 명령도 섞여 있다. 그래서 이동 방향 쪽
// 스틱은 적어도 이동 기울기만큼 나가게 한다 (main.cpp 와 PC 시험이 같은 함수를 쓴다)
inline float tourOlPitchUs(float trimUs, float legUs) {
  bool against = (legUs > 0 && trimUs < 0) || (legUs < 0 && trimUs > 0);
  return against ? legUs : legUs + trimUs;
}

// 이름으로 파라미터 바꾸기 (`tour set <이름> <값>`)
inline bool tourSetParam(TourParams &p, const char *name, float v) {
  struct Entry { const char *n; float *f; uint32_t *u; float lo, hi; };
  Entry table[] = {
      {"dwell_s", &p.dwellS, nullptr, 0, 600},
      {"align_radius", &p.alignRadiusCm, nullptr, 3, 100},
      {"align_exit", &p.alignExitCm, nullptr, 5, 200},
      {"dwell_radius", &p.dwellRadiusCm, nullptr, 3, 200},
      {"yaw_tol", &p.yawTolDeg, nullptr, 1, 45},
      {"yaw_hold_ms", nullptr, &p.yawHoldMs, 0, 10000},
      {"align_timeout_ms", nullptr, &p.alignTimeoutMs, 3000, 120000},
      {"yaw_kp", &p.yawKp, nullptr, 0, 5},
      {"yaw_rate_max", &p.yawRateMax, nullptr, 0, 45},
      {"yaw_dead", &p.yawDeadDeg, nullptr, 0, 20},
      {"reverse", &p.reverse, nullptr, 0, 1},
      {"land_at", &p.landAt, nullptr, 0, 20},
      {"mode", &p.mode, nullptr, 0, 1},
      {"ol_accel", &p.olAccelDeg, nullptr, 0, 6},
      {"ol_cruise", &p.olCruiseDeg, nullptr, 0, 4},
      {"ol_brake", &p.olBrakeDeg, nullptr, 0, 6},
      {"stuck_land_ms", nullptr, &p.stuckLandMs, 1000, 120000},
      {"dist", &p.distCm, nullptr, 0, 500},
      {"speed", &p.speedCmS, nullptr, 2, 30},
      {"transit_ms", nullptr, &p.transitMs, 1000, 60000},
      {"return_ms", nullptr, &p.returnMs, 1000, 60000},
      {"goal_tol", &p.goalTolCm, nullptr, 2, 50},
      {"heading_alpha", &p.headingAlpha, nullptr, 0.01f, 1},
      {"new_tag_cone", &p.newTagConeDeg, nullptr, 10, 180},
      {"target_lost_ms", nullptr, &p.targetLostMs, 500, 20000},
      {"pause_ms", nullptr, &p.pauseMs, 0, 10000},
      {"yaw_sign", &p.yawSign, nullptr, -1, 1},
      {"yaw_us_max", &p.yawUsMax, nullptr, 0, 250},
  };
  if (strcmp(name, "new_tag_frames") == 0) {
    if (v < 1 || v > 10) return false;
    p.newTagFrames = (uint8_t)v;
    return true;
  }
  if (strcmp(name, "yaw_sign") == 0 && v != 1 && v != -1) return false;
  for (const Entry &e : table) {
    if (strcmp(e.n, name) != 0) continue;
    if (v < e.lo || v > e.hi) return false;
    if (e.f) *e.f = v;
    else *e.u = (uint32_t)v;
    return true;
  }
  return false;
}

inline int tourParamsJson(const TourParams &p, char *out, size_t n) {
  return snprintf(out, n,
                  "{\"dwell_s\":%.1f,\"align_radius\":%.0f,\"align_exit\":%.0f,\"dwell_radius\":%.0f,\"yaw_tol\":%.1f,"
                  "\"yaw_hold_ms\":%lu,\"align_timeout_ms\":%lu,\"yaw_kp\":%.2f,\"yaw_rate_max\":%.0f,\"yaw_dead\":%.1f,\"reverse\":%.0f,\"land_at\":%.0f,\"mode\":%.0f,\"dist\":%.0f,"
                  "\"speed\":%.0f,\"transit_ms\":%lu,\"return_ms\":%lu,\"goal_tol\":%.0f,\"new_tag_frames\":%u,\"new_tag_cone\":%.0f,"
                  "\"heading_alpha\":%.2f,\"target_lost_ms\":%lu,\"pause_ms\":%lu,\"yaw_sign\":%.0f,\"yaw_us_max\":%.0f}",
                  p.dwellS, p.alignRadiusCm, p.alignExitCm, p.dwellRadiusCm, p.yawTolDeg, (unsigned long)p.yawHoldMs, (unsigned long)p.alignTimeoutMs,
                  p.yawKp, p.yawRateMax, p.yawDeadDeg, p.reverse, p.landAt, p.mode, p.distCm, p.speedCmS, (unsigned long)p.transitMs,
                  (unsigned long)p.returnMs, p.goalTolCm, p.newTagFrames, p.newTagConeDeg, p.headingAlpha,
                  (unsigned long)p.targetLostMs, (unsigned long)p.pauseMs, p.yawSign, p.yawUsMax);
}
