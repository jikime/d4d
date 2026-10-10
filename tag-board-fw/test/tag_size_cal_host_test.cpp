// 태그 크기 자동 보정을 PC에서 시험한다.
//   clang++ -std=c++17 -I src test/tag_size_cal_host_test.cpp -o /tmp/tagcal && /tmp/tagcal
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>

#include "tag_size_cal.h"

static int fails = 0;
#define CHECK(c)                                                       \
  do {                                                                 \
    if (!(c)) {                                                        \
      std::fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #c); \
      fails++;                                                         \
    }                                                                  \
  } while (0)
#define NEAR(a, b, tol) CHECK(std::fabs((a) - (b)) <= (tol))

int main() {
  // 표본 계산: 100mm 태그를 41cm 에서 보면 fx 615 기준 150px
  NEAR(tagSizeSampleMm(150.0f, 41.0f, 0.0f, 615.0f), 100.0f, 0.1f);
  NEAR(tagSizeSampleMm(150.0f, 39.0f, 2.0f, 615.0f), 100.0f, 0.1f);  // 렌즈가 센서보다 2cm 위
  CHECK(tagSizeSampleMm(0, 41, 0, 615) == 0);

  // 높이를 바꿔 가며(위아래 이동) 같은 태그를 보면 표본은 같은 값 근처 → 수렴
  TagSizeCal c;
  std::srand(1);
  for (int i = 0; i < 12; i++) {
    float h = 40.0f + 8.0f * i;                       // 40~128cm
    float side = 615.0f * 9.5f / h;                   // 실제 95mm
    side *= 1.0f + ((std::rand() % 200) - 100) / 4000.0f;  // ±2.5% 측정 잡음
    c.add(tagSizeSampleMm(side, h, 0, 615.0f));
  }
  NEAR(c.median(), 95.0f, 2.0f);
  CHECK(c.converged(8, 0.06f));

  // 튀는 값(손 그림자·순간 기울기)이 몇 개 섞여도 중앙값은 버틴다
  c.add(300.0f);
  c.add(20.0f);
  NEAR(c.median(), 95.0f, 3.0f);
  CHECK(c.converged(8, 0.06f));

  // 표본이 적거나 들쭉날쭉하면 확정하지 않는다
  TagSizeCal d;
  for (float v : {90.0f, 140.0f, 60.0f, 200.0f, 100.0f}) d.add(v);
  CHECK(!d.converged(8, 0.06f));
  for (float v : {95.0f, 180.0f, 50.0f, 120.0f}) d.add(v);
  CHECK(!d.converged(8, 0.06f));  // 9개지만 퍼짐이 큼
  CHECK(!d.add(5.0f) && !d.add(5000.0f));  // 범위 밖 버림

  // 오래된 표본은 밀려난다 (태그를 바꾸거나 크기 설정이 바뀐 경우)
  TagSizeCal e;
  for (int i = 0; i < 15; i++) e.add(200.0f);
  for (int i = 0; i < 15; i++) e.add(100.0f);
  NEAR(e.median(), 100.0f, 1e-3f);

  // 마지막 수렴값 기억 (held): 10-10 시험에서 퍼짐이 8% 로 잠깐 커지자 200mm 설정값으로 돌아가 거리가 2.3배 튐
  TagSizeCal h;
  CHECK(h.held() == 0);  // 수렴 전에는 없음 → 설정값을 쓴다
  for (int i = 0; i < 7; i++) {
    h.add(87.0f + (i % 3));
    h.refresh(8, 0.06f);
  }
  CHECK(h.held() == 0);  // 7개 — 아직
  h.add(88.0f);
  h.refresh(8, 0.06f);
  NEAR(h.held(), 88.0f, 1.0f);
  // 움직이는 중 들쭉날쭉한 표본이 15개 중 10개를 차지해 퍼짐이 커져도(MAD 라 7개까지는 버팀) held 는 그대로
  for (float v : {60.0f, 130.0f, 70.0f, 120.0f, 65.0f, 125.0f, 75.0f, 115.0f, 62.0f, 128.0f}) {
    h.add(v);
    h.refresh(8, 0.06f);
  }
  CHECK(!h.converged(8, 0.06f));
  NEAR(h.held(), 88.0f, 1.5f);
  // 다시 안정되면(다른 값으로) 새 값으로 바뀐다
  for (int i = 0; i < 15; i++) {
    h.add(80.0f);
    h.refresh(8, 0.06f);
  }
  NEAR(h.held(), 80.0f, 1e-3f);
  h.reset();
  CHECK(h.held() == 0);

  if (fails) {
    std::fprintf(stderr, "%d failures\n", fails);
    return 1;
  }
  std::puts("태그 크기 보정 테스트 통과");
  return 0;
}
