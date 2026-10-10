#pragma once
// 태그 크기 자동 보정 (하드웨어 의존 없음 — test/tag_size_cal_host_test.cpp 에서 PC로 시험)
//
// 카메라가 바로 아래를 보고 바닥(태그 면)과 나란하면, 바닥 위 물체의 화면 크기는 화면 어디에 있든
//   side_px = fx · 실제한변 / 거리   → 실제한변(mm) = side_px · 거리(cm) / fx · 10
// 거리는 거리센서(MTF-01P) 높이를 쓴다. 표본의 중앙값이 안정되면(퍼짐이 작으면) 그 값을 태그 크기로 쓴다.
// 설정값(config.h·NVS)이 틀려도, 모르고 있어도 비행하며 맞춰진다. 태그가 바닥에 있어야 맞다(손에 든 태그 X).
#include <math.h>
#include <stdint.h>
#include <string.h>

class TagSizeCal {
 public:
  static const int kCap = 15;  // 최근 표본 수

  void reset() {
    n_ = 0;
    head_ = 0;
    held_ = 0;
  }

  // 표본 하나 추가 (mm). 터무니없는 값(10mm 미만·2m 초과)은 버린다.
  bool add(float mm) {
    if (!(mm > 10.0f && mm < 2000.0f)) return false;
    buf_[head_] = mm;
    head_ = (head_ + 1) % kCap;
    if (n_ < kCap) n_++;
    return true;
  }

  int count() const { return n_; }

  // 중앙값 (표본이 없으면 0)
  float median() const {
    float s[kCap];
    int n = sorted(s);
    if (!n) return 0;
    return n % 2 ? s[n / 2] : 0.5f * (s[n / 2 - 1] + s[n / 2]);
  }

  // 상대 퍼짐: 중앙값 절대편차(MAD) / 중앙값
  float spread() const {
    float s[kCap];
    int n = sorted(s);
    if (n < 2) return 1.0f;
    float med = n % 2 ? s[n / 2] : 0.5f * (s[n / 2 - 1] + s[n / 2]);
    if (med <= 0) return 1.0f;
    float d[kCap];
    for (int i = 0; i < n; i++) d[i] = fabsf(s[i] - med);
    sortf(d, n);
    float mad = n % 2 ? d[n / 2] : 0.5f * (d[n / 2 - 1] + d[n / 2]);
    return mad / med;
  }

  // 표본이 minN 개 이상이고 퍼짐이 maxSpread 이하면 믿을 만하다
  bool converged(int minN, float maxSpread) const { return n_ >= minN && spread() <= maxSpread; }

  // 표본을 넣은 뒤 부른다. 지금 수렴해 있으면 그 중앙값을 기억한다.
  // 움직이는 중의 표본 몇 개로 퍼짐이 잠깐 커져도 설정값(예: 200mm)으로 되돌아가 거리가 몇 배로 튀지 않게,
  // 쓰는 크기는 held() — 마지막으로 수렴했던 값 — 이다. 다시 수렴하면 새 값으로 바뀐다.
  void refresh(int minN, float maxSpread) {
    if (converged(minN, maxSpread)) held_ = median();
  }

  // 마지막으로 수렴했던 크기 (mm). 한 번도 수렴하지 않았으면 0
  float held() const { return held_; }

 private:
  static void sortf(float *a, int n) {
    for (int i = 1; i < n; i++) {
      float v = a[i];
      int j = i - 1;
      while (j >= 0 && a[j] > v) {
        a[j + 1] = a[j];
        j--;
      }
      a[j + 1] = v;
    }
  }
  int sorted(float *out) const {
    memcpy(out, buf_, sizeof(float) * n_);
    sortf(out, n_);
    return n_;
  }

  float buf_[kCap] = {};
  int n_ = 0;
  int head_ = 0;
  float held_ = 0;
};

// 표본 하나를 계산: 화면 한 변(px), 거리센서 높이(cm), 카메라 렌즈가 거리센서보다 위에 있는 거리(cm), 초점거리(px)
inline float tagSizeSampleMm(float sidePx, float heightCm, float camAboveSensorCm, float fxPx) {
  float z = heightCm + camAboveSensorCm;
  if (sidePx <= 0 || z <= 0 || fxPx <= 0) return 0;
  return sidePx * z / fxPx * 10.0f;
}
