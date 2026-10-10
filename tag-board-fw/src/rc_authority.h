#pragma once

#include <stddef.h>
#include <stdint.h>

// FC 수신기 CH6가 결정하는 제어권. 유효한 AUTO 응답이 없으면 항상 MANUAL이 기본값이다.
enum class RcAuthority : uint8_t {
  Manual = 0,
  Auto = 1,
};

class RcAuthorityGate {
 public:
  RcAuthorityGate(uint8_t channelIndex, uint16_t autoMinUs, uint16_t autoMaxUs,
                  uint32_t timeoutMs)
      : channelIndex_(channelIndex),
        autoMinUs_(autoMinUs),
        autoMaxUs_(autoMaxUs),
        timeoutMs_(timeoutMs) {}

  void observe(const uint16_t *channels, size_t count, uint32_t nowMs) {
    if (!channels || count <= channelIndex_) {
      invalidate();
      return;
    }

    channelUs_ = channels[channelIndex_];
    updatedAtMs_ = nowMs;
    valid_ = channelUs_ >= kRcMinUs && channelUs_ <= kRcMaxUs;
    observed_ = valid_ && channelUs_ >= autoMinUs_ && channelUs_ <= autoMaxUs_
                    ? RcAuthority::Auto
                    : RcAuthority::Manual;
  }

  // FC 의 MSP RC OVERRIDE 활성 범위를 그대로 따른다 (FC MSP_MODE_RANGES 에서 읽음).
  // 20261011: FC 가 900~1300 으로 바뀌었는데 보드는 900~1700 고정이라, CH6 1500 에서 보드만 AUTO 로 봤다.
  // 범위가 비정상이면 무시한다. 바뀌면 다음 observe 부터 적용된다.
  bool setRange(uint8_t channelIndex, uint16_t autoMinUs, uint16_t autoMaxUs) {
    if (autoMinUs < kRcMinUs || autoMaxUs > kRcMaxUs || autoMinUs >= autoMaxUs || channelIndex >= 18) return false;
    if (channelIndex != channelIndex_) invalidate();
    channelIndex_ = channelIndex;
    autoMinUs_ = autoMinUs;
    autoMaxUs_ = autoMaxUs;
    return true;
  }
  uint8_t channelIndex() const { return channelIndex_; }
  uint16_t autoMinUs() const { return autoMinUs_; }
  uint16_t autoMaxUs() const { return autoMaxUs_; }

  void invalidate() {
    valid_ = false;
    observed_ = RcAuthority::Manual;
  }

  void expire(uint32_t nowMs) {
    if (valid_ && (uint32_t)(nowMs - updatedAtMs_) > timeoutMs_) invalidate();
  }

  RcAuthority authority(uint32_t nowMs) const {
    if (!fresh(nowMs)) return RcAuthority::Manual;
    return observed_;
  }

  bool rcTxAllowed(uint32_t nowMs) const { return authority(nowMs) == RcAuthority::Auto; }

  bool fresh(uint32_t nowMs) const {
    return valid_ && (uint32_t)(nowMs - updatedAtMs_) <= timeoutMs_;
  }

  uint16_t channelUs() const { return channelUs_; }

 private:
  static constexpr uint16_t kRcMinUs = 750;
  static constexpr uint16_t kRcMaxUs = 2250;

  uint8_t channelIndex_;
  uint16_t autoMinUs_;
  uint16_t autoMaxUs_;
  uint32_t timeoutMs_;
  RcAuthority observed_ = RcAuthority::Manual;
  uint16_t channelUs_ = 0;
  uint32_t updatedAtMs_ = 0;
  bool valid_ = false;
};
