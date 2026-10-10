#pragma once
// MSP 프레임 생성·해석 (하드웨어 의존 없음 — test/ 에서 PC로 시험한다)
//  v1 요청: '$' 'M' '<' | size | cmd | payload[size] | checksum(size ^ cmd ^ payload)
//  v2 요청: '$' 'X' '<' | flag | cmd(LE16) | size(LE16) | payload | crc8_dvb_s2(flag~payload)
//  응답은 '<' 자리에 '>'(정상) 또는 '!'(FC가 거부)
#include <stddef.h>
#include <stdint.h>

static const uint16_t MSP_MOTOR = 104;          // FC가 모터에 실제로 내보내는 값 8개
static const uint16_t MSP_RC = 105;
static const uint16_t MSP_SONAR_ALTITUDE = 58;   // FC 거리센서 높이(cm, int32, 기울기 보정). 범위 밖이면 음수
static const uint16_t MSP_RX_MAP = 64;           // rcmap[4]: 논리 채널(롤·피치·요·스로틀) → 수신기 슬롯
static const uint16_t MSP_REBOOT = 68;
static const uint16_t MSP_MODE_RANGES = 34;      // 모드 스위치 배정 40개 × (모드 permanentId, AUX 번호, 시작 단계, 끝 단계)
static const uint8_t INAV_BOX_MSP_RC_OVERRIDE = 50;  // MSP RC OVERRIDE 모드의 permanentId (INAV fc/fc_msp_box.c)

// MSP_MODE_RANGES 응답에서 한 모드의 첫 배정을 찾는다. 채널 인덱스는 0 기준(AUX1 = 4 → CH5).
// INAV 활성 조건: 900 + 25·시작 ≤ 값 < 900 + 25·끝 → 끝은 미포함이라 maxUs = 끝 − 1.
inline bool mspFindModeRange(const uint8_t *p, uint16_t size, uint8_t permanentId, uint8_t &chIndex, uint16_t &minUs,
                             uint16_t &maxUs) {
  for (uint16_t i = 0; i + 3 < size; i += 4) {
    if (p[i] != permanentId || p[i + 2] >= p[i + 3]) continue;  // 시작 ≥ 끝 이면 빈 항목
    chIndex = (uint8_t)(4 + p[i + 1]);
    minUs = (uint16_t)(900 + 25 * p[i + 2]);
    maxUs = (uint16_t)(900 + 25 * p[i + 3] - 1);
    return true;
  }
  return false;
}
static const uint16_t MSP2_COMMON_SETTING = 0x1003; // 설정 이름("...\0") → 값 (msp_override_channels 확인)
static const uint16_t MSP_SET_RAW_RC = 200;
static const uint16_t MSP_SET_MOTOR = 214;      // 아밍 전 모터 출력 직접 지정 (INAV Configurator 모터 시험과 같음)
static const uint16_t MSP_ATTITUDE = 108;       // 롤·피치(0.1°)·요(°) — 카메라 지면점 기울기 보정용
static const uint16_t MSP2_INAV_STATUS = 0x2000; // 아밍 플래그 32비트 전체
static const uint16_t MSP2_INAV_OPTICAL_FLOW = 0x2001; // 품질 u8, 유량 X/Y(°/s) i16, 자이로 X/Y(°/s) i16
static const uint16_t MSP2_INAV_ANALOG = 0x2002; // 배터리 전압·셀 수·전류

inline uint8_t mspCrc8(uint8_t crc, uint8_t a) {
  crc ^= a;
  for (int i = 0; i < 8; i++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
  return crc;
}

// out 은 9 + size 바이트 이상. 프레임 길이를 돌려준다.
inline size_t mspEncodeV2(uint16_t cmd, const uint8_t *payload, uint16_t size, uint8_t *out) {
  out[0] = '$';
  out[1] = 'X';
  out[2] = '<';
  out[3] = 0;  // flag
  out[4] = cmd & 0xFF;
  out[5] = cmd >> 8;
  out[6] = size & 0xFF;
  out[7] = size >> 8;
  uint8_t crc = 0;
  for (int i = 3; i < 8; i++) crc = mspCrc8(crc, out[i]);
  for (uint16_t i = 0; i < size; i++) {
    out[8 + i] = payload[i];
    crc = mspCrc8(crc, payload[i]);
  }
  out[8 + size] = crc;
  return 9 + size;
}

// out 은 6 + size 바이트 이상. 프레임 길이를 돌려준다.
inline size_t mspEncode(uint8_t cmd, const uint8_t *payload, uint8_t size, uint8_t *out) {
  out[0] = '$';
  out[1] = 'M';
  out[2] = '<';
  out[3] = size;
  out[4] = cmd;
  uint8_t ck = size ^ cmd;
  for (uint8_t i = 0; i < size; i++) {
    out[5 + i] = payload[i];
    ck ^= payload[i];
  }
  out[5 + size] = ck;
  return 6 + size;
}

// 채널 값(µs) 배열 → MSP_SET_RAW_RC 프레임
inline size_t mspEncodeRawRc(const uint16_t *ch, uint8_t count, uint8_t *out) {
  uint8_t payload[64];
  for (uint8_t i = 0; i < count; i++) {
    payload[2 * i] = ch[i] & 0xFF;
    payload[2 * i + 1] = ch[i] >> 8;
  }
  return mspEncode(MSP_SET_RAW_RC, payload, 2 * count, out);
}

struct MspReply {
  uint16_t cmd;
  uint8_t version;  // 1 또는 2
  bool error;       // '!' 응답 (FC가 명령을 거부)
  bool ckOk;
  uint16_t size;
  const uint8_t *payload;
};

class MspReplyParser {
 public:
  // 응답 하나가 끝나면 true 와 함께 r 을 채운다 (v1·v2 모두)
  bool feed(uint8_t b, MspReply &r) {
    switch (st_) {
      case 0: st_ = (b == '$') ? 1 : 0; return false;
      case 1: st_ = (b == 'M') ? 2 : (b == 'X') ? 20 : (b == '$') ? 1 : 0; return false;
      // ---- v1 ----
      case 2:
        if (b == '>' || b == '!') {
          err_ = (b == '!');
          st_ = 3;
        } else {
          st_ = 0;
        }
        return false;
      case 3:
        size_ = b;
        ck_ = b;
        idx_ = 0;
        st_ = (size_ <= sizeof(buf_)) ? 4 : 0;
        return false;
      case 4:
        cmd_ = b;
        ck_ ^= b;
        st_ = size_ ? 5 : 6;
        return false;
      case 5:
        buf_[idx_++] = b;
        ck_ ^= b;
        if (idx_ >= size_) st_ = 6;
        return false;
      case 6:
        return finish(1, ck_ == b, r);
      // ---- v2 ----
      case 20:
        if (b == '>' || b == '!') {
          err_ = (b == '!');
          st_ = 21;
        } else {
          st_ = 0;
        }
        return false;
      case 21: ck_ = mspCrc8(0, b); st_ = 22; return false;  // flag
      case 22: cmd_ = b; ck_ = mspCrc8(ck_, b); st_ = 23; return false;
      case 23: cmd_ |= (uint16_t)b << 8; ck_ = mspCrc8(ck_, b); st_ = 24; return false;
      case 24: size_ = b; ck_ = mspCrc8(ck_, b); st_ = 25; return false;
      case 25:
        size_ |= (uint16_t)b << 8;
        ck_ = mspCrc8(ck_, b);
        idx_ = 0;
        st_ = size_ > sizeof(buf_) ? 0 : (size_ ? 26 : 27);
        return false;
      case 26:
        buf_[idx_++] = b;
        ck_ = mspCrc8(ck_, b);
        if (idx_ >= size_) st_ = 27;
        return false;
      case 27:
        return finish(2, ck_ == b, r);
      default:
        st_ = 0;
        return false;
    }
  }

 private:
  bool finish(uint8_t ver, bool ok, MspReply &r) {
    r.cmd = cmd_;
    r.version = ver;
    r.error = err_;
    r.ckOk = ok;
    r.size = size_;
    r.payload = buf_;
    st_ = 0;
    return true;
  }
  uint8_t st_ = 0, ck_ = 0;
  uint16_t size_ = 0, cmd_ = 0, idx_ = 0;
  bool err_ = false;
  uint8_t buf_[512];
};
