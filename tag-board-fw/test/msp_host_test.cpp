// PC에서 MSP 프레임 생성·해석을 시험한다 (펌웨어와 같은 src/msp.h 사용)
//   clang++ -std=c++17 -I src test/msp_host_test.cpp -o /tmp/msp_test && /tmp/msp_test
// 표준출력 첫 줄의 SET_RAW_RC 프레임(hex)은 test/check_msp.py 가 규격대로 다시 풀어 확인한다.
#include <cassert>
#include <cstdio>
#include <vector>

#include "msp.h"

static int fails = 0;
#define CHECK(c)                                          \
  do {                                                    \
    if (!(c)) {                                           \
      fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #c); \
      fails++;                                            \
    }                                                     \
  } while (0)

static std::vector<MspReply> feedAll(MspReplyParser &p, const std::vector<uint8_t> &bytes,
                                     std::vector<std::vector<uint8_t>> &payloads) {
  std::vector<MspReply> out;
  MspReply r;
  for (uint8_t b : bytes)
    if (p.feed(b, r)) {
      out.push_back(r);
      payloads.emplace_back(r.payload, r.payload + r.size);
    }
  return out;
}

static std::vector<uint8_t> reply(char dir, uint8_t cmd, const std::vector<uint8_t> &pl, bool breakCk = false) {
  std::vector<uint8_t> f = {'$', 'M', (uint8_t)dir, (uint8_t)pl.size(), cmd};
  uint8_t ck = pl.size() ^ cmd;
  for (uint8_t b : pl) {
    f.push_back(b);
    ck ^= b;
  }
  f.push_back(breakCk ? ck ^ 0x5A : ck);
  return f;
}

int main() {
  // 1) SET_RAW_RC 프레임 → hex 출력 (Python 이 독립적으로 검증)
  const uint16_t ch[8] = {1520, 1468, 1500, 1500, 1000, 2000, 1500, 1234};
  uint8_t frame[64];
  size_t n = mspEncodeRawRc(ch, 8, frame);
  CHECK(n == 6 + 16);
  for (size_t i = 0; i < n; i++) printf("%02x", frame[i]);
  printf("\n");

  // 2) MSP_RC 요청: payload 없음
  n = mspEncode(MSP_RC, nullptr, 0, frame);
  CHECK(n == 6 && frame[3] == 0 && frame[4] == 105 && frame[5] == 105);

  // 3) 응답 파서
  MspReplyParser p;
  std::vector<std::vector<uint8_t>> pls;
  std::vector<uint8_t> rcPayload;
  for (uint16_t v : {1520, 1468, 1500, 1500, 1000, 2000, 1500, 1234}) {
    rcPayload.push_back(v & 0xFF);
    rcPayload.push_back(v >> 8);
  }
  std::vector<uint8_t> stream = {0x00, '$', 'x', '$', '$'};          // 앞쪽 잡음
  auto a = reply('>', MSP_RC, rcPayload);                              // 정상 MSP_RC
  auto b = reply('>', MSP_SET_RAW_RC, {});                             // SET_RAW_RC ack
  auto c = reply('>', MSP_RC, rcPayload, true);                        // 체크섬 깨짐
  auto d = reply('!', MSP_SET_RAW_RC, {});                             // FC 거부
  for (auto *v : {&a, &b, &c, &d}) stream.insert(stream.end(), v->begin(), v->end());
  auto rs = feedAll(p, stream, pls);
  CHECK(rs.size() == 4);
  if (rs.size() == 4) {
    CHECK(rs[0].cmd == MSP_RC && rs[0].ckOk && !rs[0].error && rs[0].size == 16);
    CHECK((pls[0][0] | (pls[0][1] << 8)) == 1520 && (pls[0][14] | (pls[0][15] << 8)) == 1234);
    CHECK(rs[1].cmd == MSP_SET_RAW_RC && rs[1].ckOk && rs[1].size == 0);
    CHECK(rs[2].cmd == MSP_RC && !rs[2].ckOk);
    CHECK(rs[3].cmd == MSP_SET_RAW_RC && rs[3].error && rs[3].ckOk);
  }
  // 4) MSP v2: 실제 MTF-01P 가 보낸 v2 프레임(거리 0x1F01) — 같은 CRC 규칙
  {
    MspReplyParser p2;
    std::vector<std::vector<uint8_t>> pl2;
    // '$X<' 요청 프레임이라 응답 파서는 무시해야 한다 → '>' 로 바꾼 응답 형태로 시험
    std::vector<uint8_t> f = {'$', 'X', '>', 0x00, 0x01, 0x1f, 0x05, 0x00, 0x34, 0xad, 0x06, 0x00, 0x00, 0xcf};
    auto rs2 = feedAll(p2, f, pl2);
    CHECK(rs2.size() == 1);
    if (rs2.size() == 1) {
      CHECK(rs2[0].version == 2 && rs2[0].cmd == 0x1F01 && rs2[0].ckOk && rs2[0].size == 5);
      CHECK((pl2[0][1] | (pl2[0][2] << 8)) == 0x06AD);
    }
    // 우리가 만드는 v2 요청(MSP2_INAV_STATUS)을 응답 형태로 바꿔 넣으면 CRC가 맞아야 한다
    uint8_t req[16];
    size_t n2 = mspEncodeV2(MSP2_INAV_STATUS, nullptr, 0, req);
    CHECK(n2 == 9 && req[4] == 0x00 && req[5] == 0x20);
    req[2] = '>';
    std::vector<uint8_t> rv(req, req + n2);
    auto rs3 = feedAll(p2, rv, pl2);
    CHECK(rs3.size() == 1 && rs3[0].ckOk && rs3[0].cmd == MSP2_INAV_STATUS && rs3[0].version == 2);
    // v1 과 v2 가 섞여 들어와도 각각 해석
    std::vector<uint8_t> mix = reply('>', MSP_RC, rcPayload);
    mix.insert(mix.end(), f.begin(), f.end());
    auto rs4 = feedAll(p2, mix, pl2);
    CHECK(rs4.size() == 2 && rs4[0].version == 1 && rs4[1].version == 2);
  }
  {
    // MSP_MODE_RANGES: 10-11 FC 실측 배정 (ARM CH5 1700~2100, ANGLE CH10 1300~1700, OVERRIDE CH6 900~1300) + 빈 항목
    uint8_t mr[160] = {};
    const uint8_t used[] = {0, 0, 32, 48, 1, 5, 16, 32, 50, 1, 0, 16};
    for (size_t i = 0; i < sizeof(used); i++) mr[i] = used[i];
    uint8_t chI = 0;
    uint16_t lo = 0, hi = 0;
    CHECK(mspFindModeRange(mr, sizeof(mr), INAV_BOX_MSP_RC_OVERRIDE, chI, lo, hi));
    CHECK(chI == 5 && lo == 900 && hi == 1299);  // CH6, 끝 미포함
    CHECK(mspFindModeRange(mr, sizeof(mr), 1, chI, lo, hi) && chI == 9 && lo == 1300 && hi == 1699);
    CHECK(!mspFindModeRange(mr, sizeof(mr), 27, chI, lo, hi));  // FAILSAFE 배정 없음
  }
  fprintf(stderr, fails ? "C++ 테스트 실패 %d건\n" : "C++ 테스트 통과\n", fails);
  return fails ? 1 : 0;
}
