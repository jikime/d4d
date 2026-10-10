// CH6 수동 우선권 게이트를 PC에서 시험한다.
//   g++ -std=c++17 -I src test/rc_authority_host_test.cpp -o rc_authority_test
#include <cstdio>

#include "rc_authority.h"

static int fails = 0;
#define CHECK(c)                                                          \
  do {                                                                    \
    if (!(c)) {                                                           \
      std::fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #c);    \
      fails++;                                                            \
    }                                                                     \
  } while (0)

int main() {
  RcAuthorityGate gate(/*channelIndex=*/5, /*autoMinUs=*/900, /*autoMaxUs=*/1700,
                       /*timeoutMs=*/250);
  uint16_t ch[8] = {1500, 1500, 1000, 1500, 1000, 2000, 1500, 1500};

  // 부팅 직후와 유효한 CH6 데이터가 없을 때는 반드시 수동이다.
  CHECK(gate.authority(0) == RcAuthority::Manual);
  CHECK(!gate.rcTxAllowed(0));
  gate.observe(ch, 5, 10);
  CHECK(gate.authority(10) == RcAuthority::Manual);

  // CH6=1000이면 자동 송신을 허용한다.
  ch[5] = 1000;
  gate.observe(ch, 8, 20);
  CHECK(gate.authority(20) == RcAuthority::Auto);
  CHECK(gate.rcTxAllowed(20));
  CHECK(gate.channelUs() == 1000);

  // CH6=2000 응답을 받는 즉시 수동이 우선한다.
  ch[5] = 2000;
  gate.observe(ch, 8, 30);
  CHECK(gate.authority(30) == RcAuthority::Manual);
  CHECK(!gate.rcTxAllowed(30));
  CHECK(gate.channelUs() == 2000);

  // FC의 MSP RC OVERRIDE 활성 범위(900~1700)와 같은 범위만 AUTO다.
  ch[5] = 899;
  gate.observe(ch, 8, 40);
  CHECK(gate.authority(40) == RcAuthority::Manual);
  ch[5] = 900;
  gate.observe(ch, 8, 50);
  CHECK(gate.authority(50) == RcAuthority::Auto);
  ch[5] = 1100;
  gate.observe(ch, 8, 60);
  CHECK(gate.authority(60) == RcAuthority::Auto);
  ch[5] = 1500;
  gate.observe(ch, 8, 70);
  CHECK(gate.authority(70) == RcAuthority::Auto);
  ch[5] = 1700;
  gate.observe(ch, 8, 80);
  CHECK(gate.authority(80) == RcAuthority::Auto);
  ch[5] = 1701;
  gate.observe(ch, 8, 90);
  CHECK(gate.authority(90) == RcAuthority::Manual);

  // FC 응답이 끊기면 마지막 AUTO 값을 계속 쓰지 않고 수동으로 복귀한다.
  ch[5] = 1000;
  gate.observe(ch, 8, 100);
  CHECK(gate.authority(350) == RcAuthority::Auto);
  gate.expire(351);
  CHECK(gate.authority(351) == RcAuthority::Manual);
  CHECK(!gate.rcTxAllowed(351));

  // 범위를 벗어난 RC 값도 유효한 자동 권한으로 취급하지 않는다.
  ch[5] = 0;
  gate.observe(ch, 8, 400);
  CHECK(gate.authority(400) == RcAuthority::Manual);

  // FC 범위 따라가기 (10-11: FC 가 900~1300 으로 바뀜 → CH6 1500 은 수동)
  RcAuthorityGate fcGate(5, 900, 1700, 250);
  CHECK(fcGate.setRange(5, 900, 1299));
  ch[5] = 1500;
  fcGate.observe(ch, 8, 10);
  CHECK(fcGate.authority(10) == RcAuthority::Manual);
  ch[5] = 1000;
  fcGate.observe(ch, 8, 20);
  CHECK(fcGate.authority(20) == RcAuthority::Auto);
  ch[5] = 1299;
  fcGate.observe(ch, 8, 25);
  CHECK(fcGate.authority(25) == RcAuthority::Auto);
  ch[5] = 1300;  // INAV 는 끝 값 미포함
  fcGate.observe(ch, 8, 30);
  CHECK(fcGate.authority(30) == RcAuthority::Manual);
  CHECK(!fcGate.setRange(5, 1500, 1200));  // 비정상 범위는 무시
  CHECK(fcGate.autoMaxUs() == 1299);
  // 다른 채널로 옮기면 이전 관측은 버리고 수동에서 시작
  ch[5] = 1000;
  fcGate.observe(ch, 8, 40);
  CHECK(fcGate.setRange(6, 900, 1299));
  CHECK(fcGate.authority(40) == RcAuthority::Manual);
  ch[6] = 1000;
  fcGate.observe(ch, 8, 50);
  CHECK(fcGate.authority(50) == RcAuthority::Auto);

  // millis() rollover 중의 짧은 간격은 유효하되, 한 번 만료된 AUTO는 다시 살아나지 않는다.
  RcAuthorityGate wrapGate(5, 900, 1700, 250);
  ch[5] = 1000;
  wrapGate.observe(ch, 8, 0xFFFFFFF0u);
  CHECK(wrapGate.authority(0x00000020u) == RcAuthority::Auto);
  wrapGate.expire(0x00000120u);
  CHECK(wrapGate.authority(0x00000120u) == RcAuthority::Manual);
  CHECK(wrapGate.authority(0xFFFFFFF1u) == RcAuthority::Manual);

  std::fprintf(stderr, fails ? "RC 권한 테스트 실패 %d건\n" : "RC 권한 테스트 통과\n", fails);
  return fails ? 1 : 0;
}
