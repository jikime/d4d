#pragma once
// BLE 시리얼 (Nordic UART Service)
// ESP32-S3는 Bluetooth Classic(SPP)이 없어 BLE로 USB 시리얼을 대신한다.
//   서비스 6E400001-…, RX(쓰기: 명령) 6E400002-…, TX(알림: JSON 줄) 6E400003-…
// USB와 같은 줄 단위 텍스트를 주고받는다.
#include <stddef.h>
#include <stdint.h>

void bleBegin();
bool bleStarted();                 // radio 설정으로 BLE 를 켰나 (-36: WiFi 만 쓰면 끔 → 내부 RAM 확보)
const char *bleName();             // 광고 이름 (TagBoard-XXXX, MAC 끝 2바이트)
bool bleConnected();               // 연결되고 TX 알림을 구독 중
uint16_t bleMtu();
// 한 줄을 MTU 크기로 나눠 보낸다. 전송 버퍼가 비기를 timeoutMs까지 기다리고, 넘으면 false.
bool bleSend(const uint8_t *data, size_t len, uint32_t timeoutMs);
// 받은 명령 바이트를 꺼낸다 (줄 조립은 호출하는 쪽에서)
size_t bleRead(uint8_t *buf, size_t max);
uint32_t bleDropped();
int bleBuffersFree();
int bleBuffersTotal();
