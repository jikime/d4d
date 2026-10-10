#pragma once
// 보드 WiFi + TCP 한 줄 통로. USB·BLE 와 같은 JSON 줄을 내보내고 같은 명령을 받는다 (클라이언트 1개).
// 접속 방식 (-36~):
//   STA: 저장된 공유기(`wifi sta <이름> <비밀번호>`, NVS)에 붙는다 → PC 는 인터넷을 유지한 채 같은 망에서
//        tagboard-xxxx.local:WIFI_TCP_PORT (mDNS) 또는 `wifi` 명령이 알려 주는 IP 로 연결한다.
//   AP : 저장된 공유기가 없거나 20초 안에 못 붙으면 보드 자체 WiFi(이름 = BLE 이름, WPA2) 192.168.4.1.
// 사진(snap)도 같은 통로로 받는다 — BLE(약 6KB/s)보다 수십 배 빠르다.
#include <stddef.h>
#include <stdint.h>

// staSsid 가 비어 있으면 바로 AP. 비밀번호는 NVS 에서 읽어 넘긴다 (소스에 두지 않는다)
bool wifiBegin(const char *name, const char *apPass, const char *staSsid, const char *staPass, uint16_t port);
bool wifiOn();                  // 망에 붙었거나 AP 가 떠 있나
const char *wifiMode();         // "sta" · "ap" · "connecting" · "sta_lost" · "off"
void wifiIp(char *out, size_t n);
int wifiRssi();                 // STA 일 때 신호 세기 dBm (아니면 0)
const char *wifiHost();         // mDNS 이름 (tagboard-xxxx, .local 붙여 씀)
bool wifiClientConnected();     // TCP 클라이언트가 붙어 있나
int wifiStations();             // AP 일 때 붙은 기기 수
// 한 줄 보내기. 소켓 버퍼가 차 있으면 timeoutMs 까지 기다리고, 그래도 안 되면 줄을 버린다.
bool wifiSend(const uint8_t *data, size_t len, uint32_t timeoutMs);
// 받은 명령 바이트를 꺼낸다 (줄 조립은 호출하는 쪽에서)
size_t wifiRead(uint8_t *buf, size_t max);
uint32_t wifiDropped();
float wifiTxDbm();             // 송신 세기 dBm
int wifiReason();               // 마지막 공유기 끊김 이유 (ESP-IDF wifi_err_reason_t: 201 공유기 없음, 15/204 비밀번호·핸드셰이크)
// 주변 WiFi 목록(2.4GHz)을 JSON 배열로. auth: 0 열림, 3 WPA2, 4 WPA/WPA2, 6 WPA3, 7 WPA2/WPA3. 블로킹 2~3초
int wifiScan(char *out, size_t n);
// WiFi 펌웨어 업데이트(ArduinoOTA, 포트 3232, 비밀번호 = AP 비밀번호). allowed() 가 false 면(FC 아밍 중) 받지 않는다.
// PC: pio run -e xiao_esp32s3_ota -t upload --upload-port <IP 또는 tagboard-xxxx.local>
void wifiOtaBegin(const char *hostname, const char *pass, bool (*allowed)());
