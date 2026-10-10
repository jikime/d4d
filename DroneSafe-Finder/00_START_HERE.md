# 드론안심이 — 휴대용 탐색기 소스

2026-10-11 기준 최신 실험판 소스입니다. 실제 사용을 인증한 완제품/정식 릴리스라는 뜻은 아닙니다.

## 이 ZIP의 용도

- 대상: NUCODE NU-54V DK(nRF54L15), SSD1306 128x64 I2C OLED(현재 주소 0x3C).
- 핵심 코드: `searcher/src/main.c`.
- 거리 필터: `searcher/src/walk_range_filter.c`, 자체 시험: `searcher/tests/`.
- 기능: 소유자 인증, Channel Sounding initiator, 정수 m OLED 표시, 거리별 LED, 보행 속도 기반 거리 필터 및 재연결.
- `common/`, `board_root/`, 라이선스 및 빌드/등록 도구를 함께 포함합니다. 폴더 구조를 유지하세요.
- 드론 비콘 소스 `drone/`는 별도 **DroneSafe Beacon** ZIP에 있습니다.

## 빌드

이 코드는 **Nordic nRF Connect SDK v3.4.1**용이며 Arduino `.ino` 파일이 아닙니다.
압축을 모두 풀고 `Build-NU54Secure.ps1`이 있는 폴더에서 PowerShell을 실행하세요.

```powershell
./Build-NU54Secure.ps1 -Role searcher -SdkInstallRoot 'C:/ncs' -SdkManager '<설치된 nrfutil-sdk-manager.exe의 실제 절대 경로>'
```

SDK 설치 위치는 자신의 PC에 맞게 바꾸세요. 기본 결과물은 `.build/searcher/searcher/zephyr/zephyr.hex`입니다.
일반 Nordic DK 설정으로 바꾸지 말고, 포함된 NU54 보드 확장을 사용하세요.

## 주의

- 완성된 바이너리, SDK, 장치 인증키, 등록 덤프, 개인 USB UID는 포함하지 않습니다.
- 업로드/등록은 자동 실행되지 않습니다. `docs/GETTING_STARTED.md`와 `docs/SECURITY.md`를 먼저 읽으세요.
- 공통 설명서에는 두 역할이 함께 나오지만 이 ZIP에서는 `-Role searcher`를 사용합니다.
- 필터는 순간적인 표시 튐을 줄이는 기능입니다. 절대거리 정확도나 실제 1m 이내를 보증하지 않습니다.
- 일반 BLE 탐지와 Channel Sounding은 다릅니다. 이 역할을 NU40(nRF52840)에 그대로 올릴 수 없습니다.
- 이번 다운로드 패키지 작성 과정에서는 보드를 다시 업로드하거나 재등록하지 않았습니다.

시험 결과와 한계: `docs/VALIDATION.md`.
