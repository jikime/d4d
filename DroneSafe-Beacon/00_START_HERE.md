# 드론안심이 — 드론 탑재 비콘 소스

2026-10-11 기준 최신 실험판 소스입니다. 실제 사용을 인증한 완제품/정식 릴리스라는 뜻은 아닙니다.

## 이 ZIP의 용도

- 대상: NUCODE NU-54V DK(nRF54L15), BNO08x/SH2 IMU(현재 주소 0x4B).
- 핵심 코드: `drone/src/main.c`.
- 기능: 충격 의심 감시, 회수 상태 유지, 독립 BLE 비콘 광고, 소유자 인증, Channel Sounding reflector.
- `common/`, `board_root/`, 라이선스 및 빌드/등록 도구를 함께 포함합니다. 폴더 구조를 유지하세요.
- 탐색기 소스 `searcher/`는 별도 **DroneSafe Finder** ZIP에 있습니다.

## 빌드

이 코드는 **Nordic nRF Connect SDK v3.4.1**용이며 Arduino `.ino` 파일이 아닙니다.
압축을 모두 풀고 `Build-NU54Secure.ps1`이 있는 폴더에서 PowerShell을 실행하세요.

```powershell
./Build-NU54Secure.ps1 -Role drone -SdkInstallRoot 'C:/ncs' -SdkManager '<설치된 nrfutil-sdk-manager.exe의 실제 절대 경로>'
```

SDK 설치 위치는 자신의 PC에 맞게 바꾸세요. 기본 결과물은 `.build/drone/drone/zephyr/zephyr.hex`입니다.
일반 Nordic DK 설정으로 바꾸지 말고, 포함된 NU54 보드 확장을 사용하세요.

## 주의

- 완성된 바이너리, SDK, 장치 인증키, 등록 덤프, 개인 USB UID는 포함하지 않습니다.
- 업로드/등록은 자동 실행되지 않습니다. `docs/GETTING_STARTED.md`와 `docs/SECURITY.md`를 먼저 읽으세요.
- 공통 설명서에는 두 역할이 함께 나오지만 이 ZIP에서는 `-Role drone`을 사용합니다.
- 현재 드라이버는 LSM6DS3TR용이 아닙니다. 센서 모델을 확인하세요.
- 거리 정확도, 충격 오탐/미탐 및 30~60일 배터리 지속시간은 검증 완료 상태가 아닙니다.
- 이번 다운로드 패키지 작성 과정에서는 보드를 다시 업로드하거나 재등록하지 않았습니다.

시험 결과와 한계: `docs/VALIDATION.md`.
