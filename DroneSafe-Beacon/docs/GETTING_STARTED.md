# Windows에서 시작하기

현재 펌웨어는 **Nordic nRF Connect SDK v3.4.1**용입니다. Arduino IDE의 보드 목록에서 임의의 Nordic DK/XIAO를 골라 업로드하는 프로젝트가 아닙니다.

## 1. 필요한 도구

- Windows PowerShell 5.1 이상 또는 PowerShell 7.
- 공식 Nordic 공급처의 NCS **v3.4.1** 및 대응 툴체인. SDK 설치 루트 아래 `v3.4.1/nrf/west.yml`이 있어야 합니다.
- 설치된 `nrfutil-sdk-manager.exe`의 실제 경로.
- 업로드 경로에 한해 검토한 `nrf_ocd 0.3.8`과 툴체인 Python의 `pyOCD 0.42.0`, `intelhex`.

SDK, IDE, 프로그래머 실행 파일은 이 저장소에 재배포하지 않습니다. 기존 개발 PC에서는 비공식 Arduino 코어에 동봉된 nrf_ocd를 사용했으며, 프로그래머 경로는 사용자가 명시해야 합니다. 다른 버전을 설치했다면 호환성과 업로드 절차부터 별도로 검토하세요.

## 2. 빌드

저장소 최상위에서 PowerShell을 실행합니다. 아래 `<...>`는 설명용 자리표시자이므로 **자신의 실제 경로로 바꿔야 합니다.**

```powershell
$taskSdk = 'C:/ncs'
$taskManager = '<nrfutil-sdk-manager.exe의 절대 경로>'
$taskProgrammer = '<nrf_ocd 0.3.8 실행 파일의 절대 경로>'

./Build-NU54Secure.ps1 -Role drone -SdkInstallRoot $taskSdk -SdkManager $taskManager
./Build-NU54Secure.ps1 -Role searcher -SdkInstallRoot $taskSdk -SdkManager $taskManager
```

기본 산출물:

- 비콘: `.build/drone/drone/zephyr/zephyr.hex`
- 탐색기: `.build/searcher/searcher/zephyr/zephyr.hex`

`BUILD_COMPLETE`와 정상 종료를 확인하세요. 빌드는 보드 업로드를 실행하지 않습니다. 스크립트는 포함된 NUCODE 보드 확장과 `nrf54l15dk/nrf54l15/cpuapp/nu54dk` 타깃을 명시합니다. `-Pristine`은 해당 역할의 빌드 디렉터리를 다시 구성하는 옵션이며 보드 삭제 명령이 아닙니다.

## 3. 보드 식별

두 보드에 물리적으로 ‘비콘’과 ‘탐색기’를 표시하고, 프로브 도구의 전체 USB UID 및 Windows COM 포트를 확인하세요. COM 번호는 연결 상황에 따라 바뀔 수 있습니다. 혼동된다면 하나씩 연결해 확인합니다.

```powershell
Get-CimInstance Win32_SerialPort | Select-Object DeviceID, Name, PNPDeviceID

$taskDroneUid = '<비콘 프로브의 전체 USB UID>'
$taskSearcherUid = '<탐색기 프로브의 전체 USB UID>'
$taskDronePort = '<비콘의 COM 번호, 예: COM5>'
$taskSearcherPort = '<탐색기의 COM 번호, 예: COM6>'
```

복합 USB 인터페이스의 PNPDeviceID는 프로브 UID와 다를 수 있습니다. 프로브 목록과 부모 USB 장치 정보까지 대조해 주세요. 문자열 일부나 같은 칩 이름만으로 고르지 마세요. 스크립트는 전체 UID가 정확히 일치하는 장치만 사용하지만, **UID 자체가 센서/화면 연결이나 펌웨어 역할을 증명하지는 않습니다.** 그 역할 매핑은 물리 보드를 확인하는 사람이 책임져야 합니다.

## 4. 업로드

모든 시리얼 모니터·플로터와 다른 SWD 프로그래머를 닫습니다. 먼저 파일과 도구만 검사합니다.

```powershell
./Upload-NU54Secure.ps1 -Role searcher -UsbId $taskSearcherUid `
  -HexPath .build/searcher/searcher/zephyr/zephyr.hex `
  -Programmer $taskProgrammer -SdkManager $taskManager `
  -SdkInstallRoot $taskSdk -ValidateOnly
```

`-ValidateOnly`는 프로브를 열거나 보드를 프로그램하지 않습니다. 파일·도구·역할 매핑이 맞으면 같은 명령에서 **`-ValidateOnly`를 제거**해 실제로 탐색기를 업로드합니다. 비콘은 `-Role drone`, 비콘 UID와 `.build/drone/drone/zephyr/zephyr.hex`를 지정합니다. 혼동을 막기 위해 두 역할을 동시에 업로드하지 마세요.

검증 정책:

- HEX 주소와 체크섬 검사, 명시한 정확한 프로브 선택.
- 기존 settings/소유자 등록과 UICR를 지우지 않으며 erase/recover/unlock을 자동 수행하지 않음.
- 기록한 앱 범위 전부를 읽어 대조하고, 성공한 뒤 재시작.
- `FULL_READBACK_MATCH`, `UPLOAD_VERIFIED` 확인. 일부 바이트 검사 성공을 전체 검증으로 해석하지 않음.
- 검증 오류 시 자동 반복 기록하지 않음. `.build/upload-logs`에서 원인을 확인.

읽기 대조와 CPU 실행 확인은 센서·OLED·인증·거리 측정의 기능 검증을 대신하지 않습니다. 보수적인 읽기 검증은 수 분 걸릴 수 있습니다.

## 5. 최초 소유자 등록

[보안 안내](SECURITY.md)를 읽고, 신뢰 PC에서 양쪽 USB를 연결합니다. 다른 시리얼 도구와 PowerShell transcript/로그 수집을 모두 중단합니다.

```powershell
./common/Register-NU54Pair.ps1 -DronePort $taskDronePort `
  -SearcherPort $taskSearcherPort -DroneUsbId $taskDroneUid `
  -SearcherUsbId $taskSearcherUid
```

이미 다른 소유자가 등록돼 있거나 데이터가 모호하면 안전하게 거부합니다. 이를 우회하려고 임의 전체 삭제나 자동 재등록을 실행하지 마세요. 등록 후에는 키를 저장소나 로그에 복사하지 않습니다.

## 6. 기능 확인

```powershell
./Test-NU54Link.ps1 -DronePort $taskDronePort `
  -SearcherPort $taskSearcherPort -DroneUsbId $taskDroneUid `
  -SearcherUsbId $taskSearcherUid -Seconds 20
```

기본은 양쪽 상태 조회입니다. 시리얼 직접 확인은 115200 8N1, 줄바꿈을 사용합니다. 시험 스크립트와 시리얼 모니터를 동시에 열지 마세요.

| 장치 | 명령 | 동작 |
| --- | --- | --- |
| 양쪽 | `s` / `?` | 상태 조회 |
| 양쪽 | `SP STATUS` | 키를 출력하지 않는 등록·인증 상태 |
| 비콘 | `b` | 수동 회수 모드 진입 |
| 비콘 | `r` | 회수 상태 해제 후 재무장 — 의도했을 때만 사용 |
| 비콘 | `p` / `m` | 감시 중 센서 플롯 / 일반 상태 출력 |
| 탐색기 | `retry` | 인증 연결을 유지한 채 CS만 재시작하는 시험 |
| 탐색기 | `pause` / `resume` | 측정·연결 중지 후 재개; 수신 전용 비콘 스캔 유지 |
| 양쪽 | `drop` | 전체 재연결 시험 |

탐색기 `RANGE_RAW`는 원값, `RANGE`는 필터값과 제한 정보입니다. `authenticated=1`은 등록 기기 인증을 뜻하며 정확한 실제 거리를 보증하지 않습니다. OLED 첫 표시는 정상 표본 3개를 기다립니다. 소유자 등록 뒤의 평상시 운용에 수동 `SP OPEN`은 필요하지 않습니다.
