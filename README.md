# d4d - EMP/재밍 환경 자율드론 도구

SPIDER 웹 콘솔은 지도 중심 작업 공간과 3D 가상 비행 시험실을 제공한다. 새 화면의 조작법·모델 범위·검증 결과는 [UI/UX 개편 기록](docs/UI_UX_REFRESH.md)을 참고한다.

패키지 두 개로 구성됩니다.

| 패키지 | 역할 | 하드웨어 |
|--------|------|----------|
| `d4d_planner` | 출발 전에 전장을 분석하고 코스를 계획하는 에이전트 | 불필요 |
| `d4d_fc` | 비행제어기(INAV)와 MSP로 통신하는 조사/모니터 도구 | USB 연결 필요 |

## 기체 실측 기록 (2026-10-10, -22 지상 세션)

실기기 제어 경로와 안전 절차는 **`docs/HARDWARE_CONTROL.md`**가 기준입니다. 2026-10-10 지상 세션의
증거는 `out/control_live/2026-10-10-ground-session.md`에 있습니다.

| 항목 | 값 |
|------|-----|
| 보조 보드 | XIAO ESP32S3 Sense, BLE `TagBoard-B8B4`, 빌드 `TAGVISION-20261010-22` |
| 보드↔FC | UART5 MSP 115200 (롤·피치만, 스로틀·요·ARM은 덮어쓰지 않음). 배선·프로토콜 확인 스케치: `hardware/reference/seeduino_communication_OK.ino` |
| MTF-01P | FC UART4 MSP에 직결, FC가 거리·광류 감지 |
| 채널 | CH5 ARM · CH6 MSP RC OVERRIDE(1000 자동 / 2000 수동) · CH7 킬 · CH8 ANGLE |
| `msp_override_channels` | 영구 64(킬만). 지상 시험 때만 RAM에 67(롤+피치+킬) |
| 검증됨 | CH6 수동 우선·자동 복귀, 중립 롤·피치 반영, 10% 모터 지상 응답 |
| 미검증 | 태그 기반 비중립 반영, 모터 순서·방향, 카메라 축, 킬 재시험 → **비행 금지** |

이 표는 당시 기록입니다. 관제 소프트웨어는 현재 -25 인터페이스와 7/71 마스크를 지원하지만, 그 사실이 현재 보드 빌드·FC 영구 설정이나 지상 시험 성공을 뜻하지 않습니다. 최신 상태는 실기기 조회로 확인합니다.
정보가 서로 다르면 실기기 조회 → GitHub 2026-10-10 문서 → 10:51 CLI 덤프 순으로 믿습니다.
아래 2026-10-03 조사표는 과거 기록으로 남겨 둡니다.

## 연결된 실제 기체 (2026-10-03 조사 결과, 과거 기록)

`d4d_fc inspect`로 확인한 내용입니다. 자세한 내용은 `out/fc/fc_readiness.md`를 보세요.
광류/거리 센서와 MSP RC override는 그 뒤 설정됐습니다(위 표 참고).

| 항목 | 값 |
|------|-----|
| 비행제어기 | MicoAir H743 V2 (`MICOAIR743V2`), INAV **9.1.0** (빌드 2026-07-08) |
| 연결 | USB VCP (STM32 VCP, `VID_0483/PID_5740`) |
| 기체 | 멀티로터 Quad X, 모터 4개, DSHOT300 |
| 센서 | 가속도계 BMI088, 기압계 SPL06, 지자기 QMC5883 |
| **광류/거리** | **없음** (`opflow_hardware=NONE`, `rangefinder_hardware=NONE`) |
| GPS | UART3에 할당(UBLOX)되어 있으나 미감지 |
| 수신기 | IBUS, UART6 (FS-A8S V2와 일치). 조사 시점에는 신호 없음 |
| 남는 UART | UART4, UART5 |
| Failsafe | **DROP(모터 정지)** |
| MSP RC override | **비활성** (`msp_override_channels=0`) |

## d4d_planner - 코스 계획 에이전트

통신(RC, 텔레메트리)과 GPS를 쓸 수 없는 EMP/재밍 환경에서, 드론이 **출발 전에 분석해 둔 전장과 코스대로** 스스로 움직이도록 코스를 만드는 지상용 프로그램입니다. 표준 라이브러리만 사용합니다.

### 왜 이런 구조인가 (Notion 드론 설계 기준)

| 드론 구성 | 이 프로그램에서의 의미 |
|-----------|-----------------------|
| MTF-01P (광류 + 거리) | 추측항법. 이동할수록 위치 오차가 쌓임 -> **드리프트 모델** |
| 카메라 + AprilTag (XIAO ESP32S3) | 위치 보정 지점. 태그를 보면 오차가 리셋됨 -> **태그를 경유하는 코스** |
| UART로 FC에 명령 (MicoAir H743 V2) | 좌표 웨이포인트를 속도/위치 명령으로 변환 -> `mission.json`, `mission_data.h` |
| FC가 명령 타임아웃 감시 | 명령 주기/타임아웃을 미션에 포함 (`uart` 항목) |
| 조종기 끊기면 자동 미션 중단 (초기 시험 설계) | `link_policy: abort` = 재밍 구역을 아예 피함 |

재밍 구역을 반드시 지나야 한다면 `link_policy: continue_mission`을 씁니다. 이 경우 구역 안에서는 조종기로 회수할 수 없으므로, 아래 [안전 주의](#안전-주의)를 읽으세요.

### 에이전트 동작

```
시나리오(JSON) -> [분석] -> [경로 계획] -> [검증(몬테카를로)] -> 통과? -> 산출물
                      ^           |              |
                      |           v              v
                      +---- [진단/수정] <---- 실패 원인별 파라미터 조정, 태그 설치 위치 제안
```

1. **분석** - 장애물/금지/재밍/광류 저하 구역을 격자로 만들고 기체 반경+마진만큼 팽창합니다. 목표 도달 가능 여부, 태그 상태, 태그 간 허용 거리를 계산합니다.
2. **계획** - 출발점과 태그를 노드로 하는 그래프를 만듭니다. 노드 사이의 레그는 격자 Dijkstra + 직선화로 구하고, 오차 예산(`z·σ`)과 장애물 이격 여유를 만족하는 레그만 남깁니다. 그 위에서 최소 시간 순서를 고릅니다.
3. **검증** - 광류 스케일 오차, 요 드리프트, 랜덤워크, 태그 인식 확률을 주입한 몬테카를로를 돌려 성공률/충돌률이 기준을 넘는지 봅니다.
4. **수정** - 충돌이 많으면 마진을 키우고, 태그 인식 실패가 많으면 태그 간격을 줄이고, 오차 예산이 모자라면 **태그를 설치할 위치를 제안**합니다. 자동으로 풀 수 없는 문제(예: abort 정책인데 재밍 통과가 필수)는 반복하지 않고 사용자 결정을 요구합니다.

모든 판단은 결정론적이며 `agent_trace.json`과 리포트에 남습니다. LLM 없이 재현 가능하고 검증 가능하도록 일부러 그렇게 만들었습니다. 자연어 시나리오 입력이나 브리핑 생성처럼 LLM이 도움이 되는 부분은 `scenario.py` 앞단에 붙이는 방식으로 확장할 수 있습니다.

### 빠른 시작

```powershell
cd C:\Users\neoar\Desktop\d4d_2
$env:PYTHONPATH = "src"

# 전장 분석만
python -m d4d_planner analyze scenarios\demo_arena.json

# 분석 -> 계획 -> 검증 -> 산출물
python -m d4d_planner plan scenarios\demo_arena.json --out out\demo_arena
python -m d4d_planner plan scenarios\demo_jam.json   --out out\demo_jam
python -m d4d_planner plan scenarios\demo_spider.json --out out\demo_spider   # 노드 교신 + 메시지 운반

# 정책 덮어쓰기 / 몬테카를로 횟수 / 태그 제안 끄기
python -m d4d_planner plan scenarios\demo_jam.json --policy abort
python -m d4d_planner plan scenarios\demo_jam.json --runs 1000 --seed 3 --no-suggest-tags

# 테스트
python -m unittest discover -s tests -v
```

`pip install -e .` 하면 `d4d-plan plan ...` 으로도 실행됩니다. 종료 코드는 검증 통과 0, 미통과 1, 시나리오 오류 2입니다.

### 산출물 (`--out` 폴더)

| 파일 | 내용 |
|------|------|
| `mission.json` | 웨이포인트(좌표, 속도, 종류, 예상 오차, 링크 상태, 비상 착륙 지점), 태그 목록(설치 필요 여부), 실행 규칙 |
| `mission_data.h` | 위 내용을 C 구조체 배열로 변환. XIAO ESP32S3 펌웨어에 `#include` |
| `plan.svg` | 지도 + 경로 + 태그 + 오차 주입 시뮬 궤적 (브라우저로 열기) |
| `report.md` | 분석, 코스 표, 검증 결과, 설치할 태그, 에이전트 판단 기록, 가정/한계 |
| `agent_trace.json` | 에이전트의 단계별 판단 기록 |

웨이포인트 종류: `takeoff`, `goto`, `fix`(태그에서 위치 보정), `hover`, `land`, `comm`(노드 위에서 근접 BLE 교신), `read`(노드 디스플레이 코드 판독).

### 시나리오 형식

`scenarios/demo_arena.json`을 복사해서 수정하세요. 좌표는 필드 좌하단이 원점인 m 단위(x=동, y=북)입니다.

```json
{
  "name": "내 전장",
  "link_policy": "abort | continue_mission",
  "field": { "width_m": 40, "height_m": 24, "resolution_m": 0.25 },
  "start": { "x": 2, "y": 3 },
  "objectives": [ { "name": "관측점A", "x": 20, "y": 17, "action": "hover", "dwell_s": 3 } ],
  "goal": { "name": "착륙지점", "x": 37, "y": 4, "tolerance_m": 1.0 },
  "tags": [ { "id": 1, "x": 5.4, "y": 10.4 } ],
  "zones": [
    { "kind": "obstacle | keepout | jam | low_flow", "shape": "rect", "x": 12, "y": 0, "w": 1, "h": 15 },
    { "kind": "jam", "shape": "circle", "x": 20, "y": 5, "r": 3.5, "value": 0.9 },
    { "kind": "low_flow", "shape": "polygon", "points": [[28,1],[35,1],[35,8]], "value": 3.0 }
  ],
  "landing_sites": [ { "x": 2, "y": 20 } ],
  "drone": { "tag_detect_radius_m": 1.0, "flow_scale_sigma": 0.03 }
}
```

- `obstacle`/`keepout`: 통과 불가. `jam`: EMP/재밍으로 링크가 끊기는 구역(강도 0~1). `low_flow`: 연기/무늬 없는 바닥 등 광류가 나빠지는 구역(드리프트 배수 1 이상, 감속).
- `objectives`는 순서대로 방문합니다. 중간 목표는 다음 구간을 시작할 수 있도록 태그 인식 반경의 절반 안으로 정밀하게 도착해야 하므로, 근처에 태그가 필요합니다. 없으면 에이전트가 제안합니다.
- `drone`에서 바꿀 수 있는 항목은 `src/d4d_planner/model.py`의 `DroneSpec`을 보세요.

### 스파이더 코드 네트워크 (노드 교신 + 메시지 운반)

RF가 막힌 구역에서 드론이 **데이터 뮬**로 동작합니다. 코드가 붙은 지상 ESP32 노드 위에서 위치를 보정하면서 메시지를 싣고 내려놓습니다. 태그에 `role`을 주고 `messages`를 적으면 됩니다.

```json
"tags": [
  { "id": 2, "x": 12.5, "y": 19, "role": "ble",     "payload_bytes": 32 },
  { "id": 4, "x": 31,   "y": 19.5, "role": "mailbox" }
],
"messages": [
  { "name": "관측센서_로그", "from": 2,       "to": "goal", "bytes": 4096, "priority": 1 },
  { "name": "미션갱신_명령", "from": "start", "to": 4,      "bytes": 256,  "priority": 2 }
]
```

| 역할 | 장치 | 교신 방식 | 재밍 영향 |
|------|------|-----------|-----------|
| `marker` (기본) | 종이 AprilTag | 위치 보정만 | 없음 |
| `mailbox` | AprilTag + ESP32 디스플레이 | 카메라로 코드를 읽는 **광학** 교신 (`read`) | 없음 |
| `ble` | AprilTag + ESP32 BLE (XIAO ESP32S3, NUS) | 태그 위 `comm_altitude_m`에서 **근접 BLE** 교신 (`comm`) | 강도에 따라 유효 거리 축소 |

- `from`/`to`는 노드 id 또는 `start`(출발 지상국)/`goal`(착륙 지상국)입니다. `marker` 태그에는 메시지를 둘 수 없습니다.
- 교신 지점은 `priority`와 "싣기 전에 내려놓지 않는다" 규칙으로 순서가 정해지고, 사용자 `objectives`보다 **먼저** 방문합니다. 같은 노드를 다시 가야 하면 자동으로 두 번 넣습니다.
- 검증 지표에 **메시지별 전달률**이 추가됩니다(`min_delivery`, 기본 90%). 재밍 안 BLE 노드 때문에 전달률이 모자라면 에이전트가 그 노드를 **광학 메일박스로 전환 제안**하고 재검증합니다. 리포트의 `3-1. 네트워크` 절과 `mission.json`의 `network`, `mission_data.h`의 `MISSION_MSGS`에 결과가 들어갑니다.
- 교신 모델(`ble_range_m` 3 m, 재밍 1.0에서 범위 30%, BLE 7 KB/s, 광학 300 B/s, 판독률 90% 등)은 `기기정보/08-bluetooth.md` 실측과 링크 버짓 추정에서 나온 **가정치**입니다. BLE 성공률 vs 거리(간섭 포함)와 디스플레이 코드 판독률을 실측해 `DroneSpec`을 보정하세요.
- 카메라 수평 화각 55°(`기기정보/05`) 기준으로 고도 1.2 m에서 보이는 반폭은 약 0.62 m입니다. 기본 `tag_detect_radius_m`(1.0 m)이 이보다 크면 분석 단계에서 경고합니다.

### 데모 결과

| 시나리오 | 설명 | 결과 |
|----------|------|------|
| `demo_arena` | 재밍 구역을 우회, 태그 사전 설치 | 태그 제안 없이 통과 (성공률 약 98%) |
| `demo_jam` | 재밍 구역 통과 필수, 태그 부족 | 에이전트가 태그 4개 설치 위치를 제안하고 통과 |
| `demo_jam --policy abort` | 위와 같은데 정책만 abort | 불가능으로 진단하고 사용자 결정을 요구 |
| `demo_spider` | 재밍 구역 안 BLE 노드에서 4 KB 로그 회수, 명령 배달, 노드 간 중계 | 태그 5개 제안 + 재밍 안 노드 2를 광학 메일박스로 전환 제안, 전달률 100%로 통과 |

## d4d_fc - 비행제어기 조사 도구

INAV 비행제어기와 MSP 프로토콜로 통신합니다. **읽기 전용**입니다. 설정을 바꾸거나 모터를 돌리는 명령은 보내지 않습니다. 쓰기 명령 코드는 `codes.py`에 목록으로만 있고, 테스트가 조사 코드에서 쓰이지 않는지 확인합니다.

```powershell
python -m d4d_fc ports                                   # 포트 목록
python -m d4d_fc inspect --port COM9 --out out\fc        # 전체 조사 + 준비상태 점검
python -m d4d_fc inspect --port COM9 --no-settings       # 설정 666개 덤프 생략
python -m d4d_fc monitor --port COM9 --seconds 10 --hz 5 # 자세/고도/RC/모드 실시간
python -m d4d_fc snapshot --port COM9                    # 오프라인 개발용 전체 스냅샷
```

### 케이블 없이 개발하기

`snapshot`이 FC 응답을 **원본 바이트까지** 저장해 두므로, 케이블을 뽑은 뒤에도 모든 도구가 그대로 돌아갑니다. `--port` 대신 `--replay`를 쓰면 됩니다.

```powershell
python -m d4d_fc inspect --replay snapshots\fc_snapshot.json --out out\fc
python -m d4d_fc monitor --replay snapshots\fc_snapshot.json --seconds 5
python -m unittest discover -s tests          # 62개 테스트, 하드웨어 불필요
```

`snapshots/fc_snapshot.json`(약 315KB)에 담긴 것:

| 항목 | 내용 |
|------|------|
| 조회 응답 | 67종 (미지원 9종은 이름과 함께 기록) |
| 설정 | 666개 전부, 원본 바이트 + 해석 결과 + 이름→인덱스 색인 |
| 실시간 시계열 | 10종 × 60회 (6Hz 실측). 재생 시 순환하므로 모니터가 계속 동작 |

`snapshots/fc_settings_dump.txt`는 같은 설정을 INAV CLI의 `set 이름 = 값` 형식으로 적어 둔 것입니다. 설정을 바꾸기 전 상태를 되돌릴 기록으로 쓰거나, 나중 상태와 비교하는 데 씁니다. **CLI에 붙여넣기 전에 내용을 확인하세요.**

`ReplayClient`는 `MspClient`와 같은 인터페이스라 코드에서도 바꿔 끼울 수 있습니다.

```python
from d4d_fc.replay import ReplayClient
with ReplayClient.from_file("snapshots/fc_snapshot.json") as fc:
    print(fc.setting("failsafe_procedure")["value_name"])   # DROP
```

### PID 튜닝 분석

```powershell
python -m d4d_fc tune                                    # 현황만
python -m d4d_fc tune --symptom highfreq --out out\fc    # 증상별 변경안
```

증상 키워드: `firstflight`(아직 안 날려봤다), `highfreq`(고주파 떨림), `wobble`(저주파 요동), `bounce`(정지 후 되튐), `propwash`(하강 중 흔들림), `poshold`(위치유지 중 왕복).

**이 명령은 값을 적용하지 않습니다.** 현재 상태를 읽고 변경안을 `set` 명령 형태로 기록만 합니다. 제안값은 설정의 `min`/`max` 범위 안으로 잘립니다.

**이 기체에서 가장 중요한 발견: EZ Tune이 켜져 있습니다** (`ez_enabled = ON`). INAV는 EZ Tune이 ON인 동안 `mc_p_*`/`mc_i_*`/`mc_d_*`/`mc_cd_*`와 자이로·D텀 필터를 슬라이더에서 다시 계산합니다. 따라서 **원시 PID를 직접 수정해도 유지되지 않습니다.** 튜닝은 `ez_*` 슬라이더로 해야 합니다. 단 항법 PID(`nav_mc_*`)는 EZ Tune이 건드리지 않으므로 직접 조정합니다.

현재 슬라이더: `ez_filter_hz=110`, `ez_response=95`, `ez_damping=115`, `ez_stability=110`, `ez_aggressiveness=80`.

### ANGLE 모드 상시 활성 (이 프로젝트의 유일한 쓰기 기능)

```powershell
python -m d4d_fc angle-always --port COM9                  # 미리보기만
python -m d4d_fc angle-always --port COM9 --apply          # RAM 적용 (전원 뽑으면 원복)
python -m d4d_fc angle-always --port COM9 --apply --save   # EEPROM 영구 저장
```

INAV에는 모드를 "항상 켜기"로 두는 설정이 따로 없습니다. 대신 범위를 **900~2100us 전 구간**으로 주면, 펌웨어가 채널 값을 900~2099로 제한해 비교하므로 어떤 값이든 범위 안이 되어 항상 활성입니다. 수신기가 없어 그 채널이 존재하지 않아도 켜집니다.

`src/d4d_fc/write.py`는 허용 명령을 **`MSP_SET_MODE_RANGE`와 `MSP_EEPROM_WRITE` 두 개로만** 제한합니다(`ALLOWED_WRITES`). 그 외는 `_guard()`가 거부하고, 테스트가 이를 검증합니다.

적용 순서가 안전하게 짜여 있습니다.

1. 무장 상태면 거부
2. 현재 모드 범위를 `snapshots/mode_ranges_backup_*.json`에 백업 (INAV CLI `aux` 복구 줄 포함)
3. **RAM에만** 기록 — 전원을 뽑으면 원래대로 돌아갑니다
4. 되읽어 바이트 일치 확인, 다르면 거기서 중단하고 EEPROM에 쓰지 않음
5. `MSP_ACTIVEBOXES`로 ANGLE이 실제 켜졌는지 확인
6. `--save`를 줬을 때만 EEPROM 저장

### 블랙박스 로그 분석

```powershell
python -m d4d_fc log E:\LOGS\LOG00001.BBL --list          # 세션 목록
python -m d4d_fc log E:\LOGS\LOG00001.BBL --out out\log   # 분석
python -m d4d_fc log ... --seconds 0                      # 전체 (40MB 기준 약 36초)
```

`.BBL` 파일을 직접 해석해 자이로 스펙트럼을 내고 증상을 판정합니다. 외부 의존성 없이 표준 라이브러리만 씁니다.

- `bbl.py` — 로그 디코더. 헤더 파싱, I/P 프레임, 예측기, `TAG2_3S32`/`TAG8_4S16`/`TAG8_8SVB` 인코딩, 손상 시 재동기화
- `logan.py` — radix-2 FFT, Hann 창 Welch 평균, 대역 RMS, 봉우리 검출, 증상 분류

대역 RMS는 실제 RMS와 일치하도록 정규화돼 있습니다(정현파 진폭 30이면 RMS 21.2). 처리 속도는 약 1.1 MB/s입니다.

**알아둘 한계가 있습니다.**

- 테스트의 왕복 검증은 **이 구현의 내부 일관성**만 보장합니다. INAV가 쓰는 실제 바이트 배치와 일치하는지는 진짜 로그 파일로만 확인됩니다. 그래서 디코더는 `loopIteration` 증가, `time` 단조증가, 값 범위를 계속 검사하고, 어긋나면 `resyncs`로 보고합니다. **보고서에 재동기화 횟수가 많으면 결과를 믿지 마세요.**
- 판정 기준값(`TH_HIGH_RMS` 등)은 멀티로터 튜닝의 통용 경험치이고 이 기체로 보정한 값이 아닙니다.
- `TAG2_3SVARIABLE`(인코딩 10) 은 구현하지 않았습니다. 해당 로그는 명시적으로 오류를 냅니다.
- 자이로 단위는 펌웨어로 판단합니다. INAV는 `gyroADC`를 deg/s로 기록하지만 Betaflight는 원시값이라, 그 경우 절대 기준값 판정을 끄고 봉우리 주파수만 봅니다.

### 블랙박스 상태 읽는 방법 (주의)

`blackbox_device`는 **반드시 설정 API에서 읽어야 합니다.** INAV 9의 `MSP_BLACKBOX_CONFIG`(80)는 더 이상 지원되지 않아 항상 `00000000`을 돌려줍니다. 이 응답의 첫 바이트를 device로 해석하면 로깅이 꺼진 것으로 오진합니다.

이 기체의 실제 상태는 `blackbox_device = SDCARD`, `FEATURE_BLACKBOX` 활성, SD카드 정상(3.89/3.93GB), 기록 비율 1/2(약 1kHz)로 **이미 로깅이 가능한 상태**입니다.

### 수집 시 안전장치

`snapshot`은 **허용목록에 있는 조회 명령만** 보냅니다. 목록 밖의 코드는 코드 차원에서 예외가 발생하므로, 캘리브레이션 시작(205·206), 설정 초기화(208), EEPROM 기록(250), 재부팅(68), 플래시 소거(72) 같은 명령이 실수로 나갈 수 없습니다. 테스트가 이 차단을 검증합니다.

### 수집 내용

- **식별**: 펌웨어 종류/버전, 보드 이름, 빌드 날짜, UID
- **상태**: 감지된 센서, CPU 부하, 루프 주기, I2C 오류, 무장 차단 사유(한국어)
- **실시간**: 자세, 고도, IMU, 거리센서, 광류, 배터리, RC 18채널, 모터 출력
- **모드**: 사용 가능한 비행모드 32개, 조종기 스위치 할당, 현재 활성 모드
- **하드웨어**: UART 9개의 기능/보레이트, 모터 믹서, Failsafe 설정, feature 마스크
- **설정 전체**: INAV 설정 666개를 이름·타입·범위·선택값·현재값까지 (`fc_settings.md`)

설정 전체 덤프가 조정 툴의 기반입니다. `MSP2_COMMON_PG_LIST`로 인덱스 범위를 구하고 `MSP2_COMMON_SETTING_INFO`로 하나씩 읽습니다. 약 7초 걸립니다.

### 산출물

| 파일 | 내용 |
|------|------|
| `out/fc/fc_inspect.json` | 수집한 모든 데이터 (디코딩 실패 시 원본 hex 포함) |
| `out/fc/fc_readiness.md` | 이 프로젝트 요구사항 대조 결과와 조치 사항 |
| `out/fc/fc_settings.md` | 설정 666개 표 |
| `snapshots/fc_snapshot.json` | 오프라인 재생용 원본 스냅샷 (버전 관리 대상) |
| `snapshots/fc_settings_dump.txt` | CLI `set` 형식 설정 기록 |

### 주의

- INAV 9는 `MSP_BOXNAMES`(116)를 더 이상 제공하지 않습니다. 비행모드 이름은 `codes.py`의 `BOX_NAMES` 표로 붙입니다. 이 장치의 `BOXIDS` 순서, 가속도계 기반 모드 묶음, GPS 의존 모드 부재, PREARM 미충족 무장 차단과 교차 확인했지만 **최종 확인은 INAV Configurator로 하세요**.
- 설정 타입 번호(5=float, 6=string, 7=int32)는 이 장치의 실제 응답으로 검증했습니다. `inav_w_xy_flow_p`가 `0x3F800000`(=1.0), `opflow_scale`이 10.5로 나와 INAV 기본값과 일치합니다.
- 테스트는 `tests/fixtures_fc.json`에 캡처한 실제 응답을 쓰므로 하드웨어 없이 돌아갑니다.

## 안전 주의

- **드리프트 파라미터는 가정치**입니다 (광류 스케일 3%, 요 드리프트 0.02 rad 등). MTF-01P와 실제 FC 설정으로 비행 로그를 모아 `DroneSpec`을 보정하기 전에는 시뮬 성공률을 믿지 마세요.
- **현재 Failsafe는 DROP**입니다. 링크가 끊기면 모터가 정지해 그대로 떨어집니다. 어떤 비행 시험이든 이 값을 먼저 검토하세요.
- 조사 시점에 **수신기 신호가 없었고 FC는 FAILSAFE 상태**였습니다. 수동 회수 경로가 확보되지 않은 상태에서는 프로펠러를 달지 마세요.
- Notion 초기 시험 설계는 "조종기가 끊기면 자동 미션 중단"입니다. `continue_mission`은 이와 반대로 링크가 끊겨도 계속 비행하므로, 지오펜스, 비행 시간 제한, 물리적 격리 같은 별도 안전장치를 확보하고 사람이 없는 통제된 공간에서만 시험하세요.
- FC 펌웨어가 UART 명령 타임아웃 감시와 수동/자동 전환을 지원하는지는 아직 확인되지 않았습니다 (Notion에도 확인 항목으로 적혀 있음).
- 2D 계획입니다. 장애물이 비행 고도보다 낮은 경우는 다루지 않습니다.

## 다음 단계

실제 기체 조사 결과에 따라, 비행보다 먼저 해결해야 하는 것들입니다.

1. **MTF-01P 연결** - 지금 광류/거리 센서가 전혀 설정되어 있지 않습니다. 남는 UART(UART4 또는 UART5)에 `OPTICAL_FLOW`와 `RANGEFINDER` 기능을 함께 켜고 `opflow_hardware=MSP`, `rangefinder_hardware=MSP`로 설정해야 합니다. 이게 없으면 GPS 없는 위치 유지가 불가능합니다.
2. **Failsafe를 DROP에서 변경** - 지금은 링크가 끊기면 모터가 즉시 멈춥니다(추락). 센서를 붙인 뒤 `LAND`로 바꾸는 것을 검토하세요.
3. **MSP RC override 설정** - INAV에서 보드가 조종을 넘겨받는 방법입니다. `msp_override_channels` 비트마스크와 `MSP RC OVERRIDE` 모드 스위치가 Notion 설계의 "수동/자동 조종권 선택"에 해당합니다. 둘 다 아직 설정되지 않았습니다.
4. **XIAO 펌웨어** - `mission_data.h`를 읽어 웨이포인트를 따라가고, `fix`에서 태그를 찾고, 실패하면 `abort_land_at`으로 가는 실행기. FC에는 MSP RC override로 명령합니다.
5. **`DroneSpec` 실측 보정** - 광류 오차, 태그 인식 거리/확률을 비행 로그로 측정.

## SPIDER 지휘소 웹 UI

SPIDER 지휘소는 EMP 전에 전장을 편집하고 AI 에이전트의 계획 과정을 관찰한 뒤, 사람이 승인한 미션과 AprilTag 플랜 카드를 차폐함(둥지)에 넣기 위한 웹 앱입니다. EMP 뒤에는 API나 LLM 없이 승인된 패키지만 사용한다는 전제입니다. 제품 요구사항은 `docs/PRD.md`에 있습니다.

### Vercel 3D 훈련 시연

저장소 최상위의 `vercel.json`으로 웹 UI만 정적 배포합니다. 루트 디렉터리는 저장소 최상위, 프레임워크는 `Other`로 설정합니다. `web` 밖의 `scenarios`도 빌드에 필요합니다.

```sh
npm --prefix web ci
npm --prefix web run build:demo
npm --prefix web run preview -- --host 127.0.0.1
```

- `build:demo`는 `web/.env.demo`의 `VITE_DEMO_MODE=true`를 사용합니다. 3D 지도, 격자 경로 생성, 가상 자동 비행, AprilTag 영상 인식, 조종기 화면, 한국어·영어 전환을 브라우저에서 실행합니다.
- 시연의 기본 화질은 성능 모드입니다. 하단 화질 선택에서 고화질로 전환할 수 있습니다.
- 자연어 입력은 화면에 안내된 한국어·영어 문법으로 해석합니다. AI API를 호출하지 않습니다.
- 시연에서는 API 상태 조회, 실기체 연결, 서버의 계획 승인·저장 메뉴를 사용하지 않습니다. 비행 경험은 브라우저 저장소에만 남습니다.
- 일반 `npm run dev`와 `npm run build`는 기존 로컬 API·실기체 기능을 유지합니다.
- `.vercelignore`는 웹 소스·공개 에셋·시나리오 JSON만 업로드합니다. Python 서버, 펌웨어, 장치 로그, 로컬 환경 파일은 제외합니다.

Vercel CLI에 로그인한 뒤 저장소 최상위에서 배포할 수 있습니다.

```sh
npx vercel deploy --prod --project mango-training --scope jikimes-projects --yes
```

### 개발 실행

```powershell
# 최초 1회
pip install -e .
cd web
npm install
cd ..

# 백엔드와 프론트 개발 서버를 함께 실행
powershell -ExecutionPolicy Bypass -File scripts\dev.ps1
```

- 지휘소 UI: `http://127.0.0.1:5173`
- API 문서: `http://127.0.0.1:8765/docs`
- LLM 없이도 기본값(`D4D_LLM_PROVIDER=none`)으로 전체 데모가 동작합니다.
- OpenAI 또는 Bedrock 연결은 `.env.example`을 `.env`로 복사한 뒤 값만 채우세요. 키와 `.env`는 버전 관리하지 않습니다.
- 플랜 카드에는 시스템의 한글 글꼴이 필요합니다. Windows는 `%WINDIR%/Fonts/malgun.ttf`(맑은 고딕),
  Linux는 `/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc` 등 Noto Sans CJK 또는
  `/usr/share/fonts/truetype/nanum/NanumGothic.ttf`를 차례로 찾습니다. 글꼴을 자동 다운로드하지 않습니다.
  후보 글꼴이 없으면 깨진 PNG 대신 오류를 내므로 카드 생성과 계획 완료가 실패할 수 있습니다.
  지원 경로는 `src/d4d_web/cards.py`의 `CARD_FONT_PATHS`를 확인하세요.

### 2분 데모

1. 전장 편집기에서 `demo_spider`를 선택합니다.
2. 자연어 입력에 `노드 3을 대피소로, 사진 30KB 회수 메시지 추가`를 넣고 diff를 적용합니다.
3. 계획을 실행해 제안 태그와 BLE→광학 메일박스 전환, 최종 전달률을 확인합니다.
4. `임무 승인`을 눌러 플랜 카드와 둥지 패키지를 엽니다. 승인 전에는 패키지를 받을 수 없습니다.
5. EMP 드릴을 재생해 카나리아 정지→둥지 개방→카드 판독→비행→교신→귀환을 확인합니다.
6. **접근 시뮬레이션**에서 태그 상실·복귀 각본과 수동→자동 기준점 연결을 봅니다. 화면의
   `SIMULATION · 실기기 아님` 표기대로 보드나 FC에 명령을 보내지 않는 시연입니다.

### 실기기 제어 패널

하단 **실기기 제어** 베이에서 XIAO를 BLE로 연결하고 **제어 패널**을 열면 FC 상태, 센서, CH6 제어권,
안전 게이트, 보드 명령, RAM 마스크 시험을 한 화면에서 다룹니다.

- FC USB 조회는 `GET /api/hardware/fc/status`를 3초마다 호출합니다(COM9를 짧게 열었다 닫음).
- 보드 명령은 `info`, `tags`, `snap`, `msp on/off`, `mission reset`, `kill`, `kill off`(2단계 확인),
  `approach`, `approach params`, `approach shadow/off/reset`, `approach set <허용 이름> <범위 안 값>`,
  `approach on/legacy`(2단계 확인 + 지상 시험 게이트)만 보냅니다. `max_speed` UI 상한은 10cm/s입니다.
- 펌웨어 `-25`의 RC 프레임은 MSP 활성 시 CH6과 무관하게 50Hz 상시 송신됩니다. `rc_tx`는 송신 상태,
  최신 `authority`는 제어권입니다. 보드 스로틀은 `rcmap`의 `thr_slot`에 1000µs 고정이며 FC 스로틀은
  조종기가 제어합니다. `vz` 경고는 기록·표시만 하고 스로틀을 조정하지 않습니다.
- RAM 마스크는 7(롤·피치·요), 71(롤·피치·요·CH7), 67(롤·피치·CH7), 64(CH7 복구)만 허용합니다.
  **7은 보드 CH7 킬을 전달하지 못합니다.** 킬 버튼은 보드 요청이며 어떤 마스크에서도 FC 정지·디스아밍을
  보장하지 않습니다. 스로틀 논리 비트 8은 모든 허용 마스크에서 제외합니다.
- RAM 마스크 변경은 `D4D_HARDWARE_WRITES=1`로 API를 띄우고 확인 문구
  `프로펠러 제거 / 디스아밍`을 입력해야 하며, 30초 일회성 토큰으로 2단계 적용합니다. EEPROM에는 쓰지 않습니다.
- `-25` 텔레메트리·파라미터 지원은 소프트웨어 변경입니다. 연결된 보드의 빌드·FC 설정·실기기 동작 검증은
  별도로 확인해야 합니다. API의 적용 게이트는 `can_apply_override`이며 기존 `can_apply_67`도 유지합니다.
- 아밍, 스로틀, 모터 시험, 원시 MSP는 UI에 없습니다. 절차와 차단 조건은 `docs/HARDWARE_CONTROL.md`를 보세요.

```powershell
$env:D4D_HARDWARE_WRITES = "1"     # RAM 마스크 변경을 허용할 때만
$env:D4D_FC_PORT = "COM9"
powershell -ExecutionPolicy Bypass -File scripts\dev.ps1
```

### 산출물 상태

웹 실행 결과는 `artifacts/{run_id}/`에 저장됩니다.

- `draft`: 생성됐지만 운용자 미승인
- `approved`: 운용자가 승인해 패키지 사용 가능
- `stale`: 원본 시나리오가 바뀌어 재계획 필요
- `failed`: 계획 또는 검증 미통과

현재 기체의 Failsafe는 여전히 `DROP`입니다. UI의 `검증 통과`는 시뮬레이션 기준이며, 실기기 제어 패널의 `지상 시험 가능` 역시 프로펠러 제거 상태의 채널 시험 허용을 뜻할 뿐 비행 허가가 아닙니다.
