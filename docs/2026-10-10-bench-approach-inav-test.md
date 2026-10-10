# 지상 시험: 태그를 손으로 움직여 INAV 값 변화 확인 — 펌웨어 `TAGVISION-20261010-29`

2026-10-10 밤. 기체(프로펠러 없음)를 받침 위에 두고 카메라 아래에서 AprilTag를 손으로 움직였을 때, 보드가 계산한 조정값(롤·피치)이 FC(INAV)의 채널 값으로 실제 들어가는지 확인했다.

## 1. 요구

- 드론이 떠 있을 때 카메라가 AprilTag를 인식하고, 좌·우·앞·뒤 조정값을 FC에 보낸다.
- 태그를 인식하면 **속도를 제어**한다. 지나치면 안 되므로 멀면 빠르게, 가까우면 천천히 움직인다. 최대 속도는 10cm/s로 일정하게 둔다.
- 근거는 공식 문서·소스·논문으로 확인한다(3절).
- 지상에서 태그를 손으로 움직였을 때 **INAV(다른 PC의 Configurator) 수치가 바뀌는지** 확인한다.

## 2. 지상 시험을 위해 바꾼 것

팀원이 만든 접근 제어기(`-23`~`-26`, `approach.h`)는 이미 거리 기반 속도 프로파일을 갖고 있다. 25cm 밖은 10cm/s, 25cm 안은 거리에 비례해 감속, 5cm 안은 정지, 가속 10cm/s², 실측 12cm/s 초과 시 제동이다. 다만 아래 두 조건 때문에 작업대 위에서는 명령이 나오지 않았다.

| 막힌 이유 | 지상 시험 대응 (`approach bench`) |
|---|---|
| 높이(MTF-01P)가 20~150cm여야 함. 작업대 위에서는 몇 cm라 `FAULT/height` | `height_src 1`: 거리센서가 무효면 **태그 크기로 잰 카메라~태그 거리**를 높이로 쓴다 |
| 속도를 FC 광류로만 재는데, 기체가 서 있어 손으로 태그를 움직여도 의미가 없음 | `vel_src 1`: **카메라로 잰 상대 속도**(−Δ오차/Δt, 지수평활 `tag_vel_alpha` 0.5)를 쓴다. 광류 품질 대신 태그 신선도(`tag_stale_ms`)로 판단 |

- `approach bench` = 위 두 입력 + 출력 모드 `on`. `approach flight`로 비행 입력(거리센서·광류)으로 되돌린다. 재부팅하면 비행 입력, `shadow` 모드로 돌아간다.
- 제어 로직(프로파일·가속 제한·속도 P·제동·도착 판정)은 그대로이고, 입력 출처만 바꾼다.
- 그 밖의 변경:
  - `-28`: `mspTask` 스택 6KB → 10KB. 접근 보고(1000바이트 줄과 실수 서식)를 이 태스크가 만드는데, 측정해 보니 **약 5.6KB를 쓰고 있었다**. `info`의 `stack_free`로 확인한다.
  - `-29`: 시험에 쓴 종이 태그 **ID 1을 미션 목표로 등록**했다(크기 실측 전, 200mm 가정 → `size 1 <mm>`로 보정).
- PC 시험(`test/approach_host_test.cpp`)에 지상 시험 경우 3개를 추가했다. 카메라 속도는 품질 조건을 무시하고 600ms 신선도로 판단하는지, 태그를 가만히 들면 +20µs로 수렴하는지, 20cm/s로 가져오면 과속 제동(−20µs 미만)이 걸리는지를 본다. 기존 테스트와 함께 통과했다.

## 3. 근거 (공식 문서·소스·논문)

| 항목 | 내용 | 출처 |
|---|---|---|
| MSP RC Override | 모드가 켜져 있고 `msp_override_channels`에 비트가 있는 채널만 적용된다. 5Hz 이상 필요하고, 끊기면 약 0.7초 뒤 수신기 값으로 돌아간다(그동안 마지막 값 유지). 마스크 비트는 INAV 내부 순서(롤 0·피치 1·요 2·스로틀 3) | [Rx.md](https://github.com/iNavFlight/inav/blob/master/docs/Rx.md), [msp_override.c](https://github.com/iNavFlight/inav/blob/master/src/main/rx/msp_override.c) |
| ANGLE 모드 변환 | `기울기 = rcCommand/500 × max_angle_inclination(45°)`. **기본 `rc_expo 70`이 ANGLE에도 적용**되어 작은 변위는 약 0.3배로 약해진다(10µs ≈ 0.22°, 80µs ≈ 2.2°). Configurator Receiver 탭 값은 expo **적용 전**이다 | [fc_core.c](https://github.com/iNavFlight/inav/blob/master/src/main/fc/fc_core.c), [rc_curves.c](https://github.com/iNavFlight/inav/blob/master/src/main/fc/rc_curves.c) |
| 속도 루프 지연 | 현재 이득(2µs per cm/s, expo 70)이면 시정수가 약 1.1초다. 감속 명령 때 실제 속도가 약 5cm/s 뒤처진다 → 지나침의 주원인 후보 | 위 변환식으로 계산 |
| 지나치지 않는 속도 | `v²/(2a) + v·(L + 1/f) ≤ r`. 4fps, 지연 0.3초, a = 10cm/s², r = 5cm이면 **약 6cm/s 이하** | 직접 유도. 지연이 위상을 깎는 일반 원리: [MIT 16.30](https://ocw.mit.edu/courses/16-30-feedback-control-systems-fall-2010/cb4fd48f2435b6d67fb6d3c3ecf9bc86_MIT16_30F10_rec11.pdf) |
| 정밀착륙 사례 | PX4: 수평 반경 안에서 2초 머문 뒤 하강, 표적은 칼만 추정. ArduPilot: 촬영 시점 자세로 시선벡터 보정 + 지연(`PLND_LAG`) 앞당김, 근거리 최소 하강 0.1m/s | [PX4 precland](https://docs.px4.io/main/en/advanced_features/precland.html), [AC_PrecLand.cpp](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AC_PrecLand/AC_PrecLand.cpp), [mode.cpp](https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/mode.cpp) |
| 논문 | AprilTag 약 6.5Hz에서도 평활 후 실내 착륙오차 약 0.1m, 오차 원인은 흐림과 지연(Malyuta 등). 상대 위치·속도 KF 후 근거리 PID(Borowczyk 등) | [arXiv 1908.06381](https://arxiv.org/abs/1908.06381), [arXiv 1611.07329](https://arxiv.org/abs/1611.07329) |
| 카메라 속도 잡음 | 위치 미분 잡음 ≈ √2·σp·f. σp = 1cm, 4fps면 약 5.7cm/s → **비행 속도는 광류로, 카메라는 위치로** 쓰는 것이 맞다. 카메라 속도(`vel_src 1`)는 지상 시험 전용 | [alpha-beta filter](https://en.wikipedia.org/wiki/Alpha_beta_filter) |

## 4. 시험 절차

1. FC: 디스아밍, 프로펠러 없음, 배터리 3S(12.1V). CH6 = 1000(AUTO), FC 마스크 7(롤·피치·요).
2. XIAO: **USB 전원만** 사용(빨간 5V 선 분리, GND·신호선만 FC에 연결). FC 5V와 USB를 함께 넣으면 USB 시리얼이 자주 멈췄다.
3. 기체를 받침 위에 올려 카메라~태그 거리 약 60~85cm. 태그 위 케이블 제거.
4. 이 PC에서 카메라 창을 띄운다. 시험 명령을 자동으로 보내고, 화면에 보드 명령과 **INAV 롤·피치 값**을 표시하며, 기록을 남긴다.
   ```bash
   B=~/.local/share/uv/python/cpython-3.12.10-macos-aarch64-none/lib
   cd /tmp && TCL_LIBRARY=$B/tcl8.6 TK_LIBRARY=$B/tk8.6 uv run --no-project --python 3.12 --with pyserial --with pillow \
     python3 <D4D>/tools/cam_view.py --port /dev/cu.usbmodem1101 \
     --init "approach bench;approach set vel_kp 6;approach set tag_stale_ms 1500" --ap-log ap.jsonl
   ```
   - 이득 6은 Receiver 탭에서 잘 보이게 하려는 **지상 시험 전용** 값이다(기본 2).
   - 카메라 창이 스냅샷을 계속 받아 인식이 초당 약 1.5회로 느려지므로 태그 신선도를 1.5초로 늘린다.
5. 다른 PC: INAV Configurator Receiver 탭으로 Roll·Pitch를 본다.

## 5. 결과 (228초 기록, 접근 보고 1,053개)

**보드가 계산한 조정값이 INAV 채널 값으로 그대로 들어갔다.** 같은 순간의 보드 송신값과 FC 채널 값 차이는 중앙값 0µs다(90분위 39µs는 두 값의 갱신 시각 차이).

| 확인할 것 | 기록 | 보드 명령 → INAV | 판정 |
|---|---|---|---|
| 태그 뒤쪽 → 뒤로 | 124.5초: 뒤 13.9cm, 목표 7.2cm/s | 피치 −61µs → **1435** | ✅ |
| 태그 앞쪽 → 앞으로 | 165.5초: 앞 6.7cm | 피치 +41µs → **1556** | ✅ |
| 태그 왼쪽 → 왼쪽으로 | 19.6초: 왼쪽 22.8cm | 롤 −72µs → **1454** | ✅ |
| 태그 오른쪽 → 오른쪽으로 | 163.4초: 오른쪽 7.0cm | 롤 +13µs → **1511** | ✅ |
| 거리에 비례한 속도 | 25.3cm → 10.0, 10.3cm → 4.1, 7.0cm → 2.8cm/s | — | ✅ |
| 5cm 안 정지 | 139.7초 3.7cm, 180.9초 4.6cm | 0 → 1500 | ✅ |
| 과속 제동 | 상대 속도 12cm/s 초과 25회 (예: 198초 33cm/s) | 목표 0 → 반대 방향 명령 | ✅ |
| CH6 수동 → 조종기 | 126~135초 | 보드 명령 0, INAV는 조종기 스틱(피치 1239~1612) | ✅ |

- AUTO 중 INAV 값 범위(보드 명령으로 인한 것): 롤 약 1426~1534, 피치 약 1422~1556.
- 방향 부호(`roll_sign`·`pitch_sign` = 1)는 화면 기준으로 맞았다. 실제 기체 이동 방향은 비행 전에 다시 확인한다.

## 6. 문제와 원인

| 문제 | 원인 | 대응 |
|---|---|---|
| 처음 두 번은 태그가 전혀 인식 안 됨 | (1) 카메라~태그 26~33cm로 너무 가까워 테두리가 화면 밖으로 잘림, (2) 케이블이 태그를 가림 | 받침으로 높이고 케이블 제거. 200mm 태그는 45cm 이상 |
| 인식은 되는데 제어에 안 들어감 | 시험 태그가 **ID 1(미등록)**. 미션 목표는 ID 0·10만 | `-29`에서 ID 1 등록 |
| 명령이 작아 Receiver 탭에서 잘 안 보임 | 기체가 서 있으면 명령 ≈ 이득 2 × 목표속도 8~10 = 16~20µs | 지상 시험에서만 `vel_kp 6` |
| **태그 인식이 끊김 (기록의 47%만 인식)** | 카메라 창 스냅샷으로 인식 약 1.5회/s, 태그가 화면 가장자리, 손 움직임 흐림 | 끊기면 보드는 중립(1500)으로 수렴(안전). 비행은 카메라 창 없이, 높이는 거리센서 |
| 블루투스 기록이 37초에 멈춤 | 보드 출력 약 8.5KB/s(접근 보고 4.8KB/s 등)가 BLE 한도 약 7KB/s를 넘음 | 기록은 USB로. BLE용 출력 축소는 남은 과제 |
| USB 시리얼이 0바이트로 멈춤 (반복) | XIAO에 FC 5V와 USB가 동시에 들어감(다이오드 없음)으로 추정 | 시험 중에는 빨간 5V 선 분리. 멈추면 USB-C를 뽑았다 꽂는다 |

## 7. 비행 전에 할 일

1. `approach flight`로 되돌리고 이득을 비행용으로 다시 정한다. 지상 시험 이득 6은 쓰지 않는다.
2. 25cm 안에서는 속도를 **6cm/s 이하**로 제한한다(3절 식).
3. **가속 피드포워드**를 추가하고 `rc_expo`(ANGLE에도 적용)를 확인한다. expo를 0으로 두면 같은 µs의 효과가 약 3.3배가 된다.
4. 촬영 시점 자세로 기울기를 보정한다(`att_comp`). 1m에서 2.2° 기울면 약 3.8cm 밀려 보인다.
5. 보드 `kill`을 쓰려면 FC 마스크를 71로 바꾼다. FC 설정 변경이므로 승인이 필요하다.
6. 태그 ID 1의 실제 크기를 재서 `size 1 <mm>`로 넣는다.

> **이후 (`-30`~`-34`):** 3번은 기울기 출력(`rc_expo` 역변환, 피드포워드 0)으로 처리했다. 6번은 자동 보정으로 약 86~89mm를 쟀고, 실측 저장은 남았다. 비행 시험 순서와 광류 미검증 문제는 [비행 준비 보고서](2026-10-10-tilt-output-tagcal-flight-prep.md)에 있다.
