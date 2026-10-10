#!/usr/bin/env python3
"""태그인식 보드(XIAO ESP32S3 Sense) 모니터 — USB 시리얼 또는 블루투스(BLE).

펌웨어가 내보내는 JSON 줄을 한글로 풀어 보여준다.

  python3 tools/tag_monitor.py                 # 실시간 해석 (USB)
  uv run --with bleak --with pyserial python3 tools/tag_monitor.py --ble   # 블루투스로 연결
  python3 tools/tag_monitor.py --snap out.jpg  # 카메라 화면 한 장 저장
  python3 tools/tag_monitor.py --cmd "size 0 100"       # ID 0 태그 크기(mm)
  python3 tools/tag_monitor.py --cmd "land 10 -170 0"   # ID 10 중심 → 착륙 지점 오프셋(mm)
  python3 tools/tag_monitor.py --cmd "dec auto"         # decimate 자동 (기본)
  python3 tools/tag_monitor.py --cmd "msp off"          # FC로 MSP 송신 중지 (on 으로 재개)
  python3 tools/tag_monitor.py --cmd "mission reset"    # 미션 단계를 '정렬'로 되돌림
  python3 tools/tag_monitor.py --raw           # JSON 원문 그대로

주의: USB 포트를 열면 보드가 재부팅될 수 있다(ESP32-S3 내장 USB 특성). 블루투스는 재부팅하지 않는다.
블루투스는 BLE Nordic UART Service(이름 TagBoard-XXXX)이며, 다른 옵션은 USB와 똑같이 쓴다.
"""
import argparse
import asyncio
import base64
import glob
import json
import queue
import sys
import threading
import time

import serial


def find_port():
    ports = sorted(glob.glob("/dev/cu.usbmodem*")) + sorted(glob.glob("/dev/ttyACM*"))
    if not ports:
        sys.exit("보드 포트를 찾지 못했습니다. USB-C 연결을 확인하세요.")
    return ports[0]


def open_port(port):
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, 115200, 0.2
    s.dtr = False
    s.rts = False
    s.open()
    return s


def read_lines(s):
    buf = b""
    while True:
        chunk = s.read(4096)
        if not chunk:
            yield None
            continue
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            yield line.decode("utf-8", "replace").strip()


class BleLink:
    """BLE Nordic UART Service 연결. USB 쪽과 같은 write()/lines()를 제공한다."""

    TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # 보드 → PC (알림)
    RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # PC → 보드 (명령)

    def __init__(self, name_prefix="TagBoard", timeout=15.0):
        try:
            from bleak import BleakClient, BleakScanner
        except ImportError:
            sys.exit("블루투스 연결에는 bleak가 필요합니다:\n"
                     "  uv run --with bleak --with pyserial python3 tools/tag_monitor.py --ble")
        self.q = queue.Queue()
        self.buf = bytearray()
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self.loop.run_forever, daemon=True).start()

        async def connect():
            dev = await BleakScanner.find_device_by_filter(
                lambda d, ad: (d.name or ad.local_name or "").startswith(name_prefix), timeout=timeout)
            if dev is None:
                return None
            client = BleakClient(dev, disconnected_callback=lambda _: print("[블루투스] 연결 끊김", flush=True))
            await client.connect()
            await client.start_notify(self.TX, self._on_data)
            return client, dev

        found = asyncio.run_coroutine_threadsafe(connect(), self.loop).result(timeout + 20)
        if not found:
            sys.exit("블루투스로 보드를 찾지 못했습니다 (이름 TagBoard-…). 보드 전원과 거리를 확인하세요.")
        self.client, dev = found
        print(f"[블루투스] {dev.name} 연결됨 (MTU {self.client.mtu_size})", flush=True)

    def _on_data(self, _, data):
        self.buf.extend(data)
        while b"\n" in self.buf:
            i = self.buf.index(b"\n")
            self.q.put(bytes(self.buf[:i]).decode("utf-8", "replace").strip())
            del self.buf[:i + 1]

    def write(self, data):
        asyncio.run_coroutine_threadsafe(self.client.write_gatt_char(self.RX, data, response=False), self.loop).result(5)

    def lines(self):
        while True:
            try:
                yield self.q.get(timeout=0.2)
            except queue.Empty:
                yield None

    def close(self):
        try:
            asyncio.run_coroutine_threadsafe(self.client.disconnect(), self.loop).result(5)
        finally:
            self.loop.call_soon_threadsafe(self.loop.stop)


def wait_boot(lines, timeout=8.0):
    """재부팅 직후 명령이 무시되지 않도록 boot 메시지(또는 아무 JSON)를 기다린다."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        line = next(lines)
        if line and line.startswith("{"):
            return
    # 재부팅하지 않은 경우에도 계속 진행


def fmt_tag(o):
    reg = "" if o.get("registered") else " · 미등록"
    land = f" · 착륙점 오프셋 ({o['land_x_mm']},{o['land_y_mm']})mm" if (o["land_x_mm"] or o["land_y_mm"]) else ""
    return (
        f"  └ 태그 ID {o['id']} ({o['meaning']}{reg}) · 한 변 {o['tag_size_mm']}mm → {o['side_px']:.0f}px"
        f" · 회전 {o['rot_deg']:+.0f}° · 추정거리 {o['dist_est_cm']:.0f}cm · margin {o['margin']:.0f}{land}"
    )


def fmt_target(o):
    h = f"{o['height_cm']:.0f}cm" if o.get("height_cm") is not None else "없음"
    axes = "" if o.get("axes_verified") else " (축 방향 미검증)"
    many = f" · 후보 {o['candidates']}개 중 가장 크게 보이는 태그" if o["candidates"] > 1 else ""
    return (
        f"[판단] ID {o['id']} 기준{many} · 단계 {PHASE_KO.get(o.get('phase'), o.get('phase'))} · decimate {o['decimate']}\n"
        f"       {'착륙 지점 바로 위' if o['decision'] == 'OVER_TARGET' else '이동 필요'}"
        f" | 이동: {o['move_ko']}{axes}\n"
        f"       다음: {o['next_ko']}\n"
        f"       착륙 지점: 화면 중심 대비 오른쪽 {o['img_right_cm']:+.1f}cm · 위쪽 {o['img_up_cm']:+.1f}cm"
        f" | 추정거리 {o['dist_est_cm']:.0f}cm | MTF 높이 {h}"
    )


def fmt_flow(o):
    if not o.get("connected"):
        return "[MTF-01P] 연결 안 됨 (데이터 없음)"
    def num(key, fmt):
        v = o.get(key)
        return format(v, fmt) if isinstance(v, (int, float)) else "-"

    # 높이 출처가 FC(MSP_SONAR_ALTITUDE)이면 속도 필드는 null 로 온다
    return (
        f"[MTF-01P] 높이 {num('height_cm', '.1f')}cm | 앞 {num('forward_cm_s', '+.1f')} · 오른쪽 {num('right_cm_s', '+.1f')}"
        f" · 위 {num('up_cm_s', '+.1f')} cm/s | 품질 {o.get('quality')} | 패킷 {o.get('packets')} 오류 {o.get('errors')}"
        f" | 출처 {o.get('height_src', '?')}"
    )


def fmt_vision(o):
    if not o.get("camera"):
        return "[카메라] 초기화 실패"
    seen = f"ID {o['ids']} (미션 태그 아님)" if o["tags"] else "태그 안 보임"
    return (
        f"[카메라] {o['fps']:.1f}fps · 검출 {o['detect_ms']}ms · {seen}"
        f" · decimate {o['decimate_used']}→{o['decimate_next']} ({o['dec_mode']}: {DEC_REASON.get(o['dec_reason'], o['dec_reason'])})"
    )


PHASE_KO = {
    "ALIGN": "정렬",
    "GOTO_50": "50cm로 이동",
    "HOLD_50": "50cm 유지",
    "GOTO_100": "100cm로 이동",
    "HOVER": "호버링(완료)",
}


def fmt_msp(o):
    authority = o.get("authority", "unknown")
    ch6 = o.get("ch6")
    rc_tx = o.get("rc_tx", bool(o.get("out")))
    authority_text = (
        f"{'AUTO' if authority == 'auto' else 'MANUAL'}"
        f"(CH6={ch6 if ch6 is not None else '없음'})"
        if authority in ("auto", "manual")
        else "UNKNOWN"
    )
    if not o["enabled"] and not rc_tx:
        return f"[MSP→FC] 제어권 {authority_text} · 자율 송신 꺼짐 (msp on 으로 켜기)"
    names = ["롤", "피치", "스로틀", "요"]
    out = " ".join(f"{n}={v}" for n, v in zip(names, o.get("out", []))) if rc_tx else "정지"
    link = "FC 응답 OK" if o["fc_link"] else "FC 응답 없음(배선·INAV Ports MSP 확인)"
    fc = ""
    if o["fc_rc"]:
        # MSP_RC 응답은 INAV 내부 순서(롤·피치·요·스로틀)다. 보내는 값(AETR)과 3·4번 자리가 다르다.
        fc_names = ["롤", "피치", "요", "스로틀", "AUX1", "AUX2", "AUX3", "AUX4"]
        fc = " | FC가 쓰는 값: " + " ".join(f"{n}={v}" for n, v in zip(fc_names, o["fc_rc"]))
    thr = "" if o["throttle_ctrl"] else " (스로틀 제어 꺼짐)"
    # -25: 50Hz 상시 송신, rcmap·마스크 확인, 핸드오버 기준점, MTF vz
    extra = ""
    if "tx_hz" in o:
        slot = o.get("thr_slot")
        if slot is not None and rc_tx and len(o.get("out", [])) > slot:
            out_vals = o["out"]
            rcmap = o.get("rcmap", "AETR")
            letter_to_name = {"A": "롤", "E": "피치", "R": "요", "T": "스로틀"}
            out = " ".join(f"{letter_to_name.get(ch, ch)}={v}" for ch, v in zip(rcmap, out_vals))
        mask = o.get("fc_mask")
        mask_ok = o.get("mask_ok")
        mask_text = ("미확인" if mask is None else
                     f"{mask}{' OK' if mask_ok else ' ⚠스로틀 비트/롤·피치 누락 → set msp_override_channels = ' + str(o.get('mask_recommended', 7))}")
        vz = o.get("vz_cm_s")
        vz_text = "없음" if vz is None else f"{vz:+.1f}cm/s{' ⚠>10' if o.get('vz_warn') else ''}"
        base = o.get("base_rpy")
        ack_hz = o.get("ack_hz")
        ack_text = "" if ack_hz is None else f" FC수신 {ack_hz}Hz{' ⚠유실' if o.get('fc_link') and ack_hz < o['tx_hz'] * 0.9 else ''}"
        extra = (f" | 송신율 {o['tx_hz']}Hz{'(상시)' if o.get('tx_always') else ''}{ack_text}"
                 f" · rcmap {o.get('rcmap')}({o.get('rcmap_src')}) · FC 마스크 {mask_text}"
                 f" · 기준점 {base} · vz {vz_text}")
    return (f"[MSP→FC] {link} · 제어권 {authority_text} · RC 송신 {'중' if rc_tx else '정지'}"
            f" · 보냄 {o['sent']} 응답 {o['acks']} 오류 {o['errors']} | 송신: {out}{thr}{fc}"
            f" | 단계 {PHASE_KO.get(o['phase'], o['phase'])}{extra}")


def fmt_authority(o):
    age = o.get("frame_age_ms")
    age_text = "프레임 없음" if age is None or age < 0 else f"마지막 프레임 {age}ms 전"
    return (f"[CH6 전환] → {'AUTO' if o['to'] == 'auto' else 'MANUAL'} (CH6={o.get('ch6')}) @ {o.get('t')}ms"
            f" · {age_text} · 송신율 {o.get('tx_hz')}Hz · 보내던 롤/피치/요 {o.get('out_rpy')}"
            f" · 조종기 롤/피치/요 {o.get('pilot_rpy')}{'' if o.get('pilot_valid') else '(미확인)'}"
            f" · 스로틀 슬롯 {o.get('thr_slot_us')} · FC 값 {o.get('fc_rc')} · 접근 {o.get('approach')}")


APPROACH_STATE_KO = {"IDLE": "대기", "APPROACH": "접근", "ARRIVING": "감속", "ARRIVED": "도착",
                     "HOLD": "정지", "FAULT": "멈춤"}
APPROACH_FAULT_KO = {"disabled": "꺼짐", "manual_authority": "CH6 수동(조종기)", "kill": "킬",
                     "mask_unknown": "FC 마스크 미확인(FC 응답 확인)", "mask": "FC 마스크에 롤·피치 없음/스로틀 있음",
                     "height": "높이 없음·범위 밖(20~150cm)", "flow": "속도 추정 없음", "velocity": "속도 이상",
                     "tag_lost": "태그 놓침"}


def fmt_approach(o, fc_rc=None):
    """접근 제어 한 줄: 태그 위치 → 목표 속도 → 보드 명령 → FC가 실제로 쓰는 롤·피치."""
    st = APPROACH_STATE_KO.get(o["state"], o["state"])
    f = o.get("fault")
    why = f"({APPROACH_FAULT_KO.get(f, f)})" if f else ""
    v = o.get("v_flow") or [0, 0]
    vel = f"오른쪽 {v[0]:+.1f}·앞 {v[1]:+.1f}" if o.get("v_flow_valid") else "없음"
    fc = f" | FC 롤={fc_rc[0]} 피치={fc_rc[1]}" if fc_rc and len(fc_rc) >= 2 else " | FC 값 없음"
    return (f"[접근 {o['mode']}] {st}{why} · 태그 오른쪽 {o['err_right_cm']:+.1f} 앞 {o['err_fwd_cm']:+.1f}cm"
            f"(거리 {o['dist_cm']:.1f}) · 목표속도 {o['target_speed']:.1f}cm/s · 상대속도[{o.get('vel_src', 'flow')}] {vel}"
            f" · 명령 롤 {o['roll_us']:+.0f} 피치 {o['pitch_us']:+.0f}µs → 보냄 롤/피치/요 {o.get('out_rpy')}{fc}"
            f" · 높이 {o['height_cm']:.0f}cm[{o.get('h_src', '?')}]{' ⚠과속→제동' if o.get('overspeed') else ''}")


def fmt_warn(o):
    if o.get("code") == "vz":
        return (f"[경고] MTF 수직 속도 {o['vz_cm_s']:+.1f}cm/s (한계 {o['limit_cm_s']:.0f})"
                f" · 높이 {o.get('height_cm')}cm({o.get('height_src')}) · 스로틀 조작 없음")
    return f"[경고] {o}"


DEC_REASON = {
    "tag_px": "보이는 태그 크기 기준",
    "mtf_height": "MTF 높이로 태그 크기 예측",
    "sweep": "주기적 넓게 찾기",
    "search": "높이 정보 없음 → 넓게 찾기",
    "manual": "수동 고정",
    "start": "시작값",
}


def fmt_boot(o):
    return (
        f"[부팅] {o['build']} | 카메라 {o['camera_sensor'] if o['camera'] else '실패'} {o['frame']}"
        f" | {o['tag_family']} 등록 태그 {o['tags_registered']}개 · decimate {o['decimate']} ({o['dec_mode']})"
        f" | PSRAM 여유 {o['psram_free'] // 1024}KB | MTF RX{o['rx']}/TX{o['tx']}"
        f" | FC MSP {'켜짐' if o.get('msp') else '꺼짐'} (GPIO{o.get('fc_tx')}→RX5, GPIO{o.get('fc_rx')}←TX5)"
        f" | BLE {o.get('ble', '없음')}{' 연결 중' if o.get('ble_connected') else ''}"
    )


def monitor(lines, raw, flow_every, deadline):
    last_flow = last_msp = last_mic = last_ap = 0.0
    last_fc_rc = None
    for line in lines:
        if deadline and time.time() > deadline:
            return
        if not line:
            continue
        if raw:
            print(line, flush=True)
            continue
        try:
            o = json.loads(line)
        except json.JSONDecodeError:
            continue  # ROM 부트 로그 등
        t = o.get("type")
        if t == "tag":
            print(fmt_tag(o), flush=True)
        elif t == "target":
            print(fmt_target(o), flush=True)
        elif t == "tags":
            reg = ", ".join(f"ID {x['id']}={x['size_mm']}mm 착륙점({x['land_x_mm']},{x['land_y_mm']})" for x in o["tags"])
            print(f"[태그 표] {reg}", flush=True)
        elif t == "mic":
            if time.time() - last_mic >= flow_every:
                last_mic = time.time()
                print(f"[마이크] 평균 {o['rms_db']:.1f}dBFS · 최대 {o['peak_db']:.1f}dBFS" if o.get("ok")
                      else "[마이크] 신호 없음 (배선 확인)", flush=True)
        elif t == "voice":
            print(f"[음성 명령] {o['keyword']} ({o['ko']}) → {o['commands']}", flush=True)
        elif t == "approach":
            if time.time() - last_ap >= flow_every:
                last_ap = time.time()
                print(fmt_approach(o, last_fc_rc), flush=True)
        elif t == "msp":
            last_fc_rc = o.get("fc_rc") or last_fc_rc
            if time.time() - last_msp >= flow_every:
                last_msp = time.time()
                print(fmt_msp(o), flush=True)
        elif t == "flow":
            if time.time() - last_flow >= flow_every:
                last_flow = time.time()
                print(fmt_flow(o), flush=True)
        elif t == "vision":
            if o.get("target_id", -1) < 0:
                print(fmt_vision(o), flush=True)
        elif t in ("boot", "info"):
            print(fmt_boot(o), flush=True)
        elif t == "authority":
            print(fmt_authority(o), flush=True)
        elif t == "warn":
            print(fmt_warn(o), flush=True)
        else:
            print(line, flush=True)


def snap(s, lines, path, timeout=15.0):
    s.write(b"snap\n")
    t0 = time.time()
    header = None
    data = []
    while time.time() - t0 < timeout:
        line = next(lines)
        if line is None:
            continue
        if header is None:
            if line.startswith("SNAP_BEGIN"):
                header = line.split()
            continue
        if line == "SNAP_END":
            b64 = "".join(data)
            if len(b64) != int(header[4]):
                print(f"경고: base64 길이 {len(b64)} ≠ 예상 {header[4]} (일부 줄 누락)")
            jpg = base64.b64decode(b64)
            open(path, "wb").write(jpg)
            print(f"저장: {path} ({header[2]}x{header[3]}, {len(jpg)} bytes)")
            return True
        if not line.startswith("{"):  # 스냅샷 줄 사이에 다른 JSON 줄이 끼어들 수 있다
            data.append(line)
    sys.exit("스냅샷을 받지 못했습니다.")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    ap.add_argument("--ble", action="store_true", help="USB 대신 블루투스(BLE)로 연결")
    ap.add_argument("--snap", metavar="JPG", help="카메라 화면 한 장을 저장하고 종료")
    ap.add_argument("--cmd", help="보드에 명령 한 줄 보내기 (예: 'tags', 'size 0 100', 'land 10 -170 0', 'dec auto', 'dec 3')")
    ap.add_argument("--raw", action="store_true", help="JSON 원문 출력")
    ap.add_argument("--flow-every", type=float, default=1.0, help="MTF-01P 줄 출력 간격(초)")
    ap.add_argument("--duration", type=float, default=0, help="이 시간(초) 후 종료 (0 = 계속)")
    args = ap.parse_args()

    if args.ble:
        s = BleLink()
        lines = s.lines()
        s.write(b"info\n")  # BLE는 재부팅이 없으니 현재 상태를 한 번 받아 둔다
    else:
        s = open_port(args.port or find_port())
        lines = read_lines(s)
        wait_boot(lines)  # USB는 포트를 열면 재부팅되므로 부팅을 기다린다

    if args.snap:
        snap(s, lines, args.snap)
        return
    if args.cmd:
        s.write((args.cmd + "\n").encode())

    deadline = time.time() + args.duration if args.duration else 0
    try:
        monitor(lines, args.raw, args.flow_every, deadline)
    except KeyboardInterrupt:
        pass
    finally:
        s.close()


if __name__ == "__main__":
    main()
