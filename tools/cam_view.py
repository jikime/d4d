"""태그 보드 카메라 USB 라이브 뷰어 (1~2 fps).

보드의 `snap` 명령을 반복 호출해 JPEG를 받고, 같은 포트로 들어오는 tag/flow/vision JSON과
FC 광류(MSP2_INAV_OPTICAL_FLOW, fcmsp 0x2001)를 함께 오버레이한다.

사용:
  uv run --with pyserial --with pillow python tools/cam_view.py --port COM10
  uv run --with pyserial --with pillow --with bleak python tools/cam_view.py --ble   (USB 없이 블루투스로)
  uv run --with pyserial --with pillow python tools/cam_view.py --wifi   (보드 자체 WiFi, -36~. Mac 을 TagBoard-… WiFi 에 먼저 연결)
  옵션:
    --rot 0|90|180|270   화살표에 쓸 카메라 장착 회전 (펌웨어 CAM_ROTATE_DEG와 같은 의미)
    --log out.csv        flow/tag 샘플을 CSV로 기록 (축 검증용)
    --dump latest.jpg    오버레이된 최신 프레임을 2초마다 파일로 저장
  키: q/Esc 종료, s 스냅샷 저장(Downloads), p 새 사진 바로 요청 (BLE 에서 유용), v 좌표↔영상 방식,
      o approach on(보드 출력) / w approach shadow(관찰) — FC 가 실제로 쓰는지는 CH6 위치
"""
from __future__ import annotations

import argparse
import asyncio
import base64
import csv
import io
import json
import math
import queue
import struct
import sys
import threading
import time
import tkinter as tk
from pathlib import Path

import serial
from PIL import Image, ImageDraw, ImageFont, ImageTk

FLOW_QUERY = b"fcmsp 0x2001\n"
FLOW_PERIOD = 0.25
SNAP_TIMEOUT = 4.0

def _to_photo(img):
    """PIL 이미지 → Tk PhotoImage. ImageTk 는 uv 독립 Python 의 Tk 와 맞지 않는 경우가 있어 PNG 데이터로 넘긴다."""
    bio = io.BytesIO()
    img.convert("RGB").save(bio, format="PNG")
    return tk.PhotoImage(data=base64.b64encode(bio.getvalue()))


def _korean_font(size, bold=False):
    """Windows(맑은 고딕) → macOS(Apple SD 고딕 Neo) → 기본 글꼴 순서로 찾는다."""
    candidates = (["C:/Windows/Fonts/malgunbd.ttf"] if bold else []) + [
        "C:/Windows/Fonts/malgun.ttf",
        "/System/Library/Fonts/AppleSDGothicNeo.ttc",
        "/System/Library/Fonts/Supplemental/AppleGothic.ttf",
    ]
    for path in candidates:
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            continue
    return ImageFont.load_default()


ARROWS = {  # rot -> (위, 아래, 오른쪽, 왼쪽) 라벨  (펌웨어 imageToBody 와 동일한 매핑)
    0: ("앞", "뒤", "오른쪽", "왼쪽"),
    90: ("오른쪽", "왼쪽", "뒤", "앞"),
    180: ("뒤", "앞", "왼쪽", "오른쪽"),
    270: ("왼쪽", "오른쪽", "앞", "뒤"),
}


class BleSerial:
    """BLE Nordic UART 를 serial.Serial 처럼(write/read/reset_input_buffer/close) 쓰게 한다. 끊기면 다시 붙는다.
    USB(HWCDC)가 멈췄거나 USB 없이 시험할 때 쓴다. 사진 한 장(약 30KB)에 1~3초 걸린다."""

    TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # 보드 → PC (알림)
    RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # PC → 보드 (명령)

    def __init__(self, name_prefix="TagBoard", timeout=0.05):
        try:
            from bleak import BleakClient, BleakScanner
        except ImportError:
            sys.exit("블루투스 연결에는 bleak가 필요합니다: uv run --with bleak --with pyserial --with pillow ...")
        self._client_cls, self._scanner = BleakClient, BleakScanner
        self.name_prefix = name_prefix
        self.timeout = timeout
        self.buf = bytearray()
        self.cv = threading.Condition()
        self.client = None
        self.closed = False
        self.ready = threading.Event()
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self.loop.run_forever, daemon=True).start()
        asyncio.run_coroutine_threadsafe(self._keep_connected(), self.loop)
        if not self.ready.wait(40):
            sys.exit("블루투스로 보드를 찾지 못했습니다 (이름 TagBoard-…). 보드 전원과 거리를 확인하세요.")

    async def _keep_connected(self):
        while not self.closed:
            try:
                dev = await self._scanner.find_device_by_filter(
                    lambda d, ad: (d.name or ad.local_name or "").startswith(self.name_prefix), timeout=10)
                if dev is None:
                    continue
                gone = asyncio.Event()
                client = self._client_cls(dev, disconnected_callback=lambda _: self.loop.call_soon_threadsafe(gone.set))
                await client.connect()
                await client.start_notify(self.TX, self._on_data)
                self.client = client
                print(f"[블루투스] {dev.name} 연결됨 (MTU {client.mtu_size})", flush=True)
                self.ready.set()
                await gone.wait()
                self.client = None
                if not self.closed:
                    print("[블루투스] 연결 끊김 — 다시 연결합니다", flush=True)
            except Exception as e:
                self.client = None
                print("[블루투스] 오류:", repr(e)[:160], flush=True)
            await asyncio.sleep(1)

    def _on_data(self, _, data):
        with self.cv:
            self.buf.extend(data)
            self.cv.notify()

    def read(self, n):
        with self.cv:
            if not self.buf:
                self.cv.wait(self.timeout)
            chunk = bytes(self.buf[:n])
            del self.buf[:n]
            return chunk

    def write(self, data: bytes):
        c = self.client
        if c is None:
            return 0
        step = max(20, min(c.mtu_size - 3, 180))

        async def send():
            for i in range(0, len(data), step):
                await c.write_gatt_char(self.RX, data[i:i + step], response=False)

        try:
            asyncio.run_coroutine_threadsafe(send(), self.loop).result(5)
        except Exception as e:
            print("[블루투스] 보내기 실패:", repr(e)[:120], flush=True)
            return 0
        return len(data)

    def reset_input_buffer(self):
        with self.cv:
            self.buf.clear()

    def close(self):
        self.closed = True
        c = self.client
        if c is not None:
            try:
                asyncio.run_coroutine_threadsafe(c.disconnect(), self.loop).result(5)
            except Exception:
                pass


class TcpSerial:
    """보드 자체 WiFi(-36~) 의 TCP 통로를 serial.Serial 처럼 쓴다. 끊기면 1초마다 다시 붙는다.
    Mac 이 보드 WiFi(TagBoard-XXXX)에 먼저 붙어 있어야 한다. 사진도 빠르다(인식 주기만큼)."""

    def __init__(self, host="192.168.4.1", port=3333, timeout=0.05):
        import socket
        self._socket_mod = socket
        self.host, self.port, self.timeout = host, port, timeout
        self.sock = None
        self.last_try = 0.0
        self.connects = 0  # 연결될 때마다 +1 → Reader 가 시험 명령을 다시 보낸다
        self.last_rx = time.time()
        self._connect()

    def _connect(self):
        self.last_try = time.time()
        try:
            s = self._socket_mod.create_connection((self.host, self.port), timeout=2)
            s.setsockopt(self._socket_mod.IPPROTO_TCP, self._socket_mod.TCP_NODELAY, 1)
            s.settimeout(self.timeout)
            self.sock = s
            self.last_rx = time.time()
            self.connects += 1
            print(f"[WiFi] {self.host}:{self.port} 연결됨", flush=True)
        except OSError as e:
            self.sock = None
            print(f"[WiFi] 연결 실패 ({e}) — Mac 이 보드 WiFi(TagBoard-…)에 붙어 있는지 확인. 다시 시도합니다", flush=True)

    def read(self, n):
        if self.sock is None:
            if time.time() - self.last_try > 1.0:
                self._connect()
            else:
                time.sleep(self.timeout)
            return b""
        try:
            data = self.sock.recv(n)
            if not data:
                raise OSError("closed")
            self.last_rx = time.time()
            return data
        except self._socket_mod.timeout:
            # 보드가 재부팅·전원 차단되면 연결이 닫히지 않고 조용히 끊긴다 → 5초 넘게 아무것도 안 오면 다시 연결
            if time.time() - self.last_rx > 5.0:
                print("[WiFi] 5초 동안 데이터 없음 — 다시 연결합니다", flush=True)
                self.close_sock()
            return b""
        except OSError:
            print("[WiFi] 끊김 — 다시 연결합니다", flush=True)
            self.close_sock()
            return b""

    def write(self, data: bytes):
        if self.sock is None:
            return 0
        try:
            self.sock.sendall(data)
            return len(data)
        except OSError:
            self.close_sock()
            return 0

    def close_sock(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None

    def reset_input_buffer(self):
        pass

    def close(self):
        self.close_sock()


class Reader(threading.Thread):
    def __init__(self, port: str, out: queue.Queue, init_cmds: list[str] | None = None, snap_period: float = 0.0):
        super().__init__(daemon=True)
        self.snap_period = snap_period  # 사진 사이 쉬는 시간(초). 사진을 받는 동안 보드의 태그 인식이 느려진다
        self.fc_flow = False  # FC 광류를 fcmsp 로 직접 묻기 (보드의 FC 설정 읽기와 겹치므로 기본 끔)
        self.next_snap_at = 0.0
        self.ble = port.upper() == "BLE"
        self.wifi = port.startswith("wifi:")
        if self.ble:
            self.ser = BleSerial()
        elif self.wifi:
            self.ser = TcpSerial(port[5:] or "192.168.4.1")
        else:
            self.ser = serial.Serial(port, 115200, timeout=0.05)
        self.snap_timeout = 25.0 if self.ble else SNAP_TIMEOUT  # BLE 는 사진 한 장에 10초 넘게 걸린다
        # -35 부터 `snap small`(320×240, 약 1/6) — BLE 에서 쓴다. 옛 펌웨어가 거부하면 `snap` 으로 돌아간다
        # WiFi 도 신호가 약하면(10-11 실측 -80dBm, 약 6KB/s) 큰 사진이 몇 초씩 걸린다 → 작은 사진이 기본
        self.snap_cmd = "snap small" if (self.ble or self.wifi) else "snap"
        # -37~ WiFi: 보드가 요청 없이 작은 사진을 계속 보낸다(snap stream) → 요청·응답 왕복 지연 없이 거의 실시간
        self.stream = self.wifi
        self.stream_ms = 200
        self.last_frame_t = 0.0
        # 사진 허용: 보드 출력(approach on)이나 CH6 AUTO 중에는 멈춘다. 옛 펌웨어(-34 이하)는 사진을 보내는 동안
        # 카메라 루프가 멈춰 태그 인식이 끊긴다(BLE 약 12초) → 보드가 조종할 수 있을 때는 사진보다 인식이 우선
        self.photo_allowed = True
        self.out = out
        self.init_cmds = init_cmds or []
        self.init_sent = not self.init_cmds
        self.t_open = time.time()
        self.stop = threading.Event()
        self.snap_pending = 0.0
        self.last_flow_q = 0.0

    def send_init(self):
        for cmd in self.init_cmds:
            self.ser.write(cmd.strip().encode() + b"\n")
            time.sleep(0.05)
        if self.stream:
            self.ser.write(f"snap stream {self.stream_ms}\n".encode())
        elif self.wifi:
            self.ser.write(b"snap stream off\n")  # 좌표 방식: 이전 뷰어가 켜 둔 영상도 끈다
        self.init_sent = True
        self.out.put(("json", {"type": "_init_sent", "cmds": self.init_cmds}, None))

    def request_snap(self):
        self.ser.write(self.snap_cmd.encode() + b"\n")
        self.snap_pending = time.time()

    def run(self):
        time.sleep(0.3)
        self.ser.reset_input_buffer()
        buf = b""
        header = None
        b64: list[str] = []
        binparts: list[bytes] = []
        bin_need = 0
        manual = self.snap_period < 0  # 음수: p 키로만 사진 (옛 펌웨어는 사진을 보내는 동안 태그 인식이 멈춤)
        if not manual and not self.stream:
            self.request_snap()
        while not self.stop.is_set():
            now = time.time()
            if self.fc_flow and now - self.last_flow_q >= FLOW_PERIOD:
                self.ser.write(FLOW_QUERY)
                self.last_flow_q = now
            if self.snap_pending and now - self.snap_pending > self.snap_timeout:
                header, b64 = None, []
                if manual:
                    self.snap_pending = 0.0
                else:
                    self.request_snap()
            if self.next_snap_at and now >= self.next_snap_at and self.photo_allowed:
                self.next_snap_at = 0.0
                self.request_snap()
            # USB 포트를 열면 보드가 재부팅된다 → boot 줄을 본 뒤(또는 5초 뒤) 시험 명령을 보낸다. BLE 는 재부팅 없음 → 바로
            if not self.init_sent and now - self.t_open > (1.0 if self.ble or self.wifi else 5.0):
                self.send_init()
            # WiFi: 뷰어를 먼저 켜 두고 Mac 을 나중에 보드 WiFi 로 바꿔도 되게, (재)연결될 때마다 시험 명령·사진 요청
            if self.wifi and self.ser.sock is not None and self.ser.connects != getattr(self, "_seen_connects", 0):
                self._seen_connects = self.ser.connects
                self.send_init()  # stream 이면 여기서 snap stream 도 보낸다
                header, b64 = None, []
                if not self.stream and self.snap_period >= 0:
                    self.request_snap()
            chunk = self.ser.read(8192)
            if not chunk:
                continue
            buf += chunk
            while True:
                # -38 WiFi 스트림: "SNAPC <n>\n" 뒤 n 바이트가 사진 원본 조각 (줄바꿈 없음, 상태 줄이 사이에 끼어도 됨)
                if bin_need:
                    take = buf[:bin_need]
                    buf = buf[len(take):]
                    binparts.append(take)
                    bin_need -= len(take)
                    if bin_need:
                        break  # 조각을 더 받아야 한다
                    continue
                if b"\n" not in buf:
                    break
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("utf-8", "replace").strip()
                if not line:
                    continue
                if line.startswith("SNAPC "):
                    try:
                        bin_need = int(line.split()[1])
                    except (IndexError, ValueError):
                        bin_need = 0
                    continue
                if line.startswith("SNAPB "):
                    header, b64, binparts = line.split(), [], []
                    continue
                if line.startswith("SNAP_BEGIN"):
                    header, b64 = line.split(), []
                    continue
                if line == "SNAP_END":
                    if header:
                        if header[0] == "SNAPB":
                            jpg_bin = b"".join(binparts)
                            ok = len(header) > 1 and header[1].isdigit() and len(jpg_bin) == int(header[1])
                            joined, expected = "", 0
                        else:
                            jpg_bin = None
                            joined = "".join(b64)
                            expected = int(header[4]) if len(header) > 4 and header[4].isdigit() else len(joined)
                            ok = len(joined) == expected
                        if ok:
                            try:
                                jpg = jpg_bin if jpg_bin is not None else base64.b64decode(joined)
                                now_f = time.time()
                                if self.stream:  # 영상 간격 = 앞 사진과의 시간
                                    dt = now_f - self.last_frame_t if self.last_frame_t else 0.0
                                else:
                                    dt = now_f - self.snap_pending
                                self.last_frame_t = now_f
                                self.out.put(("frame", jpg, dt))
                            except Exception:
                                self.out.put(("drop", None, None))
                        else:
                            self.out.put(("drop", None, None))
                    header, b64 = None, []
                    self.snap_pending = 0.0
                    if self.stream:
                        continue  # 다음 사진은 보드가 알아서 보낸다
                    if self.snap_period > 0:
                        self.next_snap_at = time.time() + self.snap_period
                    elif self.snap_period == 0:
                        self.request_snap()
                    continue
                # 오류 재부팅 기록(Guru Meditation·Backtrace 등)은 JSON 이 아니다 → 원인 분석용으로 그대로 넘긴다
                if any(k in line for k in ("Guru", "Backtrace", "panic", "abort()", "Stack canary", "rst:", "Core ", "EXCVADDR", "PC      :")):
                    self.out.put(("json", {"type": "_raw", "line": line[:300]}, None))
                if line.startswith("{"):
                    try:
                        d = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    if self.init_cmds and d.get("type") == "boot":
                        # 시험 도중 보드가 재부팅돼도 시험 설정을 다시 넣는다 (재부팅하면 기본값으로 돌아감)
                        time.sleep(1.0)
                        self.send_init()
                    if d.get("type") in ("cmd_error", "help") and self.snap_pending and self.snap_cmd != "snap":
                        self.snap_cmd = "snap"  # 옛 펌웨어: `snap small` 모름
                        self.request_snap()
                    self.out.put(("json", d, None))
                elif header is not None:
                    b64.append(line)
        self.ser.close()


class Viewer:
    def __init__(self, args):
        self.args = args
        self.q: queue.Queue = queue.Queue()
        init_cmds = [x for x in (args.init or "").split(";") if x.strip()]
        if args.wifi:
            args.port = "wifi:" + args.wifi  # 사진은 640×480 그대로, 쉬는 시간 없이 (snap-period 0)
        elif args.ble:
            args.port = "BLE"
            if args.snap_period == 0:
                args.snap_period = 2.0  # BLE: 사진 사이 2초 — 상태 줄(INAV 값)이 끊기지 않게
        self.reader = Reader(args.port, self.q, init_cmds, args.snap_period)
        self.reader.stream_ms = args.stream_ms
        # 좌표 방식(기본): 사진 없이 보드의 위치 줄(pos)로 그린다. 영상 방식: 예전처럼 사진 (`--mode video` 또는 v 키)
        self.coords = args.mode == "coords"
        if self.coords:
            self.reader.stream = False
            self.reader.snap_period = -1  # 사진 자동 요청 안 함 (p 키로 한 장은 가능)
        self.trail: list = []  # 최근 태그 중심 (시각, x, y)
        self.pos_times: list = []
        self.reader.fc_flow = args.fc_flow
        self.state = {"tag": None, "tag_t": 0.0, "height": None, "flow": None, "vision": None,
                      "frame_dt": None, "frames": 0, "msp": None, "approach": None, "init": None, "tour": None}
        self.ap_log = open(args.ap_log, "w", encoding="utf-8") if args.ap_log else None
        self.font = _korean_font(22, bold=True)
        self.small = _korean_font(15)
        self.image: Image.Image | None = None
        bio = io.BytesIO()
        Image.new("RGB", (640, 480), (25, 25, 25)).save(bio, format="JPEG")
        self.blank_jpg = bio.getvalue()  # 사진이 아직 없을 때 오버레이 바탕
        self.last_jpg: bytes | None = None  # 마지막 사진 — 사진이 뜸한(BLE) 동안에도 오버레이는 이 위에 계속 새로 그린다
        self.last_jpg_t = 0.0
        self.last_render = 0.0
        self.last_dump = 0.0
        self.log = None
        if args.log:
            self.log_f = open(args.log, "w", newline="", encoding="utf-8")
            self.log = csv.writer(self.log_f)
            self.log.writerow(["t", "height", "tag_id", "tag_right", "tag_up", "tag_cx", "tag_cy",
                               "flow_q", "flow_fx", "flow_fy", "flow_bx", "flow_by"])
        self.t0 = time.time()

        self.root = tk.Tk()
        self.root.title(f"TagBoard 카메라 · {args.port} · rot {args.rot}")
        self.root.configure(bg="#111")
        self.label = tk.Label(self.root, bg="#111")
        self.label.pack()
        self.status = tk.Label(self.root, bg="#111", fg="#ddd", font=("Malgun Gothic", 10), anchor="w", justify="left")
        self.status.pack(fill="x")
        self.root.bind("<Key>", self.on_key)
        self.photo = None
        # 터미널·백그라운드에서 띄우면 macOS 에서 창이 다른 창 뒤에 숨는다 → 처음에 잠깐 맨 앞으로
        self.root.lift()
        self.root.attributes("-topmost", True)
        self.root.after(1500, lambda: self.root.attributes("-topmost", False))

    def on_key(self, e):
        if e.keysym in ("q", "Escape"):
            self.root.destroy()
        elif e.keysym == "s" and self.image is not None:
            p = Path.home() / "Downloads" / f"tagboard_live_{time.strftime('%Y%m%d_%H%M%S')}.jpg"
            self.image.save(p, quality=92)
            print("저장:", p)
        elif e.keysym in ("o", "w"):
            # 현장 조작: o = 보드 출력 켬(approach on), w = 관찰(shadow, FC 에는 1500).
            # 실제로 FC 가 보드 값을 쓰는지는 조종기 CH6(OVERRIDE) 위치가 정한다
            cmd = "approach on" if e.keysym == "o" else "approach shadow"
            self.reader.ser.write(cmd.encode() + b"\n")
            print("보냄:", cmd)
        elif e.keysym == "v":
            # 좌표 방식 ↔ 영상 방식 (재시작 없이)
            self.coords = not self.coords
            if self.reader.wifi:
                self.reader.stream = not self.coords
                self.reader.snap_period = -1 if self.coords else 0
                cmd = f"snap stream {self.reader.stream_ms}" if self.reader.stream else "snap stream off"
                self.reader.ser.write(cmd.encode() + b"\n")
            else:
                self.reader.snap_period = -1 if self.coords else 2.0
                if not self.coords and not self.reader.snap_pending:
                    self.reader.request_snap()
            print("화면 방식:", "좌표" if self.coords else "영상")
        elif e.keysym == "t":
            # 태그 투어 켜기/끄기 (-41): 태그 위 → 화살표 방향으로 돌기 → 머무름 → 다음 태그
            on = not ((self.state.get("tour") or {}).get("on"))
            cmd = "tour on" if on else "tour off"
            self.reader.ser.write(cmd.encode() + b"\n")
            print("보냄:", cmd)
        elif e.keysym == "p" and not self.reader.snap_pending:
            self.reader.next_snap_at = 0.0
            self.reader.request_snap()

    def handle_json(self, d: dict):
        t = d.get("type")
        s = self.state
        if t == "tag":
            s["tag"], s["tag_t"] = d, time.time()
            if self.ap_log:  # 태그 크기 자동 보정(tag_size_mm·cal_n·cal_spread) 분석용
                self.ap_log.write(json.dumps({"t": round(time.time() - self.t0, 2), "tag": d}, ensure_ascii=False) + "\n")
        elif t == "flow":
            s["height"] = d.get("height_cm")
        elif t == "vision":
            s["vision"] = d
            if self.ap_log:
                self.ap_log.write(json.dumps({"t": round(time.time() - self.t0, 2), "vision": d}, ensure_ascii=False) + "\n")
            if not d.get("tags"):
                s["tag"] = None
        elif t == "pos":
            now = time.time()
            s["pos"], s["pos_t"] = d, now
            self.pos_times = [x for x in self.pos_times if now - x < 2.0] + [now]
            if d.get("id", -1) >= 0:
                self.trail = [x for x in self.trail if now - x[0] < 3.0] + [(now, d["cx"], d["cy"])]
            if self.ap_log:
                self.ap_log.write(json.dumps({"t": round(now - self.t0, 2), "pos": d}, ensure_ascii=False) + "\n")
            return
        elif t == "msp":
            s["msp"] = d
            if self.ap_log:
                self.ap_log.write(json.dumps({"t": round(time.time() - self.t0, 2), "msp": d}, ensure_ascii=False) + "\n")
        elif t == "fc":
            if self.ap_log:
                self.ap_log.write(json.dumps({"t": round(time.time() - self.t0, 2), "fc": d}, ensure_ascii=False) + "\n")
            return
        elif t == "tour":
            s["tour"], s["tour_t"] = d, time.time()
            if self.ap_log:
                self.ap_log.write(json.dumps({"t": round(time.time() - self.t0, 2), "tour": d}, ensure_ascii=False) + "\n")
            return
        elif t == "approach":
            s["approach"] = d
            # 사용자 요청(10-11): CH6 AUTO·출력 중에도 사진을 계속 받는다. 옛 펌웨어는 그동안 인식이 멈춰 보드가 1500 → 화면에 경고
            if self.ap_log:
                fc = (s["msp"] or {}).get("fc_rc")
                self.ap_log.write(json.dumps({"t": round(time.time() - self.t0, 2), "ap": d, "fc_rc": fc},
                                             ensure_ascii=False) + "\n")
                self.ap_log.flush()
            return
        elif t in ("_init_sent", "approach_set", "approach_mode", "cmd_error", "exp", "boot", "_raw", "tagcal",
                   "tour_mode", "tour_set"):
            s["init"] = d
            if self.ap_log:
                self.ap_log.write(json.dumps({"t": round(time.time() - self.t0, 2), "evt": d}, ensure_ascii=False) + "\n")
            return
        elif t == "fcmsp" and d.get("cmd") == 0x2001 and d.get("ok") and not d.get("error"):
            raw = bytes.fromhex(d["hex"])
            if len(raw) >= 9:
                qv, fx, fy, bx, by = struct.unpack("<Bhhhh", raw[:9])
                s["flow"] = {"q": qv, "fx": fx, "fy": fy, "bx": bx, "by": by}
        else:
            return
        if self.log and t in ("tag", "flow", "fcmsp"):
            tag, fl = s["tag"] or {}, s["flow"] or {}
            self.log.writerow([round(time.time() - self.t0, 2), s["height"], tag.get("id"), tag.get("img_right_cm"),
                               tag.get("img_up_cm"), tag.get("cx_px"), tag.get("cy_px"),
                               fl.get("q"), fl.get("fx"), fl.get("fy"), fl.get("bx"), fl.get("by")])

    @staticmethod
    def auto_readiness(ap: dict) -> tuple[bool, str]:
        """CH6 수동 중에도 AUTO 로 넘겨도 될지: 높이·속도(광류 품질 40)·태그. 제어기는 수동이면 이 판정 전에 멈춘다."""
        h = ap.get("height_cm") or 0
        h_ok = bool(ap.get("height_valid")) and 20 <= h <= 300
        if ap.get("vel_src") == "kf":  # -39~: 칼만(명령 기울기 + 카메라) — 태그를 보고 있으면 준비됨
            v_ok, v_txt = bool((ap.get("kf") or {}).get("on")), "추정"
        elif ap.get("vel_src") == "tag":
            vt = ap.get("v_tag") or [None]
            v_ok, v_txt = vt[0] is not None, "카메라 속도"
        else:
            v_ok, v_txt = bool(ap.get("v_flow_valid")) and (ap.get("flow_q") or 0) >= 40, f"광류 q{ap.get('flow_q', 0)}"
        tag_ok = ap.get("tag") == "fresh"
        d = math.hypot(ap.get("err_right_cm") or 0, ap.get("err_fwd_cm") or 0)
        ready = h_ok and v_ok and tag_ok
        ox = lambda ok: "O" if ok else "X"
        return ready, (f"CH6 수동 · AUTO 준비 {'됨' if ready else '안 됨'}: 높이 {h:.0f}cm {ox(h_ok)} · "
                       f"{v_txt}{'' if v_ok or ap.get('vel_src') == 'tag' else '(40 미만)'} {ox(v_ok)} · "
                       f"태그 {f'{d:.0f}cm ' if tag_ok else ''}{ox(tag_ok)}")

    @staticmethod
    def command_reason(ap: dict) -> tuple[str, str]:
        """INAV 롤·피치가 1500 에서 움직이는지와 그 이유 (approach.h step 의 판정 순서 그대로).
        반환 (종류, 글): active 명령 중 · ready 수동이지만 AUTO 준비됨 · idle 1500 · warn 위험한 조합."""
        fault, state, dist = ap.get("fault"), ap.get("state"), ap.get("dist_cm", 0.0)
        if ap.get("mode") == "shadow" and ap.get("authority") == "auto":
            return "warn", "경고: 관찰 모드 + CH6 AUTO → FC 롤·피치 1500 고정. CH6 수동으로 또는 o"
        if fault == "manual_authority":
            ready, txt = Viewer.auto_readiness(ap)
            return ("ready" if ready else "idle"), txt
        reasons = {
            "disabled": "접근 제어 꺼짐",
            "manual_authority": "CH6 수동 — 조종기가 제어",
            "kill": "킬 스위치",
            "mask_unknown": "FC 오버라이드 마스크 확인 중",
            "mask": "FC 오버라이드 마스크가 롤·피치를 안 받음",
            "height": f"높이 {ap.get('height_cm', 0):.0f}cm — 20~300cm 밖 (거리센서)",
            "flow": ("태그를 3초 넘게 못 봄 — 추정 멈춤, 중립(조종자가 잡을 것)" if ap.get("vel_src") == "kf"
                     else "태그 속도 없음 — 태그 미인식 또는 막 다시 보임(2장 필요)" if ap.get("vel_src") == "tag"
                     else "광류 속도 없음"),
            "velocity": "너무 빠르게 움직임 (60cm/s 초과)",
            "tag_lost": "태그 놓침 — 중립",
        }
        if fault:
            return "idle", "INAV 1500 · " + reasons.get(fault, fault)
        if state == "ARRIVED":
            return "idle", "INAV 1500 · 도착(5cm 안 2초) — 5cm 밖으로 벗어나야 다시 동작"
        if dist <= 5.0:
            return "idle", f"INAV 1500 · 태그가 중심에서 {dist:.1f}cm — 정지 반경 5cm 안"
        if (ap.get("kf") or {}).get("pred"):
            return "warn", f"태그 놓침 — 예측 위치({dist:.0f}cm)로 돌아가는 중 (3초 안에 못 찾으면 중립)"
        if ap.get("mode") == "shadow":
            return "ready", f"관찰: 출력이면 명령 · 태그 {dist:.0f}cm → 목표 {ap.get('target_speed', 0):.1f}cm/s"
        return "active", f"INAV 명령 중 · 태그 {dist:.0f}cm 떨어짐 → 목표 속도 {ap.get('target_speed', 0):.1f}cm/s"

    def draw_coords(self, d, W, H, cx, cy, label):
        """좌표 방식: 보드가 보낸 위치 줄(pos)로 태그 상자·지나온 궤적·추정기 위치를 그린다 (사진 없이 실시간)."""
        now = time.time()
        tr = [x for x in self.trail if now - x[0] < 3.0]
        for i in range(1, len(tr)):  # 궤적: 오래될수록 어둡게
            a = 1 - (now - tr[i][0]) / 3.0
            col = (int(60 + 140 * a), int(120 + 135 * a), int(60 + 60 * a))
            d.line([(tr[i - 1][1], tr[i - 1][2]), (tr[i][1], tr[i][2])], fill=col, width=3)
        p = self.state.get("pos")
        fresh = p is not None and now - self.state.get("pos_t", 0) < 1.0
        if fresh and p.get("id", -1) >= 0:
            tx, ty, half, rot = p["cx"], p["cy"], p["s"] / 2, math.radians(p.get("rot", 0))
            corners = []
            for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):  # 회전한 정사각형
                corners.append((tx + half * (sx * math.cos(rot) - sy * math.sin(rot)),
                                ty + half * (sx * math.sin(rot) + sy * math.cos(rot))))
            d.polygon(corners, outline=(0, 255, 0), width=3)
            self.draw_arrow(d, tx, ty, p.get("rot", 0), max(30.0, p["s"] * 0.9), (0, 255, 0))  # 태그 화살표(인쇄물 위쪽)
            d.ellipse([tx - 5, ty - 5, tx + 5, ty + 5], fill=(0, 255, 0))
            d.line([(cx, cy), (tx, ty)], fill=(0, 255, 0), width=2)
            label(f"ID {p['id']}  오른쪽 {p.get('r', 0):+.1f}cm  앞 {p.get('f', 0):+.1f}cm  한 변 {p['s']:.0f}px"
                  f"{'  · 주변 탐색' if p.get('roi') else ''}", (W - 6, H - 6), "rb", fill=(0, 255, 0), font=self.small)
        # 추정기 위치 (cm → 화면 px: 높이로 환산, 장착 회전 0 기준). 태그를 놓쳤을 때 예측 위치가 특히 중요
        ap = self.state.get("approach") or {}
        h = ap.get("height_cm") or self.state.get("height")
        kf_on = bool((ap.get("kf") or {}).get("on")) or bool(p and (p.get("kp") or p.get("kon")))
        if fresh and h and h > 20 and kf_on:
            k = 615.0 * W / 640.0 / h
            ex, ey = cx + p["ke"][0] * k, cy - p["ke"][1] * k
            if p.get("kp"):
                d.ellipse([ex - 14, ey - 14, ex + 14, ey + 14], outline=(255, 160, 60), width=3)
                label("예측 위치", (ex + 16, ey), "lm", fill=(255, 160, 60), font=self.small)
            else:
                d.ellipse([ex - 6, ey - 6, ex + 6, ey + 6], outline=(0, 220, 255), width=2)
        rate = len([x for x in self.pos_times if now - x < 2.0]) / 2.0
        label(f"좌표 방식 · 위치 {rate:.1f}회/s · v 영상으로", (W - 8, 130), "rt", fill=(200, 200, 200), font=self.small)

    @staticmethod
    def draw_arrow(d, x, y, deg, length, color, width=3):
        """화면에서 위쪽 기준 시계방향 deg 로 화살표."""
        a = math.radians(deg)
        ex, ey = x + length * math.sin(a), y - length * math.cos(a)
        d.line([(x, y), (ex, ey)], fill=color, width=width)
        for side in (-1, 1):
            b = a + math.pi + side * 0.45
            d.line([(ex, ey), (ex + 12 * math.sin(b), ey - 12 * math.cos(b))], fill=color, width=width)

    def tour_rows(self) -> list[tuple[str, tuple]]:
        """투어 상태 두 줄 (상태·머무름·행동 / 기수 오차·요 명령)."""
        tr = self.state.get("tour")
        if not tr or time.time() - self.state.get("tour_t", 0) > 2.5:
            return []
        if not tr.get("on"):
            return [("투어 꺼짐 · t 로 켜기", (180, 180, 180))]
        st = tr.get("state")
        head = f"투어 · {tr.get('state_ko', st)}"
        if tr.get("target", -1) >= 0:
            head += f" · ID {tr['target']}"
        if st == "DWELL":
            head += f" · {tr.get('dwell_ms', 0) / 1000:.1f}/{tr.get('dwell_need_ms', 0) / 1000:.0f}초"
        elif st in ("TRANSIT", "RETURN"):
            head += f" · {tr.get('elapsed_ms', 0) / 1000:.1f}초 (떠난 태그 {tr.get('last', -1)})"
        color = {"DWELL": (0, 255, 0), "ALIGN": (120, 255, 200), "TRANSIT": (255, 220, 0), "RETURN": (255, 160, 60),
                 "LOST": (255, 60, 60), "DONE": (0, 220, 255)}.get(st, (255, 255, 255))
        rows = [(head, color)]
        detail = f"행동: {tr.get('act_ko', '-')}"
        if tr.get("yaw_on") or st in ("ALIGN", "DWELL"):
            detail += f" · 기수 오차 {tr.get('yaw_err', 0):+.0f}° · 요 {tr.get('yaw_us', 0):+.0f}us"
        if st == "TRANSIT":
            g = tr.get("goal") or [0, 0]
            detail += f" · 목표점 {math.hypot(g[0], g[1]):.0f}cm 앞"
        rows.append((detail, (220, 220, 220)))
        if tr.get("reason"):
            rows.append((tr["reason"], (170, 170, 170)))
        return rows

    def compose(self, jpg: bytes) -> Image.Image:
        img = Image.open(io.BytesIO(jpg)).convert("RGB")
        if img.width != 640:  # `snap small` 320×240 → 태그 좌표(640×480 기준)에 맞게 키운다
            img = img.resize((640, round(img.height * 640 / img.width)))
        W, H = img.size
        d = ImageDraw.Draw(img)
        cx, cy = W // 2, H // 2
        if self.coords:  # 좌표 방식: 옅은 격자를 먼저 (가운데 십자선 아래로)
            for x in range(0, W, 80):
                d.line([(x, 0), (x, H)], fill=(45, 45, 45))
            for y in range(0, H, 80):
                d.line([(0, y), (W, y)], fill=(45, 45, 45))
        up, down, right, left = ARROWS[self.args.rot]
        yellow = (255, 220, 0)
        for p0, p1 in (((cx, cy - 30), (cx, 60)), ((cx, cy + 30), (cx, H - 60)),
                       ((cx + 30, cy), (W - 60, cy)), ((cx - 30, cy), (60, cy))):
            d.line([p0, p1], fill=yellow, width=3)
        d.line([(cx - 20, cy), (cx + 20, cy)], fill=(0, 255, 255), width=2)
        d.line([(cx, cy - 20), (cx, cy + 20)], fill=(0, 255, 255), width=2)

        def label(text, xy, anchor, fill=yellow, font=None):
            font = font or self.font
            box = d.textbbox(xy, text, font=font, anchor=anchor)
            d.rectangle([box[0] - 5, box[1] - 3, box[2] + 5, box[3] + 3], fill=(0, 0, 0))
            d.text(xy, text, font=font, fill=fill, anchor=anchor)

        label(up, (cx, 34), "mt")
        label(down, (cx, H - 34), "mb")
        label(right, (W - 8, cy), "rm")
        label(left, (8, cy), "lm")

        if self.coords:
            self.draw_coords(d, W, H, cx, cy, label)
        # 투어 진행 방향 (기체 기준: 진행 방향 − 지금 기수). 기수와 맞으면 위쪽을 가리킨다
        tr = self.state.get("tour") or {}
        if tr.get("on") and tr.get("travel") is not None and tr.get("yaw") is not None \
                and time.time() - self.state.get("tour_t", 0) < 2.5:
            self.draw_arrow(d, cx, cy, tr["travel"] - tr["yaw"], 110, (0, 220, 255), width=4)
        tag = self.state["tag"]
        if not self.coords and tag and time.time() - self.state["tag_t"] < 1.5:
            tx, ty, half = tag["cx_px"], tag["cy_px"], tag["side_px"] / 2
            color = (0, 255, 0) if tag.get("registered") else (255, 140, 0)
            d.rectangle([tx - half, ty - half, tx + half, ty + half], outline=color, width=3)
            d.ellipse([tx - 5, ty - 5, tx + 5, ty + 5], fill=color)
            d.line([(cx, cy), (tx, ty)], fill=color, width=2)
            txt = (f"ID {tag['id']} {'' if tag.get('registered') else '(미등록)'}  "
                   f"R {tag.get('img_right_cm', 0):+.1f}cm  U {tag.get('img_up_cm', 0):+.1f}cm  "
                   f"거리 {tag.get('dist_est_cm', '?')}cm  크기 {tag.get('tag_size_mm', '?')}mm"
                   f"({'자동' if tag.get('size_src') == 'auto' else '설정'} · 표본 {tag.get('cal_n', 0)})")
            label(txt, (W - 6, H - 6), "rb", fill=color, font=self.small)

        fl, h, v = self.state["flow"], self.state["height"], self.state["vision"]
        rows = [f"rot={self.args.rot} 가정 · 거리 {h if h is not None else '-'}cm"]
        if fl:
            rows.append(f"광류 q{fl['q']}  X{fl['fx']:+d}  Y{fl['fy']:+d} deg/s")
        if v:
            rows.append(f"검출 {v.get('fps', 0):.1f}fps  {v.get('detect_ms', 0)}ms  {v.get('decision', '')}")
        if self.state["frame_dt"]:
            rows.append(f"영상 {1 / max(self.state['frame_dt'], 0.01):.1f}fps")
        age = time.time() - self.last_jpg_t
        ap_now = self.state.get("approach") or {}
        # 옛 펌웨어(-34 이하, `snap small` 을 몰라 `snap` 으로 돌아간 경우)만: 사진을 보내는 동안 인식이 멈춘다
        old_fw = (self.reader.ble or self.reader.wifi) and self.reader.snap_cmd == "snap"
        if old_fw and self.reader.snap_pending and ap_now.get("authority") == "auto":
            rows.append("AUTO 중 사진 받는 중 → 태그 인식 멈춤 → 보드 1500 (옛 펌웨어)")
        if not self.last_jpg and not self.coords:
            rows.append("사진 없음 · p 로 요청" + (" (받는 중)" if self.reader.snap_pending else ""))
        elif age > 2.5 and not self.coords:
            rows.append(f"사진 {age:.0f}초 전" + (" · 새 사진 받는 중(그동안 값 멈춤)" if self.reader.snap_pending else " · p 새 사진"))
        y = 6
        for r in rows:
            label(r, (6, y), "lt", fill=(255, 255, 255), font=self.small)
            y += 22

        # 접근 제어 → FC(INAV) 값. FC 값은 FC가 실제로 쓰는 채널(MSP_RC, Configurator Receiver 탭과 같은 값)
        ap, m = self.state["approach"], self.state["msp"] or {}
        if ap:
            fault = ap.get("fault")
            v = ap.get("v_flow") or [0, 0]
            mode = {"on": "출력", "shadow": "관찰", "off": "꺼짐", "legacy": "legacy"}.get(ap.get("mode"), ap.get("mode"))
            label(f"[{mode}·{ap.get('vel_src', '-')}] 접근 {ap['state']}{'(' + fault + ')' if fault else ''} · 목표 {ap['target_speed']:.1f}cm/s"
                  f" · 상대속도 R{v[0]:+.1f} F{v[1]:+.1f}", (6, y + 4), "lt", fill=(120, 220, 255), font=self.small)
            tilt = ap.get("tilt") or [0, 0]
            label(f"보드 명령  롤 {ap['roll_us']:+.0f}us({tilt[0]:+.1f}°)  피치 {ap['pitch_us']:+.0f}us({tilt[1]:+.1f}°)"
                  f"  expo {ap.get('expo', '-')}", (6, y + 28), "lt", fill=(120, 220, 255), font=self.small)
            kind, why = self.command_reason(ap)
            color = {"active": (0, 255, 0), "ready": (120, 255, 200), "warn": (255, 60, 60)}.get(kind, (255, 160, 60))
            label(why, (6, y + 52), "lt", fill=color, font=self.small)
            y += 76
        for text, col in self.tour_rows():
            label(text, (6, y + 4), "lt", fill=col, font=self.small)
            y += 24
        # -34 부터 approach 줄(5Hz)에 FC 롤·피치(fc_rp)가 실린다 — msp 줄(1Hz)보다 짧은 명령을 덜 놓친다
        fc = (ap or {}).get("fc_rp") or m.get("fc_rc") or []
        if len(fc) >= 2:
            changed = abs(fc[0] - 1500) > 3 or abs(fc[1] - 1500) > 3
            label(f"INAV 롤 {fc[0]}  피치 {fc[1]}", (W - 8, 70), "rt",
                  fill=(0, 255, 0) if changed else (255, 255, 255), font=self.font)
            label(f"CH6 {m.get('ch6', '-')} · {('AUTO' if m.get('authority') == 'auto' else 'MANUAL')}", (W - 8, 100),
                  "rt", fill=(255, 255, 255), font=self.small)
        return img

    def render(self, jpg: bytes):
        self.image = self.compose(jpg)
        self.last_render = time.time()
        self.photo = _to_photo(self.image)
        self.label.configure(image=self.photo)
        if self.args.dump and time.time() - self.last_dump > 2:
            self.image.save(self.args.dump, quality=90)
            self.last_dump = time.time()

    def tick(self):
        try:
            while True:
                kind, payload, extra = self.q.get_nowait()
                if kind == "json":
                    self.handle_json(payload)
                elif kind == "drop":
                    self.state["drops"] = self.state.get("drops", 0) + 1
                else:
                    try:
                        self.render(payload)
                    except OSError:
                        self.state["drops"] = self.state.get("drops", 0) + 1
                        continue
                    self.last_jpg, self.last_jpg_t = payload, time.time()
                    self.state["frame_dt"] = extra
                    if self.ap_log:  # 영상 속도 분석용 (사진 도착 시각·간격·크기)
                        self.ap_log.write(json.dumps({"t": round(time.time() - self.t0, 2),
                                                      "frame": {"dt": round(extra or 0, 3), "bytes": len(payload)}}) + "\n")
                    self.state["frames"] += 1
        except queue.Empty:
            pass
        # 새 사진이 없어도 태그·INAV 값 오버레이는 0.25초마다 마지막 사진 위에 다시 그린다
        if time.time() - self.last_render > (0.1 if self.coords else 0.25):
            self.render(self.last_jpg or self.blank_jpg)
        m = self.state["msp"] or {}
        self.status.configure(text=(
            f"프레임 {self.state['frames']} (손실 {self.state.get('drops', 0)})  ·  FC {'연결' if m.get('fc_link') else '-'}  "
            f"제어권 {m.get('authority', '-')}  CH6 {m.get('ch6', '-')}  ·  o 출력 켬 · w 관찰 · t 투어 · v 좌표/영상 · p 사진 · s 저장 · q 종료"))
        self.root.after(40, self.tick)

    def run(self):
        self.reader.start()
        self.root.after(40, self.tick)
        try:
            self.root.mainloop()
        finally:
            self.reader.stop.set()
            if self.log:
                self.log_f.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--ble", action="store_true", help="USB 대신 블루투스(TagBoard-…)로 연결. bleak 필요")
    ap.add_argument("--mode", choices=("coords", "video"), default="coords",
                    help="coords(기본): 사진 없이 위치 좌표로 그림 — 보드 부담·전송량 최소. video: 예전처럼 사진. v 키로 바꿈")
    ap.add_argument("--stream-ms", type=int, default=100, help="WiFi: 보드가 사진을 보내는 최소 간격(ms). 실제로는 인식 주기·WiFi 속도만큼")
    ap.add_argument("--wifi", nargs="?", const="192.168.4.1", default=None,
                    help="보드 자체 WiFi(-36~)로 연결. Mac 을 먼저 WiFi TagBoard-…(비밀번호 config.h WIFI_AP_PASS)에 붙인다")
    ap.add_argument("--rot", type=int, default=0, choices=(0, 90, 180, 270))
    ap.add_argument("--log")
    ap.add_argument("--dump")
    ap.add_argument("--init", help="보드가 켜진 뒤 보낼 명령(세미콜론 구분). 예: \"approach bench;approach set vel_kp 6\"")
    ap.add_argument("--ap-log", help="approach 보고와 FC 채널 값을 JSON 줄로 기록")
    ap.add_argument("--fc-flow", action="store_true", help="FC 광류를 fcmsp 0x2001 로 직접 물어 표시 (보드 질의와 겹칠 수 있음)")
    ap.add_argument("--snap-period", type=float, default=0.0,
                    help="사진 사이 쉬는 시간(초). 0 = 쉬지 않음, 음수 = p 키로만. 시험 중에는 2 정도로 두면 태그 인식 속도를 덜 떨어뜨린다")
    Viewer(ap.parse_args()).run()


if __name__ == "__main__":
    sys.exit(main())
