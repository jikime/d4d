"""태그 보드 카메라 USB 라이브 뷰어 (1~2 fps).

보드의 `snap` 명령을 반복 호출해 JPEG를 받고, 같은 포트로 들어오는 tag/flow/vision JSON과
FC 광류(MSP2_INAV_OPTICAL_FLOW, fcmsp 0x2001)를 함께 오버레이한다.

사용:
  uv run --with pyserial --with pillow python tools/cam_view.py --port COM10
  uv run --with pyserial --with pillow --with bleak python tools/cam_view.py --ble   (USB 없이 블루투스로)
  옵션:
    --rot 0|90|180|270   화살표에 쓸 카메라 장착 회전 (펌웨어 CAM_ROTATE_DEG와 같은 의미)
    --log out.csv        flow/tag 샘플을 CSV로 기록 (축 검증용)
    --dump latest.jpg    오버레이된 최신 프레임을 2초마다 파일로 저장
  키: q/Esc 종료, s 스냅샷 저장(Downloads), p 새 사진 바로 요청 (BLE 에서 유용)
"""
from __future__ import annotations

import argparse
import asyncio
import base64
import csv
import io
import json
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


class Reader(threading.Thread):
    def __init__(self, port: str, out: queue.Queue, init_cmds: list[str] | None = None, snap_period: float = 0.0):
        super().__init__(daemon=True)
        self.snap_period = snap_period  # 사진 사이 쉬는 시간(초). 사진을 받는 동안 보드의 태그 인식이 느려진다
        self.fc_flow = False  # FC 광류를 fcmsp 로 직접 묻기 (보드의 FC 설정 읽기와 겹치므로 기본 끔)
        self.next_snap_at = 0.0
        self.ble = port.upper() == "BLE"
        self.ser = BleSerial() if self.ble else serial.Serial(port, 115200, timeout=0.05)
        self.snap_timeout = 25.0 if self.ble else SNAP_TIMEOUT  # BLE 는 사진 한 장에 10초 넘게 걸린다
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
        self.init_sent = True
        self.out.put(("json", {"type": "_init_sent", "cmds": self.init_cmds}, None))

    def request_snap(self):
        self.ser.write(b"snap\n")
        self.snap_pending = time.time()

    def run(self):
        time.sleep(0.3)
        self.ser.reset_input_buffer()
        buf = b""
        header = None
        b64: list[str] = []
        self.request_snap()
        while not self.stop.is_set():
            now = time.time()
            if self.fc_flow and now - self.last_flow_q >= FLOW_PERIOD:
                self.ser.write(FLOW_QUERY)
                self.last_flow_q = now
            if self.snap_pending and now - self.snap_pending > self.snap_timeout:
                header, b64 = None, []
                self.request_snap()
            if self.next_snap_at and now >= self.next_snap_at:
                self.next_snap_at = 0.0
                self.request_snap()
            # USB 포트를 열면 보드가 재부팅된다 → boot 줄을 본 뒤(또는 5초 뒤) 시험 명령을 보낸다. BLE 는 재부팅 없음 → 바로
            if not self.init_sent and now - self.t_open > (1.0 if self.ble else 5.0):
                self.send_init()
            chunk = self.ser.read(8192)
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("utf-8", "replace").strip()
                if not line:
                    continue
                if line.startswith("SNAP_BEGIN"):
                    header, b64 = line.split(), []
                    continue
                if line == "SNAP_END":
                    if header:
                        joined = "".join(b64)
                        expected = int(header[4]) if len(header) > 4 and header[4].isdigit() else len(joined)
                        if len(joined) == expected:
                            try:
                                jpg = base64.b64decode(joined)
                                self.out.put(("frame", jpg, time.time() - self.snap_pending))
                            except Exception:
                                self.out.put(("drop", None, None))
                        else:
                            self.out.put(("drop", None, None))
                    header, b64 = None, []
                    self.snap_pending = 0.0
                    if self.snap_period > 0:
                        self.next_snap_at = time.time() + self.snap_period
                    else:
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
                    self.out.put(("json", d, None))
                elif header is not None:
                    b64.append(line)
        self.ser.close()


class Viewer:
    def __init__(self, args):
        self.args = args
        self.q: queue.Queue = queue.Queue()
        init_cmds = [x for x in (args.init or "").split(";") if x.strip()]
        if args.ble:
            args.port = "BLE"
        self.reader = Reader(args.port, self.q, init_cmds, args.snap_period)
        self.reader.fc_flow = args.fc_flow
        self.state = {"tag": None, "tag_t": 0.0, "height": None, "flow": None, "vision": None,
                      "frame_dt": None, "frames": 0, "msp": None, "approach": None, "init": None}
        self.ap_log = open(args.ap_log, "w", encoding="utf-8") if args.ap_log else None
        self.font = _korean_font(22, bold=True)
        self.small = _korean_font(15)
        self.image: Image.Image | None = None
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
        elif t == "msp":
            s["msp"] = d
        elif t == "approach":
            s["approach"] = d
            if self.ap_log:
                fc = (s["msp"] or {}).get("fc_rc")
                self.ap_log.write(json.dumps({"t": round(time.time() - self.t0, 2), "ap": d, "fc_rc": fc},
                                             ensure_ascii=False) + "\n")
                self.ap_log.flush()
            return
        elif t in ("_init_sent", "approach_set", "approach_mode", "cmd_error", "exp", "boot", "_raw", "tagcal"):
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
    def command_reason(ap: dict) -> tuple[bool, str]:
        """INAV 롤·피치가 1500 에서 움직이는지와 그 이유 (approach.h step 의 판정 순서 그대로)."""
        fault, state, dist = ap.get("fault"), ap.get("state"), ap.get("dist_cm", 0.0)
        reasons = {
            "disabled": "접근 제어 꺼짐",
            "manual_authority": "CH6 수동 — 조종기가 제어",
            "kill": "킬 스위치",
            "mask_unknown": "FC 오버라이드 마스크 확인 중",
            "mask": "FC 오버라이드 마스크가 롤·피치를 안 받음",
            "height": f"높이 {ap.get('height_cm', 0):.0f}cm — 20~300cm 밖 (거리센서)",
            "flow": ("태그 속도 없음 — 태그 미인식 또는 막 다시 보임(2장 필요)" if ap.get("vel_src") == "tag"
                     else "광류 속도 없음"),
            "velocity": "너무 빠르게 움직임 (60cm/s 초과)",
            "tag_lost": "태그 놓침 — 중립",
        }
        if fault:
            return False, "INAV 1500 · " + reasons.get(fault, fault)
        if state == "ARRIVED":
            return False, "INAV 1500 · 도착(5cm 안 2초) — 5cm 밖으로 벗어나야 다시 동작"
        if dist <= 5.0:
            return False, f"INAV 1500 · 태그가 중심에서 {dist:.1f}cm — 정지 반경 5cm 안"
        return True, f"INAV 명령 중 · 태그 {dist:.0f}cm 떨어짐 → 목표 속도 {ap.get('target_speed', 0):.1f}cm/s"

    def compose(self, jpg: bytes) -> Image.Image:
        img = Image.open(io.BytesIO(jpg)).convert("RGB")
        W, H = img.size
        d = ImageDraw.Draw(img)
        cx, cy = W // 2, H // 2
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

        tag = self.state["tag"]
        if tag and time.time() - self.state["tag_t"] < 1.5:
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
        if age > 2.5:
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
            label(f"접근 {ap['state']}{'(' + fault + ')' if fault else ''} · 목표 {ap['target_speed']:.1f}cm/s"
                  f" · 상대속도 R{v[0]:+.1f} F{v[1]:+.1f}", (6, y + 4), "lt", fill=(120, 220, 255), font=self.small)
            tilt = ap.get("tilt") or [0, 0]
            label(f"보드 명령  롤 {ap['roll_us']:+.0f}us({tilt[0]:+.1f}°)  피치 {ap['pitch_us']:+.0f}us({tilt[1]:+.1f}°)"
                  f"  expo {ap.get('expo', '-')}", (6, y + 28), "lt", fill=(120, 220, 255), font=self.small)
            active, why = self.command_reason(ap)
            label(why, (6, y + 52), "lt", fill=(0, 255, 0) if active else (255, 160, 60), font=self.small)
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
                    self.state["frames"] += 1
        except queue.Empty:
            pass
        # 새 사진이 없어도 태그·INAV 값 오버레이는 0.25초마다 마지막 사진 위에 다시 그린다
        if self.last_jpg and time.time() - self.last_render > 0.25:
            self.render(self.last_jpg)
        m = self.state["msp"] or {}
        self.status.configure(text=(
            f"프레임 {self.state['frames']} (손실 {self.state.get('drops', 0)})  ·  FC {'연결' if m.get('fc_link') else '-'}  "
            f"제어권 {m.get('authority', '-')}  CH6 {m.get('ch6', '-')}  ·  q 종료 · s 저장 · p 새 사진"))
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
    ap.add_argument("--rot", type=int, default=0, choices=(0, 90, 180, 270))
    ap.add_argument("--log")
    ap.add_argument("--dump")
    ap.add_argument("--init", help="보드가 켜진 뒤 보낼 명령(세미콜론 구분). 예: \"approach bench;approach set vel_kp 6\"")
    ap.add_argument("--ap-log", help="approach 보고와 FC 채널 값을 JSON 줄로 기록")
    ap.add_argument("--fc-flow", action="store_true", help="FC 광류를 fcmsp 0x2001 로 직접 물어 표시 (보드 질의와 겹칠 수 있음)")
    ap.add_argument("--snap-period", type=float, default=0.0,
                    help="사진 사이 쉬는 시간(초). 0 = 쉬지 않음. 시험 중에는 2 정도로 두면 태그 인식 속도를 덜 떨어뜨린다")
    Viewer(ap.parse_args()).run()


if __name__ == "__main__":
    sys.exit(main())
