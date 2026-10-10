"""TagBoard의 읽기 전용 Wi-Fi 영상·계측 중계. FC 제어 명령은 제공하지 않는다."""
from __future__ import annotations

import base64
import copy
import ipaddress
import json
import math
import os
import re
import socket
import struct
import threading
import time
from typing import Any, Callable

from .hardware import HardwareError
from .viewer_bridge import ViewerProcess, ViewerReader, find_viewer


class BoardFrames:
    """분할 TCP·JSON/사진 혼합 스트림. 완성·검증된 JPEG만 게시한다."""

    MAX_LINE = 16_384
    MAX_IMAGE = 500_000

    def __init__(self) -> None:
        self.buffer = b""
        self.header: tuple[int, int, int, int] | None = None
        self.parts: list[bytes] = []
        self.size = 0
        self.errors = 0
        self.binary = False
        self.chunk_remaining = 0

    def reset_image(self) -> None:
        self.header, self.parts, self.size = None, [], 0
        self.binary, self.chunk_remaining = False, 0

    def feed(self, chunk: bytes) -> list[tuple[str, Any]]:
        self.buffer += chunk
        events: list[tuple[str, Any]] = []
        while self.buffer:
            # SNAPC의 원본 바이트에는 줄바꿈·JSON처럼 생긴 내용도 들어갈 수 있다.
            # 길이만큼 먼저 소비해야 상태 보고나 사진 종료 표시로 오인하지 않는다.
            if self.chunk_remaining:
                if len(self.buffer) < self.chunk_remaining:
                    break
                part, self.buffer = self.buffer[:self.chunk_remaining], self.buffer[self.chunk_remaining:]
                self.chunk_remaining = 0
                if self.header is not None and self.binary:
                    self.parts.append(part)
                    self.size += len(part)
                continue
            if b"\n" not in self.buffer:
                break
            raw, self.buffer = self.buffer.split(b"\n", 1)
            line = raw.strip()
            if len(line) > self.MAX_LINE:
                self.errors += 1
                self.reset_image()
                continue
            if line.startswith(b"SNAPB "):
                self.reset_image()
                try:
                    count, width, height = map(int, line.split()[1:])
                    if not (0 < count <= self.MAX_IMAGE and 0 < width <= 1920 and 0 < height <= 1080):
                        raise ValueError
                    self.header = count, width, height, count
                    self.binary = True
                except ValueError:
                    self.errors += 1
            elif line.startswith(b"SNAPC "):
                try:
                    parts = line.split()
                    if len(parts) != 2:
                        raise ValueError
                    count = int(parts[1])
                    if not 0 < count <= self.MAX_LINE:
                        raise ValueError
                    if self.header is None or not self.binary or self.size + count > self.header[3]:
                        self.errors += 1
                        self.reset_image()
                    # 잘못된 사진도 유효한 조각 길이는 건너뛰어 뒤의 JSON을 보존한다.
                    self.chunk_remaining = count
                except ValueError:
                    self.errors += 1
                    self.reset_image()
            elif line.startswith(b"SNAP_BEGIN "):
                self.reset_image()
                try:
                    count, width, height, encoded = map(int, line.split()[1:])
                    if not (0 < count <= self.MAX_IMAGE and 0 < width <= 1920 and 0 < height <= 1080
                            and encoded == 4 * ((count + 2) // 3)):
                        raise ValueError
                    self.header = count, width, height, encoded
                except ValueError:
                    self.errors += 1
            elif line == b"SNAP_END":
                try:
                    if self.header is None or self.size != self.header[3]:
                        raise ValueError
                    payload = b"".join(self.parts)
                    image = payload if self.binary else base64.b64decode(payload, validate=True)
                    if len(image) != self.header[0] or not image.startswith(b"\xff\xd8") or not image.endswith(b"\xff\xd9"):
                        raise ValueError
                    events.append(("image", (image, self.header[1], self.header[2])))
                except ValueError:
                    self.errors += 1
                self.reset_image()
            elif line.startswith(b"{"):
                try:
                    row = json.loads(line, parse_constant=lambda _: None)
                    if isinstance(row, dict) and isinstance(row.get("type"), str):
                        events.append(("telemetry", row))
                except (ValueError, UnicodeDecodeError):
                    self.errors += 1
            elif self.header is not None and line and not self.binary:
                self.parts.append(line)
                self.size += len(line)
                if self.size > self.header[3]:
                    self.errors += 1
                    self.reset_image()
        if len(self.buffer) > self.MAX_LINE:
            self.buffer = b""
            self.reset_image()
            self.errors += 1
        return events


class BoardWifiService:
    # 쓰기·조종 명령을 받을 수 있는 범용 전송 API를 의도적으로 두지 않는다.
    READ_COMMANDS = frozenset({b"info\n", b"wifi\n", b"fcmsp 108\n", b"snap small\n", b"snap stream 200\n"})
    TYPES = frozenset({"info", "boot", "wifi", "flow", "msp", "fc", "approach", "vision", "gimbal", "fcmsp", "snap_stream", "snap_error"})

    def __init__(self, *, connector: Callable[..., Any] = socket.create_connection,
                 now: Callable[[], float] = time.monotonic,
                 viewer_finder: Callable[[str], ViewerProcess | None] = find_viewer) -> None:
        self.host = os.environ.get("D4D_BOARD_HOST", "192.168.0.151")
        self.port = 3333
        self._connector, self._now = connector, now
        self._viewer_finder = viewer_finder
        self.source = "direct_wifi"
        self._lock = threading.RLock()
        self._control = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._socket: Any = None
        self.state = "disconnected"
        self.error = ""
        self.session = 0
        self.sequence = 0
        self.latest: dict[str, dict[str, Any]] = {}
        self.last_at: float | None = None
        self.image: bytes | None = None
        self.frame: dict[str, Any] = {"sequence": 0, "width": 0, "height": 0, "at": None, "transfer_ms": None}
        self.parse_errors = 0
        self._photo_pending: float | None = None

    @staticmethod
    def validate_host(host: str) -> str:
        try:
            address = ipaddress.IPv4Address(host)
        except ipaddress.AddressValueError as exc:
            raise HardwareError("invalid_board_address", "보드의 Wi-Fi IPv4 주소를 입력해 주세요.", 422) from exc
        private = any(address in ipaddress.IPv4Network(net) for net in ("10.0.0.0/8", "172.16.0.0/12", "192.168.0.0/16", "127.0.0.0/8"))
        if not private or address.is_unspecified or address.is_multicast:
            raise HardwareError("invalid_board_address", "같은 로컬 망의 보드 주소만 연결할 수 있습니다.", 422)
        return str(address)

    def connect(self, host: str) -> dict[str, Any]:
        host = self.validate_host(host)
        with self._control:
            if self._thread and self._thread.is_alive() and host == self.host and not self._stop.is_set():
                return self.snapshot()
            self._disconnect()
            with self._lock:
                self.host = host
                self.state, self.error = "connecting", ""
            self._stop = threading.Event()
            self._thread = threading.Thread(target=self._run, name="tagboard-wifi", daemon=True)
            self._thread.start()
        return self.snapshot()

    def _disconnect(self) -> None:
        self._stop.set()
        sock = self._socket
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        if self._thread:
            self._thread.join(timeout=4)
        with self._lock:
            self.state = "disconnected"
            self.error = ""
            self._socket = None

    def disconnect(self) -> dict[str, Any]:
        with self._control:
            self._disconnect()
        return self.snapshot()

    def _send(self, command: bytes) -> None:
        if command not in self.READ_COMMANDS:
            raise ValueError("조회 명령만 허용됩니다")
        self._socket.sendall(command)

    def _record(self, row: dict[str, Any], now: float) -> None:
        kind = row.get("type")
        if kind not in self.TYPES:
            return
        self.sequence += 1
        self.latest[kind] = {"data": row, "at_ms": now * 1000, "sequence": self.sequence}
        self.last_at = now
        # 일부 접근 보고는 자세 질의 사이에 att=null을 싣는다. 마지막 유효한 계측 시각을 따로 보존한다.
        att = row.get("att")
        if kind == "approach" and isinstance(att, list) and len(att) >= 2 and all(
            isinstance(v, (int, float)) and math.isfinite(v) and -180 <= v <= 180 for v in att[:2]
        ):
            self.latest["attitude_rp"] = {"data": {"roll_deg": att[0], "pitch_deg": att[1]},
                                          "at_ms": now * 1000, "sequence": self.sequence}
        if kind == "fcmsp" and row.get("cmd") == 108 and row.get("ok") is True and row.get("error") is False:
            try:
                raw = bytes.fromhex(row["hex"])
                if len(raw) != 6:
                    return
                roll, pitch, yaw = struct.unpack("<3h", raw)
                self.latest["attitude"] = {"data": {"roll_deg": roll / 10, "pitch_deg": pitch / 10, "yaw_deg": yaw},
                                           "at_ms": now * 1000, "sequence": self.sequence}
            except (ValueError, TypeError, KeyError):
                pass

    def _camera_allowed(self, now: float) -> bool:
        info = self.latest.get("info", self.latest.get("boot", {})).get("data", {})
        build = re.fullmatch(r"TAGVISION-(\d{8})-(\d+)", str(info.get("build", "")))
        # 이전 빌드는 사진 전송 중 검출을 멈추므로 자동으로 사진을 요청하지 않는다.
        safe_build = bool(build and (int(build[1]), int(build[2])) >= (20261011, 35))
        return safe_build and info.get("camera") is True and self.last_at is not None and now - self.last_at < 3

    def _stream_supported(self) -> bool:
        info = self.latest.get("info", self.latest.get("boot", {})).get("data", {})
        build = re.fullmatch(r"TAGVISION-(\d{8})-(\d+)", str(info.get("build", "")))
        # -38부터 확인한 SNAPB/SNAPC 바이너리 스트림. 이전 보드는 개별 사진 조회를 유지한다.
        return bool(build and (int(build[1]), int(build[2])) >= (20261011, 38))

    def _run(self) -> None:
        delay = 1.0
        while not self._stop.is_set():
            try:
                viewer = self._viewer_finder(self.host)
                if viewer is not None:
                    self._run_shared(viewer)
                    continue
                sock = self._connector((self.host, self.port), timeout=2)
                sock.settimeout(0.1)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                with self._lock:
                    self._socket = sock
                    self.source = "direct_wifi"
                    self.session += 1
                    self.latest, self.last_at, self.image = {}, None, None
                    self.frame = {"sequence": 0, "width": 0, "height": 0, "at": None, "transfer_ms": None}
                    self.state, self.error = "connecting", ""
                    self._photo_pending = None
                parser = BoardFrames()
                start = self._now()
                info_at = attitude_at = photo_at = start
                viewer_check_at = start
                stream_at = None
                self._send(b"info\n")
                self._send(b"wifi\n")
                while not self._stop.is_set():
                    now = self._now()
                    try:
                        chunk = sock.recv(16_384)
                        if not chunk:
                            raise ConnectionError("보드 연결이 종료됐습니다. 별도 카메라 뷰어의 동시 연결도 확인해 주세요.")
                    except socket.timeout:
                        chunk = b""
                    with self._lock:
                        for kind, value in parser.feed(chunk):
                            if kind == "telemetry":
                                self._record(value, now)
                            else:
                                self.image, width, height = value
                                transfer = (now - self._photo_pending) * 1000 if self._photo_pending is not None else None
                                self.frame = {"sequence": self.frame["sequence"] + 1, "width": width, "height": height,
                                              "at": now, "transfer_ms": transfer}
                                self._photo_pending = None
                                photo_at = now + 0.25
                        self.parse_errors = parser.errors
                        if self.last_at is not None:
                            self.state, self.error, delay = "connected", "", 1.0
                    if now - (self.last_at if self.last_at is not None else start) > 8:
                        raise TimeoutError("보드 응답이 없습니다. Wi-Fi 망과 보드 주소를 확인해 주세요.")
                    if now - viewer_check_at > 3:
                        viewer_check_at = now
                        if self._viewer_finder(self.host) is not None:
                            break
                    if now - info_at > 5:
                        self._send(b"info\n")
                        self._send(b"wifi\n")
                        info_at = now
                    # FC 자체 상태 질의를 방해하지 않도록 짧은 ATTITUDE 조회만 최대 2Hz.
                    if self.last_at is not None and now - attitude_at >= 0.5:
                        self._send(b"fcmsp 108\n")
                        attitude_at = now
                    if self._photo_pending is not None and now - self._photo_pending > 8:
                        self._photo_pending = None
                        parser.reset_image()
                        photo_at = now + 1
                    if self._camera_allowed(now):
                        if self._stream_supported():
                            frame_at = self.frame["at"]
                            if stream_at is None or (now - stream_at > 8 and (frame_at is None or now - frame_at > 8)):
                                self._send(b"snap stream 200\n")
                                stream_at = now
                        elif now >= photo_at and self._photo_pending is None:
                            self._send(b"snap small\n")
                            self._photo_pending = now
            except OSError as exc:
                with self._lock:
                    self.state = "reconnecting"
                    self.error = str(exc)
            finally:
                sock = self._socket
                if sock is not None:
                    try:
                        sock.close()
                    except OSError:
                        pass
                self._socket = None
            if self._stop.wait(delay):
                break
            delay = min(delay * 2, 8)

    def _run_shared(self, viewer: ViewerProcess) -> None:
        reader = ViewerReader(viewer)
        generation = -1
        with self._lock:
            self.source = "camera_viewer"
        while not self._stop.is_set() and viewer.alive():
            now = self._now()
            events = reader.telemetry(now)
            with self._lock:
                if reader.generation != generation:
                    generation = reader.generation
                    self.session += 1
                    self.latest, self.last_at, self.image = {}, None, None
                    self.frame = {"sequence": 0, "width": 0, "height": 0, "at": None, "transfer_ms": None}
                for row, at in events:
                    self._record(row, at)
                photo = reader.camera(now)
                if photo:
                    self.image, width, height, at = photo
                    self.frame = {"sequence": self.frame["sequence"] + 1, "width": width, "height": height,
                                  "at": at, "transfer_ms": reader.transfer_ms}
                fresh = self.last_at is not None and now - self.last_at < 3
                self.state = "connected" if fresh else "connecting"
                self.error = "" if fresh else (
                    "기존 카메라 뷰어에 --ap-log와 --dump 설정이 필요합니다."
                    if viewer.log is None or viewer.image is None else "기존 카메라 뷰어의 실측 보고 갱신을 기다립니다. 뷰어의 보드 연결을 확인해 주세요."
                )
            self._stop.wait(.1)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            now = self._now()
            latest = copy.deepcopy(self.latest)
            for item in latest.values():
                item["age_ms"] = max(0, now * 1000 - item["at_ms"])
            frame = {k: v for k, v in self.frame.items() if k != "at"}
            frame["age_ms"] = None if self.frame["at"] is None else max(0, (now - self.frame["at"]) * 1000)
            info = latest.get("info", latest.get("boot", {})).get("data", {})
            build = info.get("build", "")
            return {"state": self.state, "host": self.host, "port": self.port,
                    "source": self.source,
                    "session": self.session, "server_ms": now * 1000, "read_only": True,
                    "error": self.error, "latest": latest, "frame": frame,
                    "received_age_ms": None if self.last_at is None else (now - self.last_at) * 1000,
                    "parse_errors": self.parse_errors, "build": build}

    def camera_frame(self) -> bytes | None:
        with self._lock:
            if self.state != "connected" or self.frame["at"] is None or self._now() - self.frame["at"] > 10:
                return None
            return self.image
