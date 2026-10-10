"""프로젝트 카메라 뷰어의 관측 산출물을 공유한다. 소켓·프로세스는 건드리지 않는다."""
from __future__ import annotations

import io
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import time
from dataclasses import dataclass
from typing import Any

from PIL import Image

VIEWER = Path(__file__).resolve().parents[2] / "tools" / "cam_view.py"


@dataclass(frozen=True)
class ViewerProcess:
    pid: int
    log: Path | None
    image: Path | None

    def alive(self) -> bool:
        try:
            os.kill(self.pid, 0)
            return True
        except ProcessLookupError:
            return False


def find_viewer(host: str) -> ViewerProcess | None:
    try:
        listing = subprocess.run(["ps", "-axo", "pid=,args="], capture_output=True, text=True, timeout=1, check=True).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    for line in reversed(listing.splitlines()):
        if str(VIEWER) not in line:
            continue
        try:
            pid, command = line.strip().split(None, 1)
            args = shlex.split(command)
            if str(VIEWER) not in args or "--wifi" not in args:
                continue
            def option(name: str) -> str | None:
                index = args.index(name) + 1 if name in args else len(args)
                return args[index] if index < len(args) and not args[index].startswith("--") else None
            if (option("--wifi") or "192.168.4.1") != host:
                continue
            log, image = option("--ap-log"), option("--dump")
            return ViewerProcess(int(pid), Path(log) if log else None, Path(image) if image else None)
        except (ValueError, IndexError):
            continue
    return None


class ViewerReader:
    def __init__(self, viewer: ViewerProcess):
        self.viewer = viewer
        self.offset = 0
        self.pending = b""
        self.inode: tuple[int, int] | None = None
        self.generation = 0
        self.photo_mtime = 0
        self.frame_at: float | None = None
        self.transfer_ms: float | None = None

    def telemetry(self, now: float) -> list[tuple[dict[str, Any], float]]:
        path = self.viewer.log
        if path is None:
            return []
        try:
            stat = path.stat()
            identity = stat.st_dev, stat.st_ino
            if self.inode != identity or stat.st_size < self.offset:
                self.inode = identity
                self.offset = max(0, stat.st_size - 65_536)
                self.pending = b""
                self.generation += 1
                self.frame_at = None
                skip_partial = self.offset > 0
            else:
                skip_partial = False
            with path.open("rb") as file:
                file.seek(self.offset)
                chunk = file.read(262_144)
                self.offset += len(chunk)
            if skip_partial:
                chunk = chunk.partition(b"\n")[2]
            lines = (self.pending + chunk).split(b"\n")
            self.pending = lines.pop()
            if len(self.pending) > 65_536:
                self.pending = b""
            rows = []
            for line in lines:
                try:
                    row = json.loads(line, parse_constant=lambda _: None)
                    if isinstance(row, dict) and isinstance(row.get("t"), (float, int)) and math.isfinite(row["t"]):
                        rows.append(row)
                except (ValueError, UnicodeDecodeError):
                    continue
            if not rows:
                return []
            newest = max(row["t"] for row in rows)
            anchor = now - max(0, time.time() - stat.st_mtime)
            events = []
            for row in rows:
                at = anchor - max(0, newest - row["t"])
                for key in ("ap", "msp", "fc", "vision", "evt"):
                    value = row.get(key)
                    if isinstance(value, dict) and isinstance(value.get("type"), str):
                        events.append((value, at))
                frame = row.get("frame")
                if isinstance(frame, dict):
                    self.frame_at = at
                    dt = frame.get("dt")
                    self.transfer_ms = dt * 1000 if isinstance(dt, (int, float)) and math.isfinite(dt) else None
            return events
        except OSError:
            return []

    def camera(self, now: float) -> tuple[bytes, int, int, float] | None:
        path = self.viewer.image
        if path is None or self.frame_at is None:
            return None
        try:
            stat = path.stat()
            if stat.st_mtime_ns == self.photo_mtime or not 0 < stat.st_size <= 2_000_000:
                return None
            image = path.read_bytes()
            with Image.open(io.BytesIO(image)) as decoded:
                width, height = decoded.size
                if decoded.format != "JPEG" or not (0 < width <= 1920 and 0 < height <= 1080):
                    return None
                decoded.verify()
            self.photo_mtime = stat.st_mtime_ns
            # 오버레이만 다시 저장해도 새 카메라 사진으로 간주하지 않는다.
            photo_at = min(self.frame_at, now - max(0, time.time() - stat.st_mtime))
            return image, width, height, photo_at
        except (OSError, ValueError):
            return None
