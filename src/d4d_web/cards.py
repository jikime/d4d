"""현장 설치용 결정론적 플랜 카드 PNG 생성."""
from __future__ import annotations

import hashlib
import io
import os
import re
import urllib.request
from pathlib import Path
from typing import Any


CARD_FONT_PATHS = (
    str(Path(os.environ.get("WINDIR", "C:/Windows")) / "Fonts" / "malgun.ttf"),
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJKkr-Regular.otf",
    "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/google-noto-cjk/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
    "/System/Library/Fonts/AppleSDGothicNeo.ttc",
)


def _card_font(size: int) -> Any:
    """설치된 한글 글꼴을 사용하고, 네모 글자로 된 카드는 만들지 않는다."""
    from PIL import ImageFont

    for path in CARD_FONT_PATHS:
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            continue
    raise RuntimeError(
        "플랜 카드용 한글 글꼴이 없습니다. Windows 맑은 고딕 또는 Linux "
        "Noto Sans CJK/NanumGothic 글꼴이 필요합니다. README 개발 실행 항목을 확인하세요."
    )


def _text_lines(draw: Any, text: str, font: Any, max_width: int) -> list[str]:
    """공백과 긴 한글 문자열 모두 지정한 인쇄 영역 안에서 줄바꿈한다."""
    lines: list[str] = []
    line = ""
    for char in text:
        if line and draw.textlength(line + char, font=font) > max_width:
            lines.append(line.rstrip())
            line = char.lstrip()
        else:
            line += char
    if line:
        lines.append(line.rstrip())
    return lines


def _draw_text_block(draw: Any, text: str, xy: tuple[int, int], font: Any, max_width: int) -> int:
    x, y = xy
    line_height = font.size + 8
    for line in _text_lines(draw, text, font, max_width):
        draw.text((x, y), line, fill="black", font=font)
        y += line_height
    return y


def _marker_bits(tag_id: int, size: int = 8) -> list[list[int]]:
    digest = hashlib.sha256(f"SPIDER:{tag_id}".encode()).digest()
    bits = [[0] * size for _ in range(size)]
    cursor = 0
    for y in range(1, size - 1):
        for x in range(1, size - 1):
            bits[y][x] = (digest[cursor // 8] >> (cursor % 8)) & 1
            cursor += 1
    # 방향을 판별할 수 있는 고정 코너
    bits[1][1], bits[1][size - 2], bits[size - 2][1] = 1, 0, 0
    return bits


def create_plan_card(tag: dict[str, Any], output: Path, scenario_name: str = "MANGO") -> Path:
    try:
        from PIL import Image, ImageDraw
    except ImportError as exc:
        raise RuntimeError("플랜 카드 생성에는 Pillow가 필요합니다.") from exc

    width, height = 1200, 760
    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image)
    font = _card_font(20)
    title_font = _card_font(24)
    tag_id = int(tag["id"])
    role = str(tag.get("role", "marker"))

    draw.rectangle((20, 20, width - 20, height - 20), outline="black", width=6)
    # 제목은 마커 왼쪽에만 배치한다. 긴 시나리오 이름도 마커 위로 침범하지 않는다.
    display_name = re.sub(r"spider|스파이더", "MANGO", scenario_name, flags=re.IGNORECASE)
    title = f"MANGO PLAN CARD / {display_name}"
    title_lines = _text_lines(draw, title, title_font, 540)
    if len(title_lines) > 2:
        last_line = title_lines[1]
        while draw.textlength(last_line + "…", font=title_font) > 540:
            last_line = last_line[:-1]
        title_lines = [title_lines[0], last_line + "…"]
    for index, line in enumerate(title_lines):
        draw.text((60, 42 + index * 32), line, fill="black", font=title_font)
    draw.text((60, 115), f"TAG {tag_id}   ROLE: {role.upper()}", fill="black", font=font)
    draw.text((60, 155), f"POSITION: X={float(tag['x']):.2f} m  Y={float(tag['y']):.2f} m", fill="black", font=font)

    cell, left, top = 58, 660, 95
    official = False
    if 0 <= tag_id < 587:
        try:
            url = (
                "https://raw.githubusercontent.com/AprilRobotics/apriltag-imgs/"
                f"master/tag36h11/tag36_11_{tag_id:05d}.png"
            )
            with urllib.request.urlopen(url, timeout=1.5) as response:
                marker = Image.open(io.BytesIO(response.read())).convert("1")
            marker = marker.resize((8 * cell, 8 * cell), Image.Resampling.NEAREST).convert("RGB")
            image.paste(marker, (left, top))
            official = True
        except Exception:
            pass
    if not official:
        bits = _marker_bits(tag_id)
        draw.rectangle((left - 20, top - 20, left + 8 * cell + 20, top + 8 * cell + 20), fill="white", outline="black", width=4)
        for y, row in enumerate(bits):
            for x, bit in enumerate(row):
                if bit:
                    draw.rectangle(
                        (left + x * cell, top + y * cell, left + (x + 1) * cell - 1, top + (y + 1) * cell - 1),
                        fill="black",
                    )

    notes = [
        "설치: 카드 중심을 지정 좌표에 정렬",
        "방향: 상단 MANGO 문구가 북쪽",
        "MARKER: tag36h11 공식 코드" if official else "MARKER: 오프라인 대체 식별자 (비행 전 공식 카드로 교체)",
        f"CHECK: MANGO-{tag_id:04d}-{hashlib.sha1(str(tag_id).encode()).hexdigest()[:8].upper()}",
    ]
    note_y = 260
    for line in notes:
        note_y = _draw_text_block(draw, line, (60, note_y), font, 540) + 22
    output.parent.mkdir(parents=True, exist_ok=True)
    image.save(output, "PNG", optimize=True)
    return output


def create_cards(tags: list[dict[str, Any]], output_dir: Path, scenario_name: str) -> list[Path]:
    return [
        create_plan_card(tag, output_dir / f"tag_{int(tag['id'])}.png", scenario_name)
        for tag in sorted(tags, key=lambda item: int(item["id"]))
    ]
