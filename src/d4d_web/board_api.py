"""브라우저 ↔ 로컬 백엔드 ↔ TagBoard TCP. 소켓은 서버가 한 개만 소유한다."""
import asyncio
import json
from urllib.parse import urlparse

from fastapi import APIRouter, Depends, Request
from fastapi.responses import Response, StreamingResponse
from pydantic import BaseModel, ConfigDict

from .board_wifi import BoardWifiService
from .hardware import HardwareError

board_wifi = BoardWifiService()


def local_request(request: Request) -> None:
    origin = request.headers.get("origin")
    if request.headers.get("sec-fetch-site") == "cross-site" or (origin and urlparse(origin).hostname not in {"localhost", "127.0.0.1", "::1"}):
        raise HardwareError("local_only", "실기체 연결은 로컬 관제 화면에서만 사용할 수 있습니다.", 403)
    if request.client and request.client.host not in {"127.0.0.1", "::1", "localhost", "testclient"}:
        raise HardwareError("local_only", "실기체 연결은 로컬 백엔드에서만 사용할 수 있습니다.", 403)


router = APIRouter(prefix="/api/hardware/wifi", dependencies=[Depends(local_request)])


class ConnectInput(BaseModel):
    model_config = ConfigDict(extra="forbid")
    host: str


@router.get("/status")
def status():
    return board_wifi.snapshot()


@router.post("/connect")
def connect(payload: ConnectInput):
    return board_wifi.connect(payload.host)


@router.post("/disconnect")
def disconnect():
    return board_wifi.disconnect()


@router.get("/events")
async def events(request: Request):
    async def stream():
        while not await request.is_disconnected():
            yield "data: " + json.dumps(board_wifi.snapshot(), ensure_ascii=False, allow_nan=False) + "\n\n"
            await asyncio.sleep(0.1)

    return StreamingResponse(stream(), media_type="text/event-stream", headers={"Cache-Control": "no-store", "X-Accel-Buffering": "no"})


@router.get("/camera.jpg")
def camera():
    image = board_wifi.camera_frame()
    if image is None:
        return Response(status_code=503, headers={"Cache-Control": "no-store"})
    return Response(image, media_type="image/jpeg", headers={"Cache-Control": "no-store", "X-Content-Type-Options": "nosniff"})
