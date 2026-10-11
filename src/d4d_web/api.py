"""SPIDER 지휘소 FastAPI 애플리케이션."""
from __future__ import annotations

import asyncio
from contextlib import asynccontextmanager
import json
import uuid
from pathlib import Path
from typing import Any, AsyncIterator

from fastapi import Body, FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, ConfigDict, Field

from d4d_planner.agent import AgentConfig, run_agent
from d4d_planner.analysis import analyze
from d4d_planner.export import mission_dict, write_all
from d4d_planner.grid import Grid
from d4d_planner.model import PlanParams
from d4d_planner.scenario import ScenarioError, load_scenario, parse_scenario

from .cards import create_cards, create_plan_card
from .drill import build_timeline
from .hardware import HardwareError, HardwareService
from .board_api import router as board_router, board_wifi
from .library import (
    ARTIFACTS_DIR,
    ROOT,
    SCENARIOS_DIR,
    LibraryError,
    approve_run,
    create_run,
    delete_run,
    list_runs,
    package_run,
    read_metadata,
    refresh_status,
    run_dir,
    safe_id,
    scenario_hash,
    write_metadata,
)
from .llm import briefing_with_llm, patch_with_llm


class ScenarioCreate(BaseModel):
    id: str
    scenario: dict[str, Any]
    overwrite: bool = False


class ScenarioInput(BaseModel):
    scenario_id: str | None = None
    scenario: dict[str, Any] | None = None
    params: dict[str, Any] = Field(default_factory=dict)


class PlanInput(ScenarioInput):
    model_config = ConfigDict(extra="forbid")
    run_id: str | None = None
    config: dict[str, Any] = Field(default_factory=dict)


class ApprovalInput(BaseModel):
    approved_by: str = "operator"


class LLMInput(ScenarioInput):
    instruction: str = ""
    provider: str | None = None


class OverrideChallengeInput(BaseModel):
    model_config = ConfigDict(extra="forbid")
    target: int
    confirmation: str


class OverrideApplyInput(BaseModel):
    model_config = ConfigDict(extra="forbid")
    target: int
    token: str


@asynccontextmanager
async def lifespan(_: FastAPI):
    yield
    await asyncio.to_thread(board_wifi.disconnect)


app = FastAPI(title="MANGO Command Post API", version="1.0.0", lifespan=lifespan)
app.include_router(board_router)
hardware_service = HardwareService()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.exception_handler(LibraryError)
async def library_error_handler(_: Request, exc: LibraryError) -> JSONResponse:
    return JSONResponse(status_code=400, content={"detail": str(exc)})


@app.exception_handler(HardwareError)
async def hardware_error_handler(_: Request, exc: HardwareError) -> JSONResponse:
    return JSONResponse(
        status_code=exc.status_code,
        content={"detail": exc.detail()},
    )


def _error(exc: Exception, status: int = 400) -> HTTPException:
    return HTTPException(status_code=status, detail=str(exc))


def _scenario_path(scenario_id: str) -> Path:
    return SCENARIOS_DIR / f"{safe_id(scenario_id, 'scenario_id')}.json"


def _read_scenario(scenario_id: str) -> dict[str, Any]:
    path = _scenario_path(scenario_id)
    if not path.is_file():
        raise HTTPException(status_code=404, detail=f"시나리오를 찾을 수 없습니다: {scenario_id}")
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        load_scenario(path)
        return data
    except (json.JSONDecodeError, ScenarioError) as exc:
        raise _error(exc, 422)


def _resolve_scenario(payload: ScenarioInput) -> tuple[str | None, dict[str, Any]]:
    if payload.scenario is not None:
        return payload.scenario_id, payload.scenario
    if payload.scenario_id:
        return payload.scenario_id, _read_scenario(payload.scenario_id)
    raise HTTPException(status_code=422, detail="scenario 또는 scenario_id가 필요합니다.")


def _params(values: dict[str, Any]) -> PlanParams:
    try:
        return PlanParams(**values)
    except TypeError as exc:
        raise _error(exc, 422)


def _validate(data: dict[str, Any]) -> None:
    try:
        parse_scenario(data)
    except (ScenarioError, TypeError, ValueError) as exc:
        raise _error(exc, 422)


def _sse(event: str, data: Any) -> str:
    return f"event: {event}\ndata: {json.dumps(data, ensure_ascii=False, default=str)}\n\n"


@app.get("/api/health")
def health() -> dict[str, Any]:
    return {"status": "ok", "root": str(ROOT), "artifacts": str(ARTIFACTS_DIR)}


@app.get("/api/hardware/profile")
def hardware_profile() -> dict[str, Any]:
    return hardware_service.profile()


@app.get("/api/hardware/fc/status")
def hardware_fc_status() -> dict[str, Any]:
    return hardware_service.status()


@app.post("/api/hardware/fc/override/challenge")
def hardware_override_challenge(
    payload: OverrideChallengeInput,
) -> dict[str, Any]:
    return hardware_service.challenge(payload.target, payload.confirmation)


@app.post("/api/hardware/fc/override/apply")
def hardware_override_apply(payload: OverrideApplyInput) -> dict[str, Any]:
    return hardware_service.apply(payload.target, payload.token)


@app.get("/api/scenarios")
def scenarios_list() -> list[dict[str, Any]]:
    SCENARIOS_DIR.mkdir(parents=True, exist_ok=True)
    result = []
    for path in sorted(SCENARIOS_DIR.glob("*.json")):
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            result.append(
                {
                    "id": path.stem,
                    "name": path.stem,
                    "title": data.get("name", path.stem),
                    "hash": scenario_hash(data),
                }
            )
        except (OSError, json.JSONDecodeError) as exc:
            result.append({"id": path.stem, "error": str(exc)})
    return result


@app.get("/api/scenarios/{scenario_id}")
def scenario_get(scenario_id: str) -> dict[str, Any]:
    return _read_scenario(scenario_id)


@app.post("/api/scenarios", status_code=201)
def scenario_create(payload: ScenarioCreate) -> dict[str, Any]:
    try:
        path = _scenario_path(payload.id)
    except LibraryError as exc:
        raise _error(exc, 422)
    if path.exists() and not payload.overwrite:
        raise HTTPException(status_code=409, detail="같은 ID의 시나리오가 이미 있습니다.")
    _validate(payload.scenario)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload.scenario, ensure_ascii=False, indent=2), encoding="utf-8")
    return {"id": payload.id, "hash": scenario_hash(payload.scenario)}


@app.put("/api/scenarios/{scenario_id}")
def scenario_update(scenario_id: str, scenario: dict[str, Any] = Body(...)) -> dict[str, Any]:
    path = _scenario_path(scenario_id)
    if not path.exists():
        raise HTTPException(status_code=404, detail="시나리오를 찾을 수 없습니다.")
    _validate(scenario)
    path.write_text(json.dumps(scenario, ensure_ascii=False, indent=2), encoding="utf-8")
    return {"id": scenario_id, "hash": scenario_hash(scenario)}


@app.delete("/api/scenarios/{scenario_id}")
def scenario_delete(scenario_id: str) -> dict[str, bool]:
    path = _scenario_path(scenario_id)
    if not path.exists():
        raise HTTPException(status_code=404, detail="시나리오를 찾을 수 없습니다.")
    path.unlink()
    return {"deleted": True}


@app.post("/api/scenarios/validate")
def scenario_validate(payload: ScenarioInput) -> dict[str, Any]:
    scenario_id, data = _resolve_scenario(payload)
    _validate(data)
    return {"valid": True, "scenario_id": scenario_id, "hash": scenario_hash(data)}


@app.post("/api/scenarios/analyze")
def scenario_analyze(payload: ScenarioInput) -> dict[str, Any]:
    scenario_id, data = _resolve_scenario(payload)
    try:
        scenario = parse_scenario(data)
        params = _params(payload.params)
        result = analyze(scenario, Grid(scenario, params), params)
    except (ScenarioError, ValueError) as exc:
        raise _error(exc, 422)
    return {"scenario_id": scenario_id, "hash": scenario_hash(data), "analysis": result.to_dict()}


@app.post("/api/plan/stream")
async def plan_stream(payload: PlanInput) -> StreamingResponse:
    scenario_id, raw = _resolve_scenario(payload)
    _validate(raw)
    run_id = safe_id(payload.run_id or uuid.uuid4().hex, "run_id")

    async def generate() -> AsyncIterator[str]:
        yield _sse("start", {"run_id": run_id, "scenario_id": scenario_id})
        try:
            out, meta = create_run(run_id, scenario_id, raw)
            plan_card_id = 500 + (len(list_runs()) - 1) % 87
            meta["plan_card_id"] = plan_card_id
            scenario = parse_scenario(raw)
            params = _params(payload.params)
            config = AgentConfig(**payload.config)
            loop = asyncio.get_running_loop()
            steps: asyncio.Queue[Any] = asyncio.Queue()

            def on_step(step: Any) -> None:
                loop.call_soon_threadsafe(steps.put_nowait, step)

            planning = asyncio.create_task(asyncio.to_thread(run_agent, scenario, params, config, on_step))
            index = 0
            while not planning.done() or not steps.empty():
                try:
                    step = await asyncio.wait_for(steps.get(), timeout=0.1)
                except TimeoutError:
                    continue
                yield _sse(
                    "trace",
                    {"index": index, "it": step.it, "tool": step.tool, "summary": step.summary, "data": step.data},
                )
                index += 1
            result = await planning
            files = await asyncio.to_thread(write_all, result, out, plan_card_id)
            card = await asyncio.to_thread(
                create_plan_card,
                {"id": plan_card_id, "role": "mission", "x": scenario.start[0], "y": scenario.start[1]},
                out / "plan_card.png",
                scenario.name,
            )
            files.append(card)
            mission_path = out / "mission.json"
            if mission_path.is_file():
                timeline_path = out / "drill_timeline.json"
                timeline_path.write_text(
                    json.dumps(
                        build_timeline(json.loads(mission_path.read_text(encoding="utf-8"))),
                        ensure_ascii=False,
                        indent=2,
                    ),
                    encoding="utf-8",
                )
                files.append(timeline_path)
            meta.update(
                success=result.success,
                status="draft" if result.success else "failed",
                issues=result.issues,
                files=sorted({p.relative_to(out).as_posix() for p in files} | {"scenario.json"}),
            )
            write_metadata(run_id, meta)
            mission = mission_dict(result, plan_card_id) if result.plan and result.plan.ok else None
            yield _sse(
                "complete",
                {"run_id": run_id, "success": result.success, "issues": result.issues, "metadata": meta, "mission": mission},
            )
        except Exception as exc:
            try:
                meta = read_metadata(run_id)
                meta.update(success=False, error=str(exc))
                write_metadata(run_id, meta)
            except Exception:
                pass
            yield _sse("error", {"run_id": run_id, "detail": str(exc)})

    return StreamingResponse(
        generate(),
        media_type="text/event-stream",
        headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
    )


@app.get("/api/library")
def library_list() -> list[dict[str, Any]]:
    rows = []
    for item in list_runs():
        meta = refresh_status(item["run_id"])
        mission_path = run_dir(meta["run_id"]) / "mission.json"
        success_rate = None
        if mission_path.is_file():
            trace_path = run_dir(meta["run_id"]) / "agent_trace.json"
            if trace_path.is_file():
                trace = json.loads(trace_path.read_text(encoding="utf-8"))
                verifies = [step for step in trace if step.get("tool") == "verify"]
                if verifies:
                    summary = verifies[-1].get("summary", "")
                    try:
                        success_rate = float(summary.split("성공률 ", 1)[1].split("%", 1)[0])
                    except (IndexError, ValueError):
                        pass
        rows.append(
            {
                **meta,
                "id": meta["run_id"],
                "name": meta.get("scenario_id") or "임시 시나리오",
                "success_rate": success_rate,
            }
        )
    return rows


@app.get("/api/library/{run_id}")
def library_get(run_id: str) -> dict[str, Any]:
    try:
        return refresh_status(run_id)
    except LibraryError as exc:
        raise _error(exc, 404)


@app.delete("/api/library/{run_id}")
def library_delete(run_id: str) -> dict[str, bool]:
    try:
        delete_run(run_id)
    except LibraryError as exc:
        raise _error(exc, 404)
    return {"deleted": True}


@app.post("/api/library/{run_id}/approve")
def approve(run_id: str, payload: ApprovalInput) -> dict[str, Any]:
    try:
        return approve_run(run_id, payload.approved_by)
    except LibraryError as exc:
        raise _error(exc, 409)


@app.post("/api/library/{run_id}/package")
def package(run_id: str) -> dict[str, Any]:
    try:
        path = package_run(run_id)
        return {"run_id": run_id, "file": path.name, "download": f"/api/library/{run_id}/package"}
    except LibraryError as exc:
        raise _error(exc, 409)


@app.get("/api/library/{run_id}/package")
def package_download(run_id: str) -> FileResponse:
    try:
        meta = refresh_status(run_id)
        path = run_dir(run_id) / str(meta.get("package", f"{run_id}.zip"))
        if meta.get("status") != "approved" or not path.is_file():
            raise LibraryError("다운로드 가능한 승인 패키지가 없습니다.")
        return FileResponse(path, media_type="application/zip", filename=path.name)
    except LibraryError as exc:
        raise _error(exc, 409)


@app.post("/api/library/{run_id}/cards")
def cards_generate(run_id: str) -> dict[str, Any]:
    try:
        out = run_dir(run_id)
        raw = json.loads((out / "scenario.json").read_text(encoding="utf-8"))
        paths = create_cards(raw.get("tags", []), out / "cards", raw.get("name", "MANGO"))
        meta = read_metadata(run_id)
        meta["files"] = sorted({*meta.get("files", []), *(p.relative_to(out).as_posix() for p in paths)})
        write_metadata(run_id, meta)
        return {"run_id": run_id, "cards": [p.name for p in paths]}
    except (LibraryError, FileNotFoundError, RuntimeError) as exc:
        raise _error(exc, 400)


@app.get("/api/library/{run_id}/cards/{filename}")
def card_download(run_id: str, filename: str) -> FileResponse:
    safe_id(filename.removesuffix(".png"), "card")
    path = run_dir(run_id) / "cards" / filename
    if path.suffix.lower() != ".png" or not path.is_file():
        raise HTTPException(status_code=404, detail="카드를 찾을 수 없습니다.")
    return FileResponse(path, media_type="image/png", filename=path.name)


@app.post("/api/library/{run_id}/drill")
def drill_generate(run_id: str) -> dict[str, Any]:
    out = run_dir(run_id)
    mission_path = out / "mission.json"
    if not mission_path.is_file():
        raise HTTPException(status_code=409, detail="유효한 mission.json이 필요합니다.")
    timeline = build_timeline(json.loads(mission_path.read_text(encoding="utf-8")))
    path = out / "drill_timeline.json"
    path.write_text(json.dumps(timeline, ensure_ascii=False, indent=2), encoding="utf-8")
    meta = read_metadata(run_id)
    meta["files"] = sorted({*meta.get("files", []), path.name})
    write_metadata(run_id, meta)
    return timeline


@app.post("/api/llm/patch")
def llm_patch(payload: LLMInput) -> dict[str, Any]:
    _, raw = _resolve_scenario(payload)
    return patch_with_llm(payload.instruction, raw, payload.provider)


@app.post("/api/llm/briefing")
def llm_briefing(payload: LLMInput) -> dict[str, Any]:
    _, raw = _resolve_scenario(payload)
    analysis_data = None
    try:
        scenario = parse_scenario(raw)
        params = _params(payload.params)
        analysis_data = analyze(scenario, Grid(scenario, params), params).to_dict()
    except Exception:
        pass
    return briefing_with_llm(raw, analysis_data, payload.provider)


# 짧은 별칭은 해커톤 UI와 외부 클라이언트가 쓰는 안정 API다.
@app.post("/api/validate")
def validate_alias(scenario: dict[str, Any] = Body(...)) -> dict[str, Any]:
    _validate(scenario)
    return {"valid": True, "status": "통과", "hash": scenario_hash(scenario)}


@app.post("/api/analyze")
def analyze_alias(scenario: dict[str, Any] = Body(...)) -> dict[str, Any]:
    parsed = parse_scenario(scenario)
    params = PlanParams()
    result = analyze(parsed, Grid(parsed, params), params).to_dict()
    return {"analysis": result, "summary": f"지도상 통과 영역 {result['free_frac']:.0%}, 재밍 {result['jam_frac']:.0%}"}


@app.post("/api/agent/patch")
def patch_alias(payload: LLMInput) -> dict[str, Any]:
    return llm_patch(payload)


@app.post("/api/runs/{run_id}/approve")
def approve_alias(run_id: str, payload: ApprovalInput = ApprovalInput()) -> dict[str, Any]:
    return approve(run_id, payload)


def _require_approved(run_id: str) -> dict[str, Any]:
    meta = refresh_status(run_id)
    if meta.get("status") != "approved":
        raise LibraryError("승인된 최신 계획만 열 수 있습니다.")
    return meta


@app.get("/api/runs/{run_id}/package")
def package_alias(run_id: str) -> FileResponse:
    try:
        meta = _require_approved(run_id)
        package_path = run_dir(run_id) / str(meta.get("package", f"{run_id}.zip"))
        if not package_path.is_file():
            package_path = package_run(run_id)
        return FileResponse(package_path, media_type="application/zip", filename=package_path.name)
    except LibraryError as exc:
        raise _error(exc, 409)


@app.get("/api/cards/{run_id}.png")
def plan_card_alias(run_id: str) -> FileResponse:
    try:
        _require_approved(run_id)
    except LibraryError as exc:
        raise _error(exc, 409)
    path = run_dir(run_id) / "plan_card.png"
    if not path.is_file():
        raise HTTPException(status_code=404, detail="플랜 카드를 찾을 수 없습니다.")
    return FileResponse(path, media_type="image/png", filename=f"{run_id}_plan_card.png")


@app.get("/api/drill/{run_id}")
def drill_alias(run_id: str) -> dict[str, Any]:
    try:
        _require_approved(run_id)
    except LibraryError as exc:
        raise _error(exc, 409)
    path = run_dir(run_id) / "drill_timeline.json"
    if path.is_file():
        return json.loads(path.read_text(encoding="utf-8"))
    return drill_generate(run_id)


_web_dist = ROOT / "web" / "dist"
if _web_dist.is_dir():
    app.mount("/", StaticFiles(directory=_web_dist, html=True), name="frontend")
