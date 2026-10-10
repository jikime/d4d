import {FormSelect,FormSlider} from "./components/form-controls";
import {Input} from "./components/ui/input";
import {Textarea} from "./components/ui/textarea";
import {Field,FieldLabel,FieldDescription} from "./components/ui/field";
import {Button} from "./components/ui/button";
import {
  Activity, AlertTriangle, ArrowUpRight, Bot, Box, Check, ChevronRight, Compass,
  CircleDot, Crosshair, Download, FileJson, Gauge, Library, Lock,
  PackageCheck, Pause, Play, RefreshCw, Route, Send, ShieldCheck,
  Sparkles, Terminal, Upload, WifiOff, Zap, Layers, Plus, Minus, Maximize, MousePointer2, Pencil, Eye, X,
} from "lucide-react";
import { FormEvent, KeyboardEvent, type MouseEvent, type PointerEvent, ReactNode, useCallback, useEffect, useMemo, useRef, useState, lazy, Suspense } from "react";

import HardwareControlBay from "./HardwareControlBay";
import { buildPlanRequest } from "./scenarioRequest";
import "./command.css";
import trainingCamp from "../../scenarios/demo_training_camp.json";
import type {MissionWaypoint} from "./simulation/missionWorld";
const MissionControl=lazy(()=>import("./MissionControl"));
const LiveAircraft=lazy(()=>import("./LiveAircraft"));

type Point = { x: number; y: number };
type Tag = Point & { id: string | number; name?: string; role?: "marker" | "ble" | "mailbox"; payload_bytes?: number; suggested?: boolean };
type Zone = {
  kind: "obstacle" | "keepout" | "jam" | "low_flow";
  shape: "rect" | "circle" | "polygon";
  name?: string;
  x?: number; y?: number; w?: number; h?: number; r?: number;
  points?: [number, number][]; value?: number;
};
type Objective = Point & { name: string; action?: string; dwell_s?: number };
type Message = { name: string; from: string | number; to: string | number; bytes: number; priority: number };
type Scenario = {
  name: string;
  link_policy: "abort" | "continue_mission";
  field: { width_m: number; height_m: number; resolution_m?: number };
  start: Point;
  goal: Point & { name?: string; tolerance_m?: number };
  objectives: Objective[];
  tags: Tag[];
  messages?: Message[];
  zones: Zone[];
  landing_sites?: Point[];
  [key: string]: unknown;
};
type Selected = { type: "start" | "goal" | "tag" | "objective"; index?: number } | null;
type LogLine = { time: string; type: "info" | "ok" | "warn"; text: string };
type LibraryItem = { id?: string; name: string; status?: string; success?: boolean | null; success_rate?: number; updated_at?: string };
type Metrics = { success: number | null; delivery: number | null; length: number | null; time: number | null };
type DrillEvent = { label?: string; t_s?: number; kind?: string };
type DrillSample = { id?: number; label: string; points?: number[][] };
type DrillTimeline = { events?: DrillEvent[]; samples?: DrillSample[] };
type PlanPhase = "idle" | "planning" | "success" | "failed";
type DrillStatus = { live: boolean; event: string; demo: boolean };
type CoreLink = "pending" | "online" | "offline";

const ORTHO_SRC = "/terrain/spider-ortho-v2.png";
const ORTHO_FALLBACK = "/terrain/spider-ortho.jpg";

const DEMO_EVENTS: DrillEvent[] = [
  { label: "카나리아 하트비트 정상" },
  { label: "LINK LOST · 하트비트 중단" },
  { label: "정숙 시간 확인" },
  { label: "둥지 개방 · 플랜 카드 판독" },
  { label: "교신" },
  { label: "귀환" },
];

const sample: Scenario = trainingCamp as Scenario;

const now = () => new Date().toLocaleTimeString("ko-KR", { hour12: false });

async function jsonApi<T>(url: string, init?: RequestInit): Promise<T> {
  const response = await fetch(url, {
    ...init,
    headers: { "Content-Type": "application/json", ...init?.headers },
  });
  if (!response.ok) {
    const payload = await response.json().catch(() => null) as { detail?: string | { message?: string } } | null;
    const detail = typeof payload?.detail === "string" ? payload.detail : payload?.detail?.message;
    throw new Error(detail || `${response.status} ${response.statusText}`);
  }
  return response.json() as Promise<T>;
}

async function parseSse(
  response: Response,
  onEvent: (event: string, data: unknown) => void,
  signal: AbortSignal,
) {
  if (!response.ok || !response.body) throw new Error(`스트림 연결 실패 (${response.status})`);
  const reader = response.body.getReader();
  const decoder = new TextDecoder();
  let buffer = "";
  while (!signal.aborted) {
    const { done, value } = await reader.read();
    if (done) break;
    buffer += decoder.decode(value, { stream: true }).replace(/\r\n/g, "\n");
    const frames = buffer.split("\n\n");
    buffer = frames.pop() ?? "";
    for (const frame of frames) {
      let event = "message";
      const chunks: string[] = [];
      for (const line of frame.split("\n")) {
        if (line.startsWith("event:")) event = line.slice(6).trim();
        if (line.startsWith("data:")) chunks.push(line.slice(5).trimStart());
      }
      if (!chunks.length) continue;
      const raw = chunks.join("\n");
      try { onEvent(event, JSON.parse(raw)); } catch { onEvent(event, raw); }
    }
  }
}

function roleLabel(role?: Tag["role"]) {
  if (role === "ble") return "BLE";
  if (role === "mailbox") return "광학";
  if (role === "marker") return "마커";
  return "";
}

function jamText(zone: Zone) {
  if (typeof zone.value !== "number") return zone.name ?? "재밍";
  if (zone.value >= 0 && zone.value <= 1) return `${zone.name ?? "재밍"} ${(zone.value * 100).toFixed(0)}%`;
  return `${zone.name ?? "재밍"} ${zone.value}`;
}

function safetyLines(scenario: Scenario) {
  const ranked = [...scenario.zones].sort((a, b) => {
    const order = { keepout: 0, jam: 1, low_flow: 2, obstacle: 3 };
    return order[a.kind] - order[b.kind];
  });
  const lines = ranked.slice(0, 4).map((zone) => {
    if (zone.kind === "keepout") return { tone: "red" as const, text: zone.name ?? "접근 금지" };
    if (zone.kind === "jam") return { tone: "amber" as const, text: jamText(zone) };
    if (zone.kind === "low_flow") return { tone: "muted" as const, text: zone.name ?? (typeof zone.value === "number" ? `저광류 ${zone.value}` : "저광류") };
    return { tone: "muted" as const, text: zone.name ?? "장애물" };
  });
  if (scenario.link_policy === "abort") lines.unshift({ tone: "red", text: "링크 두절 시 임무 중단" });
  return lines.slice(0, 4);
}

function Pill({ tone = "cyan", children }: { tone?: "cyan" | "amber" | "red" | "muted"; children: ReactNode }) {
  return <span className={`pill ${tone}`}><i />{children}</span>;
}

function PanelTitle({ icon: Icon, title, aside }: { icon: typeof Activity; title: string; aside?: ReactNode }) {
  return (
    <div className="panel-title">
      <div><Icon size={14} aria-hidden="true" /><span>{title}</span></div>
      {aside}
    </div>
  );
}

function UtcClock() {
  const [text, setText] = useState(now);
  useEffect(() => {
    const timer = window.setInterval(() => setText(now()), 1000);
    return () => window.clearInterval(timer);
  }, []);
  return <time className="utc">{text}</time>;
}

function onItemKey(event: KeyboardEvent, action: () => void) {
  if (event.key === "Enter" || event.key === " ") {
    event.preventDefault();
    event.stopPropagation();
    action();
  }
}

function MapView({
  scenario, selected, onSelect, onAddTag, linkLost, eventLabel, demoEvent, demoData, trace,
  editing, onEditing,
  plannedRoute,
}: {
  scenario: Scenario; selected: Selected;
  onSelect: (next: Selected) => void; onAddTag: (point: Point) => void;
  linkLost: boolean; eventLabel: string; demoEvent: boolean; demoData: boolean; trace: string;
  editing: boolean; onEditing: (value: boolean) => void;
  plannedRoute: Point[];
}) {
  const [ortho, setOrtho] = useState<"loading" | "ready" | "fallback">("loading");
  const [imageSource, setImageSource] = useState(ORTHO_SRC);
  const [photo, setPhoto] = useState(!scenario.name.includes("청솔"));
  const [legend, setLegend] = useState(false);
  const [layers, setLayers] = useState({ zones: true, comms: false, route: true });
  const [addMode, setAddMode] = useState(false);
  const [zoom, setZoom] = useState(1);
  const [pan, setPan] = useState({ x: 0, y: 0 });
  const [viewport, setViewport] = useState({ width: 0, height: 0 });
  const shell = useRef<HTMLDivElement>(null);
  const dragging = useRef<{ x: number; y: number; panX: number; panY: number; moved: boolean } | null>(null);
  const suppressClick = useRef(false);
  const { width_m: width, height_m: height, resolution_m: resolution } = scenario.field;
  const scaleX = 100 / width;
  const scaleY = scaleX;
  const mapHeight = height * scaleX;
  const fittedWidth = Math.min(viewport.width, viewport.height * width / height);
  const fittedHeight = fittedWidth * height / width;
  const p = (point: Point) => ({ x: point.x * scaleX, y: mapHeight - point.y * scaleY });
  const sharedEndpoint = Math.hypot(scenario.start.x - scenario.goal.x, scenario.start.y - scenario.goal.y) < .001;
  // 핀은 실제 좌표에 고정하고 라벨만 빈 방향으로 배치한다.
  type LabelBox = { left: number; top: number; right: number; bottom: number };
  const occupiedLabels: LabelBox[] = [...scenario.tags, ...scenario.objectives, scenario.start, scenario.goal].map((point) => {
    const xy = p(point);
    return { left: xy.x - 2.2, top: xy.y - 2.2, right: xy.x + 2.2, bottom: xy.y + 2.2 };
  });
  const textWidth = (text: string, size: number) => [...text].reduce((total, char) => total + (/[^\u0000-\u007f]/.test(char) ? size : size * .62), 0);
  const tagLabels = scenario.tags.map((tag) => {
    const xy = p(tag);
    const title = tag.name ? `${tag.id} · ${tag.name}` : `노드 ${tag.id}`;
    const subtitle = `${tag.suggested ? "제안 · " : ""}${roleLabel(tag.role)}`;
    const labelWidth = Math.max(textWidth(title, 1.85), textWidth(subtitle, 1.4)) + .8;
    const directions = xy.x > 76 ? [-1, 1] : [1, -1];
    const candidates = [0, -5.5, 5.5, -10.5, 10.5].flatMap((dy) => directions.map((direction) => {
      const dx = direction * 2.8;
      const left = xy.x + dx - (direction < 0 ? labelWidth : 0);
      const box = { left, top: xy.y + dy - 2.5, right: left + labelWidth, bottom: xy.y + dy + 2.2 };
      const overlap = occupiedLabels.reduce((total, item) => total + Math.max(0, Math.min(box.right, item.right) - Math.max(box.left, item.left)) * Math.max(0, Math.min(box.bottom, item.bottom) - Math.max(box.top, item.top)), 0);
      const outside = Math.max(0, 1 - box.left) + Math.max(0, box.right - 99) + Math.max(0, 1 - box.top) + Math.max(0, box.bottom - mapHeight + 1);
      return { dx, dy, anchor: direction < 0 ? "end" as const : "start" as const, box, score: overlap * 100 + outside * 1000 + Math.abs(dy) + (direction !== directions[0] ? 1 : 0) };
    }));
    const chosen = candidates.reduce((best, candidate) => candidate.score < best.score ? candidate : best);
    occupiedLabels.push(chosen.box);
    return { ...chosen, title, subtitle };
  });
  const hasPlanRoute = plannedRoute.length > 1;
  const route = (hasPlanRoute ? plannedRoute : [scenario.start, ...scenario.objectives, scenario.goal]).map((v) => `${p(v).x},${p(v).y}`).join(" ");
  const endpoint = (value: string | number): Point | undefined => value === "start" ? scenario.start
    : value === "goal" ? scenario.goal : scenario.tags.find((tag) => String(tag.id) === String(value));
  const safePan = (x: number, y: number, nextZoom = zoom) => {
    const maxX = Math.max(0, (fittedWidth * nextZoom - viewport.width) / 2);
    const maxY = Math.max(0, (fittedHeight * nextZoom - viewport.height) / 2);
    return { x: Math.max(-maxX, Math.min(maxX, x)), y: Math.max(-maxY, Math.min(maxY, y)) };
  };
  useEffect(() => {
    const element = shell.current;
    if (!element) return;
    const measure = () => setViewport({ width: element.clientWidth, height: element.clientHeight });
    measure();
    const observer = new ResizeObserver(measure);
    observer.observe(element);
    return () => observer.disconnect();
  }, []);
  useEffect(() => {
    const maxX = Math.max(0, (fittedWidth * zoom - viewport.width) / 2);
    const maxY = Math.max(0, (fittedHeight * zoom - viewport.height) / 2);
    setPan((point) => ({ x: Math.max(-maxX, Math.min(maxX, point.x)), y: Math.max(-maxY, Math.min(maxY, point.y)) }));
  }, [fittedWidth, fittedHeight, viewport.width, viewport.height, zoom]);
  const changeZoom = (next: number) => {
    const value = Math.max(1, Math.min(4, next));
    setZoom(value); setPan((point) => safePan(point.x, point.y, value));
  };
  useEffect(() => { setPan({ x: 0, y: 0 }); setZoom(1); setAddMode(false); }, [width, height, scenario.name]);
  useEffect(() => { if (!editing) setAddMode(false); }, [editing]);
  const clickMap = (event: MouseEvent<SVGSVGElement>) => {
    if (suppressClick.current) { suppressClick.current = false; return; }
    if ((event.target as Element).closest("[data-map-item]")) return;
    if (!editing || !addMode) { onSelect(null); return; }
    const matrix = event.currentTarget.getScreenCTM();
    if (!matrix) return;
    const position = new DOMPoint(event.clientX, event.clientY).matrixTransform(matrix.inverse());
    if (position.x < 0 || position.x > 100 || position.y < 0 || position.y > mapHeight) return;
    onAddTag({ x: +(position.x / scaleX).toFixed(1), y: +((mapHeight - position.y) / scaleY).toFixed(1) });
    setAddMode(false);
  };
  const pointerDown = (event: PointerEvent<SVGSVGElement>) => {
    if (event.button !== 0 || addMode || (event.target as Element).closest("[data-map-item]")) return;
    dragging.current = { x: event.clientX, y: event.clientY, panX: pan.x, panY: pan.y, moved: false };
    event.currentTarget.setPointerCapture(event.pointerId);
  };
  const pointerMove = (event: PointerEvent<SVGSVGElement>) => {
    const start = dragging.current;
    if (!start) return;
    const dx = event.clientX - start.x; const dy = event.clientY - start.y;
    if (Math.hypot(dx, dy) > 4) start.moved = true;
    if (start.moved) setPan(safePan(start.panX + dx, start.panY + dy));
  };
  const pointerUp = (event: PointerEvent<SVGSVGElement>) => {
    suppressClick.current = dragging.current?.moved ?? false;
    dragging.current = null;
    if (event.currentTarget.hasPointerCapture(event.pointerId)) event.currentTarget.releasePointerCapture(event.pointerId);
  };
  const scaleMeters = Math.max(1, Math.round(width / 5 / zoom));
  const chooseItem = (value: Selected) => { onSelect(value); setAddMode(false); };

  return <div ref={shell} className={`map-shell cmd-map ${photo && ortho === "ready" ? "has-ortho" : "is-fallback"} ${addMode ? "is-adding" : ""}`}>
    <div className="cmd-map-world" style={{ width: fittedWidth, height: fittedHeight, left: (viewport.width - fittedWidth) / 2, top: (viewport.height - fittedHeight) / 2, transform: `translate(${pan.x}px, ${pan.y}px) scale(${zoom})` }}>
      {ortho !== "fallback" && <img className={`ortho-plate ${ortho !== "ready" || !photo ? "is-pending" : ""}`} src={imageSource} alt=""
        onLoad={() => setOrtho("ready")} onError={() => {
          if (imageSource !== ORTHO_FALLBACK) setImageSource(ORTHO_FALLBACK); else setOrtho("fallback");
        }} />}
      <div className="cmd-map-tone" aria-hidden="true" />
      <svg className="battle-map" viewBox={`0 0 100 ${mapHeight}`} preserveAspectRatio="xMidYMid meet" role="application"
        aria-label={`${scenario.name} 임무 지도. ${addMode ? "태그를 추가할 위치를 클릭하세요." : "개체를 선택하거나 확대 후 드래그해 이동합니다."}`}
        onClick={clickMap} onPointerDown={pointerDown} onPointerMove={pointerMove} onPointerUp={pointerUp} onPointerCancel={pointerUp}>
        <defs>
          <pattern id="grid" width="5" height="5" patternUnits="userSpaceOnUse"><path d="M 5 0 L 0 0 0 5" fill="none" stroke="currentColor" strokeWidth=".1" /></pattern>
          <pattern id="hatch" width="3" height="3" patternUnits="userSpaceOnUse" patternTransform="rotate(45)"><line x1="0" y1="0" x2="0" y2="3" stroke="currentColor" strokeWidth=".45" /></pattern>
          <filter id="glow"><feGaussianBlur stdDeviation=".2" result="b" /><feMerge><feMergeNode in="b" /><feMergeNode in="SourceGraphic" /></feMerge></filter>
          <marker id="message-arrow" markerWidth="5" markerHeight="5" refX="4" refY="2.5" orient="auto"><path d="M0,0 L5,2.5 L0,5 Z" className="message-arrow" /></marker>
        </defs>
        <rect width="100" height={mapHeight} className="map-grid" />
        {layers.zones && scenario.zones.map((zone, index) => {
          const cls = `zone zone-${zone.kind}`;
          if (zone.shape === "rect") return <rect key={index} className={cls} x={(zone.x ?? 0) * scaleX} y={mapHeight - ((zone.y ?? 0) + (zone.h ?? 0)) * scaleY} width={(zone.w ?? 0) * scaleX} height={(zone.h ?? 0) * scaleY} />;
          if (zone.shape === "circle") return <circle key={index} className={cls} cx={(zone.x ?? 0) * scaleX} cy={mapHeight - (zone.y ?? 0) * scaleY} r={(zone.r ?? 0) * scaleX} />;
          return <polygon key={index} className={cls} points={(zone.points ?? []).map(([x, y]) => `${x * scaleX},${mapHeight - y * scaleY}`).join(" ")} />;
        })}
        {layers.route && <><polyline className="route-under" points={route} /><polyline key={route} className={`route-line ${hasPlanRoute ? "cmd-planned-route" : ""}`} pathLength={100} points={route} /></>}
        {layers.comms && (scenario.messages ?? []).map((message, index) => {
          const from = endpoint(message.from); const to = endpoint(message.to);
          if (!from || !to) return null;
          const a = p(from); const b = p(to);
          return <g key={`${message.name}-${index}`} className="message-flow"><line x1={a.x} y1={a.y} x2={b.x} y2={b.y} markerEnd="url(#message-arrow)" /><text fontSize="1.6" x={(a.x + b.x) / 2} y={(a.y + b.y) / 2 - 1}>{message.name} · {Math.round(message.bytes / 1024 * 10) / 10}KB</text></g>;
        })}
        {scenario.objectives.map((item, index) => {
          const xy = p(item); const choose = () => chooseItem({ type: "objective", index });
          const left = xy.x > 76;
          return <g key={`${item.name}-${index}`} data-map-item="" className={`map-item cmd-objective ${selected?.type === "objective" && selected.index === index ? "selected" : ""}`} transform={`translate(${xy.x} ${xy.y})`} onClick={(event) => { event.stopPropagation(); choose(); }} onKeyDown={(event) => onItemKey(event, choose)} role="button" tabIndex={0} aria-label={`경유지 ${index + 1} ${item.name}`}>
            <rect className="mark" x="-1.8" y="-1.8" width="3.6" height="3.6" rx=".5" />
            <text className="cmd-pin-number" textAnchor="middle" fontSize="1.9" y=".65">{index + 1}</text>
            <text fontSize="1.9" textAnchor={left ? "end" : "start"} x={left ? -3 : 3} y="-1.8">{item.name}</text>
          </g>;
        })}
        {scenario.tags.map((tag, index) => {
          const xy = p(tag); const choose = () => chooseItem({ type: "tag", index });
          const label = tagLabels[index];
          return <g key={`${tag.id}-${index}`} data-map-item="" className={`map-item tag cmd-tag-${tag.role ?? "marker"} ${tag.suggested ? "suggested" : ""} ${selected?.type === "tag" && selected.index === index ? "selected" : ""}`} transform={`translate(${xy.x} ${xy.y})`} onClick={(event) => { event.stopPropagation(); choose(); }} onKeyDown={(event) => onItemKey(event, choose)} role="button" tabIndex={0} aria-label={`태그 ${tag.id} ${tag.name ?? ""} ${roleLabel(tag.role)}${tag.suggested ? " 제안" : ""}`}>
            {label.dy !== 0 && <path className="cmd-label-leader" d={`M0 0 L${label.dx} ${label.dy}`} />}
            <circle className="mark" r="1.65" />
            {tag.role === "ble" ? <path className="cmd-pin-icon" d="M0 -1v2l.8-.6L-.6-.6M0-1l.8.6L-.6.6" /> : tag.role === "mailbox" ? <><rect className="cmd-pin-icon" x="-.8" y="-.6" width="1.6" height="1.2" /><path className="cmd-pin-icon" d="M-.8-.6L0 0l.8-.6" /></> : <><rect className="cmd-pin-icon" x="-.7" y="-.7" width="1.4" height="1.4" /><circle className="core" r=".3" /></>}
            {tag.suggested && <circle className="suggest-ring" r="2.2" />}
            <text fontSize="1.85" textAnchor={label.anchor} x={label.dx} y={label.dy - .5}>{label.title}</text>
            <text className="cmd-pin-sub" fontSize="1.4" textAnchor={label.anchor} x={label.dx} y={label.dy + 1.7}>{label.subtitle}</text>
          </g>;
        })}
        {([["start", scenario.start, "출발 / 둥지"], ["goal", scenario.goal, "귀환 목표"]] as const).map(([type, point, label]) => {
          const xy = p(point); const choose = () => chooseItem({ type }); const left = xy.x > 75;
          return <g key={type} data-map-item="" className={`map-item endpoint ${type} ${selected?.type === type ? "selected" : ""}`} transform={`translate(${xy.x} ${xy.y})`} onClick={(event) => { event.stopPropagation(); choose(); }} onKeyDown={(event) => onItemKey(event, choose)} role="button" tabIndex={0} aria-label={label}>
            {type === "start" ? <path className="mark" d="M -2 1.9 L 0 -2.2 L 2 1.9 Z" /> : <><circle className="mark" r="2" /><path className="cross" d="M-2.6 0h5.2M0-2.6v5.2" /></>}
            {(!sharedEndpoint || type === "goal") && <text fontSize="1.9" textAnchor={left ? "end" : "start"} x={left ? -3 : 3} y="3.1">{sharedEndpoint ? "출발·귀환" : label}</text>}
          </g>;
        })}
      </svg>
    </div>
    <div className="cmd-map-top"><div className="cmd-map-caption"><span><i /> MISSION MAP</span><strong>{scenarioKeyLabel(scenario.name)}</strong><small>합성 지형 · 로컬 좌표 {demoData ? "· 데모 시나리오" : ""}</small><small className="cmd-route-source">{hasPlanRoute ? `생성된 계획 경로 · ${plannedRoute.length}개 경유점` : "임무 연결선 · 계획 전"}</small></div>
      <button type="button" className={`cmd-map-pill ${editing ? "active" : ""}`} aria-pressed={editing} onClick={() => onEditing(!editing)}><Pencil size={15} />{editing ? "편집 중" : "지도 편집"}</button>
    </div>
    <div className="cmd-map-tools" aria-label="지도 도구">
      <button type="button" aria-label="선택 도구" aria-pressed={!addMode} className={!addMode ? "active" : ""} onClick={() => setAddMode(false)}><MousePointer2 size={18} /></button>
      <button type="button" aria-label="태그 추가 도구" title={editing ? "지도에서 추가 위치를 선택" : "지도 편집을 켜면 태그를 추가할 수 있습니다"} disabled={!editing} aria-pressed={addMode} className={addMode ? "active" : ""} onClick={() => setAddMode(!addMode)}><Plus size={18} /></button>
      <span />
      <button type="button" aria-label="지도 확대" disabled={zoom >= 4} onClick={() => changeZoom(zoom + .5)}><Plus size={18} /></button>
      <output aria-label="지도 확대율">{Math.round(zoom * 100)}%</output>
      <button type="button" aria-label="지도 축소" disabled={zoom <= 1} onClick={() => changeZoom(zoom - .5)}><Minus size={18} /></button>
      <button type="button" aria-label="지도 전체 맞춤" onClick={() => { setZoom(1); setPan({ x: 0, y: 0 }); }}><Maximize size={17} /></button>
    </div>
    <div className="cmd-map-layers" aria-label="지도 레이어"><Layers size={15} /><button type="button" aria-pressed={photo} onClick={() => setPhoto(!photo)}>{photo ? "실사 지형" : "도식 지도"}</button>{([['zones', '구역'], ['route', '경로'], ['comms', '교신']] as const).map(([key, name]) => <button type="button" key={key} className={layers[key] ? "active" : ""} aria-pressed={layers[key]} onClick={() => setLayers((v) => ({ ...v, [key]: !v[key] }))}>{name}</button>)}<button type="button" aria-expanded={legend} onClick={() => setLegend(!legend)}><Eye size={14} /> 범례</button></div>
    {legend && <div className="cmd-map-legend"><span><i className="jam" />재밍</span><span><i className="keepout" />금지 구역</span><span><i className="flow" />광류 저하</span><span><i className="obstacle" />장애물</span><span><i className="route" />{hasPlanRoute ? "생성된 계획 경로" : "임무 연결선 · 계획 전"}</span></div>}
    {addMode && <div className="cmd-map-instruction" role="status"><Plus size={16} />지도에서 새 태그 위치를 선택하세요<button type="button" onClick={() => setAddMode(false)}>취소</button></div>}
    {linkLost && <div className="link-lost" role="status"><WifiOff size={16} /><b>LINK LOST</b><span>{demoEvent ? "데모 · " : ""}{eventLabel}</span></div>}
    <div className="cmd-map-bottom"><span className="cmd-map-coordinate">{width.toFixed(0)} × {height.toFixed(0)}m · 간격 {resolution ?? "--"}m</span><span className="cmd-map-scale" style={{ width: scaleMeters / width * fittedWidth * zoom }}><i />{scaleMeters}m</span><span className="cmd-map-north"><Compass size={18} />N</span></div>
    <span className="sr-only">마지막 계획 이벤트: {trace || "아직 없음"}</span>
  </div>;
}

function scenarioKeyLabel(name: string) {
  return name.split("(")[0].trim().replace(/^demo_/, "").replaceAll("_", " ");
}

function PropertyPanel({
  scenario, selected, onChange,
}: { scenario: Scenario; selected: Selected; onChange: (next: Scenario) => void }) {
  const [check, setCheck] = useState("");
  const target = selected?.type === "tag" ? scenario.tags[selected.index ?? 0]
    : selected?.type === "objective" ? scenario.objectives[selected.index ?? 0]
    : selected?.type === "start" ? scenario.start : selected?.type === "goal" ? scenario.goal : null;
  useEffect(() => { setCheck(""); }, [selected?.type, selected?.index, target?.x, target?.y]);
  if (!target || !selected) return <div className="empty-state"><Crosshair size={22} /><b>개체 없음</b><span>지도에서 출발점, 목표, 태그, 경유지를 선택합니다.</span></div>;

  const update = (key: string, value: string | number) => {
    const next = structuredClone(scenario);
    const obj = selected.type === "tag" ? next.tags[selected.index ?? 0]
      : selected.type === "objective" ? next.objectives[selected.index ?? 0]
      : selected.type === "start" ? next.start : next.goal;
    Object.assign(obj, { [key]: value });
    onChange(next);
  };
  const inspect = () => {
    const { width_m: w, height_m: h } = scenario.field;
    const inside = target.x >= 0 && target.x <= w && target.y >= 0 && target.y <= h;
    setCheck(inside ? "범위 안" : "범위 밖");
  };
  return (
    <div className="properties">
      <div className="selection-id"><CircleDot size={13} /> {selected.type.toUpperCase()} {selected.index != null ? `#${selected.index + 1}` : ""}</div>
      {"id" in target && <label>식별자<Input value={String(target.id)} onChange={(e) => update("id", e.target.value)} /></label>}
      {"name" in target && <label>호출명<Input value={String(target.name ?? "")} onChange={(e) => update("name", e.target.value)} /></label>}
      <div className="input-row">
        <label>X / EAST<Input type="number" step=".1" value={target.x} onChange={(e) => update("x", +e.target.value)} /></label>
        <label>Y / NORTH<Input type="number" step=".1" value={target.y} onChange={(e) => update("y", +e.target.value)} /></label>
      </div>
      {"role" in target && <label>노드 역할<FormSelect aria-label="노드 역할" value={String(target.role ?? "marker")} onValueChange={value=>update("role",value)} options={[{value:"marker",label:"AprilTag"},{value:"ble",label:"BLE NUS"},{value:"mailbox",label:"광학 메일박스"}]}/></label>}
      {"suggested" in target && target.suggested === true && <p className="suggest-note">역할 제안이 지도에 반영되어 있습니다.</p>}
      <button type="button" className="button secondary full" onClick={inspect}><RefreshCw size={14} /> 좌표 정합성 검사</button>
      <p className="check-result">{check || "NO DATA"}</p>
    </div>
  );
}

function Kpis({ streaming, metrics, stale }: { streaming: boolean; metrics: Metrics; stale: boolean }) {
  const values = [
    { label: "성공률", value: metrics.success, unit: "%", icon: ShieldCheck },
    { label: "전달률", value: metrics.delivery, unit: "%", icon: Send },
    { label: "경로", value: metrics.length, unit: "m", icon: Route },
    { label: "시간", value: metrics.time, unit: "s", icon: Gauge, clock: true },
  ];
  return <div className="kpi-grid">{values.map(({ label, value, unit, icon: Icon, clock }) => {
    const has = value != null && Number.isFinite(value);
    const shown = !has ? "--" : clock ? `${Math.floor(value / 60).toString().padStart(2, "0")}:${Math.round(value % 60).toString().padStart(2, "0")}` : value.toFixed(1);
    return (
      <article className="kpi" key={label}>
        <div><Icon size={13} aria-hidden="true" /><span>{label}</span></div>
        <strong>{shown}{has && !clock ? <small>{unit}</small> : null}</strong>
        <em>{!has ? "계획 전" : stale ? "이전 계획값" : streaming ? "계산 중 · 계획값" : "계획값"}</em>
        {unit === "%" && <span className="meter" aria-hidden="true"><i style={{ width: has ? `${Math.max(0, Math.min(100, value))}%` : "0%" }} /></span>}
      </article>
    );
  })}</div>;
}

function Console({ logs, active, briefing }: { logs: LogLine[]; active: boolean; briefing: string }) {
  const end = useRef<HTMLDivElement>(null);
  useEffect(() => { end.current?.scrollIntoView({ block: "nearest" }); }, [logs]);
  return <div className="console" id="trace-panel" aria-live="polite">
    <div className="console-head"><span><Terminal size={14} aria-hidden="true" /> TRACE</span><Pill tone={active ? "cyan" : "muted"}>{active ? "수신 중" : "대기"}</Pill></div>
    <div className="console-body">{logs.length === 0
      ? <p className="no-data">NO DATA</p>
      : logs.map((line, index) =>
        <div className={`log ${line.type}`} key={`${line.time}-${index}`}><time>{line.time}</time><i>{line.type === "ok" ? "✓" : line.type === "warn" ? "!" : "›"}</i><span>{line.text}</span></div>,
      )}<div ref={end} /></div>
    <p className="briefing"><b>BRIEF</b>{briefing || "NO DATA"}</p>
  </div>;
}

function phaseFromRun(status?: string, success?: boolean | null): PlanPhase {
  if (status === "failed" || success === false) return "failed";
  if (status === "stale") return "failed";
  if (status === "approved" || (status === "draft" && success === true)) return "success";
  return "idle";
}

function DrillBay({ runId, released, onStatus }: { runId: string; released: boolean; onStatus: (status: DrillStatus) => void }) {
  const [playing, setPlaying] = useState(false);
  const [speed, setSpeed] = useState(1);
  const [eventIndex, setEventIndex] = useState(0);
  const [selectedSampleIndex, setSelectedSampleIndex] = useState(0);
  const [selectedSample, setSelectedSample] = useState<DrillSample | null>(null);
  const [timeline, setTimeline] = useState<DrillTimeline | null>(null);
  useEffect(() => {
    setEventIndex(0);
    setSelectedSampleIndex(0);
    setSelectedSample(null);
    setPlaying(false);
    if (!runId || !released || runId.startsWith("RUN-")) { setTimeline(null); return; }
    let cancel = false;
    jsonApi<DrillTimeline>(`/api/drill/${encodeURIComponent(runId)}`).then((data) => {
      if (!cancel) setTimeline(data);
    }).catch(() => { if (!cancel) setTimeline(null); });
    return () => { cancel = true; };
  }, [runId, released]);
  const demo = !timeline?.events?.length;
  const events = timeline?.events?.length ? timeline.events : DEMO_EVENTS;
  const samples = timeline?.samples ?? [];
  const current = events[Math.min(eventIndex, Math.max(events.length - 1, 0))];
  const eventLabel = current?.label || "NO DATA";
  const activeSample = samples[selectedSampleIndex] ?? selectedSample;
  useEffect(() => {
    const item = timeline?.samples?.[selectedSampleIndex];
    if (item) setSelectedSample({ label: item.label, points: item.points });
  }, [timeline, selectedSampleIndex]);
  useEffect(() => {
    onStatus({ live: playing, event: eventLabel, demo });
  }, [playing, eventLabel, demo, onStatus]);
  useEffect(() => {
    if (!playing || events.length < 2) return;
    if (eventIndex >= events.length - 1) { setPlaying(false); return; }
    const timer = window.setTimeout(() => {
      setEventIndex((index) => Math.min(index + 1, events.length - 1));
    }, Math.max(450, 1800 / speed));
    return () => window.clearTimeout(timer);
  }, [playing, speed, events.length, eventIndex]);

  return <div className="drill-bay">
    <div className="drill-head">
      <span><Zap size={13} aria-hidden="true" /> EMP</span>
      <Pill tone={playing ? "red" : "muted"}>{playing ? "시뮬레이션" : eventIndex === events.length - 1 ? "완료" : "대기"}</Pill>
    </div>
    <div className="drill-body">
      <div className={`radar ${playing ? "playing" : ""}`} aria-hidden="true">
        <div className="radar-ring r1" /><div className="radar-ring r2" /><div className="radar-ring r3" />
        <div className="sweep" style={{ animationDuration: `${2.4 / speed}s` }} />
      </div>
      <div className="drill-readout">
        <p className="drill-event" aria-live="polite">{demo ? "DEMO · " : ""}{eventLabel}</p>
        <div className="drill-stats">
          <span>LINK<em>{playing ? "LOST" : "--"}</em></span>
          <span>T<em>{typeof current?.t_s === "number" ? current.t_s.toFixed(1) : "--"}</em></span>
          <span>SAMPLE<em>{activeSample?.label ?? (demo ? "DEMO" : "--")}</em></span>
          <span>PTS<em>{activeSample?.points?.length ?? "--"}</em></span>
        </div>
        <label className="range-label">속도 <output>{speed.toFixed(1)}×</output>
          <FormSlider min={.25} max={2} step={.25} value={speed} onValueChange={setSpeed} aria-label="드릴 재생 속도" />
        </label>
        <div className="drill-controls">
          <button type="button" className="icon-button" onClick={() => setPlaying((value) => {
            if (!value) setEventIndex(0);
            return !value;
          })} aria-label={playing ? "드릴 일시정지" : "드릴 재생"} aria-pressed={playing}>{playing ? <Pause /> : <Play />}</button>
          <FormSelect value={String(samples.length ? Math.min(selectedSampleIndex, samples.length - 1) : 0)} onValueChange={value => {
            const index = Number(value);
            setSelectedSampleIndex(index);
            const item = samples[index];
            if (item) setSelectedSample({ label: item.label, points: item.points });
          }} aria-label="드릴 샘플" options={samples.length?samples.map((item,index)=>({value:String(index),label:item.label})):[{value:"0",label:demo?"DEMO":"--"}]}/>
        </div>
      </div>
    </div>
  </div>;
}

function GateLink({
  enabled, href, className, children, external,
}: { enabled: boolean; href: string; className: string; children: ReactNode; external?: boolean }) {
  if (!enabled) {
    return <button type="button" className={`${className} is-disabled`} disabled aria-disabled="true">{children}</button>;
  }
  return <a className={className} href={href} {...(external ? { target: "_blank", rel: "noreferrer" } : { download: "" })}>{children}</a>;
}

function App() {
  const [scenario, setScenario] = useState<Scenario>(sample);
  const [scenarioNames, setScenarioNames] = useState<string[]>(["demo_spider", "demo_arena", "demo_jam"]);
  const [selected, setSelected] = useState<Selected>({ type: "tag", index: 1 });
  const [tab, setTab] = useState<"properties" | "json">("properties");
  const [jsonText, setJsonText] = useState(JSON.stringify(sample, null, 2));
  const [patchText, setPatchText] = useState("재밍 구역 안의 BLE 노드를 광학 메일박스로 바꾸고 안전 여유를 2m 늘려줘.");
  const [diff, setDiff] = useState<string[]>([]);
  const [pendingScenario, setPendingScenario] = useState<Scenario | null>(null);
  const [logs, setLogs] = useState<LogLine[]>([]);
  const [trace, setTrace] = useState("");
  const [streaming, setStreaming] = useState(false);
  const [busy, setBusy] = useState("");
  const [metrics, setMetrics] = useState<Metrics>({ success: null, delivery: null, length: null, time: null });
  const [plannedRoute, setPlannedRoute] = useState<MissionWaypoint[]>([]);
  const [briefing, setBriefing] = useState("계획을 실행하면 추적 기록 기반 브리핑이 표시됩니다.");
  const [library, setLibrary] = useState<LibraryItem[]>([]);
  const [activeRun, setActiveRun] = useState("");
  const [approvedRunId, setApprovedRunId] = useState("");
  const [staleRunId, setStaleRunId] = useState("");
  const [planPhase, setPlanPhase] = useState<PlanPhase>("idle");
  const [scenarioKey, setScenarioKey] = useState("");
  const [scenarioDirty, setScenarioDirty] = useState(false);
  const [core, setCore] = useState<CoreLink>("pending");
  const [source, setSource] = useState<"builtin" | "remote">("builtin");
  const [workspaceTab, setWorkspaceTab] = useState<"edit" | "plan" | "history">("edit");
  const [mapEditing, setMapEditing] = useState(false);
  const [mapFocus, setMapFocus] = useState(false);
  const [plannerOpen,setPlannerOpen]=useState(false);
  const [blueprintOpen,setBlueprintOpen]=useState(false);
  const [liveAircraftOpen,setLiveAircraftOpen]=useState(false);
  const [pageError, setPageError] = useState("");
  const [drill, setDrill] = useState<DrillStatus>({ live: false, event: "", demo: false });
  const streamAbort = useRef<AbortController | null>(null);
  const runPick = useRef(0);
  const scenarioEpoch = useRef(0);
  const libraryRef = useRef(library);
  libraryRef.current = library;
  const onDrill = useCallback((next: DrillStatus) => {
    setDrill((prev) => prev.live === next.live && prev.event === next.event && prev.demo === next.demo ? prev : next);
  }, []);

  const addLog = useCallback((text: string, type: LogLine["type"] = "info") => {
    setLogs((prev) => [...prev.slice(-60), { time: now(), type, text }]);
    setTrace(text);
    if (type === "warn") setPageError(text);
  }, []);
  const updateScenario = (next: Scenario) => {
    const changed = JSON.stringify(next) !== JSON.stringify(scenario);
    setScenario(next); setJsonText(JSON.stringify(next, null, 2));
    if (!changed) return;
    setPlannedRoute([]);
    setScenarioDirty(true);
    if (!activeRun) return;
    if (approvedRunId === activeRun || planPhase === "success") {
      if (approvedRunId === activeRun) setApprovedRunId("");
      setStaleRunId(activeRun);
      setPlanPhase("idle");
      setLibrary((items) => items.map((item) => item.id === activeRun ? { ...item, status: "stale" } : item));
    }
  };
  const adoptRemoteScenario = useCallback((data: Scenario, id: string) => {
    scenarioEpoch.current += 1;
    runPick.current += 1;
    streamAbort.current?.abort();
    setStreaming(false);
    setScenario(data);
    setJsonText(JSON.stringify(data, null, 2));
    setScenarioKey(id);
    setSource("remote");
    setScenarioDirty(false);
    setActiveRun("");
    setApprovedRunId("");
    setStaleRunId("");
    setPlanPhase("idle");
    setMetrics({ success: null, delivery: null, length: null, time: null });
    setPlannedRoute([]);
    setBriefing("계획을 실행하면 추적 기록 기반 브리핑이 표시됩니다.");
    setPageError("");
  }, []);
  const bindRunApproval = (id: string, status?: string, success?: boolean | null) => {
    if (status) setLibrary((items) => items.map((item) => item.id === id ? { ...item, status, success: success ?? item.success } : item));
    setPlanPhase(phaseFromRun(status, success));
    if (status === "approved") {
      setApprovedRunId(id);
      setStaleRunId((current) => (current === id ? "" : current));
      return;
    }
    setApprovedRunId((current) => (current === id ? "" : current));
    setStaleRunId((current) => (status === "stale" ? id : current === id ? "" : current));
  };
  const selectRun = (id: string) => {
    setActiveRun(id);
    setPlannedRoute([]);
    setMetrics({ success: null, delivery: null, length: null, time: null });
    const ticket = ++runPick.current;
    if (!id) { setPlanPhase("idle"); return; }
    jsonApi<{ status?: string; success?: boolean | null }>(`/api/library/${encodeURIComponent(id)}`).then((meta) => {
      if (ticket !== runPick.current) return;
      bindRunApproval(id, meta.status, meta.success);
    }).catch(() => {
      if (ticket !== runPick.current) return;
      const local = libraryRef.current.find((item) => item.id === id);
      bindRunApproval(id, local?.status, local?.success);
    });
  };

  useEffect(() => {
    jsonApi<Array<string | { name: string }>>("/api/scenarios").then((data) => {
      setCore("online");
      const names = data.map((v) => typeof v === "string" ? v : v.name).filter(Boolean);
      if (names.length) {
        setScenarioNames(names);
        const preferred = names.includes("demo_training_camp") ? "demo_training_camp" : names.includes("demo_spider") ? "demo_spider" : names[0];
        jsonApi<Scenario>(`/api/scenarios/${encodeURIComponent(preferred)}`).then((loaded) => {
          adoptRemoteScenario(loaded, preferred);
          addLog(`${preferred} 시나리오 적재`, "ok");
        }).catch(() => undefined);
      }
    }).catch(() => { setCore("offline"); addLog("백엔드 미연결. 내장 시나리오를 사용합니다", "warn"); });
    jsonApi<LibraryItem[]>("/api/library").then((data) => { if (data.length) setLibrary(data); }).catch(() => undefined);
    return () => streamAbort.current?.abort();
  }, [addLog, adoptRemoteScenario]);

  useEffect(() => {
    if (drill.live) return;
    const controller = new AbortController();
    const checkHealth = () => {
      jsonApi("/api/health", { signal: controller.signal })
        .then(() => { if (!controller.signal.aborted) setCore("online"); })
        .catch(() => { if (!controller.signal.aborted) setCore("offline"); });
    };
    checkHealth();
    const timer = window.setInterval(checkHealth, 30_000);
    return () => { controller.abort(); window.clearInterval(timer); };
  }, [drill.live]);

  const loadScenario = async (name: string) => {
    setBusy("load"); setPageError("");
    try {
      const data = await jsonApi<Scenario>(`/api/scenarios/${encodeURIComponent(name)}`);
      setCore("online"); adoptRemoteScenario(data, name); setSelected(null); addLog(`${name} 시나리오 불러옴`, "ok");
    } catch (error) { addLog(`시나리오 요청 실패: ${(error as Error).message}`, "warn"); }
    finally { setBusy(""); }
  };

  const validate = async () => {
    setBusy("validate"); addLog("전장 정합성 검증 시작");
    try {
      const result = await jsonApi<Record<string, unknown>>("/api/validate", { method: "POST", body: JSON.stringify(scenario) });
      addLog(`검증 완료 · ${String(result.message ?? result.status ?? "통과")}`, "ok");
    } catch (error) { addLog(`검증 실패: ${(error as Error).message}`, "warn"); }
    finally { setBusy(""); }
  };

  const analyze = async () => {
    setBusy("analyze"); addLog("위험도 및 경로 분석 요청");
    try {
      const result = await jsonApi<Record<string, unknown>>("/api/analyze", { method: "POST", body: JSON.stringify(scenario) });
      addLog(`분석 완료 · ${String(result.summary ?? "KPI 갱신")}`, "ok");
    } catch (error) { addLog(`분석 실패: ${(error as Error).message}`, "warn"); }
    finally { setBusy(""); }
  };

  const requestPatch = async (event: FormEvent) => {
    event.preventDefault(); if (!patchText.trim()) return;
    setBusy("patch"); addLog(`자연어 패치 분석: “${patchText}”`);
    try {
      const result = await jsonApi<{ scenario?: Scenario; patch?: unknown; diff?: string[] }>("/api/agent/patch", {
        method: "POST", body: JSON.stringify({ scenario, instruction: patchText }),
      });
      setDiff(result.diff ?? [JSON.stringify(result.patch ?? result, null, 2)]);
      setPendingScenario(result.scenario ?? null);
      addLog("에이전트 패치 생성 완료. 검토가 필요합니다", "ok");
    } catch {
      const next = structuredClone(scenario);
      next.tags = next.tags.map((v) => v.role === "ble" ? { ...v, role: "mailbox" } : v);
      setDiff(["-  role: \"ble\"", "+  role: \"mailbox\"", "+  safety_margin_m: 2.0"]);
      setPendingScenario(next); addLog("오프라인 규칙으로 패치 미리보기를 만들었습니다", "warn");
    } finally { setBusy(""); }
  };

  const startPlan = async () => {
    if (streaming) { streamAbort.current?.abort(); setStreaming(false); addLog("계획 스트림 중단", "warn"); return; }
    setWorkspaceTab("plan"); setPlannerOpen(true); setMapEditing(false); setPageError("");
    setPlannedRoute([]);
    setMetrics({ success: null, delivery: null, length: null, time: null });
    const controller = new AbortController(); streamAbort.current = controller; setStreaming(true);
    const epoch = scenarioEpoch.current;
    setPlanPhase("planning");
    setApprovedRunId("");
    setStaleRunId("");
    addLog("에이전트 계획 스트림 연결");
    let outcome: PlanPhase = "planning";
    try {
      const response = await fetch("/api/plan/stream", {
        method: "POST", headers: { "Content-Type": "application/json" },
        body: JSON.stringify(buildPlanRequest(source, scenarioKey, scenarioDirty, scenario)), signal: controller.signal,
      });
      await parseSse(response, (event, data) => {
        if (epoch !== scenarioEpoch.current) return;
        const object = typeof data === "object" && data ? data as Record<string, unknown> : {};
        const text = String(object.summary ?? object.message ?? object.text ?? object.detail ?? event);
        addLog(text, event === "error" ? "warn" : event === "complete" ? "ok" : "info");
        if (event === "error") { outcome = "failed"; setPlanPhase("failed"); }
        if (event === "complete") {
          outcome = object.success === true ? "success" : "failed";
          setPlanPhase(outcome);
          jsonApi<LibraryItem[]>("/api/library").then(setLibrary).catch(() => undefined);
        }
        if (object.run_id) setActiveRun(String(object.run_id));
        if (event === "trace" && object.tool === "verify") {
          const success=text.match(/성공률\s+([\d.]+)%/),delivery=text.match(/전달률\s+([\d.]+)%/),duration=text.match(/예상\s+([\d.]+)s/);
          setMetrics(prev=>({...prev,success:success?+success[1]:prev.success,delivery:delivery?+delivery[1]:prev.delivery,time:duration?+duration[1]:prev.time}));
          const dataObject = object.data as Record<string, unknown> | undefined;
          if (typeof dataObject?.length_m === "number") setMetrics((prev) => ({ ...prev, length: dataObject.length_m as number }));
        }
        if (event === "complete" && object.mission && typeof object.mission === "object") {
          const mission = object.mission as {
            waypoints?: Array<{ x_m?: number; y_m?: number; tag_id?:string|number|null;kind?:string;z_m?:number;speed_mps?:number;dwell_s?:number;label?:string }>;
            estimate?: { total_length_m?: number; total_time_s?: number };
            network?: { delivery_rate?: number; nodes?: Array<{ id: number; role: Tag["role"]; role_change_suggested?: boolean }> };
            tags?: Array<{ id: number; x_m: number; y_m: number; role: Tag["role"]; install_required?: boolean }>;
          };
          setPlannedRoute((Array.isArray(mission.waypoints) ? mission.waypoints : [])
            .filter((point) => typeof point.x_m === "number" && Number.isFinite(point.x_m) && typeof point.y_m === "number" && Number.isFinite(point.y_m))
            .map((point) => ({ x: point.x_m as number, y: point.y_m as number,tagId:point.tag_id,kind:point.kind,height:point.z_m,speed:point.speed_mps,dwell:point.dwell_s,label:point.label })));
          setMetrics((prev) => ({
            ...prev,
            length: mission.estimate?.total_length_m ?? prev.length,
            time: mission.estimate?.total_time_s ?? prev.time,
            delivery: mission.network?.delivery_rate != null ? mission.network.delivery_rate * 100 : prev.delivery,
          }));
          setScenario((current) => {
            const next = structuredClone(current);
            for (const resultTag of mission.tags ?? []) {
              const known = next.tags.find((tag) => String(tag.id) === String(resultTag.id));
              if (known) {
                known.role = resultTag.role;
                known.suggested = Boolean(resultTag.install_required);
              } else if (resultTag.install_required) {
                next.tags.push({ id: resultTag.id, x: resultTag.x_m, y: resultTag.y_m, role: resultTag.role, suggested: true });
              }
            }
            for (const node of mission.network?.nodes ?? []) {
              const known = next.tags.find((tag) => String(tag.id) === String(node.id));
              if (known && node.role_change_suggested) { known.role = node.role; known.suggested = true; }
            }
            if (JSON.stringify(next) === JSON.stringify(current)) return current;
            setScenarioDirty(true);
            setJsonText(JSON.stringify(next, null, 2));
            return next;
          });
          jsonApi<{ briefing: string }>("/api/llm/briefing", {
            method: "POST", body: JSON.stringify({ scenario }),
          }).then((result) => setBriefing(result.briefing)).catch(() => undefined);
        }
      }, controller.signal);
      if (epoch !== scenarioEpoch.current) return;
      if (!controller.signal.aborted) addLog("계획 생성 스트림 완료", "ok");
      if (outcome === "planning") { outcome = "failed"; setPlanPhase("failed"); }
    } catch (error) {
      if (epoch !== scenarioEpoch.current) return;
      if (!controller.signal.aborted) {
        outcome = "failed";
        setPlanPhase("failed");
        addLog(`계획 스트림 오류: ${(error as Error).message}`, "warn");
      }
    } finally {
      setStreaming(false);
      if (epoch !== scenarioEpoch.current) return;
      if (controller.signal.aborted || outcome === "planning") setPlanPhase("failed");
    }
  };

  const approve = async () => {
    if (!activeRun) { addLog("승인할 런이 없습니다", "warn"); return; }
    setBusy("approve");
    try {
      await jsonApi(`/api/runs/${encodeURIComponent(activeRun)}/approve`, { method: "POST", body: "{}" });
      const approvedId = activeRun;
      setApprovedRunId(approvedId);
      setStaleRunId((current) => (current === approvedId ? "" : current));
      setLibrary((items) => items.map((item) => item.id === approvedId ? { ...item, status: "approved" } : item));
      addLog(`${approvedId} 승인 완료`, "ok");
    } catch (error) { addLog(`승인 실패: ${(error as Error).message}`, "warn"); }
    finally { setBusy(""); }
  };

  const applyJson = () => {
    try { updateScenario(JSON.parse(jsonText) as Scenario); addLog("JSON 편집 내용 적용", "ok"); }
    catch (error) { addLog(`JSON 문법 오류: ${(error as Error).message}`, "warn"); }
  };

  const addTag = (point: Point) => {
    const next = structuredClone(scenario);
    const nextId = Math.max(0, ...next.tags.map((tag) => Number(tag.id) || 0)) + 1;
    next.tags.push({ id: nextId, ...point, role: "marker" });
    updateScenario(next); setSelected({ type: "tag", index: next.tags.length - 1 });
    setWorkspaceTab("edit"); setTab("properties");
    addLog(`태그 추가 (${point.x}, ${point.y})`, "ok");
  };

  const missionCount = useMemo(() => scenario.objectives.length + scenario.tags.length, [scenario]);
  const released = Boolean(activeRun) && approvedRunId === activeRun;
  const approvalStatus: "draft" | "approved" | "stale" = released ? "approved" : activeRun && staleRunId === activeRun ? "stale" : "draft";
  const demoData = source === "builtin" || scenarioKey.startsWith("demo_") || /^demo[_-]/i.test(scenario.name);
  const unsavedVariant = source === "remote" && scenarioDirty;
  const warnings = safetyLines(scenario);
  const approvalLabel = approvalStatus === "approved"
    ? "승인됨"
    : approvalStatus === "stale"
      ? "재계획 필요"
      : planPhase === "failed"
        ? "계획 실패"
        : activeRun ? "승인 대기" : "계획 전";
  const approvalTone = approvalStatus === "approved" ? "cyan" : approvalStatus === "stale" || planPhase === "failed" ? "red" : "amber";
  const canApprove = Boolean(activeRun) && busy !== "approve" && planPhase === "success" && approvalStatus !== "approved" && approvalStatus !== "stale";
  const approvalNote = planPhase === "planning"
    ? "계획을 생성하는 동안에는 승인할 수 없습니다."
    : approvalStatus === "stale"
      ? "시나리오가 바뀌었습니다. 다시 승인해야 링크가 열립니다."
      : planPhase === "failed"
        ? "계획이 실패했거나 검증을 통과하지 않아 승인할 수 없습니다."
        : approvalStatus === "approved"
          ? "승인됨. 패키지, 카드, 드릴을 열 수 있습니다."
          : "승인 전에는 패키지, 카드, 드릴 링크가 잠겨 있습니다.";

  const missionStage = released ? 3 : planPhase === "planning" || planPhase === "success" ? 2 : 1;
  const selectedName = selected?.type === "tag" ? `노드 ${scenario.tags[selected.index ?? 0]?.id ?? ""}`
    : selected?.type === "objective" ? scenario.objectives[selected.index ?? 0]?.name
      : selected?.type === "start" ? "출발점" : selected?.type === "goal" ? "귀환 목표" : "선택한 개체 없음";
  const showSelection = (next: Selected) => { setSelected(next); if (next) { setWorkspaceTab("edit"); setTab("properties"); } };
  const openSimulator = () => window.dispatchEvent(new CustomEvent("spider:view-controller"));

  return (
    <div className={`app cmd-app cmd-unified${plannerOpen?' has-planner':''}${mapFocus ? ' is-map-focused' : ''}`} data-mode={drill.live ? "emp" : "online"}>
      <p className="sr-only" aria-live="polite">{drill.live ? `오프라인 리허설. LINK LOST. ${drill.event}` : core === "online" ? "로컬 API 응답 확인" : "서버 연결 확인 필요"}</p>
      <header className="topbar cmd-topbar">
        <a className="brand" href="#main" aria-label="SPIDER 전술지휘소 메인"><div className="brand-mark" aria-hidden="true"><Crosshair /><span /></div><div><b>SPIDER</b><small>DEFENSE SYSTEMS</small></div></a>
        <h1 className="sr-only">SPIDER 전술지휘소</h1>
        <div className="scenario-select cmd-scenario-select"><label htmlFor="scenario">임무</label>
          <FormSelect id="scenario" aria-label="임무 선택" value={scenarioKey || "builtin"} title={scenario.name} onValueChange={value=>{if(value!=="builtin")void loadScenario(value);}} disabled={busy === "load" || streaming} options={[...(!scenarioKey?[{value:"builtin",label:`내장 임무 · ${scenarioKeyLabel(scenario.name)}`}]:[]),...scenarioNames.map(name=>({value:name,label:name==='demo_training_camp'?'청솔 훈련장':name}))]}/>
        </div>
        <div className="cmd-server-status"><Pill tone={core === "online" ? "cyan" : core === "offline" ? "amber" : "muted"}>{core === "online" ? "API 응답 확인" : core === "offline" ? "API 미연결" : "서버 확인 중"}</Pill><small>{source === "builtin" ? "내장 데이터 사용" : "로컬 작업공간"}</small></div>
        <div className="head-actions cmd-head-actions">{!liveAircraftOpen&&<><button type="button" className="button secondary" data-nav="simulator" onClick={openSimulator}><Box size={16} />조종기 보기</button><button type="button" className="button secondary" aria-pressed={plannerOpen} onClick={()=>setPlannerOpen(v=>!v)}><Pencil size={15}/>임무 패널</button><button type="button" id="plan-action" className="button primary" onClick={startPlan} disabled={!!busy && !streaming}><Sparkles />{streaming ? "생성 중단" : "계획 생성"}</button></>}<button type="button" className={`button ${liveAircraftOpen?'primary':'ghost'} mc-hardware-link`} aria-pressed={liveAircraftOpen} onClick={()=>{setPlannerOpen(false);setBlueprintOpen(false);setLiveAircraftOpen(true);}}>{liveAircraftOpen?'실기체 관제':'기체 연결'}</button></div>
        <div className="cmd-clock"><UtcClock /><span>로컬 시각</span></div>
      </header>

      <div className="cmd-workspace" id="main">
        <section className="cmd-briefing-bar" aria-label="임무 개요">
          <div className="cmd-operation-heading"><span><i/>UNMANNED OPERATIONS / 01</span><h2>무인기 임무 통제</h2><p>임무 설계부터 비행 리허설까지</p></div>
          <dl className="cmd-operation-stats"><div><dt>훈련 구역</dt><dd>{scenario.field.width_m}<small>×</small>{scenario.field.height_m}<small>m</small></dd></div><div><dt>등록 노드</dt><dd>{scenario.tags.length.toString().padStart(2,'0')}<small>개</small></dd></div><div><dt>링크 두절 정책</dt><dd className="cmd-policy-value">{scenario.link_policy==='continue_mission'?'임무 지속':'임무 중단'}<span>{scenario.link_policy==='continue_mission'?'CONTINUE':'ABORT'}</span></dd></div></dl>
          <button type="button" className="cmd-focus-toggle" aria-pressed={mapFocus} onClick={()=>setMapFocus((value)=>!value)}><Maximize size={16}/><span>{mapFocus?'작업 패널 복원':'지도 집중'}</span></button>
        </section>
        <div className="cmd-stagebar" aria-label="임무 준비 단계">
          <div className="cmd-stage-list">{([
            [1, "임무 설정", "edit"], [2, "계획 검증", "plan"], [3, "승인·리허설", "plan"],
          ] as const).map(([step, label, destination]) => <button type="button" key={step} onClick={() => setWorkspaceTab(destination)} className={`${missionStage === step ? "current" : ""} ${missionStage > step ? "done" : ""}`} aria-current={missionStage === step ? "step" : undefined}><span>{missionStage > step ? <Check size={13} /> : step}</span>{label}{step < 3 && <ChevronRight className="cmd-step-arrow" size={15} />}</button>)}</div>
          <div className="cmd-stage-meta">{demoData && <span className="cmd-meta-chip">데모 데이터</span>}{unsavedVariant && <span className="cmd-meta-chip amber">저장 전 변형</span>}<span>{missionCount}개 지점</span><Pill tone={approvalTone}>{approvalLabel}</Pill></div>
        </div>
        {(pageError || busy === "load" || core === "offline") && <div className={`cmd-page-notice ${pageError ? "warn" : ""}`} role={pageError ? "alert" : "status"}><AlertTriangle size={16} /><span>{busy === "load" ? "임무 데이터를 불러오고 있습니다." : pageError || "API가 연결되지 않았습니다. 내장 임무를 살펴보거나 시뮬레이터를 사용할 수 있습니다."}</span>{core === "offline" && <button type="button" onClick={() => loadScenario(scenarioKey || "demo_spider")} disabled={!!busy}>다시 연결</button>}{pageError && <button type="button" aria-label="알림 닫기" onClick={() => setPageError("")}><X size={16} /></button>}</div>}
        <div className="cockpit cmd-cockpit">
          <section className="map-panel" id="map-panel" aria-label={liveAircraftOpen?'실기체 3D 관제':'통합 3D 훈련 관제'}><Suspense fallback={<div className="mc-loading"><RefreshCw className="spin"/><strong>3D 관제 준비 중</strong></div>}>{liveAircraftOpen?<LiveAircraft onBack={()=>setLiveAircraftOpen(false)} onDiagnostics={()=>window.dispatchEvent(new CustomEvent('spider:open-hardware'))}/>:<MissionControl scenario={scenario} plannedRoute={plannedRoute} plannerOpen={plannerOpen||blueprintOpen} planning={streaming} onPlanner={()=>setPlannerOpen(v=>!v)} onTag={id=>{showSelection({type:'tag',index:scenario.tags.findIndex(t=>String(t.id)===id)});setPlannerOpen(true);}} onApproach={()=>window.dispatchEvent(new CustomEvent('spider:open-simulator',{detail:{initialMode:'approach'}}))}/>}</Suspense></section>
          {blueprintOpen&&<section className="mc-blueprint" aria-label="태그 배치 도면"><header><strong>태그 배치 · 지도와 3D가 같은 좌표를 사용합니다</strong><button type="button" aria-label="도면 닫기" onClick={()=>setBlueprintOpen(false)}><X size={18}/></button></header><MapView scenario={scenario} selected={selected} onSelect={showSelection} onAddTag={addTag} linkLost={drill.live} eventLabel={drill.event} demoEvent={drill.demo} demoData={demoData} trace={trace} editing={mapEditing&&!streaming} onEditing={setMapEditing} plannedRoute={plannedRoute}/></section>}
          <aside className="rail cmd-mission-rail" aria-label="임무 작업 패널">
            <div className="cmd-rail-heading"><button type="button" aria-label="임무 패널 닫기" onClick={()=>setPlannerOpen(false)}><X size={17}/></button><div><small>MISSION WORKSPACE</small><h2>{workspaceTab === "edit" ? "임무 편집" : workspaceTab === "plan" ? "계획과 승인" : "실행 기록"}</h2></div><span className="cmd-rail-count">{workspaceTab === "history" ? library.length : scenario.tags.length}<small>{workspaceTab === "history" ? "기록" : "노드"}</small></span></div>
            <div className="cmd-workspace-tabs" role="tablist" aria-label="임무 작업 선택">{([['edit', '편집', Pencil], ['plan', '계획', Route], ['history', '기록', Library]] as const).map(([key, label, Icon], index) => <button type="button" key={key} id={`cmd-tab-${key}`} role="tab" data-nav={key} aria-selected={workspaceTab === key} aria-controls={`cmd-panel-${key}`} tabIndex={workspaceTab === key ? 0 : -1} onClick={() => setWorkspaceTab(key)} onKeyDown={(event) => {
              if (event.key !== "ArrowRight" && event.key !== "ArrowLeft") return;
              event.preventDefault(); const keys = ['edit', 'plan', 'history'] as const; const next = keys[(index + (event.key === "ArrowRight" ? 1 : 2)) % 3]; setWorkspaceTab(next); document.getElementById(`cmd-tab-${next}`)?.focus();
            }}><Icon size={16} />{label}{key === "plan" && streaming && <i className="cmd-tab-busy" />}</button>)}</div>
            <div className="cmd-rail-scroll" id={`cmd-panel-${workspaceTab}`} role="tabpanel" aria-labelledby={`cmd-tab-${workspaceTab}`}>
              {workspaceTab === "edit" && <><button type="button" className="button secondary full" onClick={()=>{setBlueprintOpen(true);setMapEditing(true);}}><Plus size={14}/>도면에서 태그 배치</button>
                <section className="panel agent-panel cmd-card" id="agent-panel"><PanelTitle icon={Bot} title="말로 임무 수정" aside={<Pill tone="muted">{busy === "patch" ? "분석 중" : "검토 후 적용"}</Pill>} /><FieldDescription className="cmd-card-description">변경할 내용을 적으면 먼저 수정안을 보여드립니다.</FieldDescription><form onSubmit={requestPatch}><Field><FieldLabel htmlFor="mission-patch" className="sr-only">패치 지시</FieldLabel><Textarea id="mission-patch" value={patchText} onChange={(e) => setPatchText(e.target.value)} aria-label="패치 지시" placeholder="예: 노드 3을 대피소로, 사진 30KB 회수 메시지 추가" disabled={streaming} /></Field><Button className="button secondary" type="submit" disabled={!!busy || streaming || !patchText.trim()}><Send size={15} />{busy === "patch" ? "수정안 만드는 중" : "수정안 만들기"}</Button></form>
                  {diff.length > 0 && <div className="diff"><div><span>변경 미리보기</span><b>{diff.length}개 변경</b></div><pre>{diff.map((line, i) => <code className={line.startsWith("+") ? "add" : line.startsWith("-") ? "remove" : ""} key={i}>{line}{"\n"}</code>)}</pre><div className="cmd-diff-actions"><button type="button" className="button primary" disabled={!pendingScenario || streaming} onClick={() => { if (pendingScenario) updateScenario(pendingScenario); setPendingScenario(null); setDiff([]); addLog("패치 변경사항 적용 확정", "ok"); }}><Check />변경 적용</button><button type="button" className="button ghost" onClick={() => { setPendingScenario(null); setDiff([]); addLog("패치 제안을 취소했습니다"); }}>취소</button></div></div>}
                </section>
                <section className="panel inspector cmd-card" id="inspector"><PanelTitle icon={Crosshair} title={selectedName || "선택한 개체"} aside={<button type="button" className="cmd-text-button" onClick={() => setMapEditing(!mapEditing)} disabled={streaming}>{mapEditing ? "편집 종료" : "속성 편집"}</button>} /><div className="tabs" role="tablist" aria-label="개체 속성"><button type="button" role="tab" id="inspector-tab-properties" aria-selected={tab === "properties"} aria-controls="inspector-panel-properties" className={tab === "properties" ? "active" : ""} onClick={() => setTab("properties")}><Gauge size={14} />속성</button><button type="button" role="tab" id="inspector-tab-json" aria-selected={tab === "json"} aria-controls="inspector-panel-json" className={tab === "json" ? "active" : ""} onClick={() => setTab("json")}><FileJson size={14} />JSON</button></div>
                  <div role="tabpanel" id="inspector-panel-properties" aria-labelledby="inspector-tab-properties" hidden={tab !== "properties"}><fieldset disabled={!mapEditing || streaming} className="cmd-property-fields"><PropertyPanel scenario={scenario} selected={selected} onChange={updateScenario} /></fieldset>{!mapEditing && selected && <p className="cmd-field-hint"><Lock size={12} />속성 편집을 켜면 좌표를 변경할 수 있습니다.</p>}</div>
                  <div role="tabpanel" id="inspector-panel-json" className="json-editor" aria-labelledby="inspector-tab-json" hidden={tab !== "json"}><Textarea spellCheck={false} value={jsonText} onChange={(e) => setJsonText(e.target.value)} aria-label="시나리오 JSON" disabled={streaming} /><button type="button" className="button secondary full" onClick={applyJson} disabled={streaming}><Upload size={14} />JSON 적용</button></div>
                </section>
                <div className="cmd-edit-actions"><button type="button" className="button ghost" onClick={validate} disabled={!!busy || streaming}><ShieldCheck />입력 검증</button><button type="button" className="button ghost" onClick={analyze} disabled={!!busy || streaming}><Activity />위험 분석</button></div>
              </>}
              {workspaceTab === "plan" && <>
                <section className="mc-plan-metrics"><Kpis streaming={streaming} metrics={metrics} stale={approvalStatus === "stale"}/></section><section className="cmd-card cmd-plan-state"><span className={`cmd-plan-symbol ${planPhase}`}><Route size={25} /></span><div><h3>{streaming ? "계획을 검증하고 있습니다" : planPhase === "success" ? "계획 검증 완료" : planPhase === "failed" ? "계획을 다시 확인하세요" : "첫 계획을 생성하세요"}</h3><p>{streaming ? "분석 → 계획 → 보완 → 검증" : released ? "승인한 산출물로 리허설을 시작할 수 있습니다." : "계획 결과를 확인한 뒤 직접 승인합니다."}</p></div></section>
                <section className="panel approval cmd-card" id="approval-panel"><PanelTitle icon={approvalStatus === "stale" || planPhase === "failed" ? AlertTriangle : ShieldCheck} title="승인과 산출물" aside={<Pill tone={approvalTone}>{approvalLabel}</Pill>} /><div className="approval-body"><div className="approval-run"><span>실행 ID</span><strong title={activeRun}>{activeRun || "계획 생성 후 표시"}</strong></div><div className="approval-actions"><button type="button" className="button approve" onClick={approve} disabled={!canApprove}><Check />{busy === "approve" ? "승인 중" : approvalStatus === "approved" ? "승인됨" : "임무 승인"}</button><GateLink enabled={released} className="button secondary" href={`/api/runs/${encodeURIComponent(activeRun)}/package`}><Download />{released ? "패키지 받기" : "패키지 잠김"}</GateLink><GateLink enabled={released} className="button ghost" external href={`/api/cards/${encodeURIComponent(activeRun)}.png`}><FileJson />플랜 카드</GateLink><GateLink enabled={released} className="button ghost" external href={`/api/drill/${encodeURIComponent(activeRun)}`}><ArrowUpRight />드릴 데이터</GateLink></div><p id="gate-note"><Lock size={13} />{approvalNote}</p></div></section>
                <Console logs={logs} active={streaming} briefing={briefing} />
                <details className="cmd-card mc-rehearsal"><summary>기존 통신 두절 리허설</summary><DrillBay runId={activeRun} released={released} onStatus={onDrill}/></details>
                <section className="cmd-card cmd-risk-card"><PanelTitle icon={AlertTriangle} title="시나리오 주의사항" /><ul className="warn-list">{warnings.length ? warnings.map((line) => <li key={line.text} className={line.tone}>{line.text}</li>) : <li className="muted">등록된 주의 구역 없음</li>}</ul><p className="cmd-card-description">계획 검증 결과는 실제 비행 허가가 아닙니다.</p></section>
              </>}
              {workspaceTab === "history" && <section className="cmd-card cmd-history" id="library-panel"><PanelTitle icon={Library} title="임무 라이브러리" aside={<span>{library.length}건</span>} /><p className="cmd-card-description">실행을 선택해 승인 상태와 산출물을 확인하세요.</p><div className="mission-list">{library.length === 0 ? <div className="empty-state"><Library size={28} /><b>저장된 실행이 없습니다</b><span>첫 계획을 생성하면 여기에 기록됩니다.</span></div> : library.map((item) => <button type="button" key={item.id ?? item.name} className={item.id === activeRun ? "active" : ""} onClick={() => { selectRun(item.id ?? ""); setWorkspaceTab("plan"); }}><span className="mission-icon"><PackageCheck /></span><span><b>{item.name}</b><small>{item.status === "approved" ? "승인됨" : item.status === "stale" ? "재계획 필요" : item.status === "failed" ? "실패" : "초안"} · {item.id?.slice(0, 8) || "--"}</small></span><ChevronRight size={16} /></button>)}</div></section>}
            </div>
          </aside>
        </div>
      </div>

      <footer className="strip cmd-footer" aria-label="계획 지표와 리허설">
        <section className="bay hardware-bay" aria-label="실기기 제어"><HardwareControlBay addLog={addLog} /></section>
      </footer>
    </div>
  );
}

export default App;
