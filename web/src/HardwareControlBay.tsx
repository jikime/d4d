import {
  Activity, AlertTriangle, Antenna, Battery, Bluetooth, Camera, Check, Cpu, Crosshair, Gauge,
  Lock, OctagonX, PanelRightOpen, Radio, RefreshCw, ShieldCheck, Usb, X,
} from "lucide-react";
import { lazy, Suspense, ReactNode, useCallback, useEffect, useMemo, useRef, useState } from "react";

const Simulator = lazy(() => import("./Simulator"));
import type {SimulatorLaunch} from './Simulator';
import {
  APPROACH_PARAMS,
  BOARD_COMMANDS,
  NUS_RX_UUID,
  NUS_SERVICE_UUID,
  NUS_TX_UUID,
  OVERRIDE_CONFIRMATION,
  consumeTelemetryChunk,
  createTelemetryState,
  deriveApproachView,
  deriveAuthorityView,
  deriveBoardLinkState,
  deriveBoardThrottle,
  deriveBoardCommandAvailability,
  deriveFlightLockReasons,
  deriveMtfHealth,
  deriveRcComparison,
  freshness,
  hardwareErrorMessage,
  serializeApproachSet,
  serializeBoardCommand,
  type ApproachParamName,
  type BoardCommand,
  type FcStatus,
  type PillTone,
  type TelemetryState,
} from "./hardware";

type LogType = "info" | "ok" | "warn";
type BleState = "idle" | "connecting" | "connected" | "unsupported";
type RamTarget = 7 | 64 | 67 | 71;
interface Challenge { target: RamTarget; token: string; expiresAt: number }
interface ApplyResult { target: RamTarget; previous: unknown; value: unknown; at: number }

const FC_POLL_MS = 3_000;
const FC_STALE_MS = 8_000;
const BOARD_STALE_MS = 3_000;

interface BluetoothLike {
  requestDevice(options: unknown): Promise<BluetoothDeviceLike>;
}
interface BluetoothDeviceLike {
  name?: string;
  gatt?: {
    connected: boolean;
    connect(): Promise<{ getPrimaryService(uuid: string): Promise<GattServiceLike> }>;
    disconnect(): void;
  };
  addEventListener(type: "gattserverdisconnected", listener: () => void): void;
  removeEventListener(type: "gattserverdisconnected", listener: () => void): void;
}
interface GattServiceLike {
  getCharacteristic(uuid: string): Promise<GattCharacteristicLike>;
}
interface GattCharacteristicLike {
  startNotifications(): Promise<unknown>;
  addEventListener(type: "characteristicvaluechanged", listener: (event: Event) => void): void;
  writeValueWithoutResponse?(value: BufferSource): Promise<void>;
  writeValue(value: BufferSource): Promise<void>;
}

async function hardwareApi<T>(url: string, init?: RequestInit): Promise<T> {
  let response: Response;
  try {
    response = await fetch(url, {
      ...init,
      headers: { "Content-Type": "application/json", ...init?.headers },
    });
  } catch {
    throw new Error("백엔드 API 연결 실패 (scripts\\dev.ps1 또는 uvicorn 실행 확인)");
  }
  let payload: unknown = null;
  try { payload = await response.json(); } catch { payload = null; }
  if (!response.ok) throw new Error(hardwareErrorMessage(payload, response.status));
  return payload as T;
}

function Pill({ tone = "cyan", children }: { tone?: PillTone; children: ReactNode }) {
  return <span className={`pill ${tone}`}><i />{children}</span>;
}

function Readout({ label, value, tone }: { label: string; value: ReactNode; tone?: "bad" | "warn" | "ok" }) {
  return <span><dt>{label}</dt><dd className={tone ?? ""} title={typeof value === "string" || typeof value === "number" ? String(value) : undefined}>{value}</dd></span>;
}

function Card({ icon: Icon, title, aside, children, hidden = false }: { icon: typeof Activity; title: string; aside?: ReactNode; children: ReactNode; hidden?: boolean }) {
  return (
    <section className="hw-card" hidden={hidden}>
      <h3><Icon size={12} aria-hidden="true" /> {title} {aside && <span className="hw-card-aside">{aside}</span>}</h3>
      {children}
    </section>
  );
}

function MotorBars({ motors }: { motors: number[] | undefined }) {
  const values = Array.from({ length: 4 }, (_, index) => motors?.[index]);
  return (
    <div className="motor-bars" aria-label="모터 출력">
      {values.map((value, index) => {
        const pct = typeof value === "number" ? Math.max(0, Math.min(100, ((value - 1000) / 1000) * 100)) : 0;
        const idle = value === 1000;
        return (
          <span key={index} className={value == null ? "" : idle ? "idle" : "active"}>
            <small>M{index + 1}</small>
            <b>{value ?? "--"}</b>
            <i aria-hidden="true"><em style={{ width: `${pct}%` }} /></i>
          </span>
        );
      })}
    </div>
  );
}

export default function HardwareControlBay({ addLog }: { addLog: (text: string, type?: LogType) => void }) {
  const [bleState, setBleState] = useState<BleState>("idle");
  const [deviceName, setDeviceName] = useState("");
  const [telemetry, setTelemetry] = useState<TelemetryState>(createTelemetryState);
  const [fc, setFc] = useState<FcStatus | undefined>();
  const [fcAt, setFcAt] = useState<number | undefined>();
  const [fcError, setFcError] = useState("");
  const [fcWatch, setFcWatch] = useState(true);
  const [tick, setTick] = useState(() => Date.now());
  const [open, setOpen] = useState(false);
  const [busy, setBusy] = useState("");
  const [confirmText, setConfirmText] = useState("");
  const [applyTarget, setApplyTarget] = useState<7 | 67 | 71>(71);
  const [challenge, setChallenge] = useState<Challenge | null>(null);
  const [applyResult, setApplyResult] = useState<ApplyResult | null>(null);
  const [killOffArmed, setKillOffArmed] = useState(false);
  const [approachArmed, setApproachArmed] = useState<"approach on" | "approach legacy" | null>(null);
  const [paramName, setParamName] = useState<ApproachParamName>("vel_kp");
  const [paramValue, setParamValue] = useState("");
  const [simOpen, setSimOpen] = useState(false);
  const [simLaunch,setSimLaunch]=useState<SimulatorLaunch>({initialMode:'approach'});
  const [panelTab, setPanelTab] = useState<"status" | "approach" | "safety">("status");
  const deviceRef = useRef<BluetoothDeviceLike | null>(null);
  const rxRef = useRef<GattCharacteristicLike | null>(null);
  const closeButton = useRef<HTMLButtonElement>(null);
  const fcInFlight = useRef(false);

  const commandAvailabilityAt = (now = Date.now()) => deriveBoardCommandAvailability({
    connected: bleState === "connected", telemetry, fc, fcAt, fcQueryFailed: Boolean(fcError), now,
  });

  useEffect(() => {
    const timer = window.setInterval(() => setTick(Date.now()), 1_000);
    return () => window.clearInterval(timer);
  }, []);

  useEffect(() => {
    const openSimulator = (event:Event) => { setSimLaunch((event as CustomEvent<SimulatorLaunch>).detail??{initialMode:'flight'});setOpen(false); setSimOpen(true); };
    window.addEventListener("spider:open-simulator", openSimulator);
    return () => window.removeEventListener("spider:open-simulator", openSimulator);
  }, []);

  const pollFc = useCallback(async () => {
    if (fcInFlight.current) return;
    fcInFlight.current = true;
    try {
      const status = await hardwareApi<FcStatus>("/api/hardware/fc/status");
      setFc(status);
      setFcAt(Date.now());
      setFcError("");
    } catch (error) {
      setFcError((error as Error).message);
    } finally {
      fcInFlight.current = false;
    }
  }, []);

  useEffect(() => {
    if (!fcWatch) return;
    void pollFc();
    const timer = window.setInterval(() => { void pollFc(); }, FC_POLL_MS);
    return () => window.clearInterval(timer);
  }, [fcWatch, pollFc]);

  useEffect(() => {
    if (!open) return;
    const previousFocus = document.activeElement instanceof HTMLElement ? document.activeElement : null;
    closeButton.current?.focus();
    const onKey = (event: globalThis.KeyboardEvent) => { if (event.key === "Escape" && !simOpen) setOpen(false); };
    window.addEventListener("keydown", onKey);
    return () => { window.removeEventListener("keydown", onKey); if (previousFocus?.isConnected) previousFocus.focus(); };
  }, [open, simOpen]);

  useEffect(() => {
    if (!killOffArmed) return;
    const timer = window.setTimeout(() => setKillOffArmed(false), 6_000);
    return () => window.clearTimeout(timer);
  }, [killOffArmed]);

  useEffect(() => {
    if (!approachArmed) return;
    const timer = window.setTimeout(() => setApproachArmed(null), 6_000);
    return () => window.clearTimeout(timer);
  }, [approachArmed]);

  const disconnectBle = useCallback((reason: string, type: LogType = "warn") => {
    rxRef.current = null;
    setBleState("idle");
    setChallenge(null);
    addLog(reason, type);
  }, [addLog]);

  const connect = async () => {
    const bluetooth = (navigator as Navigator & { bluetooth?: BluetoothLike }).bluetooth;
    if (!bluetooth) {
      setBleState("unsupported");
      addLog("Web Bluetooth를 지원하지 않는 브라우저입니다 (Chrome/Edge 필요)", "warn");
      return;
    }
    try {
      setBleState("connecting");
      const device = await bluetooth.requestDevice({
        filters: [{ namePrefix: "TagBoard" }],
        optionalServices: [NUS_SERVICE_UUID],
      });
      const server = await device.gatt?.connect();
      if (!server) throw new Error("GATT 서버에 연결하지 못했습니다.");
      const service = await server.getPrimaryService(NUS_SERVICE_UUID);
      const tx = await service.getCharacteristic(NUS_TX_UUID);
      const rx = await service.getCharacteristic(NUS_RX_UUID);
      await tx.startNotifications();
      const decoder = new TextDecoder();
      tx.addEventListener("characteristicvaluechanged", (event: Event) => {
        const value = (event.target as { value?: DataView }).value;
        if (!value) return;
        const chunk = decoder.decode(value.buffer.slice(value.byteOffset, value.byteOffset + value.byteLength));
        setTelemetry((previous) => consumeTelemetryChunk(previous, chunk, Date.now()));
      });
      const onDisconnected = () => {
        device.removeEventListener("gattserverdisconnected", onDisconnected);
        if (deviceRef.current === device) deviceRef.current = null;
        disconnectBle(`${device.name ?? "XIAO"} BLE 연결 끊김. 마지막 값은 stale로 표시됩니다`);
      };
      device.addEventListener("gattserverdisconnected", onDisconnected);
      deviceRef.current = device;
      rxRef.current = rx;
      setDeviceName(device.name ?? "");
      setTelemetry(createTelemetryState());
      setBleState("connected");
      addLog(`${device.name ?? "XIAO"} NUS 링크 연결`, "ok");
      await writeCommand(rx, "info");
    } catch (error) {
      setBleState("idle");
      rxRef.current = null;
      if ((error as Error).name !== "NotFoundError") addLog(`BLE 연결 실패: ${(error as Error).message}`, "warn");
    }
  };

  const writeLine = async (rx: GattCharacteristicLike, line: string) => {
    const bytes = new TextEncoder().encode(line);
    if (rx.writeValueWithoutResponse) await rx.writeValueWithoutResponse(bytes);
    else await rx.writeValue(bytes);
  };

  const writeCommand = async (rx: GattCharacteristicLike, command: BoardCommand, confirmed = false) => {
    await writeLine(rx, serializeBoardCommand(command, confirmed));
  };

  // approach set <이름> <값>: 이름·범위는 serializeApproachSet 허용목록에서 검사한다 (RAM 값, 재부팅 시 기본값)
  const sendApproachSet = async () => {
    const rx = rxRef.current;
    if (!rx) { addLog("XIAO BLE가 연결되지 않아 명령을 보내지 않았습니다", "warn"); return; }
    if (!commandAvailabilityAt().commandsEnabled || busy) { addLog("지상 제어 상태를 확인할 수 없어 파라미터를 보내지 않았습니다", "warn"); return; }
    setBusy("approach set");
    try {
      const line = serializeApproachSet(paramName, paramValue);
      await writeLine(rx, line);
      addLog(`접근 제어 파라미터 전송: ${line.trim()}`, "ok");
      await writeCommand(rx, "approach params");
    } catch (error) {
      addLog(`접근 제어 파라미터 거부: ${(error as Error).message}`, "warn");
    } finally {
      setBusy("");
    }
  };

  const sendBoard = async (command: BoardCommand, confirmed = false) => {
    const rx = rxRef.current;
    if (!rx) { addLog("XIAO BLE가 연결되지 않아 명령을 보내지 않았습니다", "warn"); return; }
    const available = commandAvailabilityAt();
    if (command !== "kill" && (!available.commandsEnabled || busy)) { addLog("지상 제어 상태를 확인할 수 없어 명령을 보내지 않았습니다", "warn"); return; }
    if ((command === "approach on" || command === "approach legacy") && !available.approachOutputEnabled) {
      addLog("접근 출력 게이트가 닫혀 명령을 보내지 않았습니다", "warn"); return;
    }
    setBusy(command);
    try {
      await writeCommand(rx, command, confirmed);
      addLog(`보드 명령 전송: ${BOARD_COMMANDS[command].label} (${command})`, command === "kill" ? "warn" : "ok");
    } catch (error) {
      addLog(`보드 명령 실패 (${command}): ${(error as Error).message}`, "warn");
    } finally {
      setBusy("");
    }
  };

  const requestChallenge = async (target: RamTarget) => {
    const currentPanel = commandAvailabilityAt().panel;
    if (!confirmOk || !(target === 64 ? currentPanel.canRestore64 : currentPanel.canApplyOverride)) return;
    setBusy(`challenge-${target}`);
    setApplyResult(null);
    try {
      const result = await hardwareApi<{ token: string; expires_in_s: number; mask: number | null }>(
        "/api/hardware/fc/override/challenge",
        { method: "POST", body: JSON.stringify({ target, confirmation: confirmText }) },
      );
      setChallenge({ target, token: result.token, expiresAt: Date.now() + result.expires_in_s * 1000 });
      addLog(`RAM 마스크 ${target} 확인 토큰 발급 (현재 ${result.mask ?? "--"}, ${result.expires_in_s}초 유효)`, "ok");
    } catch (error) {
      setChallenge(null);
      addLog(`RAM ${target} 확인 거부: ${(error as Error).message}`, "warn");
    } finally {
      setBusy("");
    }
  };

  const applyChallenge = async () => {
    const now = Date.now();
    const currentPanel = commandAvailabilityAt(now).panel;
    if (!challenge || challenge.expiresAt <= now || !(challenge.target === 64 ? currentPanel.canRestore64 : currentPanel.canApplyOverride)) return;
    setBusy("apply");
    try {
      const result = await hardwareApi<{ previous?: unknown; value?: unknown; verified?: boolean }>(
        "/api/hardware/fc/override/apply",
        { method: "POST", body: JSON.stringify({ target: challenge.target, token: challenge.token }) },
      );
      setApplyResult({ target: challenge.target, previous: result.previous, value: result.value, at: Date.now() });
      addLog(`RAM 마스크 ${challenge.target} 적용·되읽기 완료 (이전 ${String(result.previous ?? "--")})`, "ok");
    } catch (error) {
      addLog(`RAM ${challenge.target} 적용 실패: ${(error as Error).message}`, "warn");
    } finally {
      setChallenge(null);
      setBusy("");
      void pollFc();
    }
  };

  const latest = telemetry.latest;
  const connected = bleState === "connected";
  const boardLink = deriveBoardLinkState(connected, telemetry.lastReceivedAt, tick, BOARD_STALE_MS);
  const fcFresh = !fcError && freshness(fcAt, tick, FC_STALE_MS) === "fresh";
  const mspFresh = boardLink === "online" && freshness(latest.msp?.receivedAt, tick, 2_000) === "fresh";
  const commandAvailability = commandAvailabilityAt(tick);
  const panel = commandAvailability.panel;
  const authority = deriveAuthorityView(mspFresh ? latest.msp : undefined, tick);
  const rcCompare = deriveRcComparison(mspFresh ? latest.msp : undefined, 5, tick);
  const boardThrottle = deriveBoardThrottle(latest.msp);
  const mtf = deriveMtfHealth(latest.flow, fc);
  const flightLock = useMemo(() => deriveFlightLockReasons({ info: latest.info ?? latest.boot, mtf }), [latest.info, latest.boot, mtf]);
  const boardFc = latest.fc;
  const boardFcFresh = boardLink === "online" && boardFc?.status === true && freshness(boardFc.receivedAt, tick, 2_000) === "fresh";
  const armBlockers = fcFresh && Array.isArray(fc?.arm_blockers) ? fc.arm_blockers
    : boardFcFresh && Array.isArray(boardFc?.blockers) ? boardFc.blockers : undefined;
  const boardKillState = !mspFresh ? "미확인"
    : latest.msp?.kill === true || latest.msp?.kill_tx === true ? "KILL 송신 중"
      : latest.msp?.kill === false && latest.msp?.kill_tx === false ? "해제" : "미확인";
  const motors = fc?.motors_us?.length ? fc.motors_us : boardFc?.motor;
  const armed = fc?.armed ?? boardFc?.armed;
  const challengeLeft = challenge ? Math.max(0, Math.ceil((challenge.expiresAt - tick) / 1000)) : 0;
  const challengeAlive = Boolean(challenge) && challengeLeft > 0;
  const confirmOk = confirmText === OVERRIDE_CONFIRMATION;
  const commandsEnabled = commandAvailability.commandsEnabled && !busy;
  const killEnabled = commandAvailability.killEnabled && busy !== "kill";
  const ramLocked = !panel.groundReady;

  const bleTone: PillTone = boardLink === "online" ? "cyan" : boardLink === "stale" ? "amber" : "muted";
  const fcTone: PillTone = fc && fcFresh ? "cyan" : fcError ? "red" : fcAt ? "amber" : "muted";
  const uartTone: PillTone = latest.msp?.fc_link === true && boardLink === "online" ? "cyan" : latest.msp ? "red" : "muted";
  const armedTone = armed === true ? "bad" : armed === false ? "ok" : undefined;

  const info = latest.info ?? latest.boot;
  const vision = latest.vision;
  const approach = deriveApproachView(latest.approach, info, tick);
  const ap = latest.approach;
  const apParams = latest.approach_params?.params;
  const paramSpec = APPROACH_PARAMS[paramName];
  // on/legacy 는 보드가 롤·피치를 FC로 보내기 시작한다 → RAM 마스크와 같은 지상 시험 게이트 + 2단계 확인
  const approachOutputEnabled = commandAvailability.approachOutputEnabled && !busy;
  const fmt = (value: number | null | undefined, digits = 1, sign = false) =>
    value == null || !Number.isFinite(value) ? "--" : `${sign && value >= 0 ? "+" : ""}${value.toFixed(digits)}`;
  const vec = (pair: [number | null, number | null] | undefined, digits = 1) =>
    pair ? `${fmt(pair[0], digits, true)} / ${fmt(pair[1], digits, true)}` : "--";
  const rpy = (values: [number, number, number] | undefined) => values?.map((value) => fmt(value, 0)).join(" / ") ?? "--";
  const maskLabel = (mask: number | null | undefined) => mask == null ? "--" : `${mask} (${({ 7: "롤+피치+요 · 킬 제외", 64: "킬만", 67: "롤+피치+킬", 71: "롤+피치+요+킬" } as Record<number, string>)[mask] ?? "허용목록 밖"})`;
  const killPath = !fcFresh || fc?.mask == null ? "FC CH7 킬 전달 미확인" : (fc.mask & 64) === 0 ? "FC CH7 킬 전달 제외 · 조종기 회수 필요" : "FC CH7 킬 채널 포함 · 실제 정지 확인 필요";

  return (
    <>
      <div className="hw-bay" aria-label="실기기 제어 요약">
        <div className="hw-head">
          <span className="bay-label"><Cpu size={13} aria-hidden="true" /> 실기기 제어</span>
          <Pill tone={bleTone}><Bluetooth size={11} aria-hidden="true" /> {boardLink === "online" ? "BLE" : boardLink === "stale" ? "BLE STALE" : "BLE 없음"}</Pill>
          <Pill tone={fcTone}><Usb size={11} aria-hidden="true" /> {fc && fcFresh ? `FC ${fc.port}` : fcError ? "FC 오류" : "FC 없음"}</Pill>
          <Pill tone={uartTone}><Radio size={11} aria-hidden="true" /> {latest.msp?.fc_link === true ? "UART5" : latest.msp ? "UART5 끊김" : "UART5 --"}</Pill>
          <Pill tone={panel.groundReady ? "cyan" : "amber"}><Lock size={11} aria-hidden="true" /> {panel.groundReady ? "지상 시험 가능" : "비행 잠금"}</Pill>
        </div>
        <dl className="hw-stats">
          <Readout label="ARM" value={armed == null ? "--" : armed ? "ARMED" : "DISARM"} tone={armedTone} />
          <Readout label="THR" value={fc?.throttle_us ?? latest.msp?.rc_throttle ?? "--"} tone={(fc?.throttle_us ?? latest.msp?.rc_throttle ?? 0) > 1100 ? "warn" : undefined} />
          <Readout label="M1-4" value={motors ? motors.slice(0, 4).join(" ") : "--"} tone={motors?.slice(0, 4).some((value) => value !== 1000) ? "warn" : undefined} />
          <Readout label="CH6" value={authority.label} tone={authority.state === "manual" ? "warn" : authority.state === "unknown" ? "bad" : "ok"} />
          <Readout label="MASK" value={fc?.mask ?? "--"} tone={fc?.mask != null && fc.mask !== 64 ? "warn" : undefined} />
          <Readout label="VBAT" value={boardFc?.vbat != null ? `${boardFc.vbat.toFixed(2)}V` : "--"} />
          <Readout label="APPR" value={approach.label} tone={approach.tone === "red" ? "bad" : approach.tone === "amber" ? "warn" : approach.fresh ? "ok" : undefined} />
        </dl>
        <div className="hw-actions">
          <button type="button" className="button secondary" onClick={connect} disabled={bleState === "connecting"}>
            {bleState === "connecting" ? <RefreshCw className="spin" size={14} /> : <Bluetooth size={14} />}
            {bleState === "connecting" ? "페어링 중" : connected ? `${deviceName || "XIAO"} 재연결` : "XIAO 연결"}
          </button>
          <button type="button" className="button ghost" onClick={() => setOpen(true)} aria-haspopup="dialog" aria-expanded={open}>
            <PanelRightOpen size={14} /> 제어 패널
          </button>
          <button type="button" className="button secondary" onClick={() => { setSimLaunch({initialMode:'approach'});setOpen(false);setSimOpen(true); }} aria-haspopup="dialog" aria-expanded={simOpen} title="실기기 없이 접근 제어 알고리즘을 시연합니다">
            <Crosshair size={14} /> 접근 시뮬레이션
          </button>
          <button type="button" className="button danger" onClick={() => void sendBoard("kill")} disabled={!killEnabled} title={`보드 킬 요청 · ${killPath}`}>
            <OctagonX size={14} /> 킬
          </button>
        </div>
      </div>

      {open && (
        <aside className="control-drawer" role="dialog" aria-modal="false" aria-label="실기기 제어 패널">
          <div className="drawer-head">
            <span className="bay-label"><Cpu size={13} aria-hidden="true" /> 실기기 제어 패널</span>
            <Pill tone={panel.groundReady ? "cyan" : "amber"}>{panel.groundReady ? "지상 채널 시험 가능" : "지상 제어 잠김"}</Pill>
            <Pill tone="red"><Lock size={11} aria-hidden="true" /> 비행 출력 잠금</Pill>
            <button ref={closeButton} type="button" className="icon-button" onClick={() => setOpen(false)} aria-label="제어 패널 닫기"><X /></button>
          </div>
          <div className="hardware-lock-banner"><Lock size={15} /><div><strong>{panel.groundReady ? "지상 채널 시험 조건 충족" : "실기기 명령이 잠겨 있습니다"}</strong><span>{panel.groundReady ? "비행 출력은 계속 잠깁니다. 프로펠러 제거·디스아밍 상태에서만 시험하세요." : panel.reasons[0] ?? "최신 상태 보고와 안전 조건을 확인하세요."}</span></div><button type="button" onClick={() => setPanelTab("safety")}>조건 보기</button></div>
          <div className="hardware-tabs" role="tablist" aria-label="하드웨어 패널 보기">
            {([{ id: "status", label: "연결·상태", icon: Activity }, { id: "approach", label: "접근 제어", icon: Crosshair }, { id: "safety", label: "안전·지상 시험", icon: ShieldCheck }] as const).map((item, index, items) => <button key={item.id} id={`hardware-tab-${item.id}`} type="button" role="tab" aria-selected={panelTab === item.id} aria-controls="hardware-panel-content" tabIndex={panelTab === item.id ? 0 : -1} onClick={() => setPanelTab(item.id)} onKeyDown={(event) => {
              const nextIndex = event.key === "ArrowRight" ? (index + 1) % items.length : event.key === "ArrowLeft" ? (index + items.length - 1) % items.length : event.key === "Home" ? 0 : event.key === "End" ? items.length - 1 : -1;
              if (nextIndex < 0) return;
              event.preventDefault(); setPanelTab(items[nextIndex].id); document.getElementById(`hardware-tab-${items[nextIndex].id}`)?.focus();
            }}><item.icon size={15} />{item.label}</button>)}
          </div>
          <div className="drawer-body" id="hardware-panel-content" role="tabpanel" aria-labelledby={`hardware-tab-${panelTab}`}>
            <Card icon={Antenna} title="연결" hidden={panelTab !== "status"}>
              <dl className="hw-grid">
                <Readout label="XIAO BLE" value={connected ? `${deviceName || "TagBoard"} · ${boardLink === "online" ? "수신 중" : "STALE"}` : bleState === "unsupported" ? "브라우저 미지원" : "미연결"} tone={boardLink === "online" ? "ok" : connected ? "warn" : undefined} />
                <Readout label="FC USB" value={fc ? `${fc.port} · ${fc.identity.board?.board_name ?? "--"} ${fc.identity.variant ?? ""} ${fc.identity.firmware?.version ?? ""}` : fcError ? "연결 미확인 · USB와 포트를 확인하세요" : "조회 전"} tone={fc && fcFresh ? "ok" : fcError ? "bad" : undefined} />
                <Readout label="XIAO↔FC UART5" value={latest.msp ? `${latest.msp.fc_link ? "응답 OK" : "응답 없음"} · 보냄 ${latest.msp.sent ?? 0} 응답 ${latest.msp.acks ?? 0} 오류 ${latest.msp.errors ?? 0}` : "--"} tone={latest.msp?.fc_link === true ? "ok" : latest.msp ? "bad" : undefined} />
                <Readout label="보드 빌드" value={info?.build ?? "--"} />
              </dl>
              <div className="hw-inline">
                <label className="hw-toggle"><input type="checkbox" checked={fcWatch} onChange={(e) => setFcWatch(e.target.checked)} /> FC USB 3초 주기 조회</label>
                <button type="button" className="button ghost" onClick={() => void pollFc()} disabled={fcInFlight.current}><RefreshCw size={13} /> 지금 조회</button>
              </div>
              {fcError && <details className="hardware-diagnostic"><summary>FC 연결 진단 상세</summary><p>{fcError}</p></details>}
              {telemetry.errors.length > 0 && <p className="hw-note bad">BLE 파싱 오류 {telemetry.errors.length}건 · 마지막: {telemetry.errors[telemetry.errors.length - 1].message}</p>}
            </Card>

            <Card icon={Gauge} title="FC 상태" hidden={panelTab !== "status"} aside={fcAt ? <small>{fcFresh ? "LIVE" : "STALE"}</small> : null}>
              <dl className="hw-grid">
                <Readout label="무장" value={armed == null ? "--" : armed ? "ARMED" : "DISARMED"} tone={armedTone} />
                <Readout label="차단 사유" value={armBlockers === undefined ? "미확인" : armBlockers.length ? armBlockers.join(", ") : "없음"} />
                <Readout label="스로틀 / min_check" value={`${fc?.throttle_us ?? latest.msp?.rc_throttle ?? "--"} / ${fc?.min_check ?? "--"}`} tone={(fc?.throttle_us ?? 0) >= (fc?.min_check ?? Infinity) ? "warn" : undefined} />
                <Readout label="전압 · 전류" value={boardFc?.vbat != null ? `${boardFc.vbat.toFixed(2)}V (${boardFc.cells ?? "-"}S) · ${boardFc.amps?.toFixed(2) ?? "--"}A` : "--"} />
                <Readout label="RC 채널" value={fc?.rc?.channels_us ? fc.rc.channels_us.slice(0, 8).join(" ") : latest.msp?.fc_rc?.join(" ") ?? "--"} />
                <Readout label="보드 킬" value={boardKillState} tone={boardKillState === "KILL 송신 중" ? "bad" : undefined} />
                <Readout label="킬 전달 경로" value={killPath} tone={fcFresh && fc?.mask != null && (fc.mask & 64) !== 0 ? undefined : "warn"} />
              </dl>
              <MotorBars motors={motors} />
            </Card>

            <Card icon={Camera} title="센서" hidden={panelTab !== "status"}>
              <dl className="hw-grid">
                <Readout label="카메라" value={info ? `${info.camera_sensor ?? "--"} ${info.frame ?? ""}` : "--"} />
                <Readout label="비전" value={vision ? `${vision.fps?.toFixed(1) ?? "--"}fps · ${vision.detect_ms ?? "--"}ms · 태그 ${vision.ids?.length ? vision.ids.join(",") : "없음"} · ${vision.phase ?? ""}` : "--"} />
                <Readout label="MTF-01P" value={`${mtf.state === "healthy" ? "정상" : mtf.state === "degraded" ? "확인 필요" : "정보 없음"} · ${mtf.detail}`} tone={mtf.state === "healthy" ? "ok" : mtf.state === "degraded" ? "warn" : undefined} />
                <Readout label="높이" value={latest.flow?.height_cm != null ? `${latest.flow.height_cm.toFixed(0)}cm (${latest.flow.height_src ?? "board"})` : "--"} />
                <Readout label="FC 센서" value={fc?.sensors?.length ? fc.sensors.join(", ") : "--"} />
                <Readout label="축 검증" value={info?.axes_verified === true ? "완료" : "미검증"} tone={info?.axes_verified === true ? "ok" : "warn"} />
              </dl>
            </Card>

            <Card icon={Activity} title="제어권" hidden={panelTab !== "status"} aside={<Pill tone={authority.tone}>{authority.label}</Pill>}>
              <dl className="hw-grid">
                <Readout label="CH6" value={authority.detail} />
                <Readout label="CH5 ARM 스위치" value={latest.msp?.ch5 ?? "--"} tone={(latest.msp?.ch5 ?? 0) > 1100 ? "bad" : undefined} />
                <Readout label="RAM 마스크" value={maskLabel(fc?.mask)} tone={fc?.mask != null && fc.mask !== 64 ? "warn" : undefined} />
                <Readout label="보드 프레임 송신" value={latest.msp ? `${latest.msp.rc_tx === true ? "송신 중" : latest.msp.rc_tx === false ? "정지" : "미확인"} · ${fmt(latest.msp.tx_hz, 0)}Hz${latest.msp.tx_always ? " · CH6와 무관한 상시 송신" : ""}${mspFresh ? "" : " · 오래된 보고"}` : "--"} />
                <Readout label="FC 적용 제어권" value={authority.state === "auto" ? "AUTO · 마스크에 포함된 채널" : authority.state === "manual" ? "MANUAL · 조종기 제어, 보드 프레임 미적용" : "미확인"} />
                <Readout label="프레임 채널 매핑" value={latest.msp ? `${latest.msp.rcmap ?? "구형 AETR"} · ${latest.msp.rcmap_src === "fc" ? "FC 조회" : latest.msp.rcmap_src === "default" ? "기본값" : "출처 미확인"}` : "--"} />
                <Readout label="보드 스로틀 슬롯" value={latest.msp ? `슬롯 ${boardThrottle.slot ?? "?"} · ${fmt(boardThrottle.value, 0)}µs${boardThrottle.valid ? "" : " · 매핑/값 미확인"}` : "--"} tone={boardThrottle.valid && (boardThrottle.value ?? 1100) < 1100 ? undefined : "warn"} />
                <Readout label="보드 마스크 검사" value={latest.msp ? `${maskLabel(latest.msp.fc_mask)} · ${latest.msp.mask_ok === true ? "수평 제어 허용" : latest.msp.mask_ok === false ? "수평 제어 차단" : "미확인"} · 권장 ${latest.msp.mask_recommended ?? "--"}` : "--"} tone={latest.msp?.mask_ok === true ? undefined : "warn"} />
                <Readout label="조종기 기준점 (롤/피치/요)" value={`${rpy(latest.msp?.pilot_rpy)}µs · ${latest.msp?.pilot_valid ? "유효" : "미확인"}`} />
                <Readout label="현재 기준점 (롤/피치/요)" value={`${rpy(latest.msp?.base_rpy)}µs · AUTO 경과 ${latest.msp?.auto_since_ms ?? "--"}ms`} />
              </dl>
              <div className="rc-compare" aria-label="보드 송신값과 FC 적용값 비교">
                <span><small>보드 송신 롤/피치</small><b>{rcCompare.available ? `${rcCompare.board!.roll} / ${rcCompare.board!.pitch}` : "--"}</b></span>
                <span><small>FC 적용 롤/피치</small><b>{rcCompare.available ? `${rcCompare.fc!.roll} / ${rcCompare.fc!.pitch}` : latest.msp?.fc_rc ? `${latest.msp.fc_rc[0]} / ${latest.msp.fc_rc[1]}` : "--"}</b></span>
                <span><small>차이</small><b className={rcCompare.available ? (rcCompare.matches ? "ok" : "warn") : ""}>{rcCompare.available ? `${rcCompare.delta!.roll >= 0 ? "+" : ""}${rcCompare.delta!.roll} / ${rcCompare.delta!.pitch >= 0 ? "+" : ""}${rcCompare.delta!.pitch} ${rcCompare.matches ? "일치" : "불일치"}` : authority.state === "manual" ? "수동(비교 안 함)" : "--"}</b></span>
              </div>
              {latest.authority && <p className="hw-note">최근 CH6 전환 기록: {latest.authority.to === "auto" ? "AUTO" : latest.authority.to === "manual" ? "MANUAL" : "미확인"} · 프레임 나이 {latest.authority.frame_age_ms ?? "--"}ms · 송신 {rpy(latest.authority.out_rpy)}µs · 조종기 {rpy(latest.authority.pilot_rpy)}µs · {Math.max(0, Math.floor((tick - latest.authority.receivedAt) / 1000))}초 전. 현재 제어권은 위의 주기 보고로 확인합니다.</p>}
            </Card>

            <Card icon={Crosshair} title="접근 제어" hidden={panelTab !== "approach"} aside={<Pill tone={approach.tone}>{approach.label}</Pill>}>
              <dl className="hw-grid">
                <Readout label="상태" value={approach.detail} tone={approach.tone === "red" ? "bad" : approach.tone === "amber" ? "warn" : undefined} />
                <Readout label="태그" value={ap?.tag === "fresh" ? "추적 중" : ap?.tag === "stale" ? "오래됨" : ap ? "없음" : "--"} tone={ap?.tag === "fresh" ? "ok" : ap ? "warn" : undefined} />
                <Readout label="거리 · 오차(오른쪽 / 앞)" value={ap ? `${fmt(ap.dist_cm)}cm · ${fmt(ap.err_right_cm, 1, true)} / ${fmt(ap.err_fwd_cm, 1, true)}cm` : "--"} />
                <Readout label="목표 속도 · 벡터" value={ap ? `${fmt(ap.target_speed)}cm/s · ${vec(ap.v_target)}` : "--"} />
                <Readout label="광류 속도 (q)" value={ap ? `${vec(ap.v_flow)}cm/s (q${ap.flow_q ?? "--"})${ap.v_flow_valid ? "" : " · 무효"}` : "--"} tone={ap && ap.v_flow_valid === false ? "warn" : undefined} />
                <Readout label="태그 미분 속도" value={ap ? `${vec(ap.v_tag)}cm/s` : "--"} />
                <Readout label="롤 / 피치 명령" value={ap ? `${fmt(ap.roll_us, 1, true)} / ${fmt(ap.pitch_us, 1, true)}µs${approach.outputsToFc && mspFresh && authority.state === "auto" ? " (AUTO 적용 조건 충족)" : " (접근 출력 미적용/미확인)"}` : "--"} tone={approach.outputsToFc && ap && (Math.abs(ap.roll_us ?? 0) > 0.5 || Math.abs(ap.pitch_us ?? 0) > 0.5) ? "warn" : undefined} />
                <Readout label="높이 · FC 자세(롤/피치)" value={ap ? `${fmt(ap.height_cm, 0)}cm${ap.height_valid ? "" : "(무효)"} · ${vec(ap.att)}°` : "--"} />
                <Readout label="중립 · 과속 제동" value={ap ? `${ap.neutral === true ? "중립 수렴" : ap.neutral === false ? "중립 조건 아님" : "중립 미확인"} · ${ap.overspeed === true ? "과속 제동 중" : ap.overspeed === false ? "과속 없음" : "과속 미확인"}${approach.fresh ? "" : " · 오래된 보고"}` : "--"} tone={ap?.overspeed ? "warn" : undefined} />
                <Readout label="수직 속도 (관측만)" value={ap ? `${fmt(ap.vz_cm_s, 1, true)}cm/s · ${ap.vz_warn ? "수직 속도 경고" : "경고 없음"} · 스로틀 조작 없음` : "--"} tone={ap?.vz_warn ? "warn" : undefined} />
                <Readout label="송신 롤/피치/요 · 스로틀 슬롯" value={ap ? `${rpy(ap.out_rpy)}µs · ${fmt(ap.thr_slot_us, 0)}µs · ${ap.rc_tx ? "프레임 송신 중" : "송신 정지/미확인"}` : "--"} />
              </dl>
              {latest.warn?.code === "vz" && <p className="hw-note warn">최근 수직 속도 경고: {fmt(latest.warn.vz_cm_s, 1, true)}cm/s (기준 ±{fmt(latest.warn.limit_cm_s, 0)}) · 높이 {fmt(latest.warn.height_cm, 0)}cm ({latest.warn.height_src ?? "미확인"}) · {latest.warn.throttle_action === "none" ? "스로틀 조작 없음" : "조작 여부 미확인"} · {Math.max(0, Math.floor((tick - latest.warn.receivedAt) / 1000))}초 전 · 누적 {latest.msp?.vz_warn_count ?? "--"}회.</p>}
              {latest.approach_mode?.warning && <p className="hw-note warn">모드 응답 경고: {latest.approach_mode.warning} · FC 마스크 {latest.approach_mode.fc_mask ?? "미확인"}</p>}
              <div className="hw-commands">
                <button type="button" className="button secondary" disabled={!commandsEnabled || approach.mode === "shadow"} onClick={() => void sendBoard("approach shadow")}>shadow (계산만)</button>
                <button type="button" className="button ghost" disabled={!commandsEnabled || approach.mode === "off"} onClick={() => void sendBoard("approach off")}>끔</button>
                <button type="button" className="button ghost" disabled={!commandsEnabled} onClick={() => void sendBoard("approach reset")}>적분·명령 초기화</button>
                <button type="button" className="button ghost" disabled={!commandsEnabled} onClick={() => void sendBoard("approach params")}>파라미터 조회</button>
                <button type="button" className="button ghost" onClick={() => { setSimLaunch({initialMode:'approach'});setOpen(false);setSimOpen(true); }}><Crosshair size={13} /> 시뮬레이션</button>
                {approachArmed
                  ? <button type="button" className="button approve" disabled={!approachOutputEnabled} onClick={() => { const cmd = approachArmed; setApproachArmed(null); void sendBoard(cmd, true); }}><AlertTriangle size={13} /> {approachArmed === "approach on" ? "출력 켬" : "legacy"} 확정 (6초 내)</button>
                  : (
                    <>
                      <button type="button" className="button secondary" disabled={!approachOutputEnabled || approach.mode === "on"} onClick={() => setApproachArmed("approach on")} title="접근 제어 출력을 켭니다. CH6 AUTO + 마스크 7/67/71에서 수평 제어 반영, 스로틀은 조종기 소유입니다.">출력 켬(on)…</button>
                      <button type="button" className="button ghost" disabled={!approachOutputEnabled || approach.mode === "legacy"} onClick={() => setApproachArmed("approach legacy")}>기존 위치 제어(legacy)…</button>
                    </>
                  )}
              </div>
              <div className="ram-form">
                <label>파라미터 (RAM, 재부팅 시 기본값)
                  <div className="hw-inline">
                    <select value={paramName} onChange={(e) => { setParamName(e.target.value as ApproachParamName); setParamValue(""); }} aria-label="접근 제어 파라미터 이름">
                      {Object.entries(APPROACH_PARAMS).map(([name, spec]) => (
                        <option key={name} value={name}>{spec.label} ({name}){apParams?.[name] != null ? ` = ${apParams[name]}` : ""}</option>
                      ))}
                    </select>
                    <input type="number" inputMode="decimal" value={paramValue} min={paramSpec.min} max={paramSpec.max} step={paramSpec.step} placeholder={apParams?.[paramName] != null ? String(apParams[paramName]) : `${paramSpec.min}~${paramSpec.max}`} onChange={(e) => setParamValue(e.target.value)} aria-label="접근 제어 파라미터 값" />
                    <button type="button" className="button secondary" disabled={!commandsEnabled || paramValue.trim() === ""} onClick={() => void sendApproachSet()}>적용</button>
                  </div>
                </label>
                <p className="hw-note">
                  {paramSpec.label}: {paramSpec.min}~{paramSpec.max} {paramSpec.unit}. 현재 {apParams?.[paramName] ?? "조회 전"}.
                  {" "}프레임은 CH6와 무관하게 50Hz 상시 송신하고, shadow에서는 접근 오프셋을 적용하지 않습니다. on/legacy는 지상 시험 게이트와 최신 접근 보고가 있어야 켤 수 있습니다. 수동(CH6 2000)에서는 FC가 조종기 제어로 돌아갑니다. 태그 상실은 기본 중립 수렴, lost_neutral=0은 광류 제동이며 광류·높이·마스크·속도 이상은 중지합니다. 최대 접근 속도 상한은 10cm/s입니다.
                </p>
              </div>
            </Card>

            <Card icon={ShieldCheck} title="안전 게이트" hidden={panelTab !== "safety"} aside={<Pill tone={panel.groundReady ? "cyan" : "amber"}>{panel.groundReady ? "통과" : `${panel.reasons.length}건 차단`}</Pill>}>
              <ul className="gate-list">
                {(fc?.gates ?? []).map((gate) => (
                  <li key={gate.key} className={gate.passed ? "pass" : "fail"}>
                    {gate.passed ? <Check size={12} aria-hidden="true" /> : <AlertTriangle size={12} aria-hidden="true" />}
                    <span>{gate.passed ? "통과" : "차단"} · {gate.label}: {gate.detail}{!gate.passed && gate.remediation && <small>→ {gate.remediation}</small>}</span>
                  </li>
                ))}
                {panel.reasons.filter((reason) => !(fc?.gates ?? []).some((gate) => reason.startsWith(gate.label))).map((reason) => (
                  <li key={reason} className="fail"><AlertTriangle size={12} aria-hidden="true" /><span>차단 · {reason}</span></li>
                ))}
                {!fc && panel.reasons.length === 0 && <li className="muted"><span>NO DATA</span></li>}
              </ul>
              <p className="hw-note"><Lock size={11} aria-hidden="true" /> 비행 출력 잠금 사유</p>
              <ul className="gate-list lock">
                {flightLock.map((reason) => <li key={reason} className="fail"><Lock size={12} aria-hidden="true" /><span>{reason}</span></li>)}
              </ul>
            </Card>

            <Card icon={Radio} title="보드 명령" hidden={panelTab !== "safety"} aside={<small>{boardLink === "online" ? "허용목록만 전송" : "BLE 온라인 필요"}</small>}>
              <div className="hw-commands">
                <button type="button" className="button ghost" disabled={!commandsEnabled} onClick={() => void sendBoard("info")}>상태 요청</button>
                <button type="button" className="button secondary" disabled={!commandsEnabled} onClick={() => void sendBoard("msp on")}>MSP 제어 시작</button>
                <button type="button" className="button secondary" disabled={!commandsEnabled} onClick={() => void sendBoard("msp off")}>MSP 제어 중지</button>
                <button type="button" className="button ghost" disabled={!commandsEnabled} onClick={() => void sendBoard("mission reset")}>미션 초기화</button>
                <button type="button" className="button danger" disabled={!killEnabled} onClick={() => void sendBoard("kill")}><OctagonX size={13} /> 비상 정지(kill)</button>
                {killOffArmed
                  ? <button type="button" className="button approve" disabled={!commandsEnabled} onClick={() => { setKillOffArmed(false); void sendBoard("kill off", true); }}><AlertTriangle size={13} /> 해제 확정 (6초 내)</button>
                  : <button type="button" className="button ghost" disabled={!commandsEnabled || !(latest.msp?.kill || latest.msp?.kill_tx)} onClick={() => setKillOffArmed(true)}>비상 정지 해제…</button>}
              </div>
              <p className="hw-note">아밍·스로틀·모터 시험·원시 MSP 명령은 이 화면에서 보낼 수 없습니다. {killPath}. 킬 요청 뒤 FC 무장·모터 상태를 직접 확인하세요.</p>
            </Card>

            <Card icon={Battery} title="RAM 마스크 시험" hidden={panelTab !== "safety"} aside={<Pill tone={ramLocked ? "amber" : "cyan"}>{ramLocked ? "게이트 닫힘" : "준비"}</Pill>}>
              <div className="ram-form">
                <label>확인 문구 <code>{OVERRIDE_CONFIRMATION}</code>
                  <input value={confirmText} onChange={(e) => setConfirmText(e.target.value)} placeholder={OVERRIDE_CONFIRMATION} aria-invalid={confirmText !== "" && !confirmOk} autoComplete="off" />
                </label>
                <label>적용할 수평 제어 마스크
                  <select value={applyTarget} onChange={(event) => setApplyTarget(Number(event.target.value) as 7 | 67 | 71)} disabled={challengeAlive || busy !== ""} aria-label="RAM 마스크 선택">
                    <option value={71}>71 · 롤+피치+요+킬</option>
                    <option value={7}>7 · 롤+피치+요 (CH7 킬 제외)</option>
                    <option value={67}>67 · 롤+피치+킬 (기존)</option>
                  </select>
                </label>
                <div className="hw-commands">
                  <button type="button" className="button secondary" disabled={!confirmOk || !panel.canApplyOverride || busy !== "" || challengeAlive} onClick={() => void requestChallenge(applyTarget)}>
                    {busy === `challenge-${applyTarget}` ? <RefreshCw className="spin" size={13} /> : <ShieldCheck size={13} />} {applyTarget} 적용 요청
                  </button>
                  <button type="button" className="button secondary" disabled={!confirmOk || !panel.canRestore64 || busy !== "" || challengeAlive} onClick={() => void requestChallenge(64)}>
                    {busy === "challenge-64" ? <RefreshCw className="spin" size={13} /> : <ShieldCheck size={13} />} 64 복구 요청
                  </button>
                </div>
                {challenge && (
                  <div className="ram-confirm" role="status">
                    <span>마스크 <b>{challenge.target}</b> 토큰 {challengeAlive ? `${challengeLeft}초 남음` : "만료"}</span>
                    <button type="button" className="button approve" disabled={!challengeAlive || busy !== "" || !(challenge.target === 64 ? panel.canRestore64 : panel.canApplyOverride)} onClick={() => void applyChallenge()}>
                      {busy === "apply" ? <RefreshCw className="spin" size={13} /> : <Check size={13} />} RAM 적용 확정
                    </button>
                    <button type="button" className="button ghost" onClick={() => setChallenge(null)}>취소</button>
                  </div>
                )}
                {applyResult && (
                  <p className="hw-note ok">마스크 {String(applyResult.previous ?? "--")} → {String(applyResult.value ?? applyResult.target)} RAM 적용·되읽기 완료 ({new Date(applyResult.at).toLocaleTimeString("ko-KR", { hour12: false })}). EEPROM 저장 없음, 전원 재인가 시 FC에 저장된 값으로 복귀.</p>
                )}
                <p className="hw-note">
                  {ramLocked
                    ? `차단: ${panel.reasons[0] ?? "조건 미충족"}`
                    : "RAM에만 기록합니다. 적용 뒤 CH6=1000(AUTO)에서 태그를 움직여 롤·피치 반영을 비교하고, CH6=2000 또는 MSP 중지로 회수한 뒤 64를 복구하세요."}
                  {" "}모든 허용 마스크에서 스로틀 비트 8은 제외합니다. 7은 CH7 킬 전달도 제외하므로 킬 버튼만으로 FC 정지를 보장하지 않습니다.
                </p>
              </div>
            </Card>
          </div>
        </aside>
      )}

      {simOpen && <Suspense fallback={<div className="sim-loading" role="status"><RefreshCw className="spin" /><strong>가상 시험 공간 준비 중</strong><span>3D 렌더러와 제어기를 불러옵니다.</span><button className="button ghost" onClick={() => setSimOpen(false)}>취소</button></div>}><Simulator launch={simLaunch} onClose={() => setSimOpen(false)} /></Suspense>}
    </>
  );
}
