import {t, useLanguage} from "./i18n";
import { useCallback, useEffect, useRef, useState, type PointerEvent, type ReactNode, type CSSProperties } from 'react';
import { Focus, Maximize2, Pause, Play, Power, Crosshair } from 'lucide-react';
import type { SimulationView } from './SimulationScene';
import { normalizeStick, type CameraInput } from './simulation/flightVisuals';
import { NEUTRAL_INPUT, type PilotInput } from './simulation/pilotFlight';
import './remoteController.css';

function Stick({ side, label, onChange, value, flight=false }: { side:'left'|'right'; label:string; onChange:(x:number,y:number)=>void;value?:{x:number;y:number};flight?:boolean }) {
  useLanguage();
  const [position,setPosition]=useState({x:0,y:0});const held=useRef<number|null>(null);const latest=useRef(onChange);latest.current=onChange;
  const reset=useCallback(()=>{held.current=null;setPosition({x:0,y:0});latest.current(0,0);},[]);
  useEffect(()=>{window.addEventListener('blur',reset);return()=>window.removeEventListener('blur',reset);},[reset]);
  const move=(e:PointerEvent<HTMLDivElement>)=>{
    const bounds=e.currentTarget.getBoundingClientRect();const value=normalizeStick((e.clientX-bounds.left-bounds.width/2)/(bounds.width*.36),(e.clientY-bounds.top-bounds.height/2)/(bounds.height*.36));
    setPosition(value);latest.current(value.x,value.y);
  };
  return <div className={`remote-stick remote-stick--${side}${held.current!==null?' is-held':''}`} role="group" tabIndex={0} aria-label={t(label)} aria-describedby="remote-stick-guide" title={t(label)}
    onPointerDown={(e)=>{if(e.button!==0)return;e.preventDefault();if(e.pointerType!=='touch')e.currentTarget.focus();held.current=e.pointerId;e.currentTarget.setPointerCapture(e.pointerId);move(e);}}
    onPointerMove={(e)=>{if(held.current===e.pointerId)move(e);}}
    onPointerUp={(e)=>{if(held.current===e.pointerId){reset();if(e.currentTarget.hasPointerCapture(e.pointerId))e.currentTarget.releasePointerCapture(e.pointerId);}}}
    onPointerCancel={reset} onLostPointerCapture={reset} onBlur={reset}
    onKeyDown={(e)=>{if(flight||!['ArrowLeft','ArrowRight','ArrowUp','ArrowDown'].includes(e.key))return;e.preventDefault();const x=e.key==='ArrowLeft'?-1:e.key==='ArrowRight'?1:0,y=e.key==='ArrowUp'?-1:e.key==='ArrowDown'?1:0;setPosition({x,y});latest.current(x,y);}}
    onKeyUp={(e)=>{if(!flight&&e.key.startsWith('Arrow')){e.preventDefault();reset();}}}>
    <span className="remote-stick-cap" style={{'--stick-x':`${(value??position).x*80}%`,'--stick-y':`${(value??position).y*80}%`} as CSSProperties}><i/></span>
  </div>;
}

interface Props {
  active:boolean;children:ReactNode;running:boolean;speed:number;distance:number;height:number;status:string;
  onPlayback:()=>void;onView:(view:SimulationView)=>void;onZoom:(factor:number)=>void;onReset:()=>void;onExpand:()=>void;onClose:()=>void;
  onCameraInput:(input:CameraInput)=>void;
  pilotInput?:PilotInput;onPilotInput?:(input:PilotInput)=>void;metricUnits?:boolean;heading?:number;
}
export default function RemoteController(p:Props) {
  useLanguage();
  const input=useRef<CameraInput>({panX:0,panY:0,rotateX:0,rotateY:0});const [artFailed,setArtFailed]=useState(false);
  const pilot=useRef<PilotInput>({...NEUTRAL_INPUT});
  useEffect(()=>{if(!p.active){input.current={panX:0,panY:0,rotateX:0,rotateY:0};p.onCameraInput(input.current);}},[p.active,p.onCameraInput]);
  useEffect(()=>{if(!p.active){pilot.current={...NEUTRAL_INPUT};p.onPilotInput?.(pilot.current);}},[p.active,p.onPilotInput]);
  const stick=(side:'left'|'right',x:number,y:number)=>{
    if(p.onPilotInput){pilot.current=side==='left'?{...pilot.current,yaw:x,climb:-y}:{...pilot.current,right:x,forward:-y};p.onPilotInput(pilot.current);return;}
    input.current=side==='left'?{...input.current,panX:x,panY:y}:{...input.current,rotateX:x,rotateY:y};
    if(x||y)p.onView('orbit');p.onCameraInput(input.current);
  };
  return <div className={`remote-housing${artFailed?' has-fallback-art':''}`}>
    <div className="remote-device-art" hidden={!p.active} aria-hidden="true">
      <img src="/controller/spider-controller-frame-v2.png" alt="" draggable={false} onError={()=>setArtFailed(true)}/>
      <svg className="remote-brand-plate" viewBox="0 0 160 32" focusable="false">
        <rect x=".5" y=".5" width="159" height="31" rx="4" fill="#252720" stroke="#383b31"/>
        <text x="82" y="22" textAnchor="middle" fill="#aab39a" fontSize="19" fontWeight="700" letterSpacing="3">MANGO</text>
      </svg>
    </div>
    <div className="remote-display" tabIndex={p.onPilotInput?0:undefined} aria-label={t(p.onPilotInput?'드론 비행 화면: WASD와 방향키로 조종':undefined)} onPointerDown={p.onPilotInput?(e)=>{if(e.target instanceof HTMLCanvasElement)e.currentTarget.focus({preventScroll:true});}:undefined}>
      {p.children}
      {p.active&&<div className="remote-screen-hud">
        <div className="remote-screen-status"><span><i/>MANGO 01</span><small>SIM · {t(p.status)}</small><button type="button" onClick={p.onExpand} aria-label={t("조종기 디스플레이 확대")} title={t("디스플레이 확대")}><Maximize2 size={13}/></button></div>
        <div className="remote-screen-telemetry"><span><small>{t("속도")}</small><b>{t(p.speed.toFixed(1))}<em>{t(p.metricUnits?'m/s':'cm/s')}</em></b></span><span><small>{t(p.metricUnits?'다음 경유점':'태그 거리')}</small><b>{t(p.distance.toFixed(1))}<em>{t(p.metricUnits?'m':'cm')}</em></b></span><span><small>{t("고도")}</small><b>{t(p.height.toFixed(p.metricUnits?1:0))}<em>{t(p.metricUnits?'m':'cm')}</em></b></span>{p.heading!==undefined&&<span><small>{t("방위각")}</small><b>{t(p.heading.toFixed(0))}<em>°</em></b></span>}</div>
      </div>}
    </div>
    {p.active&&<div className="remote-hardware-controls">
      <Stick side="left" label={t(p.onPilotInput?'왼쪽 스틱: 상승·하강·기체 회전':'왼쪽 스틱: 카메라 이동')} flight={!!p.onPilotInput} value={p.pilotInput?{x:p.pilotInput.yaw,y:-p.pilotInput.climb}:undefined} onChange={(x,y)=>stick('left',x,y)}/>
      <Stick side="right" label={t(p.onPilotInput?'오른쪽 스틱: 기체 전후·좌우 이동':'오른쪽 스틱: 카메라 회전')} flight={!!p.onPilotInput} value={p.pilotInput?{x:p.pilotInput.right,y:-p.pilotInput.forward}:undefined} onChange={(x,y)=>stick('right',x,y)}/>
      <button type="button" className="remote-round remote-follow" onClick={()=>p.onView('follow')} aria-label={t("조종기 기체 추적")} title={t("기체 추적")}><Focus/></button>
      <button type="button" className="remote-round remote-play" onClick={p.onPlayback} aria-label={t(p.running?'조종기 일시정지':'조종기 재생')} title={t(p.running?'일시정지':'재생')}>{p.running?<Pause fill="currentColor"/>:<Play fill="currentColor"/>}</button>
      <button type="button" className="remote-key remote-l1" onClick={()=>p.onZoom(1/1.25)} aria-label={t("조종기 축소")} title={t("축소")}>−</button>
      <button type="button" className="remote-key remote-l2" onClick={()=>p.onZoom(1.25)} aria-label={t("조종기 확대")} title={t("확대")}>+</button>
      <button type="button" className="remote-key remote-l3" onClick={p.onReset} aria-label={t("조종기 시점 초기화")} title={t("시점 초기화")}><Crosshair/></button>
      <button type="button" className="remote-key remote-r1" onClick={()=>p.onView('orbit')} aria-label={t("조종기 전체 시점")} title={t("전체 시점")}>3D</button>
      <button type="button" className="remote-key remote-r2" onClick={()=>p.onView('top')} aria-label={t("조종기 상공 시점")} title={t("상공 시점")}>TOP</button>
      <button type="button" className="remote-key remote-r3" onClick={()=>p.onView('camera')} aria-label={t("조종기 하향 카메라")} title={t("하향 카메라")}>CAM</button>
      <button type="button" className="remote-round remote-expand" onClick={p.onExpand} aria-label={t("조종기 화면 전체 확대")} title={t("화면 전체 확대")}><Maximize2/></button>
      <button type="button" className="remote-round remote-power" onClick={p.onClose} aria-label={t("조종기 전원: 시뮬레이터 닫기")} title={t("시뮬레이터 닫기")}><Power/></button>
      <div className="remote-operation-help" id="remote-stick-guide"><span><i/>{t("가상 조종기")}</span><p>{t(p.onPilotInput?'W/S 고도 · A/D 회전 · 방향키 이동 · Space 제동':'왼쪽 스틱 이동 · 오른쪽 스틱 회전 · +/− 확대')}</p><button type="button" onClick={p.onExpand}>{t("디스플레이 확대 ")}<Maximize2 size={12}/></button></div>
    </div>}
  </div>;
}
