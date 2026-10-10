import {useCallback,useEffect,useMemo,useRef,useState} from 'react';
import {Activity,Box,Camera,Crosshair,EyeOff,Focus,Gamepad2,Keyboard,LayoutDashboard,Maximize2,PanelRightClose,PanelRightOpen,Pause,Play,RotateCcw,Route,Sun,Sunset,X} from 'lucide-react';
import SimulationScene,{type SimulationView} from './SimulationScene';
import RemoteController from './RemoteController';
import {flightSceneFrame,keyboardPilotInput,mergePilotInputs,NEUTRAL_INPUT,PILOT_CODES,trainingRoute,waypointDistance,type FlightRoute,type FlightSnapshot,type PilotFlight,type PilotInput} from './simulation/pilotFlight';
import type {CameraInput} from './simulation/flightVisuals';
import './simulator.css';
import './pilotSimulator.css';

const NO_CAMERA:CameraInput={panX:0,panY:0,rotateX:0,rotateY:0};
const noop=()=>{};
const KEYS=[['KeyW','W'],['KeyA','A'],['KeyS','S'],['KeyD','D'],['ArrowUp','↑'],['ArrowLeft','←'],['ArrowDown','↓'],['ArrowRight','→']] as const;

function RouteMap({flight:s}:{flight:FlightSnapshot}) {
  const all=[...s.route.points,...s.trail,s];
  const minX=Math.min(...all.map(p=>p.x))-1,maxX=Math.max(...all.map(p=>p.x))+1,minY=Math.min(...all.map(p=>p.y))-1,maxY=Math.max(...all.map(p=>p.y))+1;
  const size=Math.max(maxX-minX,(maxY-minY)*1.65),cx=(minX+maxX)/2,cy=(minY+maxY)/2;
  const point=(p:{x:number;y:number})=>`${p.x},${-p.y}`;
  return <svg className="pilot-route-map" viewBox={`${cx-size/2} ${-cy-size/3.3} ${size} ${size/1.65}`} role="img" aria-label="계획 경로와 실제 기체 이동 궤적">
    <path className="pilot-map-grid" d={`M ${cx-size/2},${-cy} h ${size} M ${cx},${-cy-size/3.3} v ${size/1.65}`} vectorEffect="non-scaling-stroke"/>
    <polyline className="pilot-map-plan" points={s.route.points.map(point).join(' ')} vectorEffect="non-scaling-stroke"/>
    <polyline className="pilot-map-trail" points={s.trail.map(point).join(' ')} vectorEffect="non-scaling-stroke"/>
    {s.route.points.map((p,i)=>{
      if(s.route.points.slice(0,i).some(v=>v.x===p.x&&v.y===p.y))return null;
      const indices=s.route.points.flatMap((v,j)=>v.x===p.x&&v.y===p.y?[j]:[]);
      return <g key={i} transform={`translate(${p.x},${-p.y})`}><circle r={size*.013} className={indices.includes(s.waypoint)&&!s.completed?'is-current':i<s.waypoint||s.completed?'is-visited':''}/>{(s.route.points.length<=8||i===0||indices.includes(s.waypoint))&&<text x={size*.024} y={-size*.018} fontSize={size*.034}>{indices.map(j=>String(j+1).padStart(2,'0')).join('/')}</text>}</g>;
    })}
    <g transform={`translate(${s.x},${-s.y}) rotate(${s.yaw*180/Math.PI})`}><path className="pilot-map-aircraft" d={`M 0,${-size*.031} L ${size*.02},${size*.025} L 0,${size*.012} L ${-size*.02},${size*.025} Z`}/></g>
  </svg>;
}

export default function PilotSimulator({engine,mission,missionUnavailable,onClose,onApproach}:{engine:PilotFlight;mission:FlightRoute|null;missionUnavailable:boolean;onClose:()=>void;onApproach:()=>void}) {
  const [flight,setFlight]=useState(()=>engine.snapshot());
  const [running,setRunning]=useState(()=>!matchMedia('(prefers-reduced-motion: reduce)').matches);
  const [view,setView]=useState<SimulationView>('follow');
  const [controller,setController]=useState(true),[dashboard,setDashboard]=useState(true),[panel,setPanel]=useState(()=>innerWidth>900);
  const [daylight,setDaylight]=useState<'day'|'dusk'>('day'),[quality,setQuality]=useState<'high'|'standard'>('high');
  const [zoom,setZoom]=useState({sequence:0,factor:1}),[reset,setReset]=useState(0),[showPath,setShowPath]=useState(true);
  const [keys,setKeys]=useState<string[]>([]),[notice,setNotice]=useState('');
  const keyboard=useRef(new Set<string>()),pointer=useRef<PilotInput>({...NEUTRAL_INPUT}),input=useRef<PilotInput>({...NEUTRAL_INPUT});
  const dialog=useRef<HTMLDivElement>(null),close=useRef<HTMLButtonElement>(null),restore=useRef<HTMLButtonElement>(null),sensor=useRef<HTMLCanvasElement>(null);
  const publish=useCallback(()=>setFlight(engine.snapshot()),[engine]);
  const syncInput=useCallback(()=>{input.current=mergePilotInputs(keyboardPilotInput(keyboard.current),pointer.current);setKeys([...keyboard.current]);},[]);
  const clearInput=useCallback(()=>{keyboard.current.clear();pointer.current={...NEUTRAL_INPUT};syncInput();},[syncInput]);
  const onStick=useCallback((value:PilotInput)=>{pointer.current=value;syncInput();},[syncInput]);
  const requestZoom=(factor:number)=>setZoom(v=>({sequence:v.sequence+1,factor}));
  const pause=()=>{clearInput();publish();setRunning(v=>!v);setNotice('');};
  const resetFlight=(route=engine.state.route)=>{clearInput();engine.reset(route);publish();setReset(v=>v+1);setRunning(true);setNotice('출발 위치에서 다시 시작했습니다.');};
  const manual=()=>{clearInput();engine.manual();publish();setRunning(true);setNotice('기체 기준으로 전후·좌우 이동합니다. 키를 놓으면 제동 후 호버합니다.');};
  const follow=()=>{clearInput();engine.followRoute();publish();setRunning(true);setNotice('경유점을 순서대로 통과합니다. 조종 키를 누르면 수동으로 전환됩니다.');};
  const brake=()=>{clearInput();engine.brake();publish();setNotice('제동 · 현재 위치에서 호버합니다.');};
  const actions=useRef({onClose,pause,resetFlight,brake,dashboard,running});actions.current={onClose,pause,resetFlight,brake,dashboard,running};
  useEffect(()=>{
    const previousOverflow=document.body.style.overflow;document.body.style.overflow='hidden';close.current?.focus();
    const keyDown=(event:KeyboardEvent)=>{
      if(event.key==='Escape'){event.preventDefault();if(!actions.current.dashboard)setDashboard(true);else actions.current.onClose();return;}
      const target=event.target as HTMLElement|null;
      if(event.metaKey||event.ctrlKey||event.altKey||event.isComposing||target?.closest('input,select,textarea,[contenteditable=true]'))return;
      if(PILOT_CODES.has(event.code)){
        event.preventDefault();if(!keyboard.current.has(event.code)){keyboard.current.add(event.code);syncInput();}return;
      }
      if(event.repeat)return;
      if(event.code==='KeyP'){event.preventDefault();actions.current.pause();}
      if(event.code==='KeyR'){event.preventDefault();actions.current.resetFlight();}
      if(event.code==='KeyH'){event.preventDefault();setDashboard(v=>!v);}
      if(event.key!=='Tab')return;
      const list=Array.from(dialog.current?.querySelectorAll<HTMLElement>('button:not(:disabled),select,input,[tabindex="0"]')??[]).filter(el=>el.offsetParent!==null);
      if(event.shiftKey&&(document.activeElement===list[0]||!dialog.current?.contains(document.activeElement))){event.preventDefault();list.at(-1)?.focus();}
      else if(!event.shiftKey&&(document.activeElement===list.at(-1)||!dialog.current?.contains(document.activeElement))){event.preventDefault();list[0]?.focus();}
    };
    const keyUp=(event:KeyboardEvent)=>{if(keyboard.current.delete(event.code)){event.preventDefault();syncInput();}};
    const focusInput=(event:FocusEvent)=>{if((event.target as HTMLElement)?.closest('input,select,textarea,[contenteditable=true]'))clearInput();};
    const loseFocus=()=>{clearInput();publish();setRunning(false);setNotice('화면을 벗어나 일시정지했습니다. P 또는 재생 버튼으로 계속합니다.');};
    const visibility=()=>{if(document.hidden)loseFocus();};
    window.addEventListener('keydown',keyDown);window.addEventListener('keyup',keyUp);window.addEventListener('blur',loseFocus);document.addEventListener('visibilitychange',visibility);document.addEventListener('focusin',focusInput);
    if(import.meta.env.DEV)(window as unknown as {spiderPilot?:PilotFlight}).spiderPilot=engine;
    return()=>{clearInput();document.body.style.overflow=previousOverflow;window.removeEventListener('keydown',keyDown);window.removeEventListener('keyup',keyUp);window.removeEventListener('blur',loseFocus);document.removeEventListener('visibilitychange',visibility);document.removeEventListener('focusin',focusInput);if(import.meta.env.DEV)delete (window as unknown as {spiderPilot?:PilotFlight}).spiderPilot;};
  },[clearInput,syncInput,engine,publish]);
  useEffect(()=>{if(!dashboard)restore.current?.focus();else if(document.activeElement===document.body)close.current?.focus();},[dashboard]);
  useEffect(()=>{
    if(!running)return;let raf=0,previous=0,accumulator=0,lastPublished=0;
    const tick=(now:number)=>{
      if(previous)accumulator+=Math.min((now-previous)/1000,.15);previous=now;
      while(accumulator>=1/60){engine.step(input.current,1/60);accumulator-=1/60;}
      if(now-lastPublished>=50){publish();lastPublished=now;}
      raf=requestAnimationFrame(tick);
    };raf=requestAnimationFrame(tick);return()=>cancelAnimationFrame(raf);
  },[running,engine,publish]);
  const frame=useMemo(()=>flightSceneFrame(flight),[flight]);
  const heading=(flight.yaw*180/Math.PI+360)%360,speed=Math.hypot(flight.vx,flight.vy,flight.vz),distance=waypointDistance(flight);
  const status=!running?'일시정지':flight.mode==='manual'?'직접 조종':flight.completed?'경로 완료':`경로 비행 · WP ${String(flight.waypoint+1).padStart(2,'0')}`;
  const waypointCount=flight.route.points.length,progress=flight.completed?100:flight.waypoint/Math.max(1,waypointCount-1)*100;
  return <div ref={dialog} className={`simlab simlab-pilot${controller?' is-controller':''}${!dashboard?' is-cinema':''}${!panel?' is-panel-hidden':''}`} role="dialog" aria-modal="true" aria-labelledby="pilot-title">
    <div className="simlab-world" aria-label="키보드로 조종하는 3차원 비행 장면">
      <RemoteController active={controller&&dashboard} running={running} speed={speed} distance={distance} height={flight.height} status={status} heading={heading} metricUnits onPlayback={pause} onView={setView} onZoom={requestZoom} onReset={()=>setReset(v=>v+1)} onExpand={()=>setDashboard(false)} onClose={onClose} onCameraInput={noop} pilotInput={input.current} onPilotInput={onStick}>
        <SimulationScene frame={frame} frames={[]} disturbance={frame.disturbance} heightCm={flight.height*100} startXcm={0} startYcm={0} arriveTolCm={12} view={view} showGuides showRoom showPath={showPath} dashboardVisible={dashboard} panelVisible={panel} quality={quality} running={running} resetKey={reset} sensorCanvas={sensor} environment="field" daylight={daylight} playbackSpeed={1} zoomCommand={zoom} cameraInput={NO_CAMERA} compactDisplay={controller&&dashboard} flight={flight}/>
      </RemoteController>
    </div>
    <div className="simlab-overlay" hidden={!dashboard}>
      <header className="simlab-header">
        <div className="simlab-brand"><span className="simlab-brand-icon"><Crosshair size={24}/></span><div><strong>SPIDER</strong><h1 id="pilot-title">키보드 비행 시뮬레이터</h1></div></div>
        <span className="pilot-header-status"><i className={running?'is-running':''}/>{status}</span>
        <div className="simlab-header-actions">
          <button type="button" className="simlab-mode-toggle" onClick={onApproach}><Activity size={15}/><span>정밀 접근 시험</span></button>
          <button type="button" className="simlab-mode-toggle" onClick={()=>setController(v=>!v)} aria-label={controller?'관제 화면으로 전환':'조종기 화면으로 전환'}>{controller?<LayoutDashboard size={15}/>:<Gamepad2 size={15}/>}<span>{controller?'관제 화면':'조종기 보기'}</span></button>
          <button type="button" className="simlab-cinema-toggle" onClick={()=>setDashboard(false)}><EyeOff size={15}/><span>3D만 보기</span></button>
          <button type="button" ref={close} className="simlab-icon-button" aria-label="시뮬레이터 닫기" onClick={onClose}><X size={21}/></button>
        </div>
      </header>
      <div className="pilot-mission-strip"><div><span>FLIGHT CONTROL / 01</span><strong title={flight.route.name}>{flight.route.name}</strong><small>{flight.route.source==='mission'?'메인 계획 좌표 · 출발점 기준':'가상 야외 훈련장'} · {waypointCount}개 경유점</small></div><button type="button" aria-label="해질녘 조명" aria-pressed={daylight==='dusk'} onClick={()=>setDaylight(v=>v==='day'?'dusk':'day')}>{daylight==='day'?<Sun size={15}/>:<Sunset size={15}/>}<span>{daylight==='day'?'주간':'해질녘'}</span></button></div>
      <div className="pilot-view-strip" role="group" aria-label="비행 카메라와 표시">{([{id:'orbit',label:'전체 경로',icon:Box},{id:'follow',label:'기체 추적',icon:Focus},{id:'top',label:'상공 시점',icon:Maximize2},{id:'camera',label:'하향 카메라',icon:Camera}] as const).map(({id,label,icon:Icon})=><button key={id} type="button" aria-label={label} aria-pressed={view===id} onClick={()=>setView(id)}><Icon size={16}/></button>)}<button type="button" aria-label="비행 궤적 표시" aria-pressed={showPath} onClick={()=>setShowPath(v=>!v)}><Route size={16}/></button><button type="button" aria-label={panel?'관제 패널 숨기기':'관제 패널 보이기'} onClick={()=>setPanel(v=>!v)}>{panel?<PanelRightClose size={16}/>:<PanelRightOpen size={16}/>}</button></div>
      <aside className="pilot-panel" hidden={!panel} aria-label="비행 경로와 계측">
        <div className="pilot-panel-title"><div><span>FLIGHT OPERATIONS</span><h2>비행 관제</h2></div><button type="button" aria-label="비행 관제 닫기" onClick={()=>setPanel(false)}><X size={16}/></button></div>
        <div className="pilot-instruments"><div><span>기체 속도</span><output aria-label="기체 속도">{speed.toFixed(2)}<small>m/s</small></output></div><div><span>비행 고도</span><output aria-label="기체 고도">{flight.height.toFixed(2)}<small>m</small></output></div><div><span>방위각</span><output aria-label="기체 방위각">{heading.toFixed(0)}<small>°</small></output></div><div><span>실제 이동 거리</span><output aria-label="실제 이동 거리">{flight.travelled.toFixed(1)}<small>m</small></output></div></div>
        <div className="pilot-coordinates"><span>X <output aria-label="기체 X 좌표">{flight.x.toFixed(2)}</output>m</span><span>Y <output aria-label="기체 Y 좌표">{flight.y.toFixed(2)}</output>m</span><span>상승 {flight.vz.toFixed(2)}m/s</span></div>
        <section className="pilot-route-section"><div className="pilot-section-heading"><h3>경로 추적</h3><span>다음 지점 {distance.toFixed(1)}m</span></div><RouteMap flight={flight}/><div className="pilot-map-legend"><span><i/>계획 경로</span><span><i/>실제 이동 궤적</span></div>
          <label className="pilot-route-select">비행 경로<select aria-label="비행 경로" value={flight.route.source} onChange={e=>resetFlight(e.target.value==='mission'&&mission?mission:trainingRoute())}><option value="training">훈련장 순환 경로</option>{mission&&<option value="mission">메인 임무 계획 · {mission.points.length}개 지점</option>}</select></label>
          {missionUnavailable&&<p className="pilot-hint">메인 경로가 훈련장 범위(출발점 ±40m)를 벗어나 기본 경로를 불러왔습니다.</p>}
          <div className="pilot-route-progress"><progress max={100} value={progress}/><span>{flight.completed?'경로 완료':`WP ${flight.waypoint+1} / ${waypointCount}`}</span></div>
        </section>
        <section className="pilot-keyboard-guide"><h3><Keyboard size={14}/> 양손 키보드 조종</h3><div className="pilot-key-grid">{KEYS.map(([code,label])=><kbd key={code} className={keys.includes(code)?'is-pressed':''}>{label}</kbd>)}</div><p><b>W / S</b> 상승·하강 <b>A / D</b> 좌·우 회전</p><p><b>방향키</b> 기체 기준 전후·좌우 이동</p><p><b>Space</b> 제동 <b>P</b> 일시정지 <b>R</b> 초기화</p></section>
        <div className="pilot-camera"><div><Camera size={14}/><span>실시간 하향 카메라</span><button type="button" aria-label="카메라 영상 확대" onClick={()=>setView('camera')}><Maximize2 size={13}/></button></div><canvas ref={sensor} width={320} height={240} aria-label="같은 장면의 하향 카메라 영상"/></div>
        <footer className="pilot-panel-footer"><select aria-label="3D 렌더링 품질" value={quality} onChange={e=>setQuality(e.target.value as 'high'|'standard')}><option value="high">고품질 3D</option><option value="standard">성능 우선</option></select><span>가상 비행 · 위치·고도·방위각 모델</span></footer>
      </aside>
      <section className="pilot-flight-bar" aria-label="비행 조작">
        <div className="pilot-flight-actions"><div className="pilot-authority"><button type="button" aria-pressed={flight.mode==='manual'} onClick={manual}><Gamepad2 size={16}/>직접 조종</button><button type="button" aria-pressed={flight.mode==='route'} onClick={follow}><Route size={16}/>{flight.completed?'경로 다시 비행':'경로 자동 비행'}</button></div><button type="button" className="pilot-brake" onClick={brake}>제동 · 호버 <kbd>Space</kbd></button><button type="button" aria-label={running?'비행 일시정지':'비행 재개'} onClick={pause}>{running?<Pause size={15}/>:<Play size={15}/>}</button><button type="button" aria-label="비행 초기화" onClick={()=>resetFlight()}><RotateCcw size={15}/></button><output aria-label="비행 경과 시간">{(flight.tMs/1000).toFixed(1)}s</output></div>
        <div className="pilot-flight-notice"><span><i/>{status}</span><p>{flight.boundary?'훈련장 경계 또는 고도 한계에 도달했습니다.':notice||'W/S 고도 · A/D 회전 + 방향키 이동을 동시에 눌러 조종하세요.'}</p><small>H 화면 전환 · P 일시정지</small></div>
      </section>
    </div>
    {!dashboard&&<><button ref={restore} className="simlab-cinema-restore" type="button" onClick={()=>setDashboard(true)}><LayoutDashboard size={16}/>대시보드 보기<kbd>H</kbd></button><div className="pilot-cinema-controls"><span>{status} · {speed.toFixed(1)}m/s · 고도 {flight.height.toFixed(1)}m</span><button type="button" onClick={brake}>제동</button><button type="button" onClick={pause}>{running?'일시정지':'재개'}</button></div></>}
  </div>;
}
