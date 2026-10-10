import {useEffect, useMemo, useRef, useState} from 'react';
import {Activity, ArrowLeft, Battery, Box, Camera, Crosshair, Eye, EyeOff, Focus, Gauge, Maximize2, Minimize2, Minus, Plus, Radio, RefreshCw, RotateCcw, Settings2, Wifi, WifiOff} from 'lucide-react';
import {Input} from './components/ui/input';
import {FormSelect} from './components/form-controls';
import SimulationScene, {type SimulationView} from './SimulationScene';
import {flightSceneFrame} from './simulation/pilotFlight';
import {LiveMotionTracker, type WifiSnapshot} from './liveTelemetry';
import {pulsePercent, ThrottleMonitor} from './throttleTelemetry';
import ThrottlePanel, {PulseMeter} from './ThrottlePanel';
import './liveAircraft.css';

const idleCamera = {panX:0,panY:0,rotateX:0,rotateY:0};
const fmt = (n:number|null|undefined, digits=1) => n==null || !Number.isFinite(n) ? '—' : n.toFixed(digits);
async function request(path:string, host?:string):Promise<WifiSnapshot> {
  const response=await fetch('/api/hardware/wifi/'+path, path==='status'?undefined:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(host?{host}:{})});
  const payload=await response.json();
  if(!response.ok) throw new Error(payload?.detail?.message??'Wi-Fi 연결 요청에 실패했습니다.');
  return payload;
}

export default function LiveAircraft({onBack,onDiagnostics}:{onBack:()=>void;onDiagnostics:()=>void}) {
  const [snapshot,setSnapshot]=useState<WifiSnapshot|null>(null),[host,setHost]=useState('');
  const [error,setError]=useState(''),[busy,setBusy]=useState(false),[transportUp,setTransportUp]=useState(false);
  const [clock,setClock]=useState(()=>performance.now()),[view,setView]=useState<SimulationView>('follow');
  const [quality,setQuality]=useState<'high'|'standard'>('high'),[zoom,setZoom]=useState({sequence:0,factor:1}),[resetKey,setResetKey]=useState(0);
  const [primaryView,setPrimaryView]=useState<'camera'|'pose'>('camera'),[insetVisible,setInsetVisible]=useState(true),[focused,setFocused]=useState(false);
  const [cameraFailed,setCameraFailed]=useState(false),[loadedFrame,setLoadedFrame]=useState<{session:number;at:number}|null>(null);
  const [ready,setReady]=useState(true);
  const receivedAt=useRef(0),tracker=useRef(new LiveMotionTracker()),throttleTracker=useRef(new ThrottleMonitor()),unusedCamera=useRef<HTMLCanvasElement>(null);

  useEffect(()=>{
    let active=true;
    const accept=(value:WifiSnapshot)=>{if(!active)return;setSnapshot(value);receivedAt.current=performance.now();};
    let source:EventSource;
    let lastEventAt=performance.now();
    const openStream=()=>{
      lastEventAt=performance.now();
      source=new EventSource('/api/hardware/wifi/events');
      source.onmessage=e=>{
        try {const value=JSON.parse(e.data) as WifiSnapshot;if(!value.latest||!Number.isFinite(value.server_ms))return;lastEventAt=performance.now();accept(value);setTransportUp(true);}
        catch {setTransportUp(false);}
      };
      source.onerror=()=>setTransportUp(false);
    };
    openStream();
    void request('status').then(async value=>{
      if(!active)return;setHost(value.host);accept(value);
      if(value.state==='disconnected') {setBusy(true);accept(await request('connect',value.host));}
    }).catch(e=>{if(active)setError((e as Error).message);}).finally(()=>{if(active)setBusy(false);});
    const timer=window.setInterval(()=>{
      const now=performance.now();setClock(now);
      // 프록시가 닫힌 서버의 SSE 응답을 붙잡고 있어도 새 상태 스트림으로 복구한다.
      if(now-lastEventAt>5000){source.close();setTransportUp(false);openStream();}
    },100);
    return()=>{active=false;source.close();clearInterval(timer);};
  },[]);
  useEffect(()=>{
    if(!focused)return;
    const escape=(e:KeyboardEvent)=>{if(e.key==='Escape')setFocused(false);};
    window.addEventListener('keydown',escape);return()=>window.removeEventListener('keydown',escape);
  },[focused]);
  const transportAge=Math.max(0,clock-receivedAt.current);
  const motion=useMemo(()=>tracker.current.update(snapshot,transportAge),[snapshot,transportAge]);
  const throttle=useMemo(()=>throttleTracker.current.update(snapshot,transportAge),[snapshot,transportAge]);
  const online=motion.online&&transportUp;
  const state=snapshot?.state??'connecting';
  const status=online?'실기체 수신 중':state==='disconnected'?'연결 해제':state==='reconnecting'?'다시 연결 중':state==='connected'?'수신 지연':'기체 연결 중';
  const wifi=snapshot?.latest.wifi?.data;
  const ssid=typeof wifi?.sta_ssid==='string'?wifi.sta_ssid:'D4D_Hacker_01';
  const rssi=typeof wifi?.rssi==='number'?wifi.rssi:null;
  const session=snapshot?.session??0;
  const frameSequence=snapshot?.frame.sequence??0;
  const imageUrl=frameSequence?'/api/hardware/wifi/camera.jpg?frame='+session+'-'+frameSequence:'';
  // 화면에 실제로 로드한 사진의 시각으로 지연을 계산한다. 새 사진 요청만으로 수신 완료를 표시하지 않는다.
  const hasImage=!!imageUrl&&loadedFrame?.session===session;
  const frameAge=hasImage?Math.max(0,(snapshot?.server_ms??0)+transportAge-loadedFrame.at):null;
  const imageStale=!online||frameAge===null||frameAge>3000||cameraFailed;
  const sceneFlight=useMemo(()=>({...motion.flight,height:Math.max(.15,motion.flight.height)}),[motion.flight]);
  const frame=flightSceneFrame(sceneFlight);frame.disturbance.kill=motion.motorPower===0;
  const speed=motion.rightSpeed===null||motion.forwardSpeed===null?null:Math.hypot(motion.rightSpeed,motion.forwardSpeed);

  const connect=async()=>{
    setBusy(true);setError('');
    try {setSnapshot(await request('connect',host));receivedAt.current=performance.now();}
    catch(e){setError((e as Error).message);}finally{setBusy(false);}
  };
  const disconnect=async()=>{
    setBusy(true);setError('');
    try {setSnapshot(await request('disconnect'));receivedAt.current=performance.now();}
    catch(e){setError((e as Error).message);}finally{setBusy(false);}
  };
  const zero=()=>{tracker.current.zero();setClock(performance.now());setResetKey(k=>k+1);};

  return <div className={'live-aircraft'+(focused?' is-focused':'')} data-link-state={online?'online':state}>
    <header className="live-title">
      <div><span className={'live-indicator '+(online?'is-online':'')}><i/>{status}</span><h2>실기체 영상 관제</h2></div>
      <div className="live-title-summary"><span><Battery size={14}/>{fmt(motion.voltage,2)} V</span><span>{fmt(motion.height,2)} m</span><b className={motion.armed===true?'armed':''}>{motion.armed===null?'기체 상태 대기':motion.armed?'ARMED':'DISARMED'}</b></div>
      <button className="live-button" onClick={onBack}><ArrowLeft size={14}/>훈련 화면</button>
    </header>

    <div className="live-workspace">
      <aside className="live-left-rail" aria-label="스로틀과 연결 상태">
        <ThrottlePanel reading={throttle}/>
        <section className="live-card live-connection" aria-label="Wi-Fi 기체 연결">
          <div className="live-card-heading"><Wifi size={16}/><h3>기체 연결</h3><span>모니터링</span></div>
          <div className="live-network"><strong>{ssid}</strong><small>TagBoard · {snapshot?.host??'주소 확인 중'}:{snapshot?.port??3333}</small><small>{snapshot?.source==='camera_viewer'?'기존 카메라 뷰어와 데이터 공유':'Wi-Fi 직접 수신'}</small></div>
          <form onSubmit={e=>{e.preventDefault();void connect();}}>
            <label htmlFor="live-board-host">보드 Wi-Fi 주소</label><Input id="live-board-host" value={host} onChange={e=>setHost(e.target.value)} autoComplete="off" placeholder="192.168.0.151" disabled={busy}/>
            <div className="live-connect-actions"><button className="live-button primary" type="submit" disabled={busy||!host}>{busy?<RefreshCw className="spin" size={14}/>:<Wifi size={14}/>}연결{state!=='disconnected'?' 확인':''}</button><button className="live-button" type="button" disabled={busy||state==='disconnected'} onClick={()=>void disconnect()}><WifiOff size={14}/>해제</button></div>
          </form>
          {(error||snapshot?.error)&&<p className="live-error" role="alert">{error||snapshot?.error}</p>}
          <dl className="live-link-stats"><div><dt>Wi-Fi 신호</dt><dd>{fmt(rssi,0)}<small>dBm</small></dd></div><div><dt>최근 보고</dt><dd>{fmt(snapshot?.received_age_ms==null?null:(snapshot.received_age_ms+transportAge)/1000,1)}<small>초 전</small></dd></div><div><dt>비행제어기</dt><dd className={motion.fcLinked?'is-good':''}>{motion.fcLinked?'응답 확인':'수신 대기'}</dd></div><div><dt>조종 권한</dt><dd>{motion.authority}</dd></div></dl>
          <button className="live-text-button" onClick={onDiagnostics}><Settings2 size={13}/>BLE·USB 진단 패널</button>
        </section>
      </aside>

      <section className="live-visual" aria-label="실제 영상과 3D 자세" data-primary-view={primaryView} data-inset-visible={insetVisible}>
        <div className="live-visual-toolbar">
          <div className="live-view-tabs" role="group" aria-label="중앙 화면 선택"><button aria-pressed={primaryView==='camera'} onClick={()=>setPrimaryView('camera')}><Camera size={14}/>실제 영상</button><button aria-pressed={primaryView==='pose'} onClick={()=>setPrimaryView('pose')}><Box size={14}/>3D 자세</button></div>
          <div className="live-visual-actions"><button className="live-icon" aria-label={insetVisible?'보조 화면 숨기기':'보조 화면 표시'} aria-pressed={insetVisible} onClick={()=>setInsetVisible(v=>!v)}>{insetVisible?<Eye size={16}/>:<EyeOff size={16}/>}</button><button className="live-icon" aria-label={focused?'관제 대시보드 표시':'영상 집중 보기'} aria-pressed={focused} onClick={()=>setFocused(v=>!v)}>{focused?<Minimize2 size={16}/>:<Maximize2 size={16}/>}</button></div>
        </div>
        <div className="live-visual-stage">
          <section className="live-camera-panel" aria-label="실제 드론 카메라">
            <div className="live-camera-image">
              {imageUrl&&<img key={session} src={imageUrl} alt="Wi-Fi로 수신한 실제 드론 카메라 영상" onLoad={()=>{setCameraFailed(false);setLoadedFrame({session,at:(snapshot?.server_ms??0)-(snapshot?.frame.age_ms??0)});}} onError={()=>setCameraFailed(true)}/>}
              {!hasImage&&<div className="live-camera-empty"><Camera size={36}/><strong>{state==='disconnected'?'카메라 연결 해제':'기체 카메라 수신 대기'}</strong><span>기체의 Wi-Fi 영상이 이곳에 표시됩니다.</span></div>}
              <div className={'live-video-badge '+(imageStale?'is-delayed':'')}><i/>{hasImage?(imageStale?'마지막 수신 영상':'실제 카메라'):'영상 대기'}<span>CAM 01</span></div>
              {hasImage&&imageStale&&<div className="live-video-warning">{!online?'연결을 확인하고 있습니다.':cameraFailed?'다음 사진을 요청하고 있습니다.':'영상 수신이 지연되고 있습니다.'} 마지막 사진을 표시합니다.</div>}
              {primaryView==='pose'&&<button className="live-inset-expand" aria-label="실제 카메라를 중앙으로" onClick={()=>setPrimaryView('camera')}><Maximize2 size={13}/>영상 확대</button>}
            </div>
          </section>
          <section className="live-scene-panel" aria-label="센서로 동기화한 3D 자세">
            <div className="live-scene-caption"><Box size={12}/><span>실측 자세 · 3D</span>{primaryView==='camera'&&<button className="live-icon" aria-label="3D 자세를 중앙으로" onClick={()=>setPrimaryView('pose')}><Maximize2 size={13}/></button>}</div>
            <div className="live-scene">
              <SimulationScene frame={frame} frames={[]} disturbance={frame.disturbance} heightCm={sceneFlight.height*100}
                startXcm={0} startYcm={0} arriveTolCm={0} view={primaryView==='camera'?'follow':view} showGuides={false} showRoom showPath
                dashboardVisible={false} panelVisible={false} quality={quality} running={online} resetKey={resetKey}
                environment="lab" daylight="day" playbackSpeed={1} zoomCommand={zoom} cameraInput={idleCamera}
                compactDisplay={false} sensorCanvas={unusedCamera} flight={sceneFlight} liveTelemetry={{motorPower:motion.motorPower}} onReady={setReady}/>
            </div>
            {primaryView==='pose'&&<div className="live-view-tools" role="group" aria-label="실기체 3D 시점">{[{id:'orbit',label:'실기체 전체 보기',icon:Box},{id:'follow',label:'실기체 추적',icon:Focus},{id:'top',label:'실기체 상공 보기',icon:Maximize2}].map(({id,label,icon:Icon})=><button key={id} aria-label={label} aria-pressed={view===id} onClick={()=>setView(id as SimulationView)}><Icon size={16}/></button>)}<i/><button aria-label="실기체 3D 확대" onClick={()=>setZoom(v=>({sequence:v.sequence+1,factor:1.3}))}><Plus size={16}/></button><button aria-label="실기체 3D 축소" onClick={()=>setZoom(v=>({sequence:v.sequence+1,factor:.77}))}><Minus size={16}/></button><button aria-label="실기체 시점 초기화" onClick={()=>setResetKey(k=>k+1)}><RotateCcw size={16}/></button></div>}
          </section>
        </div>
        <div className="live-video-status"><span>{snapshot?.frame.width??0} × {snapshot?.frame.height??0} · JPEG 연속 수신</span><span className={imageStale?'is-delayed':'is-good'}>{frameAge===null?'사진 수신 대기':fmt(frameAge/1000,1)+'초 전'}</span><span>전송 {fmt(snapshot?.frame.transfer_ms==null?null:snapshot.frame.transfer_ms/1000,2)}s</span></div>
        <div className="live-video-readouts"><div><span>스로틀 입력</span><strong>{throttle.inputUs??'—'}<small>µs</small></strong></div><div><span>기체 높이</span><strong>{fmt(motion.height,2)}<small>m</small></strong></div><div><span>수평 속도</span><strong>{fmt(speed,2)}<small>m/s</small></strong></div><div><span>배터리 전류</span><strong>{fmt(motion.amps,2)}<small>A</small></strong></div></div>
      </section>

      <aside className="live-right-rail" aria-label="기체 계측 대시보드">
        <section className="live-card live-metrics" aria-label="실측 기체 상태">
          <div className="live-card-heading"><Gauge size={16}/><h3>기체 상태</h3><span>{online?'수신 중':'대기'}</span></div>
          <div className="live-primary-metrics"><div><span>거리센서 높이</span><output aria-label="실기체 높이">{fmt(motion.height,2)}<small>m</small></output></div><div><span>수평 이동 속도</span><output aria-label="실기체 속도">{fmt(speed,2)}<small>m/s</small></output></div></div>
          <div className="live-attitude"><div className={'live-horizon'+(motion.roll===null||motion.pitch===null?' is-unavailable':'')} aria-hidden="true"><div style={{transform:'rotate('+(-1*(motion.roll??0))+'deg) translateY('+((motion.pitch??0)*1.4)+'px)'}}/><b>━ ✛ ━</b></div><dl><div><dt>롤</dt><dd><output aria-label="실기체 롤">{fmt(motion.roll)}°</output></dd></div><div><dt>피치</dt><dd><output aria-label="실기체 피치">{fmt(motion.pitch)}°</output></dd></div><div><dt>방위각</dt><dd><output aria-label="실기체 방위각">{fmt(motion.heading===null?null:(motion.heading+360)%360,0)}°</output></dd></div><div><dt>상승 속도</dt><dd>{fmt(motion.climbSpeed,2)} m/s</dd></div></dl></div>
          <div className="live-electrical"><span><Battery size={14}/>{fmt(motion.voltage,2)} V</span><span>{fmt(motion.amps,2)} A</span><b className={motion.armed===true?'armed':''}>{motion.armed===null?'상태 대기':motion.armed?'ARMED':'DISARMED'}</b></div>
          <div className="live-section-label">FC 모터별 출력 <small>µs</small></div>
          <div className="live-motors" aria-label="실기체 FC 모터 출력">{[0,1,2,3].map(i=><div key={i}><span>M{i+1}</span><PulseMeter value={pulsePercent(motion.motors?.[i]??null)} label={'모터 '+(i+1)+' 출력'} text={motion.motors?.[i]===undefined?'수신 대기':motion.motors[i]+' 마이크로초'} tone="motor"/><output>{motion.motors?.[i]??'—'}</output></div>)}</div>
          <p className="live-small-note">비행제어기가 보고한 출력값입니다.</p>
          <div className="live-section-label">조종기 채널 <small>µs</small></div>
          <div className="live-rc">{[{label:'롤',i:0},{label:'피치',i:1},{label:'요',i:2},{label:'스로틀',i:3}].map(({label,i})=><div key={i}><small>{label}</small><PulseMeter value={pulsePercent(motion.rc?.[i]??null)} label={label+' 채널'} text={motion.rc?.[i]===undefined?'수신 대기':motion.rc[i]+' 마이크로초'}/><b>{motion.rc?.[i]??'—'}</b></div>)}</div>
          <div className="live-position"><span><Crosshair size={13}/>연결 기준 상대 이동</span><strong>X {fmt(motion.flight.x,2)} <small>m</small> <i/> Y {fmt(motion.flight.y,2)} <small>m</small></strong><p className={motion.positionValid?'is-good':''}>{motion.positionNote}</p><small>광류 품질 {fmt(motion.flowQuality,0)} / 255</small></div>
          <div className="live-pose-options"><button className="live-button" onClick={zero}><Crosshair size={13}/>상대 위치 원점</button><FormSelect aria-label="실기체 3D 품질" value={quality} onValueChange={v=>setQuality(v as 'high'|'standard')} options={[{value:'high',label:'고품질 3D'},{value:'standard',label:'성능 우선'}]}/></div>
        </section>
      </aside>
    </div>
    <footer className="live-footer"><span><Radio size={13}/>{snapshot?.source==='camera_viewer'?'카메라 뷰어 데이터 공유':'Wi-Fi 실기체 연결'}</span><span><Activity size={13}/>{ready?'자세·높이 실측 / 수평 위치 추정':'3D 화면 연결 확인 필요'}</span><span>스로틀 조절은 실제 조종기에서 · 화면은 수신값 표시</span></footer>
  </div>;
}
