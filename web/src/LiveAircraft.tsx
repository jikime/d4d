import {useEffect, useMemo, useRef, useState} from 'react';
import {Activity, ArrowLeft, Battery, Box, Camera, Crosshair, Focus, Gauge, Maximize2, Minus, Plus, Radio, RefreshCw, RotateCcw, Settings2, Wifi, WifiOff} from 'lucide-react';
import {Input} from './components/ui/input';
import {FormSelect} from './components/form-controls';
import SimulationScene, {type SimulationView} from './SimulationScene';
import {flightSceneFrame} from './simulation/pilotFlight';
import {LiveMotionTracker, type WifiSnapshot} from './liveTelemetry';
import './liveAircraft.css';

const idleCamera = {panX:0,panY:0,rotateX:0,rotateY:0};
const fmt = (n:number|null|undefined, digits=1) => n==null || !Number.isFinite(n) ? '—' : n.toFixed(digits);
async function request(path:string, host?:string):Promise<WifiSnapshot> {
  const response=await fetch(`/api/hardware/wifi/${path}`, path==='status'?undefined:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(host?{host}:{})});
  const payload=await response.json();
  if(!response.ok) throw new Error(payload?.detail?.message??'Wi-Fi 연결 요청에 실패했습니다.');
  return payload;
}

export default function LiveAircraft({onBack,onDiagnostics}:{onBack:()=>void;onDiagnostics:()=>void}) {
  const [snapshot,setSnapshot]=useState<WifiSnapshot|null>(null),[host,setHost]=useState('');
  const [error,setError]=useState(''),[busy,setBusy]=useState(false),[transportUp,setTransportUp]=useState(false);
  const [clock,setClock]=useState(()=>performance.now()),[view,setView]=useState<SimulationView>('orbit');
  const [quality,setQuality]=useState<'high'|'standard'>('high'),[zoom,setZoom]=useState({sequence:0,factor:1}),[resetKey,setResetKey]=useState(0);
  const [cameraLarge,setCameraLarge]=useState(false),[cameraFailed,setCameraFailed]=useState(false),[cameraLoaded,setCameraLoaded]=useState(0);
  const [ready,setReady]=useState(true);
  const receivedAt=useRef(0),tracker=useRef(new LiveMotionTracker()),unusedCamera=useRef<HTMLCanvasElement>(null);

  useEffect(()=>{
    let active=true;
    const accept=(value:WifiSnapshot)=>{if(!active)return;setSnapshot(value);receivedAt.current=performance.now();};
    const source=new EventSource('/api/hardware/wifi/events');
    source.onmessage=e=>{
      try {const value=JSON.parse(e.data) as WifiSnapshot;if(!value.latest||!Number.isFinite(value.server_ms))return;accept(value);setTransportUp(true);}
      catch {setTransportUp(false);}
    };
    source.onerror=()=>setTransportUp(false);
    // 이 화면 진입은 상단 '기체 연결' 동작에 의해서만 일어난다. 재마운트 요청은 서버가 합친다.
    void request('status').then(async value=>{
      if(!active)return;setHost(value.host);accept(value);
      if(value.state==='disconnected') {setBusy(true);accept(await request('connect',value.host));}
    }).catch(e=>{if(active)setError((e as Error).message);}).finally(()=>{if(active)setBusy(false);});
    const timer=window.setInterval(()=>setClock(performance.now()),100);
    return()=>{active=false;source.close();clearInterval(timer);};
  },[]);
  const transportAge=Math.max(0,clock-receivedAt.current);
  const motion=useMemo(()=>tracker.current.update(snapshot,transportAge),[snapshot,transportAge]);
  const online=motion.online&&transportUp;
  const state=snapshot?.state??'connecting';
  const status=online?'실기체 수신 중':state==='disconnected'?'연결 해제':state==='reconnecting'?'다시 연결 중':state==='connected'?'수신 지연':'기체 연결 중';
  const wifi=snapshot?.latest.wifi?.data;
  const ssid=typeof wifi?.sta_ssid==='string'?wifi.sta_ssid:'D4D_Hacker_01';
  const rssi=typeof wifi?.rssi==='number'?wifi.rssi:null;
  const frameAge=snapshot?.frame.age_ms==null?null:snapshot.frame.age_ms+transportAge;
  const frameSequence=snapshot?.frame.sequence??0;
  const frameKey=`${snapshot?.session??0}-${frameSequence}`;
  const imageUrl=frameSequence?`/api/hardware/wifi/camera.jpg?frame=${frameKey}`:'';
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
  return <div className="live-aircraft" data-link-state={online?'online':state}>
    <div className="live-scene">
      <SimulationScene frame={frame} frames={[]} disturbance={frame.disturbance} heightCm={sceneFlight.height*100}
        startXcm={0} startYcm={0} arriveTolCm={0} view={view} showGuides={false} showRoom showPath
        dashboardVisible panelVisible={false} quality={quality} running={online} resetKey={resetKey}
        environment="lab" daylight="day" playbackSpeed={1} zoomCommand={zoom} cameraInput={idleCamera}
        compactDisplay={false} sensorCanvas={unusedCamera} flight={sceneFlight} liveTelemetry={{motorPower:motion.motorPower}} onReady={setReady}/>
    </div>
    <header className="live-title"><div><span className={`live-indicator ${online?'is-online':''}`}><i/>{status}</span><h2>실기체 움직임 관제</h2><p>실제 기체의 자세와 높이, 카메라를 같은 화면에서 확인합니다.</p></div><button className="live-button" onClick={onBack}><ArrowLeft size={14}/>훈련 화면</button></header>

    <section className="live-card live-connection" aria-label="Wi-Fi 기체 연결">
      <div className="live-card-heading"><Wifi size={16}/><h3>기체 연결</h3><span>관측 모드</span></div>
      <div className="live-network"><strong>{ssid}</strong><small>TagBoard · {snapshot?.host??'주소 확인 중'}:{snapshot?.port??3333}</small><small>{snapshot?.source==='camera_viewer'?'기존 카메라 뷰어의 수신 데이터 공유':'Wi-Fi 직접 수신'}</small></div>
      <form onSubmit={e=>{e.preventDefault();void connect();}}>
        <label htmlFor="live-board-host">보드 Wi-Fi 주소</label><Input id="live-board-host" value={host} onChange={e=>setHost(e.target.value)} autoComplete="off" placeholder="192.168.0.151" disabled={busy}/>
        <div className="live-connect-actions"><button className="live-button primary" type="submit" disabled={busy||!host}>{busy?<RefreshCw className="spin" size={14}/>:<Wifi size={14}/>}연결{state!=='disconnected'?' 확인':''}</button><button className="live-button" type="button" disabled={busy||state==='disconnected'} onClick={()=>void disconnect()}><WifiOff size={14}/>해제</button></div>
      </form>
      {(error||snapshot?.error)&&<p className="live-error" role="alert">{error||snapshot?.error}</p>}
      <dl className="live-link-stats"><div><dt>Wi-Fi 신호</dt><dd>{fmt(rssi,0)}<small>dBm</small></dd></div><div><dt>최근 보고</dt><dd>{fmt(snapshot?.received_age_ms==null?null:(snapshot.received_age_ms+transportAge)/1000,1)}<small>초 전</small></dd></div><div><dt>비행제어기</dt><dd className={motion.fcLinked?'is-good':''}>{motion.fcLinked?'응답 확인':'수신 대기'}</dd></div><div><dt>조종 권한</dt><dd>{motion.authority}</dd></div></dl>
      <div className="live-observe-note"><Activity size={15}/><p>기체를 기울이거나 들어 올리면 3D 모델에 반영됩니다. 화면 연결은 조종 출력을 변경하지 않습니다.</p></div>
      <button className="live-text-button" onClick={onDiagnostics}><Settings2 size={13}/>기존 BLE·USB 진단 패널</button>
    </section>

    <section className="live-card live-metrics" aria-label="실측 기체 상태">
      <div className="live-card-heading"><Gauge size={16}/><h3>실측 움직임</h3><span>{online?'수신 중':'대기'}</span></div>
      <div className="live-primary-metrics"><div><span>거리센서 높이</span><output aria-label="실기체 높이">{fmt(motion.height,2)}<small>m</small></output></div><div><span>수평 이동 속도</span><output aria-label="실기체 속도">{fmt(speed,2)}<small>m/s</small></output></div></div>
      <div className="live-attitude"><div className="live-horizon" aria-hidden="true"><div style={{transform:`rotate(${-1*(motion.roll??0)}deg) translateY(${(motion.pitch??0)*1.4}px)`}}/><b>━ ✛ ━</b></div><dl><div><dt>롤</dt><dd><output aria-label="실기체 롤">{fmt(motion.roll)}°</output></dd></div><div><dt>피치</dt><dd><output aria-label="실기체 피치">{fmt(motion.pitch)}°</output></dd></div><div><dt>방위각</dt><dd><output aria-label="실기체 방위각">{fmt(motion.heading===null?null:(motion.heading+360)%360,0)}°</output></dd></div><div><dt>상승 속도</dt><dd>{fmt(motion.climbSpeed,2)} m/s</dd></div></dl></div>
      <div className="live-position"><span><Crosshair size={13}/>연결 기준 상대 이동</span><strong>X {fmt(motion.flight.x,2)} <small>m</small> <i/> Y {fmt(motion.flight.y,2)} <small>m</small></strong><p className={motion.positionValid?'is-good':''}>{motion.positionNote}</p><small>광류 품질 {fmt(motion.flowQuality,0)} / 255 · 바닥 눈금 0.5 m</small></div>
      <div className="live-electrical"><span><Battery size={14}/>{fmt(motion.voltage,2)} V</span><span>{fmt(motion.amps,2)} A</span><b className={motion.armed===true?'armed':''}>{motion.armed===null?'상태 미확인':motion.armed?'ARMED':'DISARMED'}</b></div>
      <div className="live-motors" aria-label="실기체 FC 모터 출력">{[0,1,2,3].map(i=><div key={i}><span>M{i+1}</span><b>{motion.motors?.[i]??'—'}</b><div><i style={{width:`${motion.motors?Math.max(0,(motion.motors[i]-1000)/10):0}%`}}/></div></div>)}</div><p className="live-small-note">모터 표시는 FC 출력값(µs)을 따릅니다.</p>
      <div className="live-rc"><span>조종기 입력</span>{[{label:'롤',i:0},{label:'피치',i:1},{label:'요',i:2},{label:'스로틀',i:3}].map(({label,i})=><div key={i}><small>{label}</small><b>{motion.rc?.[i]??'—'}</b></div>)}</div>
    </section>

    <section className={`live-card live-camera${cameraLarge?' is-expanded':''}`} aria-label="실제 드론 카메라">
      <div className="live-card-heading"><Camera size={16}/><h3>실제 기체 카메라</h3><span className={imageStale?'':'is-good'}>{imageStale?'영상 대기':'수신 중'}</span><button className="live-icon" aria-label={cameraLarge?'실제 카메라 축소':'실제 카메라 확대'} onClick={()=>setCameraLarge(v=>!v)}><Maximize2 size={15}/></button></div>
      <div className="live-camera-image">{imageUrl&&<img src={imageUrl} alt="Wi-Fi로 수신한 실제 드론 카메라 영상" onLoad={()=>{setCameraFailed(false);setCameraLoaded(Date.now());}} onError={()=>setCameraFailed(true)}/>}{(!imageUrl||!cameraLoaded||imageStale)&&<div className="live-camera-empty"><Camera size={25}/><strong>{imageUrl&&cameraLoaded?'마지막 수신 영상':'기체 카메라 연결 중'}</strong><span>{!online?'기체 연결을 확인해 주세요.':cameraFailed?'사진을 다시 요청하고 있어요.':'약한 Wi-Fi에서는 사진 전송에 시간이 걸릴 수 있어요.'}</span></div>}<span className="live-camera-label">OV2640 · 실제 영상</span></div>
      <footer><span>{snapshot?.frame.width??0} × {snapshot?.frame.height??0}</span><span>사진 전송 {fmt(snapshot?.frame.transfer_ms==null?null:snapshot.frame.transfer_ms/1000,1)}s</span><span>{fmt(frameAge===null?null:frameAge/1000,1)}초 전</span></footer>
    </section>

    <div className="live-view-tools" role="group" aria-label="실기체 3D 시점">{[{id:'orbit',label:'실기체 전체 보기',icon:Box},{id:'follow',label:'실기체 추적',icon:Focus},{id:'top',label:'실기체 상공 보기',icon:Maximize2}].map(({id,label,icon:Icon})=><button key={id} aria-label={label} aria-pressed={view===id} onClick={()=>setView(id as SimulationView)}><Icon size={17}/></button>)}<i/><button aria-label="실기체 3D 확대" onClick={()=>setZoom(v=>({sequence:v.sequence+1,factor:1.3}))}><Plus size={17}/></button><button aria-label="실기체 3D 축소" onClick={()=>setZoom(v=>({sequence:v.sequence+1,factor:.77}))}><Minus size={17}/></button><button aria-label="실기체 시점 초기화" onClick={()=>setResetKey(k=>k+1)}><RotateCcw size={17}/></button></div>
    <footer className="live-footer"><div><Radio size={16}/><strong>{online?'실제 센서 → 3D 동기화':status}</strong><span>{ready?'자세·높이 실측 / 수평 위치 추정':'3D 화면 연결 확인 필요'}</span></div><button className="live-button" onClick={zero}><Crosshair size={14}/>현재 위치를 원점으로</button><FormSelect aria-label="실기체 3D 품질" value={quality} onValueChange={v=>setQuality(v as 'high'|'standard')} options={[{value:'high',label:'고품질 3D'},{value:'standard',label:'성능 우선'}]}/></footer>
  </div>;
}
