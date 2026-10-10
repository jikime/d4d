import {t, useLanguage} from "./i18n";
import {useEffect, useMemo, useRef, useState} from 'react';
import {Activity, ArrowLeft, Battery, Box, Camera, ChevronDown, Crosshair, Eye, EyeOff, Focus, Gauge, Maximize2, Minus, Plus, Radio, RefreshCw, RotateCcw, Settings2, Wifi, WifiOff, X} from 'lucide-react';
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
  useLanguage();
  const [snapshot,setSnapshot]=useState<WifiSnapshot|null>(null),[host,setHost]=useState('');
  const [error,setError]=useState(''),[busy,setBusy]=useState(false),[transportUp,setTransportUp]=useState(false);
  const [clock,setClock]=useState(()=>performance.now()),[view,setView]=useState<SimulationView>('follow');
  const [quality,setQuality]=useState<'high'|'standard'>('high'),[zoom,setZoom]=useState({sequence:0,factor:1}),[resetKey,setResetKey]=useState(0);
  const [primaryView,setPrimaryView]=useState<'camera'|'pose'>('camera'),[insetVisible,setInsetVisible]=useState(true),[focused,setFocused]=useState(false);
  const [imageFit,setImageFit]=useState<'contain'|'cover'>('contain'),[connectionOpen,setConnectionOpen]=useState(false);
  const [mobilePanel,setMobilePanel]=useState<'throttle'|'metrics'|'connection'|null>(null);
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

  return <div className={'live-aircraft'+(focused?' is-focused':'')} data-link-state={online?'online':state} data-inset-visible={insetVisible} data-mobile-panel={mobilePanel??'none'}>
    <header className="live-title">
      <div><span className={'live-indicator '+(online?'is-online':'')}><i/>{t(status)}</span><h2>{t("실기체 영상 관제")}</h2></div>
      <div className="live-title-summary"><span><Battery size={14}/>{t(fmt(motion.voltage,2))} V</span><span>{t(fmt(motion.height,2))} m</span><b className={motion.armed===true?'armed':''}>{t(motion.armed===null?'기체 상태 대기':motion.armed?'ARMED':'DISARMED')}</b></div>
      <button className="live-button" onClick={onBack}><ArrowLeft size={14}/>{t("훈련 화면")}</button>
    </header>

    <div className="live-workspace">
      <aside className="live-left-rail" aria-label={t("스로틀과 연결 상태")}>
        <ThrottlePanel reading={throttle}/>
        <section className="live-card live-connection" aria-label={t("Wi-Fi 기체 연결")}>
          <div className="live-card-heading"><Wifi size={16}/><h3>{t("기체 연결")}</h3><button className="live-card-toggle" aria-label={t(connectionOpen?'기체 연결 설정 접기':'기체 연결 설정 펼치기')} aria-expanded={connectionOpen} aria-controls="live-connection-details" onClick={()=>setConnectionOpen(v=>!v)}>{t(connectionOpen?'접기':'연결 설정')}<ChevronDown size={13}/></button></div>
          <div className="live-network"><strong>{t(ssid)}</strong><small>TagBoard · {t(snapshot?.host??'주소 확인 중')}:{snapshot?.port??3333}</small><small>{t(snapshot?.source==='camera_viewer'?'기존 카메라 뷰어와 데이터 공유':'Wi-Fi 직접 수신')}</small></div>
          {(error||snapshot?.error)&&<p className="live-error" role="alert">{t(error||snapshot?.error)}</p>}
          <div id="live-connection-details" hidden={!connectionOpen}>
          <form onSubmit={e=>{e.preventDefault();void connect();}}>
            <label htmlFor="live-board-host">{t("보드 Wi-Fi 주소")}</label><Input id="live-board-host" value={host} onChange={e=>setHost(e.target.value)} autoComplete="off" placeholder="192.168.0.151" disabled={busy}/>
            <div className="live-connect-actions"><button className="live-button primary" type="submit" disabled={busy||!host}>{busy?<RefreshCw className="spin" size={14}/>:<Wifi size={14}/>}{t(state!=='disconnected'?'연결 확인':'연결')}</button><button className="live-button" type="button" disabled={busy||state==='disconnected'} onClick={()=>void disconnect()}><WifiOff size={14}/>{t("해제")}</button></div>
          </form>
          <dl className="live-link-stats"><div><dt>{t("Wi-Fi 신호")}</dt><dd>{t(fmt(rssi,0))}<small>dBm</small></dd></div><div><dt>{t("최근 보고")}</dt><dd>{t(fmt(snapshot?.received_age_ms==null?null:(snapshot.received_age_ms+transportAge)/1000,1))}<small>{t("초 전")}</small></dd></div><div><dt>{t("비행제어기")}</dt><dd className={motion.fcLinked?'is-good':''}>{t(motion.fcLinked?'응답 확인':'수신 대기')}</dd></div><div><dt>{t("조종 권한")}</dt><dd>{t(motion.authority)}</dd></div></dl>
          <button className="live-text-button" onClick={onDiagnostics}><Settings2 size={13}/>{t("BLE·USB 진단 패널")}</button>
          </div>
        </section>
      </aside>

      <section className="live-visual" aria-label={t("실제 영상과 3D 자세")} data-primary-view={primaryView} data-inset-visible={insetVisible} data-image-fit={imageFit}>
        <div className="live-visual-toolbar">
          <div className="live-view-tabs" role="group" aria-label={t("중앙 화면 선택")}><button aria-pressed={primaryView==='camera'} onClick={()=>setPrimaryView('camera')}><Camera size={14}/>{t("실제 영상")}</button><button aria-pressed={primaryView==='pose'} onClick={()=>setPrimaryView('pose')}><Box size={14}/>{t("3D 자세")}</button></div>
          <div className="live-visual-actions">
            <button className="live-icon" title={t(insetVisible?'보조 화면 숨기기':'보조 화면 표시')} aria-label={t(insetVisible?'보조 화면 숨기기':'보조 화면 표시')} aria-pressed={insetVisible} onClick={()=>setInsetVisible(v=>!v)}>{primaryView==='camera'?<Box size={16}/>:<Camera size={16}/>}</button>
            {primaryView==='camera'&&<button className="live-button live-image-fit" aria-label={t(imageFit==='contain'?'영상으로 화면 채우기':'영상 원본 전체 보기')} title={t(imageFit==='contain'?'화면을 채우며 영상 가장자리를 일부 잘라 표시합니다.':'카메라 원본 전체를 표시합니다.')} aria-pressed={imageFit==='cover'} onClick={()=>setImageFit(v=>v==='contain'?'cover':'contain')}><Maximize2 size={14}/><span>{t(imageFit==='contain'?'화면 채우기':'원본 전체')}</span></button>}
            <button className="live-button live-dashboard-toggle" aria-label={t(focused?'대시보드 표시':'대시보드 숨기기')} title={t(focused?'대시보드 표시 · Esc':'대시보드 숨기기')} aria-pressed={!focused} onClick={()=>setFocused(v=>!v)}>{focused?<Eye size={15}/>:<EyeOff size={15}/>}<span>{t(focused?'대시보드 표시':'대시보드 숨기기')}</span></button>
          </div>
        </div>
        <div className="live-visual-stage">
          <section className="live-camera-panel" aria-label={t("실제 드론 카메라")}>
            <div className="live-camera-image">
              {imageUrl&&<img key={session} src={imageUrl} alt={t("Wi-Fi로 수신한 실제 드론 카메라 영상")} onLoad={()=>{setCameraFailed(false);setLoadedFrame({session,at:(snapshot?.server_ms??0)-(snapshot?.frame.age_ms??0)});}} onError={()=>setCameraFailed(true)}/>}
              {!hasImage&&<div className="live-camera-empty"><Camera size={36}/><strong>{t(state==='disconnected'?'카메라 연결 해제':'기체 카메라 수신 대기')}</strong><span>{t("기체의 Wi-Fi 영상이 이곳에 표시됩니다.")}</span></div>}
              <div className={'live-video-badge '+(imageStale?'is-delayed':'')}><i/>{t(hasImage?(imageStale?'마지막 수신 영상':'실제 카메라'):'영상 대기')}<span>CAM 01</span></div>
              {hasImage&&imageStale&&<div className="live-video-warning">{t(!online?'연결을 확인하고 있습니다.':cameraFailed?'다음 사진을 요청하고 있습니다.':'영상 수신이 지연되고 있습니다.')}{t(" 마지막 사진을 표시합니다.")}</div>}
              {primaryView==='pose'&&<button className="live-inset-expand" aria-label={t("실제 카메라를 중앙으로")} onClick={()=>setPrimaryView('camera')}><Maximize2 size={13}/>{t("영상 확대")}</button>}
            </div>
          </section>
          <section className="live-scene-panel" aria-label={t("센서로 동기화한 3D 자세")}>
            <div className="live-scene-caption"><Box size={12}/><span>{t("실측 자세 · 3D")}</span>{primaryView==='camera'&&<button className="live-icon" aria-label={t("3D 자세를 중앙으로")} onClick={()=>setPrimaryView('pose')}><Maximize2 size={13}/></button>}</div>
            <div className="live-scene">
              <SimulationScene frame={frame} frames={[]} disturbance={frame.disturbance} heightCm={sceneFlight.height*100}
                startXcm={0} startYcm={0} arriveTolCm={0} view={primaryView==='camera'?'follow':view} showGuides={false} showRoom showPath
                dashboardVisible={false} panelVisible={false} quality={quality} running={online} resetKey={resetKey}
                environment="lab" daylight="day" playbackSpeed={1} zoomCommand={zoom} cameraInput={idleCamera}
                compactDisplay={false} sensorCanvas={unusedCamera} flight={sceneFlight} liveTelemetry={{motorPower:motion.motorPower}} onReady={setReady}/>
            </div>
            {primaryView==='pose'&&<div className="live-view-tools" role="group" aria-label={t("실기체 3D 시점")}>{[{id:'orbit',label:'실기체 전체 보기',icon:Box},{id:'follow',label:'실기체 추적',icon:Focus},{id:'top',label:'실기체 상공 보기',icon:Maximize2}].map(({id,label,icon:Icon})=><button key={id} aria-label={t(label)} aria-pressed={view===id} onClick={()=>setView(id as SimulationView)}><Icon size={16}/></button>)}<i/><button aria-label={t("실기체 3D 확대")} onClick={()=>setZoom(v=>({sequence:v.sequence+1,factor:1.3}))}><Plus size={16}/></button><button aria-label={t("실기체 3D 축소")} onClick={()=>setZoom(v=>({sequence:v.sequence+1,factor:.77}))}><Minus size={16}/></button><button aria-label={t("실기체 시점 초기화")} onClick={()=>setResetKey(k=>k+1)}><RotateCcw size={16}/></button></div>}
          </section>
        </div>
        <div className="live-video-status"><span>{snapshot?.frame.width??0} × {snapshot?.frame.height??0}{t(" · JPEG 연속 수신")}</span><span className={imageStale?'is-delayed':'is-good'}>{t(frameAge===null?'사진 수신 대기':fmt(frameAge/1000,1)+'초 전')}</span><span>{t("전송 ")}{t(fmt(snapshot?.frame.transfer_ms==null?null:snapshot.frame.transfer_ms/1000,2))}s</span></div>
        <div className="live-video-readouts"><div><span>{t("스로틀 입력")}</span><strong>{t(throttle.inputUs??'—')}<small>µs</small></strong></div><div><span>{t("기체 높이")}</span><strong>{t(fmt(motion.height,2))}<small>m</small></strong></div><div><span>{t("수평 속도")}</span><strong>{t(fmt(speed,2))}<small>m/s</small></strong></div><div><span>{t("배터리 전류")}</span><strong>{t(fmt(motion.amps,2))}<small>A</small></strong></div></div>
      </section>

      <aside className="live-right-rail" aria-label={t("기체 계측 대시보드")}>
        <section className="live-card live-metrics" aria-label={t("실측 기체 상태")}>
          <div className="live-card-heading"><Gauge size={16}/><h3>{t("기체 상태")}</h3><span>{t(online?'수신 중':'대기')}</span></div>
          <div className="live-primary-metrics"><div><span>{t("거리센서 높이")}</span><output aria-label={t("실기체 높이")}>{t(fmt(motion.height,2))}<small>m</small></output></div><div><span>{t("수평 이동 속도")}</span><output aria-label={t("실기체 속도")}>{t(fmt(speed,2))}<small>m/s</small></output></div></div>
          <div className="live-attitude"><div className={'live-horizon'+(motion.roll===null||motion.pitch===null?' is-unavailable':'')} aria-hidden="true"><div style={{transform:'rotate('+(-1*(motion.roll??0))+'deg) translateY('+((motion.pitch??0)*1.4)+'px)'}}/><b>━ ✛ ━</b></div><dl><div><dt>{t("롤")}</dt><dd><output aria-label={t("실기체 롤")}>{t(fmt(motion.roll))}°</output></dd></div><div><dt>{t("피치")}</dt><dd><output aria-label={t("실기체 피치")}>{t(fmt(motion.pitch))}°</output></dd></div><div><dt>{t("방위각")}</dt><dd><output aria-label={t("실기체 방위각")}>{t(fmt(motion.heading===null?null:(motion.heading+360)%360,0))}°</output></dd></div><div><dt>{t("상승 속도")}</dt><dd>{t(fmt(motion.climbSpeed,2))} m/s</dd></div></dl></div>
          <div className="live-electrical"><span><Battery size={14}/>{t(fmt(motion.voltage,2))} V</span><span>{t(fmt(motion.amps,2))} A</span><b className={motion.armed===true?'armed':''}>{t(motion.armed===null?'상태 대기':motion.armed?'ARMED':'DISARMED')}</b></div>
          <div className="live-section-label">{t("FC 모터별 출력 ")}<small>µs</small></div>
          <div className="live-motors" aria-label={t("실기체 FC 모터 출력")}>{[0,1,2,3].map(i=><div key={i}><span>M{i+1}</span><PulseMeter value={pulsePercent(motion.motors?.[i]??null)} label={t('모터 '+(i+1)+' 출력')} text={t(motion.motors?.[i]===undefined?'수신 대기':motion.motors[i]+' 마이크로초')} tone="motor"/><output>{t(motion.motors?.[i]??'—')}</output></div>)}</div>
          <p className="live-small-note">{t("비행제어기가 보고한 출력값입니다.")}</p>
          <div className="live-section-label">{t("조종기 채널 ")}<small>µs</small></div>
          <div className="live-rc">{[{label:'롤',i:0},{label:'피치',i:1},{label:'요',i:2},{label:'스로틀',i:3}].map(({label,i})=><div key={i}><small>{t(label)}</small><PulseMeter value={pulsePercent(motion.rc?.[i]??null)} label={t(label+' 채널')} text={t(motion.rc?.[i]===undefined?'수신 대기':motion.rc[i]+' 마이크로초')}/><b>{t(motion.rc?.[i]??'—')}</b></div>)}</div>
          <div className="live-position"><span><Crosshair size={13}/>{t("연결 기준 상대 이동")}</span><strong>X {t(fmt(motion.flight.x,2))} <small>m</small> <i/> Y {t(fmt(motion.flight.y,2))} <small>m</small></strong><p className={motion.positionValid?'is-good':''}>{t(motion.positionNote)}</p><small>{t("광류 품질 ")}{t(fmt(motion.flowQuality,0))} / 255</small></div>
          <div className="live-pose-options"><button className="live-button" onClick={zero}><Crosshair size={13}/>{t("상대 위치 원점")}</button><FormSelect aria-label={t("실기체 3D 품질")} value={quality} onValueChange={v=>setQuality(v as 'high'|'standard')} options={[{value:'high',label:'고품질 3D'},{value:'standard',label:'성능 우선'}]}/></div>
        </section>
      </aside>
    </div>
    <nav className="live-mobile-tabs" aria-label={t("관제 패널 선택")}>
      {([{id:'throttle',label:'스로틀',icon:Activity},{id:'metrics',label:'기체 상태',icon:Gauge},{id:'connection',label:'연결',icon:Wifi}] as const).map(({id,label,icon:Icon})=><button key={id} aria-pressed={mobilePanel===id} onClick={()=>{setMobilePanel(p=>p===id?null:id);if(id==='connection')setConnectionOpen(true);}}><Icon size={14}/>{t(label)}</button>)}
      {mobilePanel&&<button className="live-icon" aria-label={t("계측 패널 닫기")} onClick={()=>setMobilePanel(null)}><X size={14}/></button>}
    </nav>
    <footer className="live-footer"><span><Radio size={12}/>{t(snapshot?.source==='camera_viewer'?'카메라 뷰어 데이터 공유':'Wi-Fi 실기체 연결')}</span>{!ready&&<span>{t("3D 화면 연결 확인 필요")}</span>}</footer>
  </div>;
}
