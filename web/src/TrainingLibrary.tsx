import {useEffect,useMemo,useRef,useState} from 'react';
import {Download,FolderOpen,Pause,Play,Save,Trash2,Upload} from 'lucide-react';
import {Dialog,DialogContent,DialogDescription,DialogTitle} from './components/ui/dialog';
import {Input} from './components/ui/input';
import {Button} from './components/ui/button';
import {FormSelect,FormSlider} from './components/form-controls';
import {locale,t,useLanguage} from './i18n';
import {createTrainingPlans,DEFAULT_INTENT,gridSpec,type TrainingPlan} from './simulation/gridTraining';
import {downloadJson,readCourses,restoreCourse,saveCourseData,scenarioSignature,writeCourses,type SavedCourse} from './simulation/trainingStorage';
import {replayFrame,replaySnapshot,sameWeatherConditions,type FlightRecord} from './simulation/flightRecording';
import {MissionFlight} from './simulation/missionFlight';
import {flightSceneFrame} from './simulation/pilotFlight';
import type {MissionScenario} from './simulation/missionWorld';
import SimulationScene,{type SimulationView} from './SimulationScene';
import {WEATHER_LABEL,weatherWind} from './simulation/weather';
import './trainingLibrary.css';

const CAMERA_IDLE={panX:0,panY:0,rotateX:0,rotateY:0};
function FlightReview({record,previous}:{record:FlightRecord;previous?:FlightRecord}){
  useLanguage();
  const [cursor,setCursor]=useState(0),[playing,setPlaying]=useState(false),[view,setView]=useState<SimulationView>('orbit'),[rate,setRate]=useState(1),[zoom,setZoom]=useState({sequence:0,factor:1});
  const emptyCanvas=useRef<HTMLCanvasElement|null>(null),duration=record.frames.at(-1)!.t;
  useEffect(()=>{if(!playing)return;let raf=0,last=0,drawAt=0,elapsed=cursor;const tick=(now:number)=>{if(last)elapsed=Math.min(duration,elapsed+Math.min(100,now-last)*rate);last=now;if(now-drawAt>=50||elapsed>=duration){setCursor(elapsed);drawAt=now;}if(elapsed>=duration)setPlaying(false);else raf=requestAnimationFrame(tick);};raf=requestAnimationFrame(tick);const pause=()=>{if(document.hidden)setPlaying(false);};document.addEventListener('visibilitychange',pause);return()=>{cancelAnimationFrame(raf);document.removeEventListener('visibilitychange',pause);};},[playing,rate,duration]);
  const base=useMemo(()=>new MissionFlight(record.world,record.tuning).snapshot(),[record]);
  const snapshot=useMemo(()=>replaySnapshot(record,cursor,base),[record,cursor,base]);
  const frame=useMemo(()=>flightSceneFrame(snapshot),[snapshot]);
  const summary=record.summary,weather=snapshot.weather,wind=weatherWind(weather);
  const chart=record.frames.map(f=>`${f.t/duration*600},${94-Math.min(6,f.height)/6*84}`).join(' ');
  return <div className="flight-review">
    <div className="record-status"><b>{t(record.status==='completed'?'임무 완료':record.status==='failed'?'환경 훈련 중단':'중단된 비행')}</b><span>{t(record.learning?'경험 적용':'기본 제어')} · {new Date(record.createdAt).toLocaleString(locale())}</span><Button variant="outline" size="sm" onClick={()=>downloadJson(record,`mango-flight-${record.id.slice(0,8)}.json`)}><Download size={13}/>{t('결과 내보내기')}</Button></div>
    <div className="record-stats"><div><span>{t('비행 시간')}</span><b>{summary.duration.toFixed(1)} s</b></div><div><span>{t('이동 거리')}</span><b>{summary.distance.toFixed(1)} m</b></div><div><span>{t('태그 확인')}</span><b>{summary.confirmations}</b></div><div><span>{t('평균 / 최대 정렬 오차')}</span><b>{summary.meanError===null?'—':(summary.meanError*100).toFixed(1)} / {summary.maxError===null?'—':(summary.maxError*100).toFixed(1)} cm</b></div><div><span>{t('고도 제한 준수')}</span><b>{summary.altitudeCompliance===null?'—':summary.altitudeCompliance.toFixed(1)+'%'}</b></div><div><span>{t('탐색·정렬 시간')}</span><b>{summary.searchSeconds.toFixed(1)} s</b></div></div>
    {previous&&<p className="record-comparison">{t('직전 동일 코스·설정·날씨 대비')} · {t('비행 시간')} {signed(summary.duration-previous.summary.duration)}s · {t('탐색·정렬 시간')} {signed(summary.searchSeconds-previous.summary.searchSeconds)}s <span>{t('경험 적용 전후의 관측값 비교입니다.')}</span></p>}
    {record.frames[0].weather&&<div className="record-weather"><div><strong>{t(WEATHER_LABEL[weather.kind])} · {Math.round(weather.intensity*100)}%</strong><span>{t('현재 풍속')} {wind.speed.toFixed(1)} m/s · {t('현재 적설')} {Math.round(weather.snowCover*100)}%</span></div><div><span>{t('목적지 인식률')}</span><b>{summary.detectionRate==null?'—':summary.detectionRate.toFixed(1)+'%'}</b></div><div><span>{t('최장 미인식')}</span><b>{(summary.maxTagLoss??0).toFixed(1)} s</b></div><div><span>{t('평균 경로 오차')}</span><b>{(summary.meanCrossTrack??0).toFixed(2)} m</b></div></div>}
    <div className="record-scene"><SimulationScene frame={frame} frames={[]} disturbance={frame.disturbance} heightCm={snapshot.height*100} startXcm={record.world.start.x*100} startYcm={record.world.start.y*100} arriveTolCm={record.tuning.tolerance*100} view={view} showGuides showRoom showPath dashboardVisible={false} panelVisible={false} quality="standard" running={playing} resetKey={0} environment="field" daylight={snapshot.daylight} playbackSpeed={rate} zoomCommand={zoom} cameraInput={CAMERA_IDLE} compactDisplay={false} emphasizeAircraft sensorCanvas={emptyCanvas} flight={snapshot} mission={record.world} missionState={snapshot}/><span className="record-watermark">{t('기록 재생 · 가상 훈련')}</span><div className="record-camera-tools"><FormSelect aria-label={t('재생 시점')} value={view} onValueChange={v=>setView(v as SimulationView)} options={[{value:'orbit',label:'훈련장 전체'},{value:'follow',label:'기체 추적'},{value:'top',label:'상공 지도'}]}/><Button variant="outline" aria-label={t('3D 확대')} onClick={()=>setZoom(v=>({sequence:v.sequence+1,factor:1.25}))}>+</Button><Button variant="outline" aria-label={t('3D 축소')} onClick={()=>setZoom(v=>({sequence:v.sequence+1,factor:.8}))}>−</Button></div></div>
    <div className="record-playback"><Button aria-label={t(playing?'재생 일시정지':'기록 재생')} onClick={()=>{if(cursor>=duration)setCursor(0);setPlaying(v=>!v);}}>{playing?<Pause size={15}/>:<Play size={15}/>}</Button><FormSlider aria-label={t('비행 기록 시간')} min={0} max={duration/1000} step={.1} value={cursor/1000} onValueChange={v=>{setPlaying(false);setCursor(v*1000);}}/><output>{(cursor/1000).toFixed(1)} / {(duration/1000).toFixed(1)}s</output><FormSelect aria-label={t('재생 배속')} value={String(rate)} onValueChange={v=>setRate(Number(v))} options={[1,2,4].map(v=>({value:String(v),label:`${v}×`}))}/></div>
    <div className="record-altitude"><span>{t('고도 이력')} · {replayFrame(record,cursor).height.toFixed(2)} m</span><svg viewBox="0 0 600 105" role="img" aria-label={t('시간별 비행 고도')}><path d="M0 10H600M0 52H600M0 94H600" stroke="#71805244"/><polyline points={chart} fill="none" stroke="#c7dca0" strokeWidth="2"/><line x1={cursor/duration*600} x2={cursor/duration*600} y1={0} y2={105} stroke="#ddba78"/></svg></div>
    <p className="record-note">{t('저장된 위치·자세와 날씨를 재생합니다. 카메라 영상 녹화는 포함하지 않습니다.')} {t('정렬 오차는 유효한 목적지 태그 관측 기준입니다. 고도 준수는 제한 구역 안에서 이착륙을 제외하고 허용 여유 0.15m로 집계합니다.')}</p>
    <div className="record-events" aria-label={t('기록된 비행 이벤트')}>{record.events.map((event,i)=><button key={i} className={Math.abs(event.time*1000-cursor)<500?'is-current':''} onClick={()=>{setPlaying(false);setCursor(Math.min(duration,event.time*1000));}}><time>{event.time.toFixed(1)}s</time><span>{t(event.text)}</span></button>)}</div>
  </div>;
}
function signed(value:number){return `${value>0?'+':''}${value.toFixed(1)}`;}

export default function TrainingLibrary({scenario,activePlan,records,initialTab,onLoad,onDefaults,onClose}:{scenario:MissionScenario;activePlan:TrainingPlan|null;records:FlightRecord[];initialTab:'courses'|'records';onLoad:(plan:TrainingPlan)=>void;onDefaults:()=>void;onClose:()=>void}){
  useLanguage();
  const [tab,setTab]=useState(initialTab),[courses,setCourses]=useState(readCourses),[name,setName]=useState(''),[message,setMessage]=useState(''),[error,setError]=useState(''),[selected,setSelected]=useState(records[0]?.id??'');
  const upload=useRef<HTMLInputElement>(null);
  const current=useMemo(()=>activePlan??(gridSpec(scenario)?createTrainingPlans(scenario,DEFAULT_INTENT)[0]:null),[activePlan,scenario]);
  const record=records.find(r=>r.id===selected)??records[0];
  const previous=record?records.slice(records.indexOf(record)+1).find(r=>sameWeatherConditions(r,record)&&!r.tuningChanged&&!record.tuningChanged&&r.status==='completed'&&record.status==='completed'&&r.world.key===record.world.key&&JSON.stringify(r.tuning)===JSON.stringify(record.tuning)&&r.learning!==record.learning):undefined;
  const save=()=>{if(!current)return;if(courses.length>=12){setError('코스는 최대 12개까지 저장할 수 있습니다. 기존 코스를 내보내거나 삭제해 주세요.');return;}try{const next=[saveCourseData(scenario,current,name),...courses];writeCourses(next);setCourses(next);setError('');setMessage('코스를 저장했습니다.');setName('');}catch{setError('브라우저 저장 공간이 부족합니다. 파일로 내보내 주세요.');}};
  const load=(course:SavedCourse)=>{try{onLoad(restoreCourse(scenario,course));onClose();}catch(e){setError(e instanceof Error?e.message:String(e));}};
  const remove=(id:string)=>{try{const next=courses.filter(c=>c.id!==id);writeCourses(next);setCourses(next);}catch{setError('저장한 코스를 변경하지 못했습니다.');}};
  return <Dialog open onOpenChange={open=>{if(!open)onClose();}}><DialogContent><header><small>MANGO / TRAINING LIBRARY</small><DialogTitle>{t('코스·비행 기록')}</DialogTitle><DialogDescription>{t('코스를 보관하고 비행 결과를 비교해 보세요. 기록은 이 브라우저에 저장됩니다.')}</DialogDescription></header><nav className="training-library-tabs"><Button variant={tab==='courses'?'default':'outline'} disabled={!current} onClick={()=>setTab('courses')}>{t('저장한 코스')}</Button><Button variant={tab==='records'?'default':'outline'} onClick={()=>setTab('records')}>{t('비행 결과·재생')} <span>{records.length}</span></Button></nav>
    <div className="training-library-body">{tab==='courses'&&current?<>
      <div className="course-save"><Input aria-label={t('코스 이름')} placeholder={t('코스 이름')} maxLength={80} value={name} onChange={e=>setName(e.target.value)}/><Button onClick={save}><Save size={14}/>{t('현재 코스 저장')}</Button></div>
      <p className="record-note">{t('불러온 코스는 저장 당시의 시작 위치에서 비행 준비 상태로 열립니다.')}</p>
      <div className="course-files"><Button variant="outline" onClick={()=>downloadJson(saveCourseData(scenario,current,name),'mango-course.json')}><Download size={14}/>{t('코스 내보내기')}</Button><Button variant="outline" onClick={()=>upload.current?.click()}><Upload size={14}/>{t('코스 파일 불러오기')}</Button><Input ref={upload} className="sr-only" type="file" accept=".json,application/json" aria-label={t('코스 JSON 파일')} onChange={async e=>{const file=e.target.files?.[0];e.target.value='';if(!file)return;try{if(file.size>200_000)throw new Error('코스 파일은 200KB 이하여야 합니다.');const plan=restoreCourse(scenario,JSON.parse(await file.text()));onLoad(plan);onClose();}catch(err){setError(err instanceof SyntaxError?'올바른 코스 파일이 아닙니다.':err instanceof Error?err.message:'올바른 코스 파일이 아닙니다.');}}}/></div>
      {courses.length?<div className="course-list">{courses.map(course=><div key={course.id}><FolderOpen size={18}/><div><b>{t(course.name)}</b><small>{course.intent?.visit?.join(' → ')} · {new Date(course.savedAt).toLocaleDateString(locale())}</small>{course.map!==scenarioSignature(scenario)&&<small>{t('다른 훈련장의 코스')}</small>}</div><Button variant="outline" size="sm" onClick={()=>load(course)} disabled={course.map!==scenarioSignature(scenario)}>{t('불러오기')}</Button><Button variant="ghost" size="icon" aria-label={t('코스 삭제')} onClick={()=>remove(course.id)}><Trash2 size={15}/></Button></div>)}</div>:<div className="training-empty"><FolderOpen size={26}/><b>{t('저장한 코스가 없습니다.')}</b><p>{t('현재 코스를 저장하면 다음 방문에도 불러올 수 있습니다.')}</p></div>}
      <Button className="course-default" variant="outline" onClick={()=>{onDefaults();onClose();}}>{t('기본 코스로 복원')}</Button>
    </>:record?<><FormSelect aria-label={t('비행 기록 선택')} value={record.id} onValueChange={setSelected} options={records.map(r=>({value:r.id,label:`${new Date(r.createdAt).toLocaleString(locale())} · ${t(r.status==='completed'?'임무 완료':r.status==='failed'?'환경 훈련 중단':'중단된 비행')} · ${t(r.learning?'경험 적용':'기본 제어')}`}))}/><FlightReview key={record.id} record={record} previous={previous}/></>:<div className="training-empty"><Play size={26}/><b>{t('아직 비행 기록이 없습니다.')}</b><p>{t('비행을 완료하거나 초기화하면 결과와 궤적이 여기에 저장됩니다.')}</p></div>}
    {message&&<p role="status" className="library-message">{t(message)}</p>}{error&&<p role="alert" className="library-error">{t(error)}</p>}</div>
  </DialogContent></Dialog>;
}
