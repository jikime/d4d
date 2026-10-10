import {useEffect,useMemo,useRef,useState} from 'react';
import {ArrowRight,Check,Grid2X2,LoaderCircle,Route,Send,Sparkles,X} from 'lucide-react';
import {Button} from './components/ui/button';
import {Textarea} from './components/ui/textarea';
import {FormSelect,FormSlider} from './components/form-controls';
import {t,useLanguage} from './i18n';
import {DEMO_MODE} from './appMode';
import {createTrainingPlans,DEFAULT_INTENT,gridCells,gridSpec,parseTrainingInstruction,validateIntent,type TrainingIntent,type TrainingPlan} from './simulation/gridTraining';
import type {MissionScenario,MissionWorld} from './simulation/missionWorld';
import type {MissionSnapshot} from './simulation/missionFlight';
import './gridTraining.css';

interface Props {scenario:MissionScenario;flight:MissionSnapshot;initial?:TrainingIntent;onClose:()=>void;onPreview:(world:MissionWorld|null)=>void;onApply:(plan:TrainingPlan)=>void}
export default function GridTrainingPanel({scenario,flight,initial,onClose,onPreview,onApply}:Props){
  useLanguage();
  const cells=useMemo(()=>gridCells(scenario),[scenario]),g=gridSpec(scenario)!;
  const [intent,setIntent]=useState<TrainingIntent>(()=>structuredClone(initial??DEFAULT_INTENT));
  const [mode,setMode]=useState('visit'),[instruction,setInstruction]=useState(''),[plans,setPlans]=useState<TrainingPlan[]>([]),[selected,setSelected]=useState(0);
  const [error,setError]=useState(''),[busy,setBusy]=useState(false),[source,setSource]=useState(''),request=useRef<AbortController|null>(null);
  const active=plans[selected],results=useRef<HTMLDivElement>(null);
  const home=cells.find(c=>Math.hypot(c.x+scenario.field.width_m/2-scenario.goal.x,c.y+scenario.field.height_m/2-scenario.goal.y)<.1)?.id??'HOME';
  useEffect(()=>{if(plans.length)results.current?.scrollIntoView({block:'nearest'});},[plans]);
  useEffect(()=>{onPreview(active?.world??null);return()=>onPreview(null);},[active,onPreview]);
  useEffect(()=>()=>{request.current?.abort();request.current=null;},[]);
  const change=(next:TrainingIntent)=>{request.current?.abort();request.current=null;setBusy(false);setIntent(next);setPlans([]);setError('');setSource('');};
  const toggle=(id:string)=>{const key=mode as 'visit'|'closed'|'low';change({...intent,[key]:intent[key].includes(id)?intent[key].filter(v=>v!==id):[...intent[key],id]});};
  const generate=(next=intent)=>{try{setPlans(createTrainingPlans(scenario,next,flight));setSelected(0);setError('');}catch(e){setPlans([]);setError(e instanceof Error?e.message:String(e));}};
  const interpret=async()=>{
    if(!instruction.trim())return;request.current?.abort();const controller=new AbortController();request.current=controller;setBusy(true);setError('');setPlans([]);
    const timeout=setTimeout(()=>controller.abort(),12000);
    try{
      let result:TrainingIntent|undefined,provider='';
      if(!DEMO_MODE)try{
        const response=await fetch('/api/llm/patch',{method:'POST',headers:{'Content-Type':'application/json'},signal:controller.signal,body:JSON.stringify({scenario:{...scenario,training_intent:intent},instruction:`Interpret this instruction ONLY for a virtual grid inspection exercise. Return an add/replace JSON Patch at /training_intent with {visit:string[],closed:string[],low:string[],altitude:number,lowAltitude:number}. Preserve unspecified settings. Cell IDs: ${cells.map(c=>c.id).join(',')}. Visit order must be preserved. Altitudes in meters, range 1.8 to 4. No coordinates, aircraft controls, or changes to any other scenario fields. User instruction: ${instruction}`})});
        if(response.ok){const data=await response.json();if(data.provider&&data.provider!=='none'){result=validateIntent(data.scenario?.training_intent,cells);provider=data.provider;}}
      }catch(e){if(controller.signal.aborted&&request.current!==controller)return;}
      if(request.current!==controller)return;
      if(!result){result=parseTrainingInstruction(instruction,intent,cells);setSource(DEMO_MODE?'규칙 기반 해석':'규칙 기반 해석 · AI 미연결');}else setSource(`AI · ${provider}`);
      setIntent(result);generate(result);
    }catch(e){if(request.current===controller)setError(e instanceof Error?e.message:String(e));}
    finally{clearTimeout(timeout);if(request.current===controller)setBusy(false);}
  };
  const path=active?.cells.map(id=>cells.find(c=>c.id===id)!).map(c=>`${c.col+.5},${g.rows-c.row-.5}`).join(' ');
  const profile=active?.waypoints??[];
  return <section className="grid-planner" aria-label={t('격자 훈련 계획')}>
    <header><div><span><Grid2X2 size={13}/> GRID MISSION LAB</span><h2>{t('상황별 순찰 계획')}</h2></div><button type="button" onClick={onClose} aria-label={t('격자 계획 닫기')}><X size={18}/></button></header>
    <div className="grid-planner-scroll">
      <p className="grid-intro">{t('구역을 선택하고 경로·고도를 미리 확인하세요. 현재 비행은 검토 중 일시정지됩니다.')}</p>
      <div className="grid-board-head"><strong>{g.columns} × {g.rows} <span>{t('고정 태그 맵')}</span></strong><span>{scenario.tags.length} TAGS · {g.cell_m}m / CELL</span></div>
      <div className="grid-edit-modes" role="group" aria-label={t('구역 편집 모드')}>{[['visit','방문 지점'],['closed','통로 폐쇄'],['low','고도 제한']].map(([id,label])=><button key={id} type="button" aria-pressed={mode===id} onClick={()=>setMode(id)}>{t(label)}</button>)}</div>
      <div className="grid-board" style={{gridTemplateColumns:`repeat(${g.columns},1fr)`}}>
        {[...cells].sort((a,b)=>b.row-a.row||a.col-b.col).map(c=>{
          const visit=intent.visit.indexOf(c.id),closed=intent.closed.includes(c.id),low=intent.low.includes(c.id);
          return <button key={c.id} type="button" data-cell={c.id} disabled={c.blocked||busy} className={`${c.blocked?'is-building':''} ${closed?'is-closed':''} ${low?'is-low':''} ${visit>=0?'is-stop':''}`} aria-label={t(`구역 ${c.id} · ${c.blocked?'건물':closed?'폐쇄':low?'고도 제한':'개방'}`)} aria-pressed={intent[mode as 'visit'|'closed'|'low'].includes(c.id)} onClick={()=>toggle(c.id)}><b>{c.id}</b><small>{c.blocked?t('건물'):closed?'×':visit>=0?`${visit+1}`:low?`${intent.lowAltitude}m`:`${c.tagId}`}</small></button>;
        })}
        {path&&<svg viewBox={`0 0 ${g.columns} ${g.rows}`} preserveAspectRatio="none" aria-hidden="true"><polyline points={path} fill="none" stroke="#b7e4bf" strokeWidth=".05" strokeDasharray=".09 .05"/></svg>}
      </div>
      <div className="grid-legend"><span><i/>{t('방문')}</span><span><i/>{t('폐쇄')}</span><span><i/>{t('고도 제한')}</span><span><i/>{t('건물')}</span></div>
      <div className="grid-order"><span>{t('방문 순서')}</span><strong>{intent.visit.join(' → ')||t('선택 필요')} → {home}</strong></div>
      <div className="grid-altitude"><label><span>{t('순항 고도')}<b>{intent.altitude.toFixed(1)}m</b></span><FormSlider aria-label={t('격자 순항 고도')} min={1.8} max={4} step={.1} value={intent.altitude} onValueChange={value=>change({...intent,altitude:value,lowAltitude:Math.min(value,intent.lowAltitude)})}/></label><label><span>{t('제한 구역 고도')}<b>{intent.lowAltitude.toFixed(1)}m</b></span><FormSlider aria-label={t('제한 구역 고도')} min={1.8} max={intent.altitude} step={.1} value={intent.lowAltitude} onValueChange={value=>change({...intent,lowAltitude:value})}/></label></div>
      <div className="grid-presets"><FormSelect aria-label={t('훈련 상황 예시')} value="choose" onValueChange={value=>{if(value==='normal')change(structuredClone(DEFAULT_INTENT));if(value==='closure')change({...intent,closed:[...new Set([...intent.closed,'D2'])]});if(value==='ceiling')change({...intent,low:[...new Set([...intent.low,'C1','D1'])]});}} options={[{value:'choose',label:'훈련 상황 추가'},{value:'normal',label:'기본 순찰로 초기화'},{value:'closure',label:'D2 통로 폐쇄'},{value:'ceiling',label:'C1·D1 고도 제한'}]}/><Button type="button" className="grid-generate" disabled={busy} onClick={()=>generate()}><Route size={14}/>{t('후보 코스 생성')}</Button></div>
      <details className="grid-instruction"><summary><Sparkles size={14}/>{t('자연어 지시')}</summary><Textarea aria-label={t('훈련 지시')} value={instruction} maxLength={1000} onChange={e=>{request.current?.abort();request.current=null;setBusy(false);setInstruction(e.target.value);setPlans([]);setSource('');}} placeholder={t('예: H1, H6, A6 방문; D2 폐쇄; C1, D1 고도 제한 2m')}/><p>{t(DEMO_MODE?'한국어·영어 예시 문법을 사용해 방문 지점과 고도 조건을 지정하세요.':'한국어·영어 지원. AI 연결 시 지시를 해석하고, 미연결 시 표시된 문법을 사용합니다.')}</p><Button type="button" disabled={busy||!instruction.trim()} onClick={()=>void interpret()}>{busy?<LoaderCircle className="spin" size={14}/>:<Send size={14}/>} {t(busy?'지시 해석 중':'지시 해석')}</Button>{source&&<small role="status">{t(source)}</small>}</details>
      {error&&<p className="grid-error" role="alert">{t(error)}</p>}
      {plans.length>0&&<div className="grid-results" ref={results}><div className="grid-results-head"><strong>{t('검증된 후보 코스')}</strong><span>{plans.length} {t('개')}</span></div>{plans.map((plan,i)=><button type="button" key={plan.id} aria-pressed={selected===i} onClick={()=>setSelected(i)}><span className="grid-route-number">0{i+1}</span><div><strong>{t(plan.name)}</strong><small>{plan.distance.toFixed(0)}m · ~{Math.ceil(plan.seconds/60)}{t('분')} · {plan.cells.length} {t('개 통과 구역')}</small></div>{selected===i?<Check size={16}/>:<ArrowRight size={16}/>}</button>)}<div className="grid-profile"><span>{t('계획 고도')}<b>{Math.min(...profile.map(p=>p.height!)).toFixed(1)}–{Math.max(...profile.map(p=>p.height!)).toFixed(1)}m</b></span><svg viewBox="0 0 400 64" role="img" aria-label={t('경유점별 목표 고도')}><path d="M0 52H400 M0 26H400" stroke="#94a27425"/><polyline points={profile.map((p,i)=>`${i/Math.max(1,profile.length-1)*400},${60-p.height!*13}`).join(' ')} fill="none" stroke="#c9daa4" strokeWidth="2"/></svg></div><ul>{active.reasons.map(reason=><li key={reason}>{t(reason)}</li>)}</ul></div>}
    </div>
    <footer><span><i/> {t('가상 훈련 전용')}</span><Button type="button" disabled={!active||busy} onClick={()=>active&&onApply(active)}>{t('선택한 코스로 비행')}<ArrowRight size={15}/></Button></footer>
  </section>;
}
