import {MissionFlight,type FlightEvent,type FlightTuning,type MissionPhase,type MissionSnapshot} from './missionFlight';
import {inZone,type MissionWorld} from './missionWorld';

export interface RecordedFrame {
  t:number;x:number;y:number;height:number;yaw:number;pitch:number;roll:number;
  vx:number;vy:number;vz:number;battery:number;phase:MissionPhase;waypoint:number;
  activeTag?:string;confirmed:number;error:number|null;ceiling:number|null;
}
export interface FlightRecord {
  version:1;id:string;createdAt:string;world:MissionWorld;tuning:FlightTuning;learning:boolean;
  tuningChanged?:boolean;
  status:'completed'|'interrupted';frames:RecordedFrame[];events:FlightEvent[];visited:string[];
  summary:{duration:number;distance:number;confirmations:number;meanError:number|null;maxError:number|null;altitudeCompliance:number|null;searchSeconds:number};
}
const RECORDS_KEY='mango-training-records-v1';
const MAX_FRAMES=1800;

/** 시뮬레이션 시간으로 기록한다. 재생용 샘플 간소화와 지표 집계는 분리한다. */
export class FlightRecorder {
  private frames:RecordedFrame[]=[];
  private events=new Map<string,FlightEvent>();
  private interval=200;private last=-Infinity;private start:number;private distance:number;
  private errorSum=0;private errorCount=0;private maxError=0;private observedMs=0;private compliantMs=0;private searchMs=0;private sampledAt:number;
  private initialVisits:number;private initialTime:number;private finished=false;
  private previous:MissionSnapshot;
  private tuningChanged=false;
  markTuningChanged(){this.tuningChanged=true;}
  constructor(private world:MissionWorld,private tuning:FlightTuning,private learning:boolean,state:MissionSnapshot){
    this.start=state.tMs;this.initialTime=state.tMs;this.sampledAt=state.tMs;this.distance=state.travelled;this.initialVisits=state.visited.length;this.previous=state;
    this.capture(state,true);
  }
  capture(state:MissionSnapshot,force=false){
    if(this.finished)return;
    const dt=Math.max(0,state.tMs-this.sampledAt);this.sampledAt=state.tMs;
    const detection=state.detections.find(d=>d.valid&&d.id===state.activeTag),error=detection?Math.hypot(detection.right,detection.forward):null;
    const limits=this.world.zones.filter(z=>z.kind==='height_limit'&&Number.isFinite(z.value)&&inZone(state,z)).map(z=>z.value!);
    const ceiling=limits.length?Math.min(...limits):null;
    if(dt>0){
      if(['search','align','confirm'].includes(state.phase))this.searchMs+=dt;
      if(ceiling!==null&&!['ready','takeoff','land','complete'].includes(state.phase)){this.observedMs+=dt;if(state.height<=ceiling+.15)this.compliantMs+=dt;}
    }
    // 측정 프레임이 새로 들어온 경우에만 정렬 오차 집계에 포함한다.
    if(error!==null&&state.vision.sequence!==this.previous.vision.sequence){this.errorSum+=error;this.errorCount++;this.maxError=Math.max(this.maxError,error);}
    for(const event of state.events)if(event.time*1000>=this.start)this.events.set(`${event.time}-${event.text}`,{...event,time:event.time-this.start/1000});
    const changed=state.phase!==this.previous.phase||state.visited.length!==this.previous.visited.length;
    this.previous={...state,vision:{...state.vision},visited:[...state.visited]};
    if(!force&&!changed&&state.tMs-this.last<this.interval)return;
    if(this.frames.length>=MAX_FRAMES){this.frames=this.frames.filter((_,i)=>i%2===0);this.interval*=2;}
    this.last=state.tMs;
    const frame:RecordedFrame={t:state.tMs-this.start,x:state.x,y:state.y,height:state.height,yaw:state.yaw,pitch:state.pitch,roll:state.roll,vx:state.vx,vy:state.vy,vz:state.vz,battery:state.battery,phase:state.phase,waypoint:state.waypoint,activeTag:state.activeTag,confirmed:state.visited.length,error,ceiling};
    if(this.frames.at(-1)?.t===frame.t)this.frames[this.frames.length-1]=frame;else this.frames.push(frame);
  }
  finish(state:MissionSnapshot):FlightRecord|null {
    if(this.finished)return null;this.capture(state,true);this.finished=true;
    if(state.tMs-this.initialTime<500)return null;
    return {version:1,id:crypto.randomUUID(),createdAt:new Date().toISOString(),world:structuredClone(this.world),tuning:{...this.tuning},learning:this.learning,tuningChanged:this.tuningChanged,status:state.completed?'completed':'interrupted',frames:this.frames,events:[...this.events.values()].sort((a,b)=>a.time-b.time).slice(-400),visited:[...state.visited],
      summary:{duration:(state.tMs-this.start)/1000,distance:Math.max(0,state.travelled-this.distance),confirmations:state.visited.length-this.initialVisits,meanError:this.errorCount?this.errorSum/this.errorCount:null,maxError:this.errorCount?this.maxError:null,altitudeCompliance:this.observedMs?100*this.compliantMs/this.observedMs:null,searchSeconds:this.searchMs/1000}};
  }
}

export function readFlightRecords():FlightRecord[]{try{
  const values=JSON.parse(localStorage.getItem(RECORDS_KEY)??'[]');
  if(!Array.isArray(values))return [];
  return values.filter((r:FlightRecord)=>r?.version===1&&r.world?.stops?.length&&r.world?.tags?.length&&r.summary&&r.tuning&&Array.isArray(r.events)&&Array.isArray(r.visited)&&Array.isArray(r.frames)&&r.frames.length>1&&r.frames.length<=MAX_FRAMES+1&&r.frames.every(f=>[f.t,f.x,f.y,f.height,f.yaw,f.pitch,f.roll,f.vx,f.vy,f.vz].every(Number.isFinite))).slice(0,5);
}catch{return [];}}
export function writeFlightRecords(records:FlightRecord[]){
  const keep=records.slice(0,5);
  while(keep.length){try{localStorage.setItem(RECORDS_KEY,JSON.stringify(keep));return true;}catch{keep.pop();}}
  return false;
}
export function replayFrame(record:FlightRecord,time:number):RecordedFrame {
  const frames=record.frames,t=Math.max(0,Math.min(time,frames.at(-1)!.t));
  let lo=0,hi=frames.length-1;
  while(lo+1<hi){const mid=(lo+hi)>>1;if(frames[mid].t<=t)lo=mid;else hi=mid;}
  const a=frames[lo],b=frames[hi],f=b.t===a.t?0:(t-a.t)/(b.t-a.t);
  const mix=(a:number,b:number)=>a+(b-a)*f;
  return {...(t>=b.t?b:a),t,x:mix(a.x,b.x),y:mix(a.y,b.y),height:mix(a.height,b.height),yaw:a.yaw+Math.atan2(Math.sin(b.yaw-a.yaw),Math.cos(b.yaw-a.yaw))*f,pitch:mix(a.pitch,b.pitch),roll:mix(a.roll,b.roll),vx:mix(a.vx,b.vx),vy:mix(a.vy,b.vy),vz:mix(a.vz,b.vz)};
}
export function replaySnapshot(record:FlightRecord,time:number,base=new MissionFlight(record.world,record.tuning).snapshot()):MissionSnapshot {
  const frame=replayFrame(record,time);
  return {...base,...frame,tMs:frame.t,motors:frame.height>.3?.65:0,activeTag:frame.activeTag,visited:record.visited.slice(0,frame.confirmed),completed:frame.phase==='complete',trail:record.frames.filter(f=>f.t<=time).map(f=>({x:f.x,y:f.y,height:f.height})),vision:{...base.vision,status:'ready'},events:record.events.filter(e=>e.time*1000<=time).slice(-3).reverse()};
}
