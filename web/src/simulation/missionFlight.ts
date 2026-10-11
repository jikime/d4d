import {clamp} from './flightVisuals';
import {NEUTRAL_INPUT,mergePilotInputs,type FlightSnapshot,type PilotInput} from './pilotFlight';
import {blocked,cellKey,clearSegment,emptyExperience,findPath,inZone,type Experience,type MapPoint,type MissionWorld} from './missionWorld';
import type {TagDetection} from './tagVision';
import {imageFix,measuredTags,VISION_MAX_AGE_MS,type PixelFrame,type VisionStatus} from './pixelVision';
import {advanceWeather,changeWeather,createWeather,weatherActive,weatherWind,WEATHER_LABEL,TAG_LOSS_LIMIT,type WeatherState,type WeatherSettings} from './weather';

export type MissionPhase='ready'|'takeoff'|'transit'|'approach'|'search'|'align'|'confirm'|'dwell'|'land'|'complete'|'manual'|'hold'|'aborted';
export const PHASE_LABEL:Record<MissionPhase,string>={ready:'비행 준비',takeoff:'이륙·상승',transit:'계획 경로 이동',approach:'목적지 감속 접근',search:'목적지 태그 탐색',align:'영상 정밀 정렬',confirm:'인식 확인',dwell:'위치 보정',land:'귀환 착륙',complete:'임무 완료',manual:'수동 조종',hold:'호버 대기',aborted:'환경 훈련 중단'};
export type FlightGuidance='route'|'vision'|'search';
export const GUIDANCE_LABEL:Record<FlightGuidance,string>={route:'계획 경로 이동',vision:'AprilTag 영상 정렬',search:'목적지 태그 탐색'};
export interface FlightTuning {cruise:number;observeHeight:number;tolerance:number;confirmSeconds:number;acceleration:number;wind:number}
export const DEFAULT_TUNING:FlightTuning={cruise:2.2,observeHeight:2.2,tolerance:.075,confirmSeconds:.8,acceleration:1.4,wind:.12};
export function approachProfile(t:FlightTuning,record?:Experience['tags'][string]){
  if(!record?.visits)return {height:t.observeHeight,speed:1};
  const difficulty=clamp((record.searchSeconds-t.confirmSeconds)/6,0,1),error=clamp(record.errorM/t.tolerance,0,1);
  return {height:Math.min(3.5,t.observeHeight+difficulty*.25),speed:clamp(1.1-difficulty*.35-error*.3,.55,1.1)};
}
export interface FlightEvent {time:number;text:string;tone:'ok'|'info'|'warn'}
export interface TelemetrySample {t:number;speed:number;height:number;error:number}
export interface MissionSnapshot extends FlightSnapshot {
  phase:MissionPhase;autonomous:boolean;detections:TagDetection[];activeTag?:string;lock:number;scanSeconds:number;visited:string[];events:FlightEvent[];samples:TelemetrySample[];
  battery:number;voltage:number;current:number;link:number;correction:number;crossTrack:number;motors:number;notice:string;experience:Experience;
  visionSession:number;vision:{status:VisionStatus;sequence:number;frameMs:number;processingMs:number;message:string};guidance:FlightGuidance;
  weather:WeatherState;daylight:'day'|'dusk';aborted:boolean;tagStats:{attempts:number;matches:number;lossSeconds:number};
}
const angle=(target:number,current:number)=>Math.atan2(Math.sin(target-current),Math.cos(target-current));
let nextSession=0;

/** 지점 사이는 계획 경로로 이동하고, 주요 지점의 도착 확인·정밀 정렬은 실제 태그 영상으로 수행한다. */
export class MissionFlight {
  state:MissionSnapshot;world:MissionWorld;tuning:FlightTuning;hiddenTag=false;dusk=false;useExperience=false;
  private trailAt=0;private sampleAt=0;private phaseAt=0;private integral={x:0,y:0};private correctionStart=0;private transitFrom:MapPoint;
  private lastWaypoint=-1;private loggedBoundary=false;private frame?:PixelFrame;private usedSequence=0;private stableAt=-1;private stableFrames=0;
  private fix?:MapPoint;private fixAt=-Infinity;private hoverPoint?:MapPoint;private confirmedWaypoint=-1;
  private recovery:MapPoint[]=[];private visualWaypoint=-1;private awaitingTag=false;
  private failedWaypoints=new Set<number>();
  private observations:{t:number;match:boolean}[]=[];private observationTag?:string;
  constructor(world:MissionWorld,tuning:FlightTuning={...DEFAULT_TUNING},experience=emptyExperience(),weather?:WeatherSettings){
    const launch=world.launch??{...world.start,height:.22};
    this.world=world;this.tuning={...tuning};this.transitFrom={x:launch.x,y:launch.y};
    this.state={...launch,vx:0,vy:0,vz:0,yaw:0,pitch:0,roll:0,tMs:0,travelled:0,mode:'manual',waypoint:0,completed:false,route:world.route,trail:[{...launch}],boundary:false,input:{...NEUTRAL_INPUT},phase:'ready',autonomous:false,detections:[],lock:0,scanSeconds:0,visited:[],events:[],samples:[],battery:100,voltage:16.8,current:0,link:100,correction:0,crossTrack:0,motors:0,notice:world.error,experience:structuredClone(experience),visionSession:++nextSession,vision:{status:'loading',sequence:0,frameMs:-Infinity,processingMs:0,message:''},guidance:'route',weather:createWeather(weather),daylight:'day',aborted:false,tagStats:{attempts:0,matches:0,lossSeconds:0}};
    this.log(world.error||'주요 지점의 태그와 지점 사이의 비행 경로를 불러왔습니다.',world.error?'warn':'info');
  }
  snapshot():MissionSnapshot {const s=this.state;s.daylight=this.dusk?'dusk':'day';return {...s,weather:{...s.weather},tagStats:{...s.tagStats},vision:{...s.vision},input:{...s.input},trail:s.trail.slice(),visited:[...s.visited],events:s.events.slice(),samples:s.samples.slice(),detections:s.detections.slice()};}
  setWeather(patch:Partial<WeatherSettings>){
    const s=this.state;s.weather=changeWeather(s.weather,patch);s.visionSession=++nextSession;s.vision={...s.vision,sequence:0,frameMs:-Infinity};
    this.frame=undefined;this.usedSequence=0;s.detections=[];this.observations=[];s.tagStats={attempts:0,matches:0,lossSeconds:0};this.unlock();
    this.log(`날씨 변경 · ${WEATHER_LABEL[s.weather.kind]} · 강도 ${Math.round(s.weather.intensity*100)}% · 바람 ${s.weather.windSpeed.toFixed(1)}m/s`);
  }
  setVisionStatus(status:VisionStatus,message=''){
    this.state.vision.status=status;this.state.vision.message=message;
    if(status!=='ready'){this.frame=undefined;this.state.detections=[];this.state.vision.sequence=0;this.usedSequence=0;}
  }
  acceptVision(frame:PixelFrame){
    const s=this.state;
    if(frame.session!==s.visionSession||frame.sequence<=s.vision.sequence||frame.tMs>s.tMs+1||s.tMs-frame.tMs>VISION_MAX_AGE_MS)return;
    this.frame=frame;s.vision={status:'ready',sequence:frame.sequence,frameMs:frame.tMs,processingMs:frame.processingMs,message:''};s.detections=measuredTags(frame,this.world.tags);
    if(this.observationTag!==s.activeTag){this.observationTag=s.activeTag;this.observations=[];}
    if(s.guidance!=='route'&&s.activeTag&&!['land','complete','aborted'].includes(s.phase)){
      this.observations.push({t:s.tMs,match:s.detections.some(d=>d.valid&&d.id===s.activeTag)});
    }
    this.observations=this.observations.filter(o=>s.tMs-o.t<=5000).slice(-180);
    s.tagStats={...s.tagStats,attempts:this.observations.length,matches:this.observations.filter(o=>o.match).length};
  }
  log(text:string,tone:FlightEvent['tone']='info'){this.state.events.unshift({time:this.state.tMs/1000,text,tone});this.state.events.length=Math.min(40,this.state.events.length);}
  private phase(phase:MissionPhase){if(this.state.phase===phase)return;this.state.phase=phase;this.phaseAt=this.state.tMs;if(['takeoff','search','dwell','land','complete','hold'].includes(phase))this.log(PHASE_LABEL[phase]+(this.state.activeTag?` · TAG ${this.state.activeTag}`:''),phase==='hold'?'warn':phase==='complete'?'ok':'info');}
  private unlock(){this.state.lock=0;this.stableAt=-1;this.stableFrames=0;}
  start(){const s=this.state;if(s.aborted)return;if(this.world.error){s.notice=this.world.error;return;}if(s.completed||!this.world.stops[s.waypoint])return;
    const path=findPath(this.world,s,this.world.stops[s.waypoint]);if(!path){this.hold('현재 위치에서 임무 경로로 복귀할 수 없습니다.');return;}
    this.recovery=path.slice(1,-1);this.visualWaypoint=-1;this.awaitingTag=false;this.fix=undefined;s.guidance='route';
    this.transitFrom={x:s.x,y:s.y};s.autonomous=true;s.mode='route';s.notice='';this.unlock();this.integral={x:0,y:0};this.hoverPoint=undefined;
    this.phase(s.height<1?'takeoff':'transit');this.log('자동 비행 · 계획 경로로 이동한 뒤 목적지 태그를 영상으로 확인합니다.');
  }
  manual(){if(this.state.phase!=='manual')this.log('수동 조종으로 전환했습니다.');this.state.completed=false;this.state.autonomous=false;this.state.mode='manual';this.state.notice='';this.unlock();this.phase('manual');}
  hold(reason='제동 후 현재 위치에서 호버합니다.'){this.state.autonomous=false;this.state.mode='manual';this.state.notice=reason;this.unlock();this.integral={x:0,y:0};this.phase('hold');}
  replan(world:MissionWorld):boolean {
    const s=this.state;
    if(world.error||!world.stops.length||!clearSegment(world,s,world.stops[0])){this.hold(world.error||'현재 위치에서 새 경로로 연결할 수 없습니다.');return false;}
    // 기체 위치·배터리·비행 시간·실제 궤적을 유지하며 목적지와 영상 세션만 교체한다.
    this.world=world;s.route=world.route;s.waypoint=0;s.completed=false;s.detections=[];s.activeTag=undefined;
    s.visionSession=++nextSession;s.vision={...s.vision,sequence:0,frameMs:-Infinity};this.frame=undefined;this.usedSequence=0;
    this.lastWaypoint=-1;this.confirmedWaypoint=-1;this.failedWaypoints.clear();this.fix=undefined;this.hoverPoint=undefined;
    this.log('현재 위치에서 새 훈련 경로를 적용했습니다.','ok');this.start();return s.autonomous;
  }
  tune(values:Partial<FlightTuning>){this.tuning={cruise:clamp(values.cruise??this.tuning.cruise,.4,3),observeHeight:clamp(values.observeHeight??this.tuning.observeHeight,1.2,3.5),tolerance:clamp(values.tolerance??this.tuning.tolerance,.03,.18),confirmSeconds:clamp(values.confirmSeconds??this.tuning.confirmSeconds,.4,2),acceleration:clamp(values.acceleration??this.tuning.acceleration,.5,2),wind:clamp(values.wind??this.tuning.wind,0,.5)};this.unlock();}
  step(raw:PilotInput,dt:number){if(this.state.aborted||!Number.isFinite(dt)||dt<=0)return;for(let left=Math.min(dt,.25);left>1e-9&&!this.state.aborted;left-=1/60)this.tick(raw,Math.min(left,1/60));}
  private advance(){const s=this.state,stop=this.world.stops[s.waypoint];this.transitFrom={x:s.x,y:s.y};this.integral={x:0,y:0};s.scanSeconds=0;this.unlock();
    if(stop.kind==='land'||s.waypoint===this.world.stops.length-1){this.phase('land');return;}s.waypoint++;this.fix=undefined;this.hoverPoint=undefined;this.visualWaypoint=-1;this.awaitingTag=false;s.guidance='route';this.phase('transit');
  }
  private tick(raw:PilotInput,dt:number){
    const s=this.state,input=mergePilotInputs(NEUTRAL_INPUT,raw),t=this.tuning;s.input=input;s.boundary=false;
    s.weather=advanceWeather(s.weather,dt);s.daylight=this.dusk?'dusk':'day';
    if(input.right||input.forward||input.climb||input.yaw||input.brake){if(s.autonomous||s.phase!=='manual')this.manual();if(s.height<.3)s.height=.3;}
    const stop=this.world.stops[s.waypoint],tag=this.world.tags.find(v=>v.id===stop?.tagId);
    s.activeTag=this.world.stops.slice(s.waypoint).find(point=>point.tagId)?.tagId;
    if(this.lastWaypoint!==s.waypoint){this.lastWaypoint=s.waypoint;this.correctionStart=s.tMs;s.scanSeconds=0;if(tag)this.log(`${stop.name}으로 이동 · TAG ${tag.id}`);}
    const fresh=!!this.frame&&s.vision.status==='ready'&&s.tMs-this.frame.tMs<=VISION_MAX_AGE_MS;
    if(!fresh)s.detections=[];
    let detection=fresh?s.detections.find(v=>v.id===tag?.id&&v.valid):undefined;
    const newFrame=fresh&&s.vision.sequence!==this.usedSequence;
    if(newFrame)this.usedSequence=s.vision.sequence;
    if(detection&&this.frame&&!this.recovery.length){
      const measured=imageFix(this.frame,detection);
      if(clearSegment(this.world,s,measured,.25)){
        this.fix=measured;this.fixAt=this.frame.tMs;
        if(this.visualWaypoint!==s.waypoint){this.visualWaypoint=s.waypoint;this.correctionStart=s.tMs;this.integral={x:0,y:0};this.log(`영상 ID ${detection.code} 포착 · 카메라 정렬로 전환`);}
      }else detection=undefined;
    }
    const speed=Math.hypot(s.vx,s.vy);
    let vx=0,vy=0,vz=0,yawRate=0;
    const learned=this.useExperience&&tag?s.experience.tags[tag.id]:undefined,profile=approachProfile(t,learned),observeHeight=Math.min(profile.height,stop?.ceiling??10);
    const scanHeight=Math.min(Math.max(2.2,observeHeight),stop?.ceiling??10),cruise=Math.min(t.cruise,stop?.speed&&stop.speed>0?stop.speed:3);
    const steer=(point:MapPoint,limit:number,gain=1.6)=>{const dx=point.x-s.x,dy=point.y-s.y,d=Math.hypot(dx,dy),v=Math.min(limit,d*gain,Math.sqrt(2*t.acceleration*Math.max(0,d-.01)));vx=d>.001?dx/d*v:0;vy=d>.001?dy/d*v:0;};
    const awaitImage=(message?:string)=>{
      this.hoverPoint??={x:s.x,y:s.y};
      // 시야 가장자리에서 놓친 태그는 최근 영상 측정점으로만 짧게 재접근한다.
      // 새 카메라 프레임이 없거나 장애물이 있으면 움직이지 않고 현재 위치를 유지한다.
      const reacquire=!message&&fresh&&this.fix&&this.visualWaypoint===s.waypoint&&s.tMs-this.fixAt<=6000&&Math.hypot(s.x-this.fix.x,s.y-this.fix.y)<=1.5&&clearSegment(this.world,s,this.fix);
      if(reacquire){this.hoverPoint={x:s.x,y:s.y};steer(this.fix!,.25);}else steer(this.hoverPoint,.22);
      vz=message?0:clamp(((this.confirmedWaypoint===s.waypoint?observeHeight:scanHeight)-s.height)*1.2,-.45,.65);
      this.unlock();s.guidance='search';this.phase('search');s.scanSeconds=(s.tMs-this.correctionStart)/1000;
      if(!message&&fresh&&s.tMs-this.phaseAt>12000&&!this.failedWaypoints.has(s.waypoint)){
        this.failedWaypoints.add(s.waypoint);const key=cellKey(s);s.experience.cells[key]=Math.min(3,(s.experience.cells[key]??0)+.7);
        this.log('목적지 태그 탐색 지연을 기록했습니다. 재인식하면 자동 비행을 이어갑니다.','warn');
      }
      s.notice=message??(reacquire?`목적지 TAG ${tag?.id} 재탐색 · 마지막 영상 위치로 저속 보정 중입니다.`:`목적지 TAG ${tag?.id} 미인식 · 위치를 유지하며 다시 찾습니다. ID를 확인하면 자동으로 이어갑니다.`);
    };
    if(s.autonomous&&stop){
      const remaining=this.fix?Math.hypot(this.fix.x-s.x,this.fix.y-s.y):Infinity;
      if(s.phase==='takeoff'){
        s.guidance='route';const height=Math.max(stop.height,tag?scanHeight:.6);vz=clamp((height-s.height)*1.4,-.7,.8);
        if(Math.abs(s.height-height)<.14&&Math.abs(s.vz)<.2)this.phase('transit');
      }else if(s.phase==='land'&&this.fix&&this.confirmedWaypoint===s.waypoint&&(!tag||detection||s.height<1.05)){
        // 착륙 직전 태그가 화면을 벗어나면 마지막으로 확인한 영상 측정점을 유지한다.
        s.guidance=tag?'vision':'route';s.notice='';steer(this.fix,.18,.9);vz=remaining<.14?clamp((.22-s.height)*.9,-.45,0):0;
        if(s.height<=.235&&speed<.06){s.height=.22;s.vx=s.vy=s.vz=0;s.autonomous=false;s.completed=true;s.mode='manual';s.experience.runs++;this.phase('complete');}
      }else if(!fresh){
        awaitImage(s.vision.status!=='ready'?(s.vision.message||'AprilTag 영상 디코더 준비 중 · 위치를 유지합니다.'):'카메라의 새 프레임을 기다립니다. 영상이 돌아오면 자동으로 이어갑니다.');
      }else if(!tag&&this.confirmedWaypoint===s.waypoint&&this.fix){
        s.guidance='route';s.notice='';this.phase('dwell');steer(this.fix,.16);vz=clamp((stop.height-s.height)*1.1,-.3,.3);
        if((s.tMs-this.phaseAt)/1000>=Math.max(.65,stop.dwell))this.advance();
      }else if(this.recovery.length||this.visualWaypoint!==s.waypoint){
        if(tag&&this.awaitingTag){awaitImage();}
        else{
          // 이 구간은 계획 경로와 시뮬레이터의 자기 위치로 이동한다. 태그를 추가하거나 검출을 합성하지 않는다.
          this.hoverPoint=undefined;s.guidance='route';s.notice='';
          const target=this.recovery[0]??stop,distance=Math.hypot(target.x-s.x,target.y-s.y),near=distance<Math.max(2,cruise*cruise/(2*t.acceleration)+.4);
          this.phase(near&&!this.recovery.length&&tag?'approach':'transit');steer(target,near?Math.min(cruise,profile.speed):cruise,1.3);
          const height=near&&!this.recovery.length&&tag?scanHeight:stop.height;vz=clamp((height-s.height)*1.2,-.6,.75);
          if(distance>.25)yawRate=clamp(angle(Math.atan2(target.x-s.x,target.y-s.y),s.yaw)*1.8,-.75,.75);
          if(distance<.16&&speed<.3){
            if(this.recovery.length)this.recovery.shift();
            else if(tag){this.awaitingTag=true;this.correctionStart=s.tMs;awaitImage();}
            else if(Math.abs(stop.height-s.height)<.2){
              if(stop.kind==='land'||stop.dwell>.1&&stop.kind!=='goto'){this.fix={x:stop.x,y:stop.y};this.confirmedWaypoint=s.waypoint;this.phase('dwell');}
              else this.advance();
            }
          }
        }
      }else if(!detection||!this.fix){
        // 목적지에서 영상 유도로 전환한 뒤 태그를 놓치면 좌표만으로 도착 처리하지 않는다.
        awaitImage();
      }else{
        this.hoverPoint=undefined;s.notice='';s.guidance='vision';s.correction=remaining;
        if(this.confirmedWaypoint===s.waypoint){
          this.phase('dwell');steer(this.fix,.16,1.4);vz=clamp((observeHeight-s.height)*1.1,-.3,.3);
          if((s.tMs-this.phaseAt)/1000>=Math.max(.65,stop.dwell))this.advance();
        }else if(remaining>.24){
          this.phase('approach');steer(this.fix,Math.min(cruise,profile.speed));vz=clamp((scanHeight-s.height)*1.2,-.6,.65);
          yawRate=clamp(angle(Math.atan2(this.fix.x-s.x,this.fix.y-s.y),s.yaw)*1.8,-.65,.65);this.unlock();
        }else{
          vz=clamp((observeHeight-s.height)*1.1,-.45,.45);s.scanSeconds=(s.tMs-this.correctionStart)/1000;
          const ex=this.fix.x-s.x,ey=this.fix.y-s.y;
          this.integral.x=clamp(this.integral.x+ex*dt,-.3,.3);this.integral.y=clamp(this.integral.y+ey*dt,-.3,.3);
          vx=clamp(ex*1.3+this.integral.x*.3-s.vx*.4,-.32,.32);vy=clamp(ey*1.3+this.integral.y*.3-s.vy*.4,-.32,.32);
          const stable=remaining<=t.tolerance&&speed<.12&&Math.abs(s.vz)<.12&&Math.abs(observeHeight-s.height)<.15;
          if(!stable)this.unlock();
          if(newFrame&&stable){if(this.stableAt<0)this.stableAt=this.frame!.tMs;this.stableFrames++;s.lock=this.stableFrames>=3?clamp((this.frame!.tMs-this.stableAt)/(t.confirmSeconds*1000),0,1):0;}
          this.phase(stable?'confirm':'align');
          if(s.lock>=1){
            this.confirmedWaypoint=s.waypoint;this.phase('dwell');
            const duplicate=stop.kind==='hover'&&this.world.stops[s.waypoint-1]?.tagId===tag!.id&&s.visited.at(-1)===tag!.id;
            if(!duplicate){s.visited.push(tag!.id);this.log(`영상 ID ${tag!.code} 확인 · 오차 ${(remaining*100).toFixed(1)}cm`,'ok');
              const previous=s.experience.tags[tag!.id]??{visits:0,searchSeconds:0,errorM:0},n=previous.visits+1;s.experience.tags[tag!.id]={visits:n,searchSeconds:previous.searchSeconds+(s.scanSeconds-previous.searchSeconds)/n,errorM:previous.errorM+(remaining-previous.errorM)/n};}
          }
        }
      }
    }else if(s.phase==='manual'&&!input.brake){const sin=Math.sin(s.yaw),cos=Math.cos(s.yaw);vx=(input.right*cos+input.forward*sin)*1.8;vy=(-input.right*sin+input.forward*cos)*1.8;vz=input.climb*.8;yawRate=input.yaw*.95;}
    const grounded=(s.phase==='ready'||s.phase==='complete')&&s.height<.25;
    const wind=grounded||s.phase==='hold'?0:t.wind;
    // 지속 외란을 적분 보상하는 속도 응답. 가속도와 회전 속도를 제한해 자세를 연속적으로 만든다.
    const weatherVelocity=weatherWind(s.weather),weatherGain=grounded||s.phase==='hold'?0:.065;
    const wx=wind*(.6+Math.sin(s.tMs/3100)*.4)+weatherVelocity.x*weatherGain,wy=wind*Math.cos(s.tMs/4700)*.5+weatherVelocity.y*weatherGain;
    const before={x:s.x,y:s.y,height:s.height},oldVx=s.vx,oldVy=s.vy,acc=input.brake?3.5:t.acceleration;
    const rawAx=(vx-s.vx)*3+wx,rawAy=(vy-s.vy)*3+wy,accScale=Math.min(1,acc/Math.max(1e-8,Math.hypot(rawAx,rawAy))),ax=rawAx*accScale,ay=rawAy*accScale;s.vx+=ax*dt;s.vy+=ay*dt;s.vz+=clamp((vz-s.vz)*3,-1.2,1.2)*dt;
    const next={x:s.x+s.vx*dt,y:s.y+s.vy*dt};
    if(blocked(this.world,next,.25)||!clearSegment(this.world,s,next,.25)){s.vx=s.vy=0;s.boundary=true;if(!this.loggedBoundary){this.loggedBoundary=true;this.hold('건물 또는 훈련장 경계에 접근해 정지했습니다.');}}else {s.x=next.x;s.y=next.y;this.loggedBoundary=false;}
    s.height=clamp(s.height+s.vz*dt,.22,10);if(s.height<=.22||s.height>=10)s.vz=0;
    s.yaw=angle(s.yaw+yawRate*dt,0);const sin=Math.sin(s.yaw),cos=Math.cos(s.yaw),blend=1-Math.exp(-dt*6);
    s.roll+=(-Math.atan2((s.vx-oldVx)/dt*cos-(s.vy-oldVy)/dt*sin,9.81)-s.roll)*blend;s.pitch+=(-Math.atan2((s.vx-oldVx)/dt*sin+(s.vy-oldVy)/dt*cos,9.81)-s.pitch)*blend;
    s.motors+=(grounded?0:1-s.motors)*Math.min(1,dt*3);if(grounded)s.motors=Math.max(0,s.motors-dt*2);
    s.current=grounded?0:11+Math.hypot(s.vx,s.vy)*1.4+Math.max(0,s.vz)*4;s.battery=Math.max(0,s.battery-s.current*dt/3600/1.8*100);s.voltage=13.6+3.2*s.battery/100-s.current*.018;
    s.link=clamp(100-Math.hypot(s.x-this.world.start.x,s.y-this.world.start.y)*.55-this.world.zones.filter(z=>z.kind==='jam'&&inZone(s,z)).reduce((n,z)=>n+(z.value??1)*85,0),0,100);
    if(s.autonomous&&s.link<10&&this.world.linkPolicy==='abort')this.hold('임무 정책에 따라 통신 저하 시 호버합니다.');
    if(s.autonomous&&s.battery<15)this.hold('모의 배터리가 15% 미만입니다.');
    const dx=(stop?.x??s.x)-this.transitFrom.x,dy=(stop?.y??s.y)-this.transitFrom.y;s.crossTrack=Math.abs(dx*(s.y-this.transitFrom.y)-dy*(s.x-this.transitFrom.x))/Math.max(.01,Math.hypot(dx,dy));
    s.travelled+=Math.hypot(s.x-before.x,s.y-before.y,s.height-before.height);s.tMs+=dt*1000;
    s.tagStats.lossSeconds=s.autonomous&&s.guidance==='search'&&!detection&&fresh?s.tagStats.lossSeconds+dt:0;
    if(s.tagStats.lossSeconds>=TAG_LOSS_LIMIT&&weatherActive(s.weather)){
      s.aborted=true;s.autonomous=false;s.mode='manual';s.phase='aborted';s.vx=s.vy=s.vz=0;
      s.notice='환경 훈련 중단 · 목적지 태그를 30초 동안 읽지 못했습니다. 날씨·적설을 조정하고 초기화해 주세요.';this.log(s.notice,'warn');
    }
    if(s.tMs-this.trailAt>100&&Math.hypot(s.x-s.trail.at(-1)!.x,s.y-s.trail.at(-1)!.y,s.height-s.trail.at(-1)!.height)>.025){s.trail.push({x:s.x,y:s.y,height:s.height});if(s.trail.length>4000)s.trail.shift();this.trailAt=s.tMs;}
    if(s.tMs-this.sampleAt>=250){s.samples.push({t:s.tMs/1000,speed:Math.hypot(s.vx,s.vy,s.vz),height:s.height,error:s.correction});if(s.samples.length>100)s.samples.shift();this.sampleAt=s.tMs;}
  }
}
