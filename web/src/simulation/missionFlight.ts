import {clamp} from './flightVisuals';
import {NEUTRAL_INPUT,mergePilotInputs,type FlightSnapshot,type PilotInput} from './pilotFlight';
import {blocked,cellKey,clearSegment,emptyExperience,findPath,inZone,type Experience,type MapPoint,type MissionWorld} from './missionWorld';
import {observeTag,type TagDetection} from './tagVision';

export type MissionPhase='ready'|'takeoff'|'transit'|'approach'|'search'|'align'|'confirm'|'dwell'|'land'|'complete'|'manual'|'hold';
export const PHASE_LABEL:Record<MissionPhase,string>={ready:'비행 준비',takeoff:'이륙·상승',transit:'경로 비행',approach:'감속 접근',search:'태그 탐색',align:'정밀 정렬',confirm:'인식 확인',dwell:'위치 보정',land:'귀환 착륙',complete:'임무 완료',manual:'수동 조종',hold:'호버 대기'};
export interface FlightTuning {cruise:number;observeHeight:number;tolerance:number;confirmSeconds:number;acceleration:number;wind:number}
export const DEFAULT_TUNING:FlightTuning={cruise:2.2,observeHeight:2.2,tolerance:.075,confirmSeconds:.8,acceleration:1.4,wind:.12};
/** 축적된 오차·탐색 시간에 따라 접근만 조정한다. 방문 순서와 확인 조건은 유지한다. */
export function approachProfile(t:FlightTuning,record?:Experience['tags'][string]){
  if(!record?.visits)return {height:t.observeHeight,speed:1};
  const difficulty=clamp((record.searchSeconds-t.confirmSeconds)/6,0,1),error=clamp(record.errorM/t.tolerance,0,1);
  return {height:Math.min(3.5,t.observeHeight+difficulty*.25),speed:clamp(1.1-difficulty*.35-error*.3,.55,1.1)};
}
export interface FlightEvent {time:number;text:string;tone:'ok'|'info'|'warn'}
export interface TelemetrySample {t:number;speed:number;height:number;error:number}
export interface MissionSnapshot extends FlightSnapshot {phase:MissionPhase;autonomous:boolean;detections:TagDetection[];activeTag?:string;lock:number;scanSeconds:number;visited:string[];events:FlightEvent[];samples:TelemetrySample[];battery:number;voltage:number;current:number;link:number;correction:number;crossTrack:number;motors:number;notice:string;experience:Experience}
const angle=(target:number,current:number)=>Math.atan2(Math.sin(target-current),Math.cos(target-current));

/** 고정 60Hz 훈련 모델. 계획·시야 확인·정렬·연속 확인·다음 태그를 한 상태에서 구동한다. */
export class MissionFlight {
  state:MissionSnapshot;world:MissionWorld;tuning:FlightTuning;hiddenTag=false;dusk=false;useExperience=false;
  private visionAt=-100;private trailAt=0;private sampleAt=0;private phaseAt=0;private lockFrames=0;private integral={x:0,y:0};private correctionStart=0;private transitFrom:MapPoint;private recovery:MapPoint[]=[];private lastWaypoint=-1;private loggedBoundary=false;
  constructor(world:MissionWorld,tuning:FlightTuning={...DEFAULT_TUNING},experience=emptyExperience()){
    this.world=world;this.tuning={...tuning};this.transitFrom={...world.start};
    this.state={...world.start,height:.22,vx:0,vy:0,vz:0,yaw:0,pitch:0,roll:0,tMs:0,travelled:0,mode:'manual',waypoint:0,completed:false,route:world.route,trail:[{...world.start,height:.22}],boundary:false,input:{...NEUTRAL_INPUT},phase:'ready',autonomous:false,detections:[],lock:0,scanSeconds:0,visited:[],events:[],samples:[],battery:100,voltage:16.8,current:0,link:100,correction:0,crossTrack:0,motors:0,notice:world.error,experience:structuredClone(experience)};
    this.log(world.error||'훈련장과 임무 경로를 불러왔습니다.',world.error?'warn':'info');
  }
  snapshot():MissionSnapshot {const s=this.state;return {...s,input:{...s.input},trail:s.trail.slice(),visited:[...s.visited],events:s.events.slice(),samples:s.samples.slice(),detections:s.detections.slice()};}
  log(text:string,tone:FlightEvent['tone']='info'){this.state.events.unshift({time:this.state.tMs/1000,text,tone});this.state.events.length=Math.min(40,this.state.events.length);}
  private phase(phase:MissionPhase){if(this.state.phase===phase)return;this.state.phase=phase;this.phaseAt=this.state.tMs;if(['takeoff','search','dwell','land','complete','hold'].includes(phase))this.log(PHASE_LABEL[phase]+(this.state.activeTag?` · TAG ${this.state.activeTag}`:''),phase==='hold'?'warn':phase==='complete'?'ok':'info');}
  start(){const s=this.state;if(this.world.error){s.notice=this.world.error;return;}if(s.completed)return;
    const target=this.world.stops[s.waypoint];if(!target)return;
    const route=findPath(this.world,s,target);if(!route){this.hold('현재 위치에서 임무 경로로 복귀할 수 없습니다.');return;}
    this.recovery=route.slice(1,-1);this.transitFrom={x:s.x,y:s.y};s.autonomous=true;s.mode='route';s.notice='';s.lock=0;this.lockFrames=0;this.integral={x:0,y:0};this.phase(s.height<1?'takeoff':'transit');this.log('자동 비행 · 현재 위치에서 계획을 이어갑니다.');
  }
  manual(){if(this.state.phase!=='manual')this.log('수동 조종으로 전환했습니다.');this.state.completed=false;this.state.autonomous=false;this.state.mode='manual';this.state.notice='';this.state.lock=0;this.lockFrames=0;this.phase('manual');}
  hold(reason='제동 후 현재 위치에서 호버합니다.'){this.state.autonomous=false;this.state.mode='manual';this.state.notice=reason;this.state.lock=0;this.lockFrames=0;this.integral={x:0,y:0};this.phase('hold');}
  tune(values:Partial<FlightTuning>){this.tuning={cruise:clamp(values.cruise??this.tuning.cruise,.4,3),observeHeight:clamp(values.observeHeight??this.tuning.observeHeight,1.2,3.5),tolerance:clamp(values.tolerance??this.tuning.tolerance,.03,.18),confirmSeconds:clamp(values.confirmSeconds??this.tuning.confirmSeconds,.4,2),acceleration:clamp(values.acceleration??this.tuning.acceleration,.5,2),wind:clamp(values.wind??this.tuning.wind,0,.5)};this.state.lock=0;this.lockFrames=0;}
  step(raw:PilotInput,dt:number){if(!Number.isFinite(dt)||dt<=0)return;for(let left=Math.min(dt,.25);left>1e-9;left-=1/60)this.tick(raw,Math.min(left,1/60));}
  private advance(){const s=this.state,stop=this.world.stops[s.waypoint];this.transitFrom={x:stop.x,y:stop.y};this.integral={x:0,y:0};s.lock=0;s.scanSeconds=0;this.lockFrames=0;if(stop.kind==='land'||s.waypoint===this.world.stops.length-1){this.phase('land');return;}s.waypoint++;this.phase('transit');}
  private tick(raw:PilotInput,dt:number){
    const s=this.state,input=mergePilotInputs(NEUTRAL_INPUT,raw),t=this.tuning;s.input=input;s.boundary=false;
    if(input.right||input.forward||input.climb||input.yaw||input.brake){if(s.autonomous||s.phase!=='manual')this.manual();if(s.height<.3)s.height=.3;}
    const stop=this.world.stops[s.waypoint],tag=this.world.tags.find(v=>v.id===stop?.tagId);s.activeTag=tag?.id;
    if(this.lastWaypoint!==s.waypoint){this.lastWaypoint=s.waypoint;this.correctionStart=s.tMs;s.scanSeconds=0;this.log(`${stop?.name??'경유점'}으로 이동${tag?` · TAG ${tag.id}`:''}`);}
    const isVisionTick=s.tMs-this.visionAt>=99;
    if(isVisionTick){this.visionAt=s.tMs;s.detections=this.world.tags.map(v=>observeTag(s,v,this.world,this.hiddenTag&&v.id===tag?.id,this.dusk));}
    const detection=s.detections.find(v=>v.id===tag?.id),distance=stop?Math.hypot(stop.x-s.x,stop.y-s.y):0,speed=Math.hypot(s.vx,s.vy);
    let vx=0,vy=0,vz=0,yawRate=0;
    const learned=this.useExperience&&tag?s.experience.tags[tag.id]:undefined;
    const profile=approachProfile(t,learned),observeHeight=profile.height;
    const cruise=Math.min(t.cruise,stop?.speed&&stop.speed>0?stop.speed:3);
    const steer=(point:MapPoint,limit:number,gain=1.2)=>{const dx=point.x-s.x,dy=point.y-s.y,d=Math.hypot(dx,dy),v=Math.min(limit,d*gain,Math.sqrt(2*t.acceleration*Math.max(0,d-.02)));vx=d>.001?dx/d*v:0;vy=d>.001?dy/d*v:0;};
    if(s.autonomous&&stop){
      const target=this.recovery[0]??stop,remaining=Math.hypot(target.x-s.x,target.y-s.y);
      if(s.phase==='takeoff'){vz=clamp((stop.height-s.height)*1.4,-.7,.8);if(Math.abs(s.height-stop.height)<.12&&Math.abs(s.vz)<.18)this.phase('transit');}
      if(s.phase==='transit'||s.phase==='approach'){
        const near=remaining<Math.max(2,cruise*cruise/(2*t.acceleration)+.4);if(!this.recovery.length&&tag&&near&&s.phase==='transit')this.phase('approach');
        steer(target,near?Math.min(cruise,profile.speed):cruise);vz=clamp(((s.phase==='approach'?observeHeight:stop.height)-s.height)*1.4,-.65,.75);
        if(remaining>.25)yawRate=clamp(angle(Math.atan2(target.x-s.x,target.y-s.y),s.yaw)*1.8,-.65,.65);
        if(this.recovery.length&&remaining<.18&&speed<.35)this.recovery.shift();
        else if(!this.recovery.length&&remaining<.24&&speed<.3){if(tag){if(stop.kind==='hover'&&s.visited.at(-1)===tag.id&&this.world.stops[s.waypoint-1]?.tagId===tag.id){this.phase('dwell');}else {this.phase('search');this.correctionStart=s.tMs;}}else if(Math.abs(stop.height-s.height)<.2){if(stop.dwell>.1&&stop.kind!=='goto'){this.phase('dwell');}else this.advance();}}
      }
      if(['search','align','confirm'].includes(s.phase)){
        s.scanSeconds=(s.tMs-this.correctionStart)/1000;vz=clamp((observeHeight-s.height)*1.1,-.45,.45);
        if(detection?.valid){
          const sin=Math.sin(s.yaw),cos=Math.cos(s.yaw),ex=detection.right*cos+detection.forward*sin,ey=-detection.right*sin+detection.forward*cos;
          this.integral.x=clamp(this.integral.x+ex*dt,-.3,.3);this.integral.y=clamp(this.integral.y+ey*dt,-.3,.3);
          vx=clamp(ex*1.15+this.integral.x*.25-s.vx*.5,-.32,.32);vy=clamp(ey*1.15+this.integral.y*.25-s.vy*.5,-.32,.32);
          s.correction=Math.hypot(ex,ey);
          const stable=s.correction<=t.tolerance&&speed<.12&&Math.abs(s.vz)<.12&&Math.abs(observeHeight-s.height)<.15;
          if(isVisionTick){this.lockFrames=stable?this.lockFrames+1:0;s.lock=clamp(this.lockFrames/(t.confirmSeconds*10),0,1);}
          this.phase(stable?'confirm':'align');
          if(s.lock>=1){this.phase('dwell');s.visited.push(tag!.id);this.log(`TAG ${tag!.id} 확인 · 오차 ${(s.correction*100).toFixed(1)}cm`,'ok');
            const previous=s.experience.tags[tag!.id]??{visits:0,searchSeconds:0,errorM:0},n=previous.visits+1;s.experience.tags[tag!.id]={visits:n,searchSeconds:previous.searchSeconds+(s.scanSeconds-previous.searchSeconds)/n,errorM:previous.errorM+(s.correction-previous.errorM)/n};
          }
        }else{
          this.lockFrames=0;s.lock=0;this.phase('search');
          const a=s.scanSeconds*.8,r=Math.min(.28,s.scanSeconds*.025);steer({x:stop.x+Math.sin(a)*r,y:stop.y+Math.cos(a)*r},.18,.8);
          if(s.scanSeconds>12){const k=cellKey(s);s.experience.cells[k]=Math.min(3,(s.experience.cells[k]??0)+.7);this.hold('목표 태그를 12초 동안 확인하지 못했습니다. 가림을 해제한 뒤 자동 비행을 재개하세요.');}
        }
      }
      if(s.phase==='dwell'){steer(stop,.14,.8);vz=clamp(((tag?observeHeight:stop.height)-s.height)*1.1,-.3,.3);if((s.tMs-this.phaseAt)/1000>=Math.max(.65,stop.dwell))this.advance();}
      if(s.phase==='land'){steer(stop,.18,.9);vz=distance<.14?clamp((.22-s.height)*.9,-.45,0):0;if(s.height<=.235&&speed<.06){s.height=.22;s.vx=s.vy=s.vz=0;s.autonomous=false;s.completed=true;s.mode='manual';s.experience.runs++;this.phase('complete');}}
    }else if(s.phase==='manual'&&!input.brake){const sin=Math.sin(s.yaw),cos=Math.cos(s.yaw);vx=(input.right*cos+input.forward*sin)*1.8;vy=(-input.right*sin+input.forward*cos)*1.8;vz=input.climb*.8;yawRate=input.yaw*.95;}
    const grounded=(s.phase==='ready'||s.phase==='complete')&&s.height<.25;
    const wind=grounded||s.phase==='hold'?0:t.wind;
    // 지속 외란을 적분 보상하는 속도 응답. 가속도와 회전 속도를 제한해 자세를 연속적으로 만든다.
    const wx=wind*(.6+Math.sin(s.tMs/3100)*.4),wy=wind*Math.cos(s.tMs/4700)*.5;
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
    if(s.tMs-this.trailAt>100&&Math.hypot(s.x-s.trail.at(-1)!.x,s.y-s.trail.at(-1)!.y,s.height-s.trail.at(-1)!.height)>.025){s.trail.push({x:s.x,y:s.y,height:s.height});if(s.trail.length>4000)s.trail.shift();this.trailAt=s.tMs;}
    if(s.tMs-this.sampleAt>=250){s.samples.push({t:s.tMs/1000,speed:Math.hypot(s.vx,s.vy,s.vz),height:s.height,error:s.correction});if(s.samples.length>100)s.samples.shift();this.sampleAt=s.tMs;}
  }
}
