import { defaultDisturbance, type SimFrame } from '../approachSim';
import { clamp, terrainHeight } from './flightVisuals';

/** 웹 훈련용 위치·속도·고도·방위각 모델. 실제 FC 또는 기존 접근 제어기와 독립적이다. */
export interface PilotInput { right:number; forward:number; climb:number; yaw:number; brake:boolean }
export const NEUTRAL_INPUT: PilotInput = {right:0,forward:0,climb:0,yaw:0,brake:false};
export interface FlightPoint { x:number; y:number; height:number }
export interface FlightRoute { name:string; source:'training'|'mission'; points:FlightPoint[]; origin:{x:number;y:number} }
export interface FlightSnapshot extends FlightPoint {
  vx:number; vy:number; vz:number; yaw:number; pitch:number; roll:number; tMs:number;
  travelled:number; mode:'manual'|'route'; waypoint:number; completed:boolean;
  route:FlightRoute; trail:FlightPoint[]; boundary:boolean; input:PilotInput;
}
export const PILOT_CODES=new Set(['KeyW','KeyA','KeyS','KeyD','ArrowUp','ArrowDown','ArrowLeft','ArrowRight','Space']);
export function keyboardPilotInput(keys:ReadonlySet<string>):PilotInput {
  const axis=(plus:string,minus:string)=>Number(keys.has(plus))-Number(keys.has(minus));
  const right=axis('ArrowRight','ArrowLeft'),forward=axis('ArrowUp','ArrowDown');
  const length=Math.max(1,Math.hypot(right,forward));
  return {right:right/length,forward:forward/length,climb:axis('KeyW','KeyS'),yaw:axis('KeyD','KeyA'),brake:keys.has('Space')};
}
export function mergePilotInputs(a:PilotInput,b:PilotInput):PilotInput {
  const right=clamp(a.right+b.right,-1,1),forward=clamp(a.forward+b.forward,-1,1),length=Math.max(1,Math.hypot(right,forward));
  return {right:right/length,forward:forward/length,climb:clamp(a.climb+b.climb,-1,1),yaw:clamp(a.yaw+b.yaw,-1,1),brake:a.brake||b.brake};
}
export function trainingRoute():FlightRoute {
  return {name:'훈련장 순환 경로',source:'training',origin:{x:0,y:0},points:[
    {x:0,y:0,height:1.2},{x:0,y:4,height:2.5},{x:4,y:4,height:2.5},
    {x:4,y:0,height:1.8},{x:0,y:0,height:1.2},
  ]};
}
/** 메인 계획의 미터 좌표를 출발점 기준으로 평행 이동한다. 크기·경유점 순서를 바꾸지 않는다. */
export function missionFlightRoute(points:readonly {x:number;y:number}[],name:string):FlightRoute|null {
  if(points.length<2||points.some(p=>!Number.isFinite(p.x)||!Number.isFinite(p.y)))return null;
  const origin={...points[0]};
  const relative=points.map(p=>({x:p.x-origin.x,y:p.y-origin.y,height:3}));
  if(relative.some(p=>Math.abs(p.x)>40||Math.abs(p.y)>40))return null;
  return {name,source:'mission',origin,points:relative};
}
export function waypointDistance(state:FlightSnapshot):number {
  const target=state.route.points[Math.min(state.waypoint,state.route.points.length-1)];
  return Math.hypot(target.x-state.x,target.y-state.y,target.height-state.height);
}
const angleDelta=(target:number,current:number)=>Math.atan2(Math.sin(target-current),Math.cos(target-current));
const approach=(value:number,target:number,step:number)=>value+clamp(target-value,-step,step);

export class PilotFlight {
  state:FlightSnapshot;
  private trailTime=0;
  constructor(route:FlightRoute=trainingRoute()) {
    const start=route.points[0];
    this.state={x:start.x,y:start.y,height:Math.max(1.2,terrainHeight(start.x,-start.y)+.5),vx:0,vy:0,vz:0,yaw:0,pitch:0,roll:0,tMs:0,travelled:0,mode:'manual',waypoint:0,completed:false,route:structuredClone(route),trail:[],boundary:false,input:{...NEUTRAL_INPUT}};
    this.state.trail.push({x:this.state.x,y:this.state.y,height:this.state.height});
  }
  snapshot():FlightSnapshot { return {...this.state,input:{...this.state.input},trail:this.state.trail.slice()}; }
  manual() { this.state.mode='manual'; }
  followRoute() { this.state.mode='route';if(this.state.completed){this.state.waypoint=0;this.state.completed=false;} }
  brake() { this.manual();this.state.vx=0;this.state.vy=0;this.state.vz=0;this.state.pitch=0;this.state.roll=0;this.state.input={...NEUTRAL_INPUT}; }
  reset(route=this.state.route) { this.state=new PilotFlight(route).state;this.trailTime=0; }
  step(input:PilotInput,dt:number) {
    if(!Number.isFinite(dt)||dt<=0)return;
    // 호출 지연이 큰 경우에도 동일한 작은 물리 스텝으로 진행한다.
    for(let remaining=Math.min(dt,.25);remaining>1e-9;remaining-=1/60)this.tick(input,Math.min(remaining,1/60));
  }
  private tick(raw:PilotInput,dt:number) {
    const s=this.state,input=mergePilotInputs(NEUTRAL_INPUT,raw);
    if(input.right||input.forward||input.climb||input.yaw||input.brake)this.manual();
    s.input=input;s.boundary=false;
    let targetX=0,targetY=0,targetZ=0;
    if(s.mode==='route'&&!s.completed){
      const target=s.route.points[s.waypoint],dx=target.x-s.x,dy=target.y-s.y,dz=target.height-s.height;
      const distance=Math.hypot(dx,dy),speed=Math.min(1.6,distance*1.8);
      if(distance>.001){targetX=dx/distance*speed;targetY=dy/distance*speed;}
      targetZ=clamp(dz*1.8,-.8,.8);
      if(distance>.2)s.yaw+=clamp(angleDelta(Math.atan2(dx,dy),s.yaw),-dt,dt);
      if(Math.hypot(distance,dz)<.12&&Math.hypot(s.vx,s.vy,s.vz)<.24){
        if(s.waypoint<s.route.points.length-1)s.waypoint++;
        else {s.completed=true;targetX=targetY=targetZ=0;}
      }
    } else if(!input.brake) {
      s.yaw+=input.yaw*1.15*dt;
      const sin=Math.sin(s.yaw),cos=Math.cos(s.yaw);
      targetX=(input.right*cos+input.forward*sin)*1.8;
      targetY=(-input.right*sin+input.forward*cos)*1.8;
      targetZ=input.climb*.9;
    }
    s.yaw=angleDelta(s.yaw,0);
    const before={x:s.x,y:s.y,height:s.height},vx=s.vx,vy=s.vy;
    const acceleration=input.brake?6:2.4;
    const dx=targetX-s.vx,dy=targetY-s.vy,length=Math.hypot(dx,dy),step=Math.min(1,acceleration*dt/Math.max(length,1e-9));
    s.vx+=dx*step;s.vy+=dy*step;s.vz=approach(s.vz,targetZ,dt*1.8);
    s.x+=s.vx*dt;s.y+=s.vy*dt;s.height+=s.vz*dt;
    const floor=terrainHeight(s.x,-s.y)+.28;
    if(Math.abs(s.x)>42){s.x=clamp(s.x,-42,42);s.vx=0;s.boundary=true;}
    if(Math.abs(s.y)>42){s.y=clamp(s.y,-42,42);s.vy=0;s.boundary=true;}
    if(s.height<floor||s.height>12){s.height=clamp(s.height,floor,12);s.vz=0;s.boundary=true;}
    const ax=(s.vx-vx)/dt,ay=(s.vy-vy)/dt,sin=Math.sin(s.yaw),cos=Math.cos(s.yaw);
    const blend=1-Math.exp(-dt*7);
    s.roll+=(-Math.atan2(ax*cos-ay*sin,9.81)-s.roll)*blend;
    s.pitch+=(-Math.atan2(ax*sin+ay*cos,9.81)-s.pitch)*blend;
    s.travelled+=Math.hypot(s.x-before.x,s.y-before.y,s.height-before.height);s.tMs+=dt*1000;
    if(s.tMs-this.trailTime>=100&&Math.hypot(s.x-s.trail.at(-1)!.x,s.y-s.trail.at(-1)!.y,s.height-s.trail.at(-1)!.height)>.035){
      s.trail.push({x:s.x,y:s.y,height:s.height});if(s.trail.length>3000)s.trail.shift();this.trailTime=s.tMs;
    }
  }
}

/** 동일한 3D 렌더러의 cm 좌표 인터페이스에 연결한다. 접근 제어기의 결과로 가장하지 않는다. */
export function flightSceneFrame(s:FlightSnapshot):SimFrame {
  const distance=Math.hypot(s.x,s.y)*100,visible=distance<(s.height*100-11.5)*Math.tan(27.5*Math.PI/180);
  const disturbance={...defaultDisturbance(),authorityAuto:s.mode==='route'};
  return {tMs:s.tMs,x:s.x*100,y:s.y*100,vx:s.vx*100,vy:s.vy*100,tagVisible:visible,measErrRight:-s.x*100,measErrFwd:-s.y*100,measVelRight:s.vx*100,measVelFwd:s.vy*100,disturbance,baseRpy:[1500,1500,1500],outRpy:[1500,1500,1500],out:{state:s.completed?'ARRIVED':s.mode==='route'?'APPROACH':'IDLE',fault:null,distCm:distance,targetSpeedCmS:0,vTargetRightCmS:0,vTargetFwdCmS:0,rollUs:0,pitchUs:0,arriveHeldMs:0,neutral:false,overspeed:false}};
}
