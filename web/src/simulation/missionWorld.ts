import type {FlightPoint,FlightRoute} from './pilotFlight';

export interface MapPoint {x:number;y:number}
export interface MissionWaypoint extends MapPoint {tagId?:string|number|null;kind?:string;height?:number;ceiling?:number;speed?:number;dwell?:number;label?:string}
export interface TrainingGrid {columns:number;rows:number;cell_m:number}
export interface MapZone {kind:string;shape:string;name?:string;x?:number;y?:number;w?:number;h?:number;r?:number;points?:[number,number][];value?:number}
export interface MissionScenario {name:string;field:{width_m:number;height_m:number};start:MapPoint;goal:MapPoint;tags:(MapPoint&{id:string|number;name?:string;role?:string;suggested?:boolean})[];zones:MapZone[];objectives:(MapPoint&{name:string;dwell_s?:number})[];link_policy:string;training_grid?:TrainingGrid}
export interface MissionTag extends MapPoint {id:string;code:number;name:string;size:number;role:string}
export interface MissionStop extends FlightPoint {tagId?:string;name:string;kind:string;dwell:number;ceiling?:number;speed?:number}
export interface Experience {runs:number;tags:Record<string,{visits:number;searchSeconds:number;errorM:number}>;cells:Record<string,number>}
export interface MissionWorld {key:string;layoutKey:string;name:string;width:number;depth:number;origin:MapPoint;start:MapPoint;goal:MapPoint;tags:MissionTag[];zones:MapZone[];stops:MissionStop[];route:FlightRoute;error:string;source:'plan'|'training';linkPolicy:string;grid?:TrainingGrid}
export const emptyExperience=():Experience=>({runs:0,tags:{},cells:{}});
export const cellKey=(p:MapPoint)=>`${Math.floor(p.x/2)},${Math.floor(p.y/2)}`;

function segmentDistance(p:MapPoint,a:MapPoint,b:MapPoint){const dx=b.x-a.x,dy=b.y-a.y,t=Math.max(0,Math.min(1,((p.x-a.x)*dx+(p.y-a.y)*dy)/(dx*dx+dy*dy||1)));return Math.hypot(p.x-a.x-t*dx,p.y-a.y-t*dy);}
export function inZone(p:MapPoint,z:MapZone,margin=0):boolean {
  if(z.shape==='rect')return p.x>=(z.x??0)-margin&&p.x<=(z.x??0)+(z.w??0)+margin&&p.y>=(z.y??0)-margin&&p.y<=(z.y??0)+(z.h??0)+margin;
  if(z.shape==='circle')return Math.hypot(p.x-(z.x??0),p.y-(z.y??0))<=(z.r??0)+margin;
  const pts=z.points??[];let inside=false;
  for(let i=0,j=pts.length-1;i<pts.length;j=i++){const [x,y]=pts[i],[u,v]=pts[j];if((y>p.y)!==(v>p.y)&&p.x<(u-x)*(p.y-y)/(v-y)+x)inside=!inside;if(segmentDistance(p,{x,y},{x:u,y:v})<=margin)return true;}
  return inside;
}
export function blocked(world:Pick<MissionWorld,'width'|'depth'|'zones'>,p:MapPoint,margin=.55):boolean {
  return Math.abs(p.x)>world.width/2-margin||Math.abs(p.y)>world.depth/2-margin||world.zones.some(z=>(z.kind==='obstacle'||z.kind==='keepout')&&inZone(p,z,margin));
}
export function clearSegment(world:Pick<MissionWorld,'width'|'depth'|'zones'>,a:MapPoint,b:MapPoint,margin=.55):boolean {
  const n=Math.max(1,Math.ceil(Math.hypot(b.x-a.x,b.y-a.y)/.16));for(let i=0;i<=n;i++)if(blocked(world,{x:a.x+(b.x-a.x)*i/n,y:a.y+(b.y-a.y)*i/n},margin))return false;return true;
}

/** 지도의 모든 장애물·금지 구역에 같은 여유 거리를 적용하는 8방향 A*. 모서리 지름길도 검사한다. */
export function findPath(world:Pick<MissionWorld,'width'|'depth'|'zones'>,start:MapPoint,end:MapPoint,experience?:Experience):MapPoint[]|null {
  if(blocked(world,start)||blocked(world,end))return null;
  const hasRisk=Object.values(experience?.cells??{}).some(value=>value>0);
  if(clearSegment(world,start,end)&&!hasRisk)return [start,end];
  const segmentCost=(a:MapPoint,b:MapPoint)=>{const d=Math.hypot(b.x-a.x,b.y-a.y);if(!hasRisk)return d;const n=Math.max(1,Math.ceil(d/.25));let cost=0;for(let k=0;k<n;k++){const t=(k+.5)/n;cost+=1+Math.min(3,experience?.cells[cellKey({x:a.x+(b.x-a.x)*t,y:a.y+(b.y-a.y)*t})]??0);}return d*cost/n;};
  const step=.5,nx=Math.ceil(world.width/step)+1,ny=Math.ceil(world.depth/step)+1;
  if(nx*ny>160000)return null;
  const index=(p:MapPoint)=>Math.round((p.y+world.depth/2)/step)*nx+Math.round((p.x+world.width/2)/step);
  const point=(n:number)=>({x:(n%nx)*step-world.width/2,y:Math.floor(n/nx)*step-world.depth/2});
  const first=index(start),last=index(end),cost=new Map([[first,0]]),parent=new Map<number,number>(),closed=new Set<number>();
  const heap:{id:number;f:number}[]=[];
  const push=(id:number,f:number)=>{heap.push({id,f});let n=heap.length-1;while(n){const p=(n-1)>>1;if(heap[p].f<=f)break;[heap[n],heap[p]]=[heap[p],heap[n]];n=p;}};
  const pop=()=>{const result=heap[0],tail=heap.pop()!;if(heap.length){heap[0]=tail;let n=0;while(true){let c=n*2+1;if(c>=heap.length)break;if(c+1<heap.length&&heap[c+1].f<heap[c].f)c++;if(heap[n].f<=heap[c].f)break;[heap[n],heap[c]]=[heap[c],heap[n]];n=c;}}return result.id;};
  push(first,0);
  while(heap.length){const at=pop();if(closed.has(at))continue;if(at===last){const out:MapPoint[]=[end];let n=at;while(parent.has(n)){out.push(point(n));n=parent.get(n)!;}out.push(start);out.reverse();const simple=[start];let i=0;while(i<out.length-1){let j=i+1,cost=segmentCost(out[i],out[j]);while(j+1<out.length){cost+=segmentCost(out[j],out[j+1]);if(!clearSegment(world,out[i],out[j+1])||segmentCost(out[i],out[j+1])>cost*1.02)break;j++;}simple.push(out[j]);i=j;}return simple;}
    closed.add(at);const a=point(at),col=at%nx,row=Math.floor(at/nx);
    for(const dx of [-1,0,1])for(const dy of [-1,0,1]){if(!dx&&!dy)continue;const x=col+dx,y=row+dy;if(x<0||y<0||x>=nx||y>=ny)continue;const next=y*nx+x;if(closed.has(next))continue;const b=point(next);if(!clearSegment(world,a,b))continue;const weight=1+Math.min(3,experience?.cells[cellKey(b)]??0),g=cost.get(at)!+Math.hypot(dx,dy)*step*weight;if(g>=(cost.get(next)??Infinity))continue;cost.set(next,g);parent.set(next,at);push(next,g+Math.hypot(b.x-end.x,b.y-end.y));}
  }return null;
}

export function buildMissionWorld(sc:MissionScenario,planned:readonly MissionWaypoint[]=[],experience?:Experience):MissionWorld {
  const origin={x:sc.field.width_m/2,y:sc.field.height_m/2},local=(p:MapPoint)=>({x:p.x-origin.x,y:p.y-origin.y});
  const zones=sc.zones.map(z=>({...z,x:z.x===undefined?undefined:z.x-origin.x,y:z.y===undefined?undefined:z.y-origin.y,points:z.points?.map(([x,y])=>[x-origin.x,y-origin.y] as [number,number])}));
  const used=new Set(sc.tags.map(t=>Number(t.id)).filter(id=>Number.isInteger(id)&&id>=0&&id<587));let spare=586;
  const tags=sc.tags.map(t=>{let code=Number(t.id);if(!Number.isInteger(code)||code<0||code>=587){while(used.has(spare)&&spare>=0)spare--;code=spare--;used.add(code);}return {...local(t),id:String(t.id),code,name:t.name??`태그 ${t.id}`,size:.4,role:t.role??'marker'};});
  const world:MissionWorld={key:JSON.stringify({field:sc.field,start:sc.start,goal:sc.goal,tags:sc.tags,zones:sc.zones,grid:sc.training_grid,planned}),layoutKey:'',name:sc.name,width:sc.field.width_m,depth:sc.field.height_m,origin,start:local(sc.start),goal:local(sc.goal),zones,tags,stops:[],route:{name:sc.name,source:planned.length?'mission':'training',origin,points:[]},source:planned.length?'plan':'training',error:'',linkPolicy:sc.link_policy};
  if(!Number.isFinite(world.width)||!Number.isFinite(world.depth)||world.width<=0||world.depth<=0||tags.some(t=>t.code<0)){world.error='훈련 구역 또는 태그 코드 범위를 확인해 주세요.';return world;}
  world.grid=sc.training_grid;
  const visit=(p:MapPoint,kind='fix',tagId?:string,dwell=1,name='경유점',height=3.2,speed?:number,ceiling?:number)=>world.stops.push({...p,height:Math.max(.6,Math.min(10,height)),tagId,name,kind,dwell:Math.max(0,Math.min(60,dwell)),speed,ceiling});
  if(planned.length){
    for(const p of planned){const at=local(p),tag=tags.find(t=>p.tagId!=null?t.id===String(p.tagId):!['goto','altitude'].includes(p.kind??'')&&Math.hypot(t.x-at.x,t.y-at.y)<.3);visit(at,p.kind??(tag?'fix':'goto'),tag?.id,p.dwell??1,p.label??tag?.name??'경로 통과',p.height??3.2,p.speed,p.ceiling);if(p.tagId!=null&&!tag)world.error=`계획의 태그 ${p.tagId}를 지도에서 찾을 수 없습니다.`;if(tag&&Math.hypot(tag.x-at.x,tag.y-at.y)>.5)world.error=`TAG ${tag.id}의 위치와 계획 좌표가 다릅니다. 목적지 주변에 해당 태그를 배치해 주세요.`;}
    const last=world.stops.at(-1);if(last)last.kind='land';
  }else{
    const home=tags.find(t=>Math.hypot(t.x-world.start.x,t.y-world.start.y)<.5);visit(world.start,'fix',home?.id,1,'출발점 확인');let from=world.start;
    const targets=sc.objectives.length?sc.objectives.map(p=>({...local(p),name:p.name,dwell:p.dwell_s??1})):tags.filter(t=>t!==home).map(t=>({...t,dwell:1}));
    for(const target of [...targets,{...world.goal,name:'귀환 이착륙장',dwell:1}]){const path=findPath(world,from,target,experience);if(!path){world.error=`${target.name}까지 통과 가능한 경로가 없습니다.`;break;}path.slice(1,-1).forEach(p=>visit(p,'goto',undefined,0,'회피 경유점'));const tag=tags.find(t=>Math.hypot(t.x-target.x,t.y-target.y)<.5);visit(target,'fix',tag?.id,target.dwell,target.name);from=target;}
    if(!world.error&&world.stops.length)world.stops.at(-1)!.kind='land';
  }
  let prev=world.start;for(const stop of world.stops){if(!Number.isFinite(stop.x)||!Number.isFinite(stop.y)||!clearSegment(world,prev,stop)){world.error='계획 경로가 건물·금지 구역 또는 훈련장 경계와 겹칩니다. 계획을 다시 생성해 주세요.';break;}prev=stop;}
  world.route.points=world.stops.map(({x,y,height})=>({x,y,height}));if(!world.route.points.length)world.route.points=[{...world.start,height:1.2}];
  // 계획 변경 시 이전 장면의 태그·비전 결과를 재사용하지 않는다.
  world.layoutKey=JSON.stringify({map:world.key,stops:world.stops});
  return world;
}
