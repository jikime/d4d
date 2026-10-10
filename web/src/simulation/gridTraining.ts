import {blocked,buildMissionWorld,clearSegment,inZone,type MapPoint,type MissionScenario,type MissionWaypoint,type MissionWorld,type TrainingGrid} from './missionWorld';

export interface GridCell extends MapPoint {id:string;col:number;row:number;tagId?:string;blocked:boolean}
export interface TrainingIntent {visit:string[];closed:string[];low:string[];altitude:number;lowAltitude:number}
export interface TrainingPlan {id:string;name:string;intent:TrainingIntent;waypoints:MissionWaypoint[];world:MissionWorld;cells:string[];distance:number;seconds:number;reasons:string[]}
export const DEFAULT_INTENT:TrainingIntent={visit:['H1','H6','A6','D3'],closed:[],low:[],altitude:3.2,lowAltitude:2};

export function gridSpec(scenario:MissionScenario):TrainingGrid|undefined {
  const g=scenario.training_grid;
  if(!g||!Number.isInteger(g.columns)||!Number.isInteger(g.rows)||g.columns<2||g.rows<2||g.columns>26||g.rows>20||g.columns*g.rows>500||!Number.isFinite(g.cell_m)||g.cell_m<2||g.columns*g.cell_m!==scenario.field.width_m||g.rows*g.cell_m!==scenario.field.height_m)return;
  return g;
}
export function gridCells(scenario:MissionScenario):GridCell[]{
  const g=gridSpec(scenario);if(!g)return [];
  const ox=scenario.field.width_m/2,oy=scenario.field.height_m/2;
  return Array.from({length:g.rows*g.columns},(_,n)=>{
    const col=n%g.columns,row=Math.floor(n/g.columns),x=(col+.5)*g.cell_m,y=(row+.5)*g.cell_m;
    const tag=scenario.tags.find(t=>Math.hypot(t.x-x,t.y-y)<.1);
    return {id:`${String.fromCharCode(65+col)}${row+1}`,col,row,x:x-ox,y:y-oy,tagId:tag?String(tag.id):undefined,blocked:scenario.zones.some(z=>(z.kind==='obstacle'||z.kind==='keepout')&&inZone({x,y},z,.6))};
  });
}
export function validateIntent(value:unknown,cells:GridCell[]):TrainingIntent {
  if(!value||typeof value!=='object')throw new Error('훈련 지시 형식을 확인해 주세요.');
  const input=value as Record<string,unknown>,ids=new Set(cells.map(c=>c.id));
  const list=(key:string)=>{const a=input[key];if(!Array.isArray(a)||a.length>48||a.some(v=>typeof v!=='string'||!ids.has(v)))throw new Error('지도에 없는 구역이 포함되어 있습니다.');return [...new Set(a)] as string[];};
  const result={visit:list('visit'),closed:list('closed'),low:list('low'),altitude:Number(input.altitude),lowAltitude:Number(input.lowAltitude)};
  if(!result.visit.length||result.visit.length>12)throw new Error('방문 구역을 1~12개 선택해 주세요.');
  if(!Number.isFinite(result.altitude)||!Number.isFinite(result.lowAltitude)||result.altitude<1.8||result.altitude>4||result.lowAltitude<1.8||result.lowAltitude>result.altitude)throw new Error('고도는 1.8~4m 범위로 설정해 주세요.');
  if(result.visit.some(id=>cells.find(c=>c.id===id)?.blocked||result.closed.includes(id)))throw new Error('방문 구역이 건물 또는 폐쇄 구역과 겹칩니다.');
  return result;
}

/** 지원 문법만 해석하는 로컬 대체 기능. LLM 사용 여부와 구분해서 표시한다. */
export function parseTrainingInstruction(text:string,current:TrainingIntent,cells:GridCell[]):TrainingIntent {
  const next=structuredClone(current);let changed=false;
  for(const clause of text.split(/[;\n。]/).map(s=>s.trim()).filter(Boolean)){
    const ids:string[]=clause.toUpperCase().match(/\b[A-Z]\d{1,2}\b/g)??[];
    if(ids.some(id=>!cells.some(c=>c.id===id)))throw new Error('지도에 없는 구역이 포함되어 있습니다.');
    if(/\b(?:not|don't|never|except)\b|하지\s*마|하지\s*않|말고/i.test(clause))throw new Error('지원하는 지시: 방문, 폐쇄, 고도 제한, 해제. 여러 지시는 세미콜론으로 구분해 주세요.');
    if(/해제|reopen|clear/i.test(clause)&&ids.length){next.closed=next.closed.filter(v=>!ids.includes(v));next.low=next.low.filter(v=>!ids.includes(v));changed=true;}
    else if(/폐쇄|진입\s*제한|통로\s*제한|close|exclude|block/i.test(clause)&&ids.length){next.closed=[...new Set([...next.closed,...ids])];changed=true;}
    else if(/고도\s*제한|낮은\s*고도|low|ceiling/i.test(clause)&&ids.length){next.low=[...new Set([...next.low,...ids])];const h=clause.match(/(-?\d+(?:\.\d+)?)\s*(?:m|미터)/i);if(h)next.lowAltitude=Number(h[1]);changed=true;}
    else if(/방문|순찰|점검|visit|patrol|inspect/i.test(clause)&&ids.length){next.visit=ids;changed=true;}
    else if(/고도|altitude|height/i.test(clause)){const h=clause.match(/(-?\d+(?:\.\d+)?)\s*(?:m|미터)/i);if(h){next.altitude=Number(h[1]);changed=true;}else throw new Error('구역 코드와 단위를 포함한 지시를 입력해 주세요.');}
    else throw new Error('지원하는 지시: 방문, 폐쇄, 고도 제한, 해제. 여러 지시는 세미콜론으로 구분해 주세요.');
  }
  if(!changed)throw new Error('구역 코드와 단위를 포함한 지시를 입력해 주세요.');
  return validateIntent(next,cells);
}

function cellPath(cells:GridCell[],world:MissionWorld,start:GridCell,end:GridCell,penalty:Set<string>):GridCell[]|null {
  const open=[start.id],cost=new Map([[start.id,0]]),parent=new Map<string,string>(),done=new Set<string>();
  const map=new Map(cells.map(c=>[c.id,c]));
  while(open.length){open.sort((a,b)=>cost.get(a)!-cost.get(b)!||a.localeCompare(b));const id=open.shift()!;if(done.has(id))continue;done.add(id);
    if(id===end.id){const result=[end];let p=id;while(parent.has(p)){p=parent.get(p)!;result.push(map.get(p)!);}return result.reverse();}
    const at=map.get(id)!;
    for(const next of cells){if(next.blocked||done.has(next.id)||Math.abs(next.col-at.col)+Math.abs(next.row-at.row)!==1||!clearSegment(world,at,next))continue;
      const value=cost.get(id)!+1+(penalty.has(next.id)?.65:0);if(value>=(cost.get(next.id)??Infinity))continue;cost.set(next.id,value);parent.set(next.id,id);open.push(next.id);
    }
  }return null;
}

export function createTrainingPlans(scenario:MissionScenario,raw:TrainingIntent,pose?:MapPoint&{height:number}):TrainingPlan[]{
  const g=gridSpec(scenario);if(!g)throw new Error('격자 훈련 맵을 선택해 주세요.');
  const all=gridCells(scenario),intent=validateIntent(raw,all),origin={x:scenario.field.width_m/2,y:scenario.field.height_m/2};
  const zone=(id:string,kind:string)=>{const c=all.find(c=>c.id===id)!;return {kind,shape:'rect',name:id,x:c.x+origin.x-g.cell_m/2,y:c.y+origin.y-g.cell_m/2,w:g.cell_m,h:g.cell_m,value:intent.lowAltitude};};
  const sc:MissionScenario={...scenario,zones:[...scenario.zones,...intent.closed.map(id=>zone(id,'keepout')),...intent.low.map(id=>zone(id,'height_limit'))]};
  // 모든 태그를 방문하는 기본 계획 대신 지도 검사에 필요한 최소 경로를 만든다.
  const inspection=buildMissionWorld(sc,[{...sc.start,kind:'goto'}]),start=pose??{...inspection.start,height:.22};
  if(blocked(inspection,start))throw new Error('현재 위치가 폐쇄 구역 안에 있습니다. 해당 구역을 해제한 뒤 다시 계획해 주세요.');
  const available=all.map(c=>({...c,blocked:c.blocked||intent.closed.includes(c.id)}));
  const first=available.filter(c=>!c.blocked&&clearSegment(inspection,start,c)).sort((a,b)=>Math.hypot(a.x-start.x,a.y-start.y)-Math.hypot(b.x-start.x,b.y-start.y))[0];
  const home=available.find(c=>Math.hypot(c.x-inspection.goal.x,c.y-inspection.goal.y)<.1&&!c.blocked);
  if(!first||!home)throw new Error('현재 위치 또는 귀환 지점으로 연결되는 통로가 없습니다.');
  const targets=[...intent.visit.map(id=>available.find(c=>c.id===id)!),home],plans:TrainingPlan[]=[],penalty=new Set<string>();
  for(let variant=0;variant<3;variant++){
    let from=first;const path:GridCell[]=[first],visits=new Set<number>();
    for(const target of targets){const leg=cellPath(available,inspection,from,target,penalty);if(!leg)throw new Error('선택한 구역까지 연결되는 경로가 없습니다. 폐쇄 구역을 확인해 주세요.');path.push(...leg.slice(1));visits.add(path.length-1);from=target;}
    if(plans.some(p=>p.cells.join(',')===path.map(c=>c.id).join(',')))break;
    const waypoints:MissionWaypoint[]=[],heightFor=(cell:GridCell)=>intent.low.includes(cell.id)?intent.lowAltitude:intent.altitude;
    let previous:MapPoint=start,previousHeight=start.height;
    for(let i=0;i<path.length;i++){
      const cell=path[i],height=Math.min(heightFor(cell),i>0?heightFor(path[i-1]):heightFor(first));
      if(Math.abs(height-previousHeight)>.05)waypoints.push({x:previous.x+origin.x,y:previous.y+origin.y,height,ceiling:height,kind:'altitude',dwell:0,label:'구간 진입 전 고도 조정'});
      const visit=visits.has(i)||i===0&&Math.hypot(start.x-cell.x,start.y-cell.y)<.15;
      waypoints.push({x:cell.x+origin.x,y:cell.y+origin.y,height,ceiling:height,kind:visit?'fix':'goto',tagId:visit?cell.tagId:null,dwell:visit?1:0,label:cell.id,speed:1.6});
      previous=cell;previousHeight=height;
    }
    waypoints.at(-1)!.kind='land';
    // 재계획은 현재 위치에서 검사하고 기존 이착륙장 좌표는 유지한다.
    const world=buildMissionWorld({...sc,start:{x:start.x+origin.x,y:start.y+origin.y}},waypoints);
    world.start={...inspection.start};world.goal={...inspection.goal};
    if(world.error)throw new Error(world.error);
    let prev={...start},distance=0;for(const p of waypoints){const local={x:p.x-origin.x,y:p.y-origin.y,height:p.height!};distance+=Math.hypot(local.x-prev.x,local.y-prev.y,local.height-prev.height);prev=local;}
    plans.push({id:`route-${variant}`,name:['기본 코스','대체 코스 1','대체 코스 2'][variant],intent,waypoints,world,cells:path.map(c=>c.id),distance,seconds:distance/1.3+visits.size*7,reasons:[`방문 순서: ${intent.visit.join(' → ')}`,intent.closed.length?`폐쇄 구역 제외: ${intent.closed.join(', ')}`:'모든 통로 개방',intent.low.length?`고도 제한: ${intent.low.join(', ')} · ${intent.lowAltitude}m`:`순항 고도: ${intent.altitude}m`,'목적지 AprilTag 영상 확인 후 다음 지점으로 이동']});
    path.forEach(c=>penalty.add(c.id));
  }
  return plans;
}
