import {describe,it,expect} from 'vitest';
import camp from '../../../scenarios/demo_training_camp.json';
import {buildMissionWorld,cellKey,clearSegment,emptyExperience,findPath,type MissionScenario} from './missionWorld';
import {approachProfile,DEFAULT_TUNING,MissionFlight} from './missionFlight';
import {NEUTRAL_INPUT} from './pilotFlight';
import {observeTag} from './tagVision';
import type {PixelFrame} from './pixelVision';

// 제어기 단위 시험에만 사용하는 이상적인 카메라. 제품에는 이 경로가 없다.
function feed(flight:MissionFlight){
  const s=flight.state;if(Math.round(s.tMs/(1000/60))%6)return;
  const frame:PixelFrame={session:s.visionSession,sequence:s.vision.sequence+1,tMs:s.tMs,pose:{...s},processingMs:1,detections:flight.world.tags.map(t=>observeTag(s,t,flight.world,flight.hiddenTag&&t.id===flight.world.stops[s.waypoint]?.tagId,flight.dusk)).filter(d=>d.valid).map(d=>({id:d.code,family:'tag36h11',hamming:0,decisionMargin:100,center:{x:d.center[0],y:d.center[1]},corners:d.corners.map(([x,y])=>({x,y}))}))};
  flight.acceptVision(frame);return frame;
}
function step(flight:MissionFlight){feed(flight);flight.step(NEUTRAL_INPUT,1/60);}

const scenario=camp as MissionScenario;
const small:MissionScenario={name:'두 지점',field:{width_m:20,height_m:20},start:{x:5,y:5},goal:{x:5,y:5},tags:[{id:1,x:5,y:5},{id:2,x:5,y:9}],zones:[],objectives:[{name:'태그 2',x:5,y:9}],link_policy:'continue_mission'};
describe('통합 훈련장·태그 자율 비행',()=>{
  it('막사·정비고를 피하는 같은 좌표의 경로를 만들고 태그 순서를 유지한다',()=>{
    const world=buildMissionWorld(scenario);expect(world.error).toBe('');expect(world.stops.filter(p=>p.tagId).map(p=>p.tagId)).toEqual(['1','2','3','4','5','6','1']);
    let previous=world.start;for(const point of world.stops){expect(clearSegment(world,previous,point)).toBe(true);previous=point;}
    const path=findPath(world,{x:-15,y:-7},{x:0,y:-7})!;expect(path.length).toBeGreaterThan(2);
  });
  it('계획의 태그 ID·고도·체류 시간과 경유점 순서를 보존한다',()=>{
    const world=buildMissionWorld(small,[{x:5,y:5,tagId:1,kind:'fix',height:2.4,dwell:1.8},{x:5,y:9,tagId:2,kind:'read',height:2.1,dwell:3}]);
    expect(world.stops.map(p=>[p.x,p.y,p.tagId,p.height,p.dwell])).toEqual([[-5,-5,'1',2.4,1.8],[-5,-1,'2',2.1,3]]);
    const bad=buildMissionWorld(scenario,[{x:6,y:5},{x:18,y:13}]);expect(bad.error).not.toBe('');
  });
  it('시야·크기·가림·방위각으로 모의 검출을 판정한다',()=>{
    const world=buildMissionWorld(small),tag=world.tags[0],pose={...world.start,height:2.2,yaw:0,vx:0,vy:0};
    expect(observeTag(pose,tag,world).valid).toBe(true);expect(observeTag(pose,tag,world,true).valid).toBe(false);
    expect(observeTag({...pose,x:pose.x+3},tag,world).valid).toBe(false);
    const turned=observeTag({...pose,yaw:Math.PI/2},{...tag,x:tag.x+.3},world);expect(turned.center[1]).toBeLessThan(120);
  });
  it('태그를 연속 확인한 뒤에만 다음 지점으로 가고 착륙까지 완주한다',()=>{
    const flight=new MissionFlight(buildMissionWorld(small)),phases=new Set<string>();flight.start();
    for(let i=0;i<18000&&!flight.state.completed;i++){step(flight);phases.add(flight.state.phase);}
    expect(flight.state.notice).toBe('');expect(flight.state.completed).toBe(true);expect(flight.state.visited).toEqual(['1','2','1']);
    for(const p of ['takeoff','approach','align','confirm','dwell','land','complete'])expect(phases.has(p),p).toBe(true);
    expect(flight.state.height).toBeCloseTo(.22,2);expect(flight.state.experience.runs).toBe(1);
  });
  it('태그가 가려지면 방문 처리를 하지 않고 재탐색 후 대기한다',()=>{
    const flight=new MissionFlight(buildMissionWorld(small));flight.hiddenTag=true;flight.start();for(let i=0;i<2000;i++)step(flight);
    expect(flight.state.phase).toBe('search');expect(flight.state.autonomous).toBe(true);expect(flight.state.waypoint).toBe(0);expect(flight.state.visited).toEqual([]);expect(flight.state.lock).toBe(0);expect(Object.values(flight.state.experience.cells)).toEqual([.7]);
    flight.hiddenTag=false;for(let i=0;i<2500&&!flight.state.visited.length;i++)step(flight);expect(flight.state.visited[0]).toBe('1');
  });
  it.each([false,true])('바람이 있는 전체 훈련장에서 경험 적용 %s 상태로 완주한다',learn=>{
    const world=buildMissionWorld(scenario),experience=emptyExperience();for(const tag of world.tags)experience.tags[tag.id]={visits:2,searchSeconds:4,errorM:.04};
    const flight=new MissionFlight(world,DEFAULT_TUNING,experience);flight.useExperience=learn;flight.start();
    for(let i=0;i<30000&&!flight.state.completed;i++)step(flight);
    expect(flight.state.notice).toBe('');expect(flight.state.completed).toBe(true);expect(flight.state.visited).toEqual(['1','2','3','4','5','6','1']);
    expect(Math.hypot(flight.state.x-world.goal.x,flight.state.y-world.goal.y)).toBeLessThan(.14);
    expect(flight.state.travelled).toBeGreaterThan(90);expect(flight.state.battery).toBeLessThan(90);
  });
  it('여러 축 수동 인계 후 위치를 유지하고 장애물을 관통하지 않는다',()=>{
    const world=buildMissionWorld(scenario),flight=new MissionFlight(world);flight.start();for(let i=0;i<180;i++)step(flight);const before=flight.snapshot();flight.step({...NEUTRAL_INPUT,right:1,climb:1,yaw:1},1/60);expect(flight.state.phase).toBe('manual');expect(Math.hypot(flight.state.x-before.x,flight.state.y-before.y)).toBeLessThan(.05);
    for(let i=0;i<4000;i++)flight.step({...NEUTRAL_INPUT,right:1},1/60);expect(Math.abs(flight.state.x)).toBeLessThan(world.width/2);expect(flight.state.boundary||flight.state.phase==='hold').toBe(true);
  });
  it('관측·속도 조정값을 제한하고 고정 스텝 결과가 일치한다',()=>{
    const world=buildMissionWorld(small),a=new MissionFlight(world),b=new MissionFlight(world);a.start();b.start();for(let i=0;i<180;i++)a.step(NEUTRAL_INPUT,1/30);for(let i=0;i<360;i++)b.step(NEUTRAL_INPUT,1/60);expect(a.state.x).toBeCloseTo(b.state.x,8);expect(a.state.height).toBeCloseTo(b.state.height,8);
    a.tune({cruise:999,tolerance:0,confirmSeconds:0});expect(a.tuning).toMatchObject({cruise:3,tolerance:.03,confirmSeconds:.4});expect(DEFAULT_TUNING.cruise).toBe(2.2);
  });
  it('기록된 탐색 실패 구역을 우회하고 어려웠던 태그에는 더 천천히 접근한다',()=>{
    const experience=emptyExperience(),world=buildMissionWorld(small),start={x:-5,y:-5},end={x:5,y:-5};
    for(let x=-2;x<3;x++)experience.cells[cellKey({x,y:-5})]=3;
    const direct=findPath(world,start,end)!,learned=findPath(world,start,end,experience)!;
    expect(direct).toHaveLength(2);expect(learned.some(p=>Math.abs(p.y+5)>.5)).toBe(true);
    expect(buildMissionWorld(scenario,[],emptyExperience()).stops).toEqual(buildMissionWorld(scenario).stops);
    const easy=approachProfile(DEFAULT_TUNING,{visits:3,searchSeconds:1,errorM:.015}),hard=approachProfile(DEFAULT_TUNING,{visits:3,searchSeconds:7,errorM:.07});
    expect(hard.speed).toBeLessThan(easy.speed);expect(hard.height).toBeGreaterThan(easy.height);
    const a=new MissionFlight(world),b=new MissionFlight(world,DEFAULT_TUNING,{...experience,tags:{'1':{visits:3,searchSeconds:7,errorM:.07}}});b.useExperience=true;a.start();b.start();
    for(let i=0;i<1500&&a.state.phase!=='confirm';i++)step(a);for(let i=0;i<1500&&b.state.phase!=='confirm';i++)step(b);expect(b.state.height).toBeGreaterThan(a.state.height);
  });
  it('추가 태그에 서로 다른 실제 코드를 배정하고 같은 위치의 체류를 중복 방문으로 세지 않는다',()=>{
    const map={...small,tags:[...small.tags,{id:900,x:10,y:10},{id:901,x:12,y:10}]};
    const world=buildMissionWorld(map,[{x:5,y:5,tagId:1,kind:'fix'},{x:5,y:5,tagId:1,kind:'hover',dwell:2},{x:5,y:9,tagId:2,kind:'land'}]);
    expect(world.tags.map(t=>t.code)).toEqual([1,2,586,585]);const flight=new MissionFlight(world);flight.start();
    for(let i=0;i<18000&&!flight.state.completed;i++)step(flight);
    expect(flight.state.completed).toBe(true);expect(flight.state.visited).toEqual(['1','2']);
  });
  it('대각선 조종에서도 수평 가속도 크기가 설정한 한계를 넘지 않는다',()=>{
    const flight=new MissionFlight(buildMissionWorld(small));flight.tune({acceleration:.7,wind:.5});
    for(let i=0;i<50;i++){const {vx,vy}=flight.state;flight.step({...NEUTRAL_INPUT,right:1,forward:1,climb:1},1/60);expect(Math.hypot(flight.state.vx-vx,flight.state.vy-vy)*60).toBeLessThanOrEqual(.700001);}
  });
  it('호버 중 방향키를 누르면 수동 조종을 이어받고 놓으면 감속한다',()=>{
    const flight=new MissionFlight(buildMissionWorld(small));flight.start();for(let i=0;i<240;i++)step(flight);flight.hold();
    const x=flight.state.x;for(let i=0;i<60;i++)flight.step({...NEUTRAL_INPUT,right:1},1/60);
    expect(flight.state.phase).toBe('manual');expect(flight.state.x-x).toBeGreaterThan(.3);expect(flight.state.notice).toBe('');
    for(let i=0;i<180;i++)step(flight);expect(Math.hypot(flight.state.vx,flight.state.vy)).toBeLessThan(.08);
  });
});


describe('계획 경로 이동과 목적지 영상 확인',()=>{
  it('화면 가장자리에서 TAG 119를 놓쳐도 마지막 영상 위치로 저속 보정하고 재인식한 뒤에만 완료한다',()=>{
    const map:MissionScenario={...small,start:{x:10,y:10},goal:{x:10,y:10},tags:[{id:119,x:10,y:10}],objectives:[]};
    const f=new MissionFlight(buildMissionWorld(map,[{x:10,y:10,tagId:119,kind:'land'}]));Object.assign(f.state,{x:1.2,y:.7,height:3.2,yaw:Math.PI});f.start();
    let sawTag=false,lostAfterSeeing=false;
    for(let i=0;i<3600&&!f.state.completed;i++){
      step(f);if(f.state.detections.some(d=>d.valid&&d.id==='119'))sawTag=true;
      if(sawTag&&f.state.phase==='search'){lostAfterSeeing=true;expect(f.state.visited).toEqual([]);}
    }
    expect(lostAfterSeeing).toBe(true);expect(f.state.completed).toBe(true);expect(f.state.visited).toEqual(['119']);
  });
  it('재탐색 보정 후에도 태그가 계속 가려져 있으면 도착으로 처리하지 않고 제한 시간 뒤 호버한다',()=>{
    const map:MissionScenario={...small,start:{x:10,y:10},goal:{x:10,y:10},tags:[{id:119,x:10,y:10}],objectives:[]};
    const f=new MissionFlight(buildMissionWorld(map,[{x:10,y:10,tagId:119,kind:'land'}]));Object.assign(f.state,{x:1.2,y:.7,height:3.2,yaw:Math.PI});f.start();
    for(let i=0;i<6;i++)step(f);f.hiddenTag=true;
    for(let i=0;i<600;i++)step(f);const at={x:f.state.x,y:f.state.y};
    for(let i=0;i<600;i++)step(f);
    expect(f.state.visited).toEqual([]);expect(f.state.completed).toBe(false);expect(f.state.phase).toBe('search');
    expect(Math.hypot(f.state.x-at.x,f.state.y-at.y)).toBeLessThan(.08);
    expect(f.state.notice).not.toContain('저속 보정');
  });
  it('계획·태그 배치의 식별자와 경험 저장 식별자를 구분한다',()=>{
    const normal=buildMissionWorld(small),experience=emptyExperience();experience.cells[cellKey({x:-5,y:-3})]=3;
    const learned=buildMissionWorld(small,[],experience);
    expect(learned.error).toBe('');expect(learned.key).toBe(normal.key);expect(learned.layoutKey).not.toBe(normal.layoutKey);
    const planned=buildMissionWorld(small,[{x:5,y:5,tagId:1},{x:5,y:9,tagId:2}]);
    expect(planned.key).not.toBe(normal.key);expect(planned.layoutKey).not.toBe(normal.layoutKey);
  });
  it('확인 중 같은 영상 시퀀스를 새 시각으로 반복 제출해도 방문 완료되지 않는다',()=>{
    const f=new MissionFlight(buildMissionWorld(small));f.start();let last:PixelFrame|undefined;
    for(let i=0;i<1500;i++){last=feed(f)??last;f.step(NEUTRAL_INPUT,1/60);if(f.state.lock>.15)break;}
    expect(f.state.lock).toBeGreaterThan(0);expect(f.state.visited).toEqual([]);
    for(let i=0;i<180;i++){f.acceptVision({...last!,tMs:f.state.tMs});f.step(NEUTRAL_INPUT,1/60);}
    expect(f.state.lock).toBe(0);expect(f.state.visited).toEqual([]);expect(f.state.phase).toBe('search');
  });

  it('좌표를 알고 있어도 영상이 없으면 멈춰 기다린다',()=>{
    const f=new MissionFlight(buildMissionWorld(small));f.start();for(let i=0;i<2200;i++)f.step(NEUTRAL_INPUT,1/60);
    expect(f.state.phase).toBe('search');expect(f.state.autonomous).toBe(true);expect(f.state.visited).toEqual([]);expect(f.state.waypoint).toBe(0);
    const at={x:f.state.x,y:f.state.y};for(let i=0;i<600;i++)f.step(NEUTRAL_INPUT,1/60);expect(Math.hypot(f.state.x-at.x,f.state.y-at.y)).toBeLessThan(.08);
  });
  it('같은 프레임 반복·오래된 프레임·이전 비행 프레임으로 확인 시간을 채우지 않는다',()=>{
    const f=new MissionFlight(buildMissionWorld(small));f.start();
    for(let i=0;i<900;i++)f.step(NEUTRAL_INPUT,1/60);
    feed(f);const v=f.state.vision.sequence;
    const frame:PixelFrame={session:f.state.visionSession-1,sequence:999,tMs:f.state.tMs,pose:{...f.state},processingMs:1,detections:[]};
    f.acceptVision(frame);expect(f.state.vision.sequence).toBe(v);
    f.acceptVision({...frame,session:f.state.visionSession,tMs:0});expect(f.state.vision.sequence).toBe(v);
    for(let i=0;i<150;i++)f.step(NEUTRAL_INPUT,1/60);expect(f.state.lock).toBe(0);expect(f.state.visited).toEqual([]);expect(f.state.detections).toEqual([]);
  });
  it('주요 지점의 태그만 유지하고 회피 경유점에는 태그를 만들지 않는다',()=>{
    const w=buildMissionWorld(scenario);
    expect(w.tags.map(t=>[t.id,t.x+w.origin.x,t.y+w.origin.y])).toEqual(scenario.tags.map(t=>[String(t.id),t.x,t.y]));
    expect(w.tags).toHaveLength(6);expect(w.stops.filter(p=>!p.tagId).length).toBeGreaterThan(0);
    let previous=w.start;for(const stop of w.stops){expect(clearSegment(w,previous,stop)).toBe(true);previous=stop;}
    expect(w.stops.length).toBeLessThan(15);
  });
  it('지점 사이에 태그가 없어도 이동하고, 가려진 목적지에서는 영상 확인까지 기다린다',()=>{
    const f=new MissionFlight(buildMissionWorld(small));f.start();
    for(let i=0;i<1800&&f.state.waypoint===0;i++)step(f);
    expect(f.state.visited).toEqual(['1']);f.hiddenTag=true;const startY=f.state.y;
    let movingWithoutTag=false;
    for(let i=0;i<2400;i++){step(f);if(f.state.guidance==='route'&&!f.state.detections.some(d=>d.id==='2'))movingWithoutTag||=Math.abs(f.state.y-startY)>1;}
    expect(movingWithoutTag).toBe(true);expect(f.state.phase).toBe('search');expect(f.state.waypoint).toBe(1);expect(f.state.visited).toEqual(['1']);
    expect(Math.hypot(f.state.x-f.world.tags[1].x,f.state.y-f.world.tags[1].y)).toBeLessThan(.3);
    f.hiddenTag=false;for(let i=0;i<1800&&!f.state.visited.includes('2');i++)step(f);
    expect(f.state.visited).toEqual(['1','2']);
  });
  it('태그가 없는 출발·도착 지점과 일반 경유점에서도 임의 태그를 추가하지 않고 완주한다',()=>{
    const map={...small,start:{x:5,y:3},goal:{x:5,y:11}},world=buildMissionWorld(map),f=new MissionFlight(world);f.start();
    expect(world.tags.map(t=>t.id)).toEqual(['1','2']);
    for(let i=0;i<14000&&!f.state.completed;i++)step(f);
    expect(f.state.completed).toBe(true);expect(f.state.visited).toEqual(['2']);expect(f.state.height).toBeCloseTo(.22);
  });
  it('목적지 근처에서는 계획 좌표 대신 카메라에서 읽은 태그 위치에 정렬한다',()=>{
    const world=buildMissionWorld(small),plannedX=world.stops[1].x;
    // 계획은 그대로 두고 시험 카메라가 보는 실제 표지만 30cm 옮긴다.
    world.tags[1].x+=.3;const f=new MissionFlight(world);f.start();
    for(let i=0;i<4000&&!f.state.visited.includes('2');i++)step(f);
    expect(f.state.visited).toEqual(['1','2']);expect(Math.abs(f.state.x-world.tags[1].x)).toBeLessThan(.075);
    expect(f.state.x-plannedX).toBeGreaterThan(.2);
  });
});
