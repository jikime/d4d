import {describe,it,expect} from 'vitest';
import camp from '../../../scenarios/demo_training_camp.json';
import {buildMissionWorld,cellKey,clearSegment,emptyExperience,findPath,type MissionScenario} from './missionWorld';
import {approachProfile,DEFAULT_TUNING,MissionFlight} from './missionFlight';
import {NEUTRAL_INPUT} from './pilotFlight';
import {observeTag} from './tagVision';

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
    for(let i=0;i<18000&&!flight.state.completed;i++){flight.step(NEUTRAL_INPUT,1/60);phases.add(flight.state.phase);}
    expect(flight.state.notice).toBe('');expect(flight.state.completed).toBe(true);expect(flight.state.visited).toEqual(['1','2','1']);
    for(const p of ['takeoff','approach','align','confirm','dwell','land','complete'])expect(phases.has(p),p).toBe(true);
    expect(flight.state.height).toBeCloseTo(.22,2);expect(flight.state.experience.runs).toBe(1);
  });
  it('태그가 가려지면 방문 처리를 하지 않고 재탐색 후 대기한다',()=>{
    const flight=new MissionFlight(buildMissionWorld(small));flight.hiddenTag=true;flight.start();for(let i=0;i<2000;i++)flight.step(NEUTRAL_INPUT,1/60);
    expect(flight.state.phase).toBe('hold');expect(flight.state.waypoint).toBe(0);expect(flight.state.visited).toEqual([]);expect(flight.state.lock).toBe(0);
    flight.hiddenTag=false;flight.start();for(let i=0;i<2500&&!flight.state.visited.length;i++)flight.step(NEUTRAL_INPUT,1/60);expect(flight.state.visited[0]).toBe('1');
  });
  it.each([false,true])('바람이 있는 전체 훈련장에서 경험 적용 %s 상태로 완주한다',learn=>{
    const world=buildMissionWorld(scenario),experience=emptyExperience();for(const tag of world.tags)experience.tags[tag.id]={visits:2,searchSeconds:4,errorM:.04};
    const flight=new MissionFlight(world,DEFAULT_TUNING,experience);flight.useExperience=learn;flight.start();
    for(let i=0;i<30000&&!flight.state.completed;i++)flight.step(NEUTRAL_INPUT,1/60);
    expect(flight.state.notice).toBe('');expect(flight.state.completed).toBe(true);expect(flight.state.visited).toEqual(['1','2','3','4','5','6','1']);
    expect(Math.hypot(flight.state.x-world.goal.x,flight.state.y-world.goal.y)).toBeLessThan(.14);
    expect(flight.state.travelled).toBeGreaterThan(90);expect(flight.state.battery).toBeLessThan(90);
  });
  it('여러 축 수동 인계 후 위치를 유지하고 장애물을 관통하지 않는다',()=>{
    const world=buildMissionWorld(scenario),flight=new MissionFlight(world);flight.start();for(let i=0;i<180;i++)flight.step(NEUTRAL_INPUT,1/60);const before=flight.snapshot();flight.step({...NEUTRAL_INPUT,right:1,climb:1,yaw:1},1/60);expect(flight.state.phase).toBe('manual');expect(Math.hypot(flight.state.x-before.x,flight.state.y-before.y)).toBeLessThan(.05);
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
    for(let i=0;i<360;i++){a.step(NEUTRAL_INPUT,1/60);b.step(NEUTRAL_INPUT,1/60);}expect(b.state.height).toBeGreaterThan(a.state.height);
  });
  it('추가 태그에 서로 다른 실제 코드를 배정하고 같은 위치의 체류를 중복 방문으로 세지 않는다',()=>{
    const map={...small,tags:[...small.tags,{id:900,x:10,y:10},{id:901,x:12,y:10}]};
    const world=buildMissionWorld(map,[{x:5,y:5,tagId:1,kind:'fix'},{x:5,y:5,tagId:1,kind:'hover',dwell:2},{x:5,y:9,tagId:2,kind:'land'}]);
    expect(world.tags.map(t=>t.code)).toEqual([1,2,586,585]);const flight=new MissionFlight(world);flight.start();
    for(let i=0;i<18000&&!flight.state.completed;i++)flight.step(NEUTRAL_INPUT,1/60);
    expect(flight.state.completed).toBe(true);expect(flight.state.visited).toEqual(['1','2']);
  });
  it('대각선 조종에서도 수평 가속도 크기가 설정한 한계를 넘지 않는다',()=>{
    const flight=new MissionFlight(buildMissionWorld(small));flight.tune({acceleration:.7,wind:.5});
    for(let i=0;i<50;i++){const {vx,vy}=flight.state;flight.step({...NEUTRAL_INPUT,right:1,forward:1,climb:1},1/60);expect(Math.hypot(flight.state.vx-vx,flight.state.vy-vy)*60).toBeLessThanOrEqual(.700001);}
  });
  it('호버 중 방향키를 누르면 수동 조종을 이어받고 놓으면 감속한다',()=>{
    const flight=new MissionFlight(buildMissionWorld(small));flight.start();for(let i=0;i<240;i++)flight.step(NEUTRAL_INPUT,1/60);flight.hold();
    const x=flight.state.x;for(let i=0;i<60;i++)flight.step({...NEUTRAL_INPUT,right:1},1/60);
    expect(flight.state.phase).toBe('manual');expect(flight.state.x-x).toBeGreaterThan(.3);expect(flight.state.notice).toBe('');
    for(let i=0;i<180;i++)flight.step(NEUTRAL_INPUT,1/60);expect(Math.hypot(flight.state.vx,flight.state.vy)).toBeLessThan(.08);
  });
});
