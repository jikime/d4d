import {describe,expect,it} from 'vitest';
import grid from '../../../scenarios/demo_grid_training.json';
import {createTrainingPlans,DEFAULT_INTENT,gridCells,parseTrainingInstruction,validateIntent} from './gridTraining';
import {clearSegment,inZone,type MissionScenario} from './missionWorld';
import {MissionFlight} from './missionFlight';
import {NEUTRAL_INPUT} from './pilotFlight';
import {observeTag} from './tagVision';

const scenario=grid as MissionScenario;
// 단위 시험용 이상적 영상. 제품의 AprilTag 픽셀 디코더를 대체하지 않는다.
function step(f:MissionFlight){const s=f.state;
  if(Math.round(s.tMs/(1000/60))%6===0)f.acceptVision({session:s.visionSession,sequence:s.vision.sequence+1,tMs:s.tMs,pose:{...s},processingMs:1,detections:f.world.tags.map(tag=>observeTag(s,tag,f.world)).filter(d=>d.valid).map(d=>({id:d.code,family:'tag36h11',hamming:0,decisionMargin:100,center:{x:d.center[0],y:d.center[1]},corners:d.corners.map(([x,y])=>({x,y}))}))});
  f.step(NEUTRAL_INPUT,1/60);
}
describe('고정 격자 훈련 계획',()=>{
  it('바닥 43곳의 고유 태그를 같은 격자 중심에 고정한다',()=>{
    const cells=gridCells(scenario);expect(cells).toHaveLength(48);expect(cells.filter(c=>c.blocked)).toHaveLength(5);
    expect(scenario.tags).toHaveLength(43);expect(new Set(scenario.tags.map(t=>t.id)).size).toBe(43);
    for(const c of cells.filter(c=>!c.blocked)){const tag=scenario.tags.find(t=>String(t.id)===c.tagId)!;expect(tag.x).toBe(c.x+24);expect(tag.y).toBe(c.y+18);}
  });
  it('후보마다 폐쇄 구역·건물을 피하고 방문 순서와 기존 태그 배치를 보존한다',()=>{
    const intent={...DEFAULT_INTENT,closed:['D1','D2'],low:['C1']},before=JSON.stringify(scenario),plans=createTrainingPlans(scenario,intent);
    expect(plans.length).toBeGreaterThan(1);expect(plans.length).toBeLessThanOrEqual(3);
    for(const plan of plans){expect(plan.world.error).toBe('');expect(plan.cells).not.toContain('D1');expect(plan.cells).not.toContain('D2');
      expect(plan.world.tags).toHaveLength(43);expect(plan.world.stops.filter(s=>s.tagId).map(s=>s.name)).toEqual(['A1',...intent.visit,'A1']);
      let previous=plan.world.start;for(const stop of plan.world.stops){expect(clearSegment(plan.world,previous,stop)).toBe(true);previous=stop;}
    }expect(JSON.stringify(scenario)).toBe(before);
    expect(createTrainingPlans(scenario,intent).map(p=>p.cells)).toEqual(plans.map(p=>p.cells));
  });
  it('도달 불가·잘못된 셀·고도·폐쇄된 방문 지점을 실행 가능한 계획으로 표시하지 않는다',()=>{
    expect(()=>createTrainingPlans(scenario,{...DEFAULT_INTENT,closed:['B1','A2']})).toThrow();
    expect(()=>createTrainingPlans(scenario,{...DEFAULT_INTENT,closed:['H1']})).toThrow();
    expect(()=>createTrainingPlans(scenario,{...DEFAULT_INTENT,altitude:NaN})).toThrow();
    expect(()=>createTrainingPlans(scenario,{...DEFAULT_INTENT,visit:['Z99']})).toThrow();
    expect(()=>createTrainingPlans(scenario,{...DEFAULT_INTENT,closed:['A1']})).toThrow();
    expect(()=>validateIntent({visit:['H1'],closed:'D1',low:[],altitude:3,lowAltitude:2},gridCells(scenario))).toThrow();
  });
  it('한영 지시의 방문 순서와 구역 설정을 해석하고 불명확한 입력을 거절한다',()=>{
    const cells=gridCells(scenario),ko=parseTrainingInstruction('H6, A6 방문; D2 폐쇄; C1, D1 고도 제한 2m',DEFAULT_INTENT,cells),en=parseTrainingInstruction('Visit H6, A6; Close D2; Low C1, D1 2m',DEFAULT_INTENT,cells);
    expect(ko).toEqual(en);expect(ko.visit).toEqual(['H6','A6']);expect(ko.closed).toEqual(['D2']);expect(ko.low).toEqual(['C1','D1']);
    expect(parseTrainingInstruction('D2 해제',ko,cells).closed).toEqual([]);
    expect(()=>parseTrainingInstruction('어딘가 가 줘',DEFAULT_INTENT,cells)).toThrow();
    for(const text of ['Low C1 -2m',"Do not close D2",'Reopen Z99'])expect(()=>parseTrainingInstruction(text,DEFAULT_INTENT,cells)).toThrow();
  });
  it('제한 구역 진입 전에 고도를 낮추고 선택된 목적지만 영상 확인한 뒤 귀환한다',()=>{
    const plan=createTrainingPlans(scenario,{...DEFAULT_INTENT,visit:['E1'],low:['C1','D1']})[0],f=new MissionFlight(plan.world);f.start();let samples=0;
    for(let i=0;i<30000&&!f.state.completed;i++){step(f);if(f.world.zones.some(z=>z.kind==='height_limit'&&inZone(f.state,z))){samples++;expect(f.state.height).toBeLessThanOrEqual(2.15);}}
    expect(samples).toBeGreaterThan(0);expect(f.state.completed,f.state.notice).toBe(true);expect(f.state.visited.map(id=>f.world.tags.find(t=>t.id===id)!.name)).toEqual(['A1','E1','A1']);
  });
  it('비행 중 재계획해도 위치·시간·배터리·기존 궤적을 보존하고 새 영상 세션으로 이어간다',()=>{
    const first=createTrainingPlans(scenario,{...DEFAULT_INTENT,visit:['H1']})[0],f=new MissionFlight(first.world);f.start();
    for(let i=0;i<10000&&f.state.travelled<12;i++)step(f);
    const before=f.snapshot(),next=createTrainingPlans(scenario,{...DEFAULT_INTENT,visit:['A6'],closed:['D2']},before)[0];
    expect(f.replan(next.world)).toBe(true);const after=f.snapshot();
    for(const key of ['x','y','height','tMs','travelled','battery'] as const)expect(after[key]).toBe(before[key]);
    expect(after.trail).toEqual(before.trail);expect(after.visionSession).not.toBe(before.visionSession);expect(after.detections).toEqual([]);
    for(let i=0;i<36000&&!f.state.completed;i++)step(f);
    expect(f.state.completed,f.state.notice).toBe(true);expect(f.state.visited).toContain(String(scenario.tags.find(t=>t.name==='A6')!.id));
  });
});
