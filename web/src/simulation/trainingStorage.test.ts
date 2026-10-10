import {describe,it,expect} from 'vitest';
import scenario from '../../../scenarios/demo_grid_training.json';
import {createTrainingPlans,DEFAULT_INTENT} from './gridTraining';
import {restoreCourse,saveCourseData} from './trainingStorage';
import {MissionFlight} from './missionFlight';
import {terrainLayoutKey} from './missionWorld';

describe('저장한 훈련 코스 복원',()=>{
  it('선택한 대체 코스의 방문 순서·폐쇄 구역·고도를 보존한다',()=>{
    const plans=createTrainingPlans(scenario,{...DEFAULT_INTENT,closed:['D2'],low:['C1']});
    for(const plan of plans){
      const saved=saveCourseData(scenario,plan,'점검 코스');
      const restored=restoreCourse(scenario,JSON.parse(JSON.stringify(saved)));
      expect(restored.cells).toEqual(plan.cells);expect(restored.world.stops).toEqual(plan.world.stops);
      expect(new MissionFlight(restored.world).state.phase).toBe('ready');
    }
  });
  it('비행 중 재계획한 코스를 초기화해도 원래 이착륙장으로 순간 이동하지 않는다',()=>{
    const pose={x:-15,y:-15,height:2.4};
    const plan=createTrainingPlans(scenario,{...DEFAULT_INTENT,visit:['C1']},pose)[0];
    const restored=restoreCourse(scenario,saveCourseData(scenario,plan,'재계획'));
    const flight=new MissionFlight(restored.world);
    expect(flight.state).toMatchObject(pose);expect(flight.state.autonomous).toBe(false);
    expect(restored.world.goal).toEqual(plan.world.goal);
  });
  it('다른 지도·잘못된 시작점·변조된 고도를 거부한다',()=>{
    const saved=saveCourseData(scenario,createTrainingPlans(scenario,DEFAULT_INTENT)[0],'검사');
    expect(()=>restoreCourse({...scenario,field:{...scenario.field,width_m:60}},saved)).toThrow();
    expect(()=>restoreCourse(scenario,{...saved,launch:{x:NaN,y:0,height:2}})).toThrow();
    expect(()=>restoreCourse(scenario,{...saved,intent:{...saved.intent,altitude:100}})).toThrow();
    expect(()=>restoreCourse(scenario,{...saved,variant:99})).toThrow();
  });
  it('코스와 통로 조건만 바꾸면 고정 3D 지형을 재사용하고 건물을 바꾸면 갱신한다',()=>{
    const a=createTrainingPlans(scenario,DEFAULT_INTENT)[0].world;
    const b=createTrainingPlans(scenario,{...DEFAULT_INTENT,visit:['B1'],closed:['D2'],low:['B1']})[0].world;
    expect(a.layoutKey).not.toBe(b.layoutKey);expect(terrainLayoutKey(a)).toBe(terrainLayoutKey(b));
    expect(terrainLayoutKey({...b,zones:[...b.zones,{kind:'obstacle',shape:'rect',x:0,y:0,w:2,h:2}]})).not.toBe(terrainLayoutKey(a));
  });
});
