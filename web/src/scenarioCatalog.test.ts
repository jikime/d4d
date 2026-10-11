import {describe,it,expect,vi} from 'vitest';
import {BUILTIN_SCENARIO_IDS,builtinScenario,loadScenarioData,scenarioOptions} from './scenarioCatalog';
import {buildMissionWorld,clearSegment} from './simulation/missionWorld';
import {createTrainingPlans,DEFAULT_INTENT,gridSpec} from './simulation/gridTraining';
import {buildPlanRequest} from './scenarioRequest';

describe('내장 임무 선택',()=>{
  it.each(BUILTIN_SCENARIO_IDS)('%s는 서버 요청 없이 불러오며 장애물과 겹치지 않는 비행 경로를 만든다',async id=>{
    const fetchRemote=vi.fn();const result=await loadScenarioData(id,true,fetchRemote);
    expect(fetchRemote).not.toHaveBeenCalled();expect(result.source).toBe('builtin');
    const world=gridSpec(result.data)?createTrainingPlans(result.data,DEFAULT_INTENT)[0].world:buildMissionWorld(result.data);
    expect(world.error).toBe('');expect(world.tags.length).toBeGreaterThan(0);expect(world.stops.length).toBeGreaterThan(1);
    let previous=world.start;for(const stop of world.stops){expect(clearSegment(world,previous,stop)).toBe(true);previous=stop;}
  });
  it('한 임무를 편집한 뒤 다시 선택해도 다른 사본과 원본을 바꾸지 않는다',()=>{
    const first=builtinScenario('demo_training_camp')!,original=structuredClone(first);
    first.tags[0].x=999;first.zones.length=0;
    expect(builtinScenario('demo_training_camp')).toEqual(original);
  });
  it('서버 임무 목록과 내장 목록을 중복 없이 합친다',()=>{
    const options=scenarioOptions(['demo_training_camp','custom-patrol','custom-patrol']);
    expect(options).toHaveLength(6);expect(options.at(-1)).toEqual({value:'custom-patrol',label:'custom-patrol'});
  });
  it('서버 모드는 원격 원본을 우선하고 연결 실패 시 내장 사본으로 전환한다',async()=>{
    const remote={...builtinScenario('demo_arena')!,name:'서버에서 편집한 임무'};
    expect(await loadScenarioData('demo_arena',false,async()=>remote)).toEqual({data:remote,source:'remote'});
    const result=await loadScenarioData('demo_arena',false,async()=>{throw new Error('offline');});
    expect(result.source).toBe('builtin');expect(buildPlanRequest(result.source,'demo_arena',false,result.data)).toEqual({scenario:result.data});
  });
  it('알 수 없는 임무를 기본 임무로 바꿔 성공한 것처럼 처리하지 않는다',async()=>{
    const fetchRemote=vi.fn().mockRejectedValue(new Error('missing'));
    await expect(loadScenarioData('missing',true,fetchRemote)).rejects.toThrow('선택한 내장 임무');expect(fetchRemote).not.toHaveBeenCalled();
    await expect(loadScenarioData('missing',false,fetchRemote)).rejects.toThrow('missing');
  });
});
