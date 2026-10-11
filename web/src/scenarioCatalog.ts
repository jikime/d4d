import grid from '../../scenarios/demo_grid_training.json';
import camp from '../../scenarios/demo_training_camp.json';
import network from '../../scenarios/demo_spider.json';
import arena from '../../scenarios/demo_arena.json';
import jam from '../../scenarios/demo_jam.json';
import type {Scenario} from './scenarioTypes';

export const DEFAULT_SCENARIO_ID='demo_grid_training';
const catalog=new Map<string,{label:string;data:Scenario}>([
  [DEFAULT_SCENARIO_ID,{label:'격자 훈련장',data:grid as Scenario}],
  ['demo_training_camp',{label:'청솔 훈련장',data:camp as Scenario}],
  ['demo_spider',{label:'MANGO 통신 훈련',data:network as Scenario}],
  ['demo_arena',{label:'장애물 회피 훈련',data:arena as Scenario}],
  ['demo_jam',{label:'재밍 구역 훈련',data:jam as Scenario}],
]);
export const BUILTIN_SCENARIO_IDS=[...catalog.keys()];
export const scenarioLabel=(id:string)=>catalog.get(id)?.label??id;
export function scenarioOptions(remoteNames:readonly string[]=[]){
  return [...new Set([...BUILTIN_SCENARIO_IDS,...remoteNames])].map(value=>({value,label:scenarioLabel(value)}));
}
/** 편집한 사본이 다음에 선택할 원본 임무에 섞이지 않게 한다. */
export function builtinScenario(id:string):Scenario|undefined {
  const entry=catalog.get(id);return entry?structuredClone(entry.data):undefined;
}
/** 정적 배포는 내장 파일만 사용하고, 로컬 서버가 있는 모드에서는 서버 원본을 우선한다. */
export async function loadScenarioData(id:string,demo:boolean,fetchRemote:(id:string)=>Promise<Scenario>):Promise<{data:Scenario;source:'builtin'|'remote'}>{
  const local=builtinScenario(id);
  if(demo){if(!local)throw new Error('선택한 내장 임무를 찾을 수 없습니다.');return {data:local,source:'builtin'};}
  try{return {data:await fetchRemote(id),source:'remote'};}
  catch(error){if(local)return {data:local,source:'builtin'};throw error;}
}
