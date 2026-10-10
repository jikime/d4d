import {createTrainingPlans,type TrainingIntent,type TrainingPlan} from './gridTraining';
import type {MissionScenario} from './missionWorld';

export interface SavedCourse {
  version:1; id:string; name:string; savedAt:string; map:string;
  intent:TrainingIntent; variant:number; launch:{x:number;y:number;height:number};
}
export const COURSE_STORAGE='mango-training-courses-v1';
export const ACTIVE_COURSE_STORAGE='mango-training-active-v1';
export function scenarioSignature(s:MissionScenario){return JSON.stringify({field:s.field,start:s.start,goal:s.goal,tags:s.tags,zones:s.zones,grid:s.training_grid});}
export function saveCourseData(scenario:MissionScenario,plan:TrainingPlan,name:string):SavedCourse {
  return {version:1,id:crypto.randomUUID(),name:name.trim().slice(0,80)||'저장한 코스',savedAt:new Date().toISOString(),map:scenarioSignature(scenario),intent:structuredClone(plan.intent),variant:Number(plan.id.split('-')[1])||0,launch:{...plan.world.launch??{...plan.world.start,height:.22}}};
}
/** 저장 파일의 경유점을 신뢰하지 않고 현재 지형에서 조건·통로·고도를 다시 검증한다. */
export function restoreCourse(scenario:MissionScenario,value:unknown):TrainingPlan {
  if(!value||typeof value!=='object')throw new Error('올바른 코스 파일이 아닙니다.');
  const saved=value as SavedCourse;
  if(saved.version!==1||saved.map!==scenarioSignature(scenario))throw new Error('현재 훈련장과 다른 지도의 코스입니다.');
  if(!Number.isInteger(saved.variant)||saved.variant<0||saved.variant>2||!saved.launch||![saved.launch.x,saved.launch.y,saved.launch.height].every(Number.isFinite)||saved.launch.height<.22||saved.launch.height>10)throw new Error('코스 시작 위치가 올바르지 않습니다.');
  const plan=createTrainingPlans(scenario,saved.intent,saved.launch)[saved.variant];
  if(!plan)throw new Error('저장한 후보 경로를 복원할 수 없습니다.');
  return plan;
}
export function readCourses():SavedCourse[]{try{const value=JSON.parse(localStorage.getItem(COURSE_STORAGE)??'[]');return Array.isArray(value)?value.filter(v=>v?.version===1&&typeof v.name==='string'&&typeof v.id==='string'&&Array.isArray(v.intent?.visit)&&typeof v.savedAt==='string').slice(0,12):[];}catch{return [];}}
export function writeCourses(courses:SavedCourse[]){localStorage.setItem(COURSE_STORAGE,JSON.stringify(courses.slice(0,12)));}
export function readActiveCourse(scenario:MissionScenario):TrainingPlan|null {try{const value=localStorage.getItem(ACTIVE_COURSE_STORAGE);return value?restoreCourse(scenario,JSON.parse(value)):null;}catch{return null;}}
export function rememberActiveCourse(scenario:MissionScenario,plan:TrainingPlan|null){try{if(plan)localStorage.setItem(ACTIVE_COURSE_STORAGE,JSON.stringify(saveCourseData(scenario,plan,'현재 코스')));else localStorage.removeItem(ACTIVE_COURSE_STORAGE);return true;}catch{return false;}}
export function downloadJson(value:unknown,name:string){const url=URL.createObjectURL(new Blob([JSON.stringify(value,null,2)],{type:'application/json'}));const link=document.createElement('a');link.href=url;link.download=name;link.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}
