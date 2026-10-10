import {describe,it,expect} from 'vitest';
import {FlightRecorder,replayFrame,replaySnapshot} from './flightRecording';
import {MissionFlight} from './missionFlight';
import {buildMissionWorld,type MissionScenario} from './missionWorld';

const scenario:MissionScenario={name:'기록 검사',field:{width_m:20,height_m:20},start:{x:5,y:5},goal:{x:5,y:5},tags:[{id:1,x:5,y:5}],zones:[],objectives:[],link_policy:'continue_mission'};
function setup(){const engine=new MissionFlight(buildMissionWorld(scenario));return {engine,recorder:new FlightRecorder(engine.world,engine.tuning,false,engine.snapshot())};}
describe('비행 결과와 기록 재생',()=>{
  it('완료 기록은 시간·거리·태그 확인을 보존하고 한 번만 저장한다',()=>{
    const {engine,recorder}=setup();Object.assign(engine.state,{tMs:1000,x:-4,height:2,travelled:1,phase:'transit'});recorder.capture(engine.state);
    Object.assign(engine.state,{tMs:2000,x:-3,height:.22,travelled:3,phase:'complete',completed:true,visited:['1']});
    const record=recorder.finish(engine.state)!;
    expect(record.summary).toMatchObject({duration:2,distance:3,confirmations:1});expect(record.status).toBe('completed');
    expect(recorder.finish(engine.state)).toBeNull();expect(record.summary.meanError).toBeNull();
  });
  it('재생 시 위치를 보간하고 180도 경계에서 기체를 거꾸로 회전시키지 않는다',()=>{
    const {engine,recorder}=setup();Object.assign(engine.state,{tMs:1000,x:0,height:2,yaw:179*Math.PI/180});recorder.capture(engine.state);
    Object.assign(engine.state,{tMs:2000,x:4,height:4,yaw:-179*Math.PI/180});const record=recorder.finish(engine.state)!;
    const mid=replayFrame(record,1500);expect(mid.x).toBe(2);expect(mid.height).toBe(3);expect(mid.yaw).toBeCloseTo(Math.PI);
    const before=JSON.stringify(engine.snapshot());const replay=replaySnapshot(record,1500);
    expect(replay.tMs).toBe(1500);expect(JSON.stringify(engine.snapshot())).toBe(before);expect(replayFrame(record,99999).t).toBe(2000);
  });
  it('긴 비행의 시작과 끝을 유지하면서 재생 프레임 수를 제한한다',()=>{
    const {engine,recorder}=setup();for(let i=1;i<=5000;i++){engine.state.tMs=i*250;engine.state.x=i/10000;recorder.capture(engine.state);}
    const record=recorder.finish(engine.state)!;expect(record.frames.length).toBeLessThanOrEqual(1800);expect(record.frames[0].t).toBe(0);expect(record.frames.at(-1)!.t).toBe(1250000);
  });
  it('비행하지 않은 준비 상태를 결과로 저장하지 않는다',()=>{const {engine,recorder}=setup();expect(recorder.finish(engine.state)).toBeNull();});
  it('제한 구역 진입 전에 수행하는 정상 하강을 고도 위반으로 집계하지 않는다',()=>{
    const engine=new MissionFlight(buildMissionWorld({...scenario,zones:[{kind:'height_limit',shape:'rect',x:8,y:8,w:4,h:4,value:2}]}));
    const recorder=new FlightRecorder(engine.world,engine.tuning,false,engine.snapshot());
    Object.assign(engine.state,{tMs:1000,height:3.2,phase:'transit'});recorder.capture(engine.state);
    Object.assign(engine.state,{tMs:2000,x:0,y:0,height:2});recorder.capture(engine.state);
    Object.assign(engine.state,{tMs:3000,height:2.3});recorder.capture(engine.state);
    Object.assign(engine.state,{tMs:4000,phase:'land'});const record=recorder.finish(engine.state)!;
    expect(record.summary.altitudeCompliance).toBe(50);
  });
  it('비행 도중 제어 설정을 바꾸면 동일 설정의 경험 비교에서 구분할 수 있다',()=>{
    const {engine,recorder}=setup();recorder.markTuningChanged();engine.state.tMs=1000;
    expect(recorder.finish(engine.state)!.tuningChanged).toBe(true);
  });
});
