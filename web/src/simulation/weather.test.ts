import {describe,it,expect} from 'vitest';
import {advanceWeather,changeWeather,createWeather,normalizeWeather,weatherWind,wetCameraPixels} from './weather';
import {MissionFlight} from './missionFlight';
import {buildMissionWorld,type MissionScenario} from './missionWorld';
import {NEUTRAL_INPUT} from './pilotFlight';
import {FlightRecorder,replayFrame,replaySnapshot,sameWeatherConditions} from './flightRecording';

const scenario:MissionScenario={name:'날씨 시험',field:{width_m:20,height_m:20},start:{x:5,y:5},goal:{x:5,y:5},tags:[{id:1,x:5,y:5}],zones:[],objectives:[],link_policy:'continue_mission'};
describe('훈련 날씨',()=>{
  it('적설과 지면 젖음은 모의 시간이 흐를 때만 변하며 작은 시간 간격으로 나눠도 동일하다',()=>{
    const initial=createWeather({kind:'snow',intensity:.6,snowBase:.2});let small=initial;
    for(let i=0;i<600;i++)small=advanceWeather(small,.1);
    const once=advanceWeather(initial,60);
    expect(small.snowCover).toBeCloseTo(once.snowCover,10);expect(small.elapsed).toBeCloseTo(60);
    expect(advanceWeather(initial,0)).toEqual(initial);expect(initial.snowCover).toBe(.2);
    expect(advanceWeather(createWeather({kind:'rain'}),60).wetness).toBe(1);
  });
  it('풍향은 세계 좌표를 따르고 같은 시드와 시간에 같은 돌풍을 만든다',()=>{
    const east=createWeather({windSpeed:4,windDirection:90,gust:0});expect(weatherWind(east).x).toBeCloseTo(4);expect(weatherWind(east).y).toBeCloseTo(0);
    const north=createWeather({windSpeed:4,windDirection:0,gust:0});expect(weatherWind(north).y).toBeCloseTo(4);
    expect(weatherWind(advanceWeather({...east,gust:.8},20))).toEqual(weatherWind(advanceWeather({...east,gust:.8},20)));
    expect(weatherWind(advanceWeather({...east,gust:.8},20))).not.toEqual(weatherWind(advanceWeather({...east,gust:.8},22)));
  });
  it('잘못된 저장 값과 범위를 보정하고 환경 변경은 경과 시간을 되돌리지 않는다',()=>{
    expect(normalizeWeather({intensity:9,windSpeed:Infinity,snowBase:-1,windDirection:-90})).toMatchObject({intensity:1,windSpeed:0,snowBase:0,windDirection:270});
    const state=advanceWeather(createWeather({kind:'snow'}),20);
    expect(changeWeather(state,{windSpeed:3}).elapsed).toBe(20);
    expect(changeWeather(state,{snowBase:1}).snowCover).toBe(1);
  });
  it('맑을 때는 카메라 픽셀을 보존하고 비의 렌즈 효과는 같은 시간에 재현된다',()=>{
    const input=new Uint8ClampedArray(320*240*4).fill(20),clear=input.slice();
    wetCameraPixels(clear,new Uint8ClampedArray(clear.length),createWeather(),false);expect(clear).toEqual(input);
    const a=input.slice(),b=input.slice(),rain=advanceWeather(createWeather({kind:'rain',intensity:1}),8);
    wetCameraPixels(a,new Uint8ClampedArray(a.length),rain,false);wetCameraPixels(b,new Uint8ClampedArray(b.length),rain,false);
    expect(a).toEqual(b);expect(a.some((v,i)=>v!==input[i])).toBe(true);
  });
  it('악천후에서 실제 디코더의 빈 결과가 계속되면 다음 태그로 건너뛰지 않고 훈련을 중단한다',()=>{
    const flight=new MissionFlight(buildMissionWorld(scenario),undefined,undefined,{...normalizeWeather(),kind:'snow',snowBase:1});flight.start();
    for(let i=0;i<5000&&!flight.state.aborted;i++){
      const s=flight.state;flight.acceptVision({session:s.visionSession,sequence:i+1,tMs:s.tMs,pose:{...s},processingMs:1,detections:[]});flight.step(NEUTRAL_INPUT,1/60);
    }
    expect(flight.state.aborted).toBe(true);expect(flight.state.phase).toBe('aborted');expect(flight.state.visited).toEqual([]);expect(flight.state.completed).toBe(false);
    const time=flight.state.tMs;flight.step(NEUTRAL_INPUT,1);expect(flight.state.tMs).toBe(time);
  });
  it('날씨를 바꾸기 전에 촬영한 카메라 결과는 새 환경에서 사용하지 않는다',()=>{
    const flight=new MissionFlight(buildMissionWorld(scenario)),session=flight.state.visionSession;
    flight.setWeather({kind:'rain'});flight.acceptVision({session,sequence:1,tMs:0,pose:{...flight.state},processingMs:1,detections:[]});
    expect(flight.state.vision.sequence).toBe(0);expect(flight.state.visionSession).not.toBe(session);
  });
  it('바람 설정이 가상 기체 움직임에 반영된다',()=>{
    const calm=new MissionFlight(buildMissionWorld(scenario)),windy=new MissionFlight(buildMissionWorld(scenario),undefined,undefined,normalizeWeather({windSpeed:5,windDirection:90,gust:0}));
    for(const f of [calm,windy]){f.manual();f.state.height=2;for(let i=0;i<120;i++)f.step(NEUTRAL_INPUT,1/60);}
    expect(windy.state.x).toBeGreaterThan(calm.state.x+.1);
  });
});

describe('날씨 기록과 재생',()=>{
  function record(kind:'rain'|'snow'='snow'){
    const engine=new MissionFlight(buildMissionWorld(scenario),undefined,undefined,normalizeWeather({kind}));
    const recorder=new FlightRecorder(engine.world,engine.tuning,false,engine.snapshot());
    engine.state.tMs=1000;engine.state.weather=advanceWeather(engine.state.weather,1);recorder.capture(engine.state);
    engine.state.tMs=2000;engine.state.weather=advanceWeather(engine.state.weather,1);
    return recorder.finish(engine.state)!;
  }
  it('뒤로 탐색해도 적설·바람·입자 시간이 원래 값으로 복원된다',()=>{
    const r=record(),mid=replayFrame(r,1500);expect(mid.weather!.elapsed).toBeCloseTo(1.5);
    expect(mid.weather!.snowCover).toBeGreaterThan(r.frames[0].weather!.snowCover);
    replaySnapshot(r,1900);expect(replaySnapshot(r,0).weather).toEqual(r.frames[0].weather);
  });
  it('다른 날씨나 비행 중 날씨를 바꾼 결과를 경험 효과로 비교하지 않는다',()=>{
    const a=record(),b=record(),rain=record('rain');expect(sameWeatherConditions(a,b)).toBe(true);expect(sameWeatherConditions(a,rain)).toBe(false);
    b.environmentChanged=true;expect(sameWeatherConditions(a,b)).toBe(false);
  });
  it('같은 설정이어도 돌풍의 시작 시점이 다른 비행은 비교하지 않는다',()=>{
    const a=record(),b=record();b.frames[0].weather!.elapsed=8;
    expect(sameWeatherConditions(a,b)).toBe(false);
  });
  it('날씨 변경과 중단 사유를 보존하고 기존 날씨 없는 기록도 재생한다',()=>{
    const engine=new MissionFlight(buildMissionWorld(scenario)),recorder=new FlightRecorder(engine.world,engine.tuning,false,engine.snapshot());
    engine.state.tMs=1000;engine.setWeather({kind:'rain'});recorder.capture(engine.state);
    engine.state.tMs=2000;engine.state.aborted=true;engine.state.phase='aborted';engine.state.tagStats.lossSeconds=30;
    const r=recorder.finish(engine.state)!;expect(r.environmentChanged).toBe(true);expect(r.status).toBe('failed');expect(r.summary.maxTagLoss).toBe(30);expect(r.events.some(e=>e.text.startsWith('날씨 변경'))).toBe(true);
    r.frames.forEach(f=>delete f.weather);expect(replaySnapshot(r,500).weather.kind).toBe('clear');
  });
});
