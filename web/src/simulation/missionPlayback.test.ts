import {describe, expect, it} from 'vitest';
import {MissionFlight} from './missionFlight';
import {missionPlaybackBudget} from './missionPlayback';
import {buildMissionWorld, type MissionScenario} from './missionWorld';
import {NEUTRAL_INPUT} from './pilotFlight';
import {observeTag} from './tagVision';
import type {PixelFrame} from './pixelVision';

const scenario: MissionScenario = {name:'재생 시간 검증',field:{width_m:20,height_m:20},start:{x:5,y:5},goal:{x:5,y:5},tags:[{id:1,x:5,y:5},{id:2,x:5,y:9}],zones:[],objectives:[{name:'태그 2',x:5,y:9}],link_policy:'continue_mission'};

// 단위 시험용 카메라. 실제 UI에서는 WASM이 반환한 픽셀 검출만 사용한다.
function capture(flight: MissionFlight, sequence: number): PixelFrame {
  const s=flight.state;
  return {session:s.visionSession, sequence, tMs:s.tMs, pose:{...s}, processingMs:5,
    detections:flight.world.tags.map(tag=>observeTag(s,tag,flight.world)).filter(tag=>tag.valid).map(tag=>({id:tag.code,family:'tag36h11',hamming:0,decisionMargin:100,center:{x:tag.center[0],y:tag.center[1]},corners:tag.corners.map(([x,y])=>({x,y}))}))};
}

describe('영상 처리 속도에 맞춘 시뮬레이션 재생',()=>{
  it('10fps에서 두 화면 뒤에 도착하는 카메라 결과로도 4배속 순찰을 완주한다',()=>{
    const flight=new MissionFlight(buildMissionWorld(scenario));
    flight.setVisionStatus('ready');flight.start();
    const pending:PixelFrame[]=[];
    let limited=false;
    for(let tick=0;tick<3000&&!flight.state.completed;tick++){
      pending.push(capture(flight,tick+1));
      if(pending.length>2)flight.acceptVision(pending.shift()!);
      let budget=Math.min(.4,missionPlaybackBudget(flight.state,NEUTRAL_INPUT));
      limited ||= budget<.4;
      while(budget>=1/60){flight.step(NEUTRAL_INPUT,1/60);budget-=1/60;}
    }
    expect(limited).toBe(true);
    expect(flight.state.completed).toBe(true);
    expect(flight.state.visited).toEqual(['1','2','1']);
    expect(flight.state.height).toBeCloseTo(.22);
  });

  it('새 영상이 없으면 확인 시간을 쌓지 않고, 수동 조종은 즉시 인계한다',()=>{
    const flight=new MissionFlight(buildMissionWorld(scenario));flight.setVisionStatus('ready');flight.start();
    expect(missionPlaybackBudget(flight.state,NEUTRAL_INPUT)).toBe(0);
    flight.acceptVision(capture(flight,1));
    for(let tick=0;tick<500;tick++)if(missionPlaybackBudget(flight.state,NEUTRAL_INPUT)>=1/60)flight.step(NEUTRAL_INPUT,1/60);
    expect(flight.state.visited).toEqual([]);
    expect(flight.state.tMs).toBeLessThan(700);
    const input={...NEUTRAL_INPUT,right:1};
    expect(missionPlaybackBudget(flight.state,input)).toBe(Infinity);
    flight.step(input,1/60);expect(flight.state.autonomous).toBe(false);
  });

  it('영상 오류 시 기존 위치 유지와 오류 안내 처리를 계속 실행한다',()=>{
    const flight=new MissionFlight(buildMissionWorld(scenario));flight.start();flight.setVisionStatus('error','카메라 오류');
    expect(missionPlaybackBudget(flight.state,NEUTRAL_INPUT)).toBe(Infinity);
  });
});
