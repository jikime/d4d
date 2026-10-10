import {describe,it,expect} from 'vitest';
import {PilotFlight,NEUTRAL_INPUT,keyboardPilotInput,missionFlightRoute,trainingRoute} from './pilotFlight';

describe('키보드 조종과 실제 경유점 비행',()=>{
  it('양쪽 스틱의 네 축을 함께 입력하고 대각선 속도를 정규화한다',()=>{
    const input=keyboardPilotInput(new Set(['KeyW','KeyD','ArrowUp','ArrowRight']));
    expect(input.climb).toBe(1);expect(input.yaw).toBe(1);expect(Math.hypot(input.right,input.forward)).toBeCloseTo(1);
    const flight=new PilotFlight();for(let i=0;i<120;i++)flight.step(input,1/60);
    expect(flight.state.height).toBeGreaterThan(2);expect(flight.state.yaw).toBeGreaterThan(1);
    expect(Math.hypot(flight.state.x,flight.state.y)).toBeGreaterThan(1);expect(flight.state.trail.length).toBeGreaterThan(10);
  });
  it('반대 키는 상쇄하며 한 키를 놓아도 남은 축은 계속 움직인다',()=>{
    const keys=new Set(['KeyW','KeyS','ArrowUp','ArrowRight']);
    expect(keyboardPilotInput(keys).climb).toBe(0);keys.delete('ArrowRight');
    expect(keyboardPilotInput(keys)).toMatchObject({right:0,forward:1});
  });
  it('기체 방위각을 기준으로 전진하고 키 해제 뒤 제동해 호버한다',()=>{
    const flight=new PilotFlight();flight.state.yaw=Math.PI/2;
    for(let i=0;i<120;i++)flight.step({...NEUTRAL_INPUT,forward:1},1/60);
    expect(flight.state.x).toBeGreaterThan(2);expect(Math.abs(flight.state.y)).toBeLessThan(.001);
    for(let i=0;i<90;i++)flight.step(NEUTRAL_INPUT,1/60);
    expect(Math.hypot(flight.state.vx,flight.state.vy)).toBeLessThan(.001);
    const before=flight.snapshot();for(let i=0;i<60;i++)flight.step(NEUTRAL_INPUT,1/60);
    expect(flight.state.x).toBeCloseTo(before.x,10);
  });
  it('자동 경로를 차례대로 통과하고 마지막 경유점에 정지한다',()=>{
    const flight=new PilotFlight();flight.followRoute();const visited=new Set<number>([flight.state.waypoint]);
    for(let i=0;i<9000&&!flight.state.completed;i++){flight.step(NEUTRAL_INPUT,1/60);visited.add(flight.state.waypoint);}
    expect([...visited]).toEqual([0,1,2,3,4]);expect(flight.state.completed).toBe(true);
    const goal=trainingRoute().points.at(-1)!;
    expect(Math.hypot(flight.state.x-goal.x,flight.state.y-goal.y,flight.state.height-goal.height)).toBeLessThan(.12);
    expect(flight.state.travelled).toBeGreaterThan(15);
  });
  it('수동 입력은 위치를 초기화하지 않고 자동 비행의 제어권을 가져온다',()=>{
    const flight=new PilotFlight();flight.followRoute();for(let i=0;i<180;i++)flight.step(NEUTRAL_INPUT,1/60);
    const before=flight.snapshot();flight.step({...NEUTRAL_INPUT,right:1},1/60);
    expect(flight.state.mode).toBe('manual');expect(Math.hypot(flight.state.x-before.x,flight.state.y-before.y)).toBeLessThan(.04);
    const target=flight.state.waypoint;flight.followRoute();expect(flight.state.waypoint).toBe(target);
  });
  it('메인 경로의 축척과 경유점 순서를 유지하고 범위 밖 데이터는 거부한다',()=>{
    const route=missionFlightRoute([{x:5,y:7},{x:25,y:19},{x:9,y:11}],'메인 계획')!;
    expect(route.points.map(p=>[p.x,p.y])).toEqual([[0,0],[20,12],[4,4]]);expect(route.origin).toEqual({x:5,y:7});
    expect(missionFlightRoute([{x:0,y:0},{x:100,y:0}],'범위 밖')).toBeNull();
    expect(missionFlightRoute([{x:0,y:0},{x:NaN,y:0}],'잘못된 입력')).toBeNull();
  });
  it('일정한 스텝이면 프레임 수와 무관하며 지면·훈련 구역을 넘지 않는다',()=>{
    const a=new PilotFlight(),b=new PilotFlight(),input={...NEUTRAL_INPUT,right:1,climb:-1};
    for(let i=0;i<60;i++)a.step(input,1/30);for(let i=0;i<120;i++)b.step(input,1/60);
    expect(a.state.x).toBeCloseTo(b.state.x,10);expect(a.state.height).toBeGreaterThanOrEqual(.245);
    for(let i=0;i<3000;i++)a.step(input,1/60);expect(a.state.x).toBe(42);expect(a.state.vx).toBe(0);
  });
});
