import {describe, expect, it} from 'vitest';
import {LiveMotionTracker, type WifiSnapshot} from './liveTelemetry';

function reading(ms=1000, sequence=1):WifiSnapshot {
  const sample=(data:Record<string,unknown>)=>({data,sequence,at_ms:ms,age_ms:0});
  return {state:'connected',host:'192.168.0.151',port:3333,session:1,server_ms:ms,read_only:true,error:'',
    latest:{info:sample({sim:false}),attitude:sample({roll_deg:10,pitch_deg:5,yaw_deg:90}),
      approach:sample({att:[10,5],height_valid:true,height_cm:120,h_src:'rangefinder',v_flow_valid:true,v_flow:[0,100],flow_q:100,vel_src:'flow',vz_cm_s:20}),
      fc:sample({status:true,armed:false,battery:true,vbat:12,amps:.5,motor_ok:true,motor:[1000,1000,1000,1000]})},
    frame:{sequence:0,width:0,height:0,age_ms:null,transfer_ms:null},received_age_ms:0,parse_errors:0,build:'TAGVISION-20261011-36'};
}

describe('실기체 계측 → 3D 움직임',()=>{
  it('잘못된 RC 배열과 알 수 없는 조종 권한을 계측값으로 표시하지 않는다',()=>{
    const s=reading();s.latest.msp={...s.latest.info,data:{authority_fresh:true,authority:'unknown',fc_rc:[{},1500,1500,1000]}};
    const r=new LiveMotionTracker().update(s);expect(r.rc).toBeNull();expect(r.authority).toBe('미확인');
  });
  it('기존 뷰어는 실측 높이·자세를 공유하고 없는 방위각을 만들지 않는다',()=>{
    const s=reading();s.source='camera_viewer';delete s.latest.info;delete s.latest.attitude;
    const r=new LiveMotionTracker().update(s);
    expect(r.roll).toBe(10);expect(r.height).toBe(1.2);expect(r.heading).toBeNull();expect(r.positionValid).toBe(false);
    s.latest.approach.data.h_src='tag_fallback';
    expect(new LiveMotionTracker().update(s).height).toBeNull();
  });
  it('null 자세 보고는 직전 유효 계측의 시각을 연장하지 않는다',()=>{
    const t=new LiveMotionTracker(),s=reading();delete s.latest.attitude;s.latest.approach.data.att=[null,null];
    s.latest.attitude_rp={data:{roll_deg:7,pitch_deg:-4},sequence:0,at_ms:500,age_ms:500};
    expect(t.update(s).roll).toBe(7);
    expect(t.update(s,800).roll).toBeNull();
  });
  it('기체 자세·고도는 실측 단위를 변환하고 RC 스틱값으로 자세를 만들지 않는다',()=>{
    const t=new LiveMotionTracker(),s=reading();
    s.latest.msp={...s.latest.info,data:{fc_rc:[1800,1700,1500,1000]}};
    const r=t.update(s);
    expect(r.height).toBe(1.2);
    expect(r.flight.roll).toBeCloseTo(-10*Math.PI/180);
    expect(r.flight.pitch).toBeCloseTo(5*Math.PI/180);
    expect(r.flight.yaw).toBeCloseTo(Math.PI/2);
    expect(r.flight.x).toBe(0);
    expect(r.motorPower).toBe(0);
  });
  it('방위각에 따라 기체 속도를 변환하고 새 표본 사이의 거리만 적분한다',()=>{
    const t=new LiveMotionTracker();t.update(reading());
    const next=reading(1500,2);
    const moved=t.update(next);
    expect(moved.flight.x).toBeCloseTo(.5);
    expect(moved.flight.y).toBeCloseTo(0);
    expect(t.update(next).flight.x).toBeCloseTo(.5);
    expect(t.update({...next,server_ms:1700},200).flight.x).toBeCloseTo(.5);
  });
  it('품질이 낮거나 오래된 속도와 센서 시험값으로 기체를 이동시키지 않는다',()=>{
    for(const invalidate of [
      (s:WifiSnapshot)=>{s.latest.approach.data.flow_q=15;},
      (s:WifiSnapshot)=>{s.latest.approach.age_ms=1500;},
      (s:WifiSnapshot)=>{s.latest.info.data.sim=true;},
      (s:WifiSnapshot)=>{s.latest.approach.data.vel_src='tag';},
      (s:WifiSnapshot)=>{s.latest.approach.data.height_cm=1;},
      (s:WifiSnapshot)=>{s.latest.attitude.age_ms=2000;},
    ]){
      const t=new LiveMotionTracker();t.update(reading());
      const bad=reading(1500,2);invalidate(bad);
      expect(t.update(bad).positionValid).toBe(false);
      expect(t.flight.x).toBe(0);
    }
  });
  it('통신 단절·긴 공백 뒤에도 누락 구간의 이동을 추측하지 않는다',()=>{
    const t=new LiveMotionTracker();t.update(reading());
    const broken=reading(1500,2);broken.state='reconnecting';
    expect(t.update(broken).roll).toBeNull();
    expect(t.update(broken).motors).toBeNull();
    expect(t.update(reading(3000,3)).flight.x).toBe(0);
    expect(t.update(reading(6000,4)).flight.x).toBe(0);
    expect(t.update(reading(6500,5)).flight.x).toBeCloseTo(.5);
  });
  it('새 연결과 원점 재설정은 기존 속도 표본으로 순간 이동하지 않는다',()=>{
    const t=new LiveMotionTracker();t.update(reading());t.update(reading(1500,2));t.zero();
    expect(t.update(reading(1500,2)).flight.x).toBe(0);
    expect(t.update(reading(2000,3)).flight.x).toBe(0);
    const reconnected=reading(2500,4);reconnected.session=2;
    expect(t.update(reconnected).flight.x).toBe(0);
    expect(t.flight.trail.length).toBeLessThanOrEqual(1);
  });
  it('수신이 멈추면 화면의 마지막 자세는 유지하되 수치의 유효성과 프로펠러 출력을 만료시킨다',()=>{
    const t=new LiveMotionTracker(),s=reading();s.latest.fc.data.motor=[1150,1100,1100,1100];
    expect(t.update(s).motorPower).toBeCloseTo(.15);
    const stale=t.update(s,4000);
    expect(stale.roll).toBeNull();expect(stale.height).toBeNull();expect(stale.motorPower).toBe(0);
    expect(stale.flight.roll).toBeCloseTo(-10*Math.PI/180);
  });
  it('부분적 자세 수신이 중단되면 임의의 0도로 돌리지 않는다',()=>{
    const t=new LiveMotionTracker();t.update(reading());
    const s=reading(1500,2);s.latest.attitude.data={roll_deg:null,pitch_deg:null,yaw_deg:null};s.latest.approach.data.att=[null,null];
    const r=t.update(s);expect(r.roll).toBeNull();expect(r.heading).toBeNull();
    expect(r.flight.roll).toBeCloseTo(-10*Math.PI/180);
  });
});
