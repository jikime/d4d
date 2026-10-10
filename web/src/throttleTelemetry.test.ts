import {describe, expect, it} from 'vitest';
import type {WifiSnapshot} from './liveTelemetry';
import {pulsePercent, signalPath, ThrottleMonitor} from './throttleTelemetry';

function reading(at=1000): WifiSnapshot {
  const sample=(data:Record<string,unknown>)=>({data,sequence:at,at_ms:at,age_ms:0});
  return {state:'connected',host:'192.168.0.151',port:3333,source:'camera_viewer',session:1,server_ms:at,read_only:true,error:'',
    latest:{msp:sample({fc_link:true,authority_fresh:true,fc_rc:[1500,1500,1700,1250],rc_throttle:1250,
      out:[1500,1500,1000,1700],thr_slot:2,thr_slot_us:1000,throttle_ctrl:false}),
      fc:sample({motor_ok:true,motor:[1300,1400,1200,1500]})},
    received_age_ms:0,frame:{sequence:0,width:0,height:0,age_ms:null,transfer_ms:null},parse_errors:0,build:''};
}

describe('실제 스로틀 관측',()=>{
  it('RC 입력과 모터 평균을 구분하며 마스킹된 보드 슬롯을 스로틀로 표시하지 않는다',()=>{
    const r=new ThrottleMonitor().update(reading());
    expect(r.inputUs).toBe(1250);expect(r.inputPercent).toBe(25);
    expect(r.motorMeanUs).toBe(1350);expect(r.motorPercent).toBe(35);expect(r.boardControl).toBe(false);
    const s=reading();delete s.latest.msp.data.rc_throttle;
    expect(new ThrottleMonitor().update(s).inputUs).toBe(1250);
  });
  it('FC 링크나 최신 조종 권한이 없으면 보관된 RC 값으로 출력을 만들지 않는다',()=>{
    for(const key of ['fc_link','authority_fresh']) {
      const s=reading();s.latest.msp.data[key]=false;
      const r=new ThrottleMonitor().update(s);expect(r.inputUs).toBeNull();expect(r.inputHistory).toHaveLength(0);
    }
  });
  it('입력과 출력의 유효기간을 따로 확인하고 연결 단절을 0µs로 표시하지 않는다',()=>{
    const monitor=new ThrottleMonitor(),s=reading();monitor.update(s);s.latest.msp.age_ms=2300;
    const stale=monitor.update(s);expect(stale.inputUs).toBeNull();expect(stale.motorMeanUs).toBe(1350);
    expect(monitor.update(s,2400).motorMeanUs).toBeNull();
    s.state='reconnecting';const lost=monitor.update(s);expect(lost.inputUs).toBeNull();expect(lost.motorMeanUs).toBeNull();
  });
  it('같은 보고를 여러 번 렌더링해도 기록을 늘리지 않고 새 연결에서 초기화한다',()=>{
    const monitor=new ThrottleMonitor();monitor.update(reading());
    expect(monitor.update(reading(),100).inputHistory).toHaveLength(1);
    expect(monitor.update(reading(2000)).inputHistory).toHaveLength(2);
    const s=reading(3000);s.session=2;expect(monitor.update(s).inputHistory).toHaveLength(1);
    expect(monitor.update(s,61_000).inputHistory).toHaveLength(0);
  });
  it('잘못된 값은 제외하고 정상 범위를 벗어난 보정 펄스는 표시 눈금만 제한한다',()=>{
    const s=reading();s.latest.msp.data.rc_throttle=NaN;s.latest.msp.data.fc_rc=[];
    s.latest.fc.data.motor=[1300,null,1400,1500];
    const r=new ThrottleMonitor().update(s);expect(r.inputUs).toBeNull();expect(r.motorMeanUs).toBeNull();
    expect(pulsePercent(null)).toBeNull();expect(pulsePercent(988)).toBe(0);expect(pulsePercent(2012)).toBe(100);
  });
  it('그래프는 수신 공백을 연결하거나 미래·화면 밖의 표본을 표시하지 않는다',()=>{
    const p=signalPath([{at:100,value:1000},{at:1000,value:1200},{at:1500,value:1300},{at:4500,value:1400},{at:6000,value:1800}],5000,4000);
    expect(p.match(/M/g)).toHaveLength(2);expect(p.match(/L/g)).toHaveLength(1);expect(p).not.toContain('NaN');
  });
});
