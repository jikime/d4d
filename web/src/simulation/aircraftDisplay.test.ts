import {describe,expect,it} from 'vitest';
import {AIRCRAFT_DIAMETER,aircraftDisplayScale} from './aircraftDisplay';

describe('전체 보기 기체 표시 크기',()=>{
  it('근접 시점에서는 기체의 실제 크기를 유지한다',()=>{
    expect(aircraftDisplayScale(3,37,1,900,2.2)).toBe(1);
  });
  it('원거리에서는 화면에 72px 폭으로 보이며 줌에 따라 확대량을 줄인다',()=>{
    const scale=aircraftDisplayScale(40,37,1,900,2.2);
    const pixels=AIRCRAFT_DIAMETER*scale/(2*40*Math.tan(37*Math.PI/360))*900;
    expect(pixels).toBeCloseTo(72);
    expect(aircraftDisplayScale(40,37,2,900,2.2)).toBeCloseTo(scale/2);
  });
  it('지도에서 멀어져도 확대 상한을 지키고 저고도에서 착륙 다리가 지면을 뚫지 않는다',()=>{
    expect(aircraftDisplayScale(100,37,1,900,3)).toBe(8);
    for(const height of [.22,.5,1]){
      const scale=aircraftDisplayScale(100,37,1,900,height);
      expect(height-.115*scale).toBeGreaterThanOrEqual(.0099);
    }
  });
});
