import { describe, expect, it } from "vitest";
import { damp, rotorWash, terrainHeight, seededRandom, visualAttitude, ROTOR_DIRECTIONS, normalizeStick } from "./flightVisuals";

describe("제어 모델과 분리한 비행 시각 효과", () => {
  it("스틱의 중립 흔들림과 잘못된 입력은 이동을 만들지 않는다",()=>{
    expect(normalizeStick(.03,-.02)).toEqual({x:0,y:0});
    expect(normalizeStick(Infinity,1)).toEqual({x:0,y:0});
  });
  it("스틱을 원 바깥으로 끌어도 대각선 입력의 크기가 1을 넘지 않는다",()=>{
    const value=normalizeStick(4,4);expect(Math.hypot(value.x,value.y)).toBeCloseTo(1,12);
    expect(normalizeStick(.4,-.3)).toEqual({x:.4,y:-.3});
  });
  it("같은 대각선 모터는 같은 방향, 인접 모터는 반대 방향으로 돈다", () => {
    expect(ROTOR_DIRECTIONS[0]).toBe(ROTOR_DIRECTIONS[3]);
    expect(ROTOR_DIRECTIONS[1]).toBe(ROTOR_DIRECTIONS[2]);
    expect(ROTOR_DIRECTIONS[0]).toBe(-ROTOR_DIRECTIONS[1]);
  });
  it("하강풍은 모터 정지 때 사라지고 거리와 고도에 따라 줄어든다", () => {
    expect(rotorWash(1.2, 0, 0)).toBe(0);
    expect(rotorWash(1.2, 1, 0)).toBeGreaterThan(rotorWash(1.2, 1, 2));
    expect(rotorWash(1, 1, 0)).toBeGreaterThan(rotorWash(3, 1, 0));
    expect(rotorWash(0, 3, 0)).toBe(1);
  });
  it("태그 접근 구역은 평탄하고 외곽 지형은 변한다", () => {
    for (const [x,z] of [[0,0], [1,-1], [-1.4,1.4]]) expect(terrainHeight(x,z)).toBe(-0.035);
    expect(terrainHeight(12,12)).not.toBe(terrainHeight(-12,-12));
  });
  it("프레임 수와 무관하게 같은 시간 후 같은 모터 응답을 낸다", () => {
    const at30 = Array.from({length:30}).reduce<number>((power) => damp(power,1,4,1/30),0);
    const at60 = Array.from({length:60}).reduce<number>((power) => damp(power,1,4,1/60),0);
    expect(at30).toBeCloseTo(at60,12);
    expect(damp(0,1,4,0)).toBe(0);
  });
  it("시각 자세는 제한되고 지형 배치는 재생성해도 동일하다", () => {
    expect(visualAttitude(10000,-10000)).toEqual({roll:-0.18,pitch:-0.18});
    const a=seededRandom(42), b=seededRandom(42);
    expect(Array.from({length:20},a)).toEqual(Array.from({length:20},b));
  });
});
