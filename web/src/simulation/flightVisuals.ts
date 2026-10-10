/** 표시 전용 모델. 제어기의 위치·속도·출력에는 영향을 주지 않는다. */
export const ROTOR_DIRECTIONS = [1, -1, -1, 1] as const;
export const clamp = (value: number, low: number, high: number) => Math.max(low, Math.min(high, value));
export type CameraInput={panX:number;panY:number;rotateX:number;rotateY:number};
export function normalizeStick(x:number,y:number) {
  if(!Number.isFinite(x)||!Number.isFinite(y))return {x:0,y:0};
  const distance=Math.hypot(x,y);if(distance<.08)return {x:0,y:0};
  const scale=distance>1?1/distance:1;return {x:x*scale,y:y*scale};
}
export function damp(current: number, target: number, rate: number, dt: number) {
  return current + (target - current) * (1 - Math.exp(-rate * Math.max(0, dt)));
}
export function rotorWash(heightM: number, rotorPower: number, distanceM: number) {
  const radius = 0.48 + Math.max(0, heightM) * 0.42;
  return clamp(rotorPower, 0, 1) * Math.exp(-Math.max(0, heightM) * 0.52) * Math.exp(-Math.pow(distanceM / radius, 2));
}
export function terrainHeight(x: number, z: number) {
  const distance = Math.hypot(x, z);
  const t = clamp((distance - 2.2) / 7, 0, 1);
  const blend = t * t * (3 - 2 * t);
  const hills = 0.35 + Math.sin(x * 0.19 + z * 0.09) * 0.7 + Math.cos(z * 0.28 - x * 0.07) * 0.5;
  const detail = Math.sin(x * 1.32) * Math.cos(z * 1.15) * 0.07;
  return -0.035 + blend * (hills + detail);
}
export function seededRandom(seed: number) {
  let value = seed >>> 0;
  return () => { value = (Math.imul(value, 1664525) + 1013904223) >>> 0; return value / 4294967296; };
}

export function visualAttitude(rollUs: number, pitchUs: number) {
  // 가속 방향에 맞는 작은 기울기. 절대 항공기 자세나 RPM 측정값이 아니다.
  return { pitch: clamp(pitchUs * 0.0018, -0.18, 0.18), roll: clamp(-rollUs * 0.0018, -0.18, 0.18) };
}
