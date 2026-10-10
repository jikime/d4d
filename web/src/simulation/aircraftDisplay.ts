export const AIRCRAFT_DIAMETER = 0.482;

/** 관제 지도에서만 최소 가독 크기를 보장한다. 비행·센서 모델에는 사용하지 않는다. */
export function aircraftDisplayScale(depth: number, fov: number, zoom: number, viewportHeight: number, altitude: number) {
  const metresPerPixel = 2 * Math.max(.01, depth) * Math.tan(fov * Math.PI / 360) / Math.max(.01, zoom) / Math.max(1, viewportHeight);
  const readable = 72 * metresPerPixel / AIRCRAFT_DIAMETER;
  // 저고도에서 확대된 착륙 다리가 지면을 뚫지 않도록 제한한다.
  const groundClearance = Math.max(1, (altitude - .01) / .115);
  return Math.max(1, Math.min(8, readable, groundClearance));
}
