/** 제어기의 cm 좌표와 렌더러의 m 좌표를 연결하는 유일한 변환. */
export function worldPosition(xCm: number, yCm: number, heightCm: number): [number, number, number] {
  return [xCm / 100, heightCm / 100, -yCm / 100];
}

export const SENSOR_ASPECT = 4 / 3;
export const SENSOR_HORIZONTAL_FOV = 55;
export const SENSOR_VERTICAL_FOV = 2 * Math.atan(Math.tan(SENSOR_HORIZONTAL_FOV * Math.PI / 360) / SENSOR_ASPECT) * 180 / Math.PI;

export function sensorFootprint(heightCm: number): [number, number] {
  const width = 2 * heightCm / 100 * Math.tan(SENSOR_HORIZONTAL_FOV * Math.PI / 360);
  return [width, width / SENSOR_ASPECT];
}

export function sensorViewport(width: number, height: number) {
  const w = Math.min(width, height * SENSOR_ASPECT);
  const h = w / SENSOR_ASPECT;
  return { width: w, height: h, left: (width - w) / 2, top: (height - h) / 2 };
}
