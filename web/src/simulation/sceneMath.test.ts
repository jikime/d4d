import { describe, expect, it } from "vitest";
import { SENSOR_ASPECT, SENSOR_VERTICAL_FOV, sensorFootprint, sensorViewport, worldPosition } from "./sceneMath";

describe("시뮬레이션과 3D 장면의 좌표 계약", () => {
  it("오른쪽·앞·고도를 m 단위의 x·-z·y로 보존한다", () => {
    expect(worldPosition(55, -20, 120)).toEqual([0.55, 1.2, 0.2]);
    expect(worldPosition(-30, 45, 80)).toEqual([-0.3, 0.8, -0.45]);
  });
  it("가로 화각 55도와 4:3 센서 비율을 함께 보존한다", () => {
    const [w, h] = sensorFootprint(120);
    expect(w / h).toBeCloseTo(SENSOR_ASPECT);
    expect(h).toBeCloseTo(2 * 1.2 * Math.tan(SENSOR_VERTICAL_FOV * Math.PI / 360));
    expect(sensorFootprint(240)[0]).toBeCloseTo(w * 2);
  });
  it.each([[1920, 1080], [390, 844], [1024, 768]])("%i × %i 화면에서도 영상이 늘어나거나 잘리지 않는다", (width, height) => {
    const rect = sensorViewport(width, height);
    expect(rect.width / rect.height).toBeCloseTo(4 / 3);
    expect(rect.width).toBeLessThanOrEqual(width);
    expect(rect.height).toBeLessThanOrEqual(height);
    expect(rect.left * 2 + rect.width).toBeCloseTo(width);
    expect(rect.top * 2 + rect.height).toBeCloseTo(height);
  });
});
