import type {MissionSnapshot} from './missionFlight';
import type {PilotInput} from './pilotFlight';
import {VISION_MAX_AGE_MS} from './pixelVision';

// 배속 시간이 GPU/Worker의 영상 시각을 앞질러 매 프레임 인식을 취소하지 않도록 한다.
// 실제 기체 제어기가 아니라 브라우저 시뮬레이션의 재생 시계만 제한한다.
export function missionPlaybackBudget(state: Pick<MissionSnapshot, 'autonomous' | 'vision' | 'tMs'>, input: PilotInput): number {
  if (!state.autonomous || input.right || input.forward || input.climb || input.yaw || input.brake || state.vision.status !== 'ready') return Infinity;
  if (!Number.isFinite(state.vision.frameMs)) return 0;
  return Math.max(0, (VISION_MAX_AGE_MS - 1 - (state.tMs - state.vision.frameMs)) / 1000);
}
