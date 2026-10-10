import type {WifiSnapshot} from './liveTelemetry';

const FRESH_MS = 2200;
const HISTORY_MS = 60_000;
const pulse = (value: unknown): number | null => typeof value === 'number'
  && Number.isFinite(value) && value >= 750 && value <= 2250 ? value : null;

/** 1000–2000µs를 표시용 눈금으로 환산한다. 추력이나 RPM의 백분율이 아니다. */
export const pulsePercent = (value: number | null) => value === null ? null : Math.min(100, Math.max(0, (value - 1000) / 10));

export interface SignalPoint {at: number; value: number}
export interface ThrottleReading {
  inputUs: number | null;
  inputPercent: number | null;
  inputAge: number | null;
  motorMeanUs: number | null;
  motorPercent: number | null;
  boardControl: boolean | null;
  inputHistory: SignalPoint[];
  motorHistory: SignalPoint[];
  now: number;
}

/** 새 FC/RC 보고만 기록한다. 같은 SSE 표본을 다시 그려도 그래프 표본을 만들지 않는다. */
export class ThrottleMonitor {
  private session = -1;
  private inputSequence = -1;
  private motorSequence = -1;
  private inputs: SignalPoint[] = [];
  private motors: SignalPoint[] = [];

  update(snapshot: WifiSnapshot | null, transportAge = 0): ThrottleReading {
    if (snapshot && snapshot.session !== this.session) {
      this.session = snapshot.session;
      this.inputSequence = this.motorSequence = -1;
      this.inputs = []; this.motors = [];
    }
    const now = (snapshot?.server_ms ?? 0) + transportAge;
    const online = snapshot?.state === 'connected' && snapshot.received_age_ms !== null
      && snapshot.received_age_ms + transportAge < 3000;
    const mspSample = snapshot?.latest.msp, fcSample = snapshot?.latest.fc;
    const msp = online && mspSample && mspSample.age_ms + transportAge <= FRESH_MS ? mspSample.data : undefined;
    const fc = online && fcSample && fcSample.age_ms + transportAge <= FRESH_MS ? fcSample.data : undefined;
    // fc_rc는 FC 정규 순서 R/P/Y/T. out은 AETR 등 송신 순서이므로 스로틀 계측으로 사용하지 않는다.
    const rcThrottle = Array.isArray(msp?.fc_rc) ? pulse(msp.fc_rc[3]) : null;
    const inputUs = msp?.fc_link === true && msp.authority_fresh === true
      ? pulse(msp.rc_throttle) ?? rcThrottle : null;
    const motorValues = fc?.motor_ok === true && Array.isArray(fc.motor) && fc.motor.length === 4
      && fc.motor.every(v => pulse(v) !== null) ? fc.motor as number[] : null;
    const motorMeanUs = motorValues ? motorValues.reduce((sum, n) => sum + n, 0) / 4 : null;

    if (inputUs !== null && mspSample && this.inputSequence !== mspSample.sequence) {
      this.inputs.push({at: mspSample.at_ms, value: inputUs});
      this.inputSequence = mspSample.sequence;
    }
    if (motorMeanUs !== null && fcSample && this.motorSequence !== fcSample.sequence) {
      this.motors.push({at: fcSample.at_ms, value: motorMeanUs});
      this.motorSequence = fcSample.sequence;
    }
    this.inputs = this.inputs.filter(p => p.at >= now - HISTORY_MS).slice(-600);
    this.motors = this.motors.filter(p => p.at >= now - HISTORY_MS).slice(-600);
    return {
      inputUs, inputPercent: pulsePercent(inputUs), inputAge: inputUs !== null && mspSample ? mspSample.age_ms + transportAge : null,
      motorMeanUs, motorPercent: pulsePercent(motorMeanUs),
      boardControl: typeof msp?.throttle_ctrl === 'boolean' ? msp.throttle_ctrl : null,
      inputHistory: this.inputs.slice(), motorHistory: this.motors.slice(), now,
    };
  }
}

/** 수신 공백은 이어 그리지 않는다. 기록이 없거나 끊긴 구간을 0% 출력으로 오해하지 않게 한다. */
export function signalPath(points: SignalPoint[], now: number, windowMs: number): string {
  let previous = -Infinity;
  return points.filter(p => p.at >= now - windowMs && p.at <= now).map(p => {
    const x = 8 + (p.at - now + windowMs) / windowMs * 264;
    const y = 76 - pulsePercent(p.value)! * .64;
    const command = p.at - previous > FRESH_MS ? 'M' : 'L';
    previous = p.at;
    return command + x.toFixed(1) + ',' + y.toFixed(1);
  }).join(' ');
}
