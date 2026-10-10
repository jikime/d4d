import {NEUTRAL_INPUT, type FlightSnapshot} from './simulation/pilotFlight';

export interface BoardSample {
  data: Record<string, unknown>;
  sequence: number;
  at_ms: number;
  age_ms: number;
}
export interface WifiSnapshot {
  state: 'disconnected' | 'connecting' | 'connected' | 'reconnecting';
  host: string;
  port: number;
  session: number;
  server_ms: number;
  read_only: boolean;
  source?: 'direct_wifi' | 'camera_viewer';
  error: string;
  latest: Record<string, BoardSample>;
  frame: {sequence: number; width: number; height: number; age_ms: number | null; transfer_ms: number | null};
  received_age_ms: number | null;
  parse_errors: number;
  build: string;
}
export interface LiveMotion {
  flight: FlightSnapshot;
  online: boolean;
  roll: number | null;
  pitch: number | null;
  heading: number | null;
  height: number | null;
  rightSpeed: number | null;
  forwardSpeed: number | null;
  climbSpeed: number | null;
  flowQuality: number | null;
  positionValid: boolean;
  positionNote: string;
  armed: boolean | null;
  voltage: number | null;
  amps: number | null;
  motors: number[] | null;
  motorPower: number;
  rc: number[] | null;
  authority: string;
  fcLinked: boolean;
}
const number = (value: unknown, min = -Infinity, max = Infinity): number | null =>
  typeof value === 'number' && Number.isFinite(value) && value >= min && value <= max ? value : null;
const pair = (value: unknown): [number, number] | null => Array.isArray(value) && value.length >= 2
  && number(value[0]) !== null && number(value[1]) !== null ? [value[0], value[1]] : null;
const radians = (degrees: number) => degrees * Math.PI / 180;

function initialFlight(): FlightSnapshot {
  return {x:0, y:0, height:0, vx:0, vy:0, vz:0, yaw:0, pitch:0, roll:0, tMs:0, travelled:0,
    mode:'manual', waypoint:0, completed:false, boundary:false, input:{...NEUTRAL_INPUT}, trail:[],
    route:{name:'실기체 상대 이동', source:'training', origin:{x:0,y:0}, points:[{x:0,y:0,height:0}]}};
}

/** 실제 계측만 사용한다. 위치는 유효한 속도 표본 사이에서만 적분하며 끊긴 시간은 추측하지 않는다. */
export class LiveMotionTracker {
  flight = initialFlight();
  private session = -1;
  private velocitySample: {at:number; seq:number; vx:number; vy:number} | null = null;
  private originAt = 0;
  private trailAt = 0;
  private zeroAfter = -Infinity;

  zero() {
    this.zeroAfter = this.velocitySample?.at ?? this.zeroAfter;
    this.flight = {...this.flight, x:0, y:0, travelled:0, trail:[]};
    this.velocitySample = null;
  }

  update(snapshot: WifiSnapshot | null, transportAge = 0): LiveMotion {
    if (snapshot && snapshot.session !== this.session) {
      this.session = snapshot.session;
      this.flight = initialFlight();
      this.velocitySample = null;
      this.originAt = snapshot.server_ms;
      this.trailAt = 0;
      this.zeroAfter = -Infinity;
    }
    const online = snapshot?.state === 'connected' && snapshot.received_age_ms !== null
      && snapshot.received_age_ms + transportAge < 3000;
    const sample = (kind:string, age=1200): BoardSample | undefined => {
      const s = snapshot?.latest[kind];
      return online && s && s.age_ms + transportAge <= age ? s : undefined;
    };
    const attitude = sample('attitude', 1600), approach = sample('approach'), flow = sample('flow');
    const attitudeRP = sample('attitude_rp');
    const info = sample('info', 15_000)?.data ?? sample('boot', 15_000)?.data;
    const fc = sample('fc', 2200)?.data, msp = sample('msp', 2200)?.data;
    const measured = info?.sim === false || (info===undefined && snapshot?.source==='camera_viewer' && approach?.data.h_src==='rangefinder');
    const ap = approach?.data;
    const apAtt = pair(ap?.att);
    let roll = number(attitude?.data.roll_deg, -180, 180), pitch = number(attitude?.data.pitch_deg, -180, 180);
    if (attitudeRP && (!attitude || attitudeRP.at_ms > attitude.at_ms)) {
      roll=number(attitudeRP.data.roll_deg,-180,180);pitch=number(attitudeRP.data.pitch_deg,-180,180);
    }
    if (apAtt && (!attitude || approach!.at_ms > attitude.at_ms)) {
      roll = number(apAtt[0], -180, 180); pitch = number(apAtt[1], -180, 180);
    }
    const heading = number(attitude?.data.yaw_deg, -360, 360);
    const heightCm = measured && ap?.height_valid === true && ap.h_src === 'rangefinder'
      ? number(ap.height_cm, 0, 2000)
      : measured && flow?.data.connected === true && flow.data.sim === false
        ? number(flow.data.height_cm, 0, 2000) : null;
    const height = heightCm === null ? null : heightCm / 100;
    const flowQuality = number(ap?.flow_q, 0, 255) ?? number(flow?.data.quality, 0, 255);
    let velocity: [number,number] | null = null;
    let velocityRecord = approach;
    if (measured && ap?.v_flow_valid === true && ap.vel_src === 'flow') velocity = pair(ap.v_flow);
    if (!velocity && measured && flow?.data.connected === true && flow.data.sim === false) {
      velocity = pair([flow.data.right_cm_s, flow.data.forward_cm_s]);
      velocityRecord = flow;
    }
    const reliable = velocity !== null && velocity.every(v=>Math.abs(v)<=1000) && flowQuality !== null && flowQuality>=40;
    const positionValid = reliable && heading !== null && height !== null && height >= .2;
    const rightSpeed = reliable ? velocity![0]/100 : null, forwardSpeed = reliable ? velocity![1]/100 : null;
    const vz = measured && ap?.height_valid === true ? number(ap.vz_cm_s, -1000, 1000) : null;
    const climbSpeed = vz === null ? null : vz/100;
    const positionNote = !online ? '수신 대기 · 마지막 위치 유지'
      : !measured ? '보드에 시험용 센서값이 설정되어 있습니다'
      : height === null ? '실제 높이 수신 대기'
      : height < .2 ? '지면 근처 · 자세와 높이 추적 중'
      : !reliable ? '광류 품질 부족 · 수평 위치 유지'
      : heading === null ? '방위각 수신 대기 · 수평 위치 유지'
      : '광류로 추정한 상대 위치 · 절대 좌표 아님';

    const s = this.flight;
    s.tMs = snapshot ? Math.max(0, snapshot.server_ms - this.originAt) : 0;
    // MSP: 오른쪽 기울기 +roll, 기수 올림 +pitch, 시계 방향 +yaw.
    // Three.js 기체는 -Z를 보므로 roll과 yaw의 회전 부호가 반대다.
    if (roll !== null) s.roll = -radians(roll);
    if (pitch !== null) s.pitch = radians(pitch);
    if (heading !== null) s.yaw = radians(heading);
    if (height !== null) s.height = height;
    s.vz = climbSpeed ?? 0;
    s.vx = s.vy = 0;
    if (positionValid && velocityRecord) {
      const yaw = radians(heading!), r=rightSpeed!, f=forwardSpeed!;
      const vx=r*Math.cos(yaw)+f*Math.sin(yaw), vy=-r*Math.sin(yaw)+f*Math.cos(yaw);
      s.vx=vx; s.vy=vy;
      if (this.velocitySample?.seq !== velocityRecord.sequence && velocityRecord.at_ms > this.zeroAfter) {
        const previous = this.velocitySample;
        const dt = previous ? (velocityRecord.at_ms-previous.at)/1000 : 0;
        if (previous && dt>0 && dt<=.75) {
          const dx=(previous.vx+vx)*.5*dt, dy=(previous.vy+vy)*.5*dt;
          s.x+=dx; s.y+=dy; s.travelled+=Math.hypot(dx,dy);
        }
        this.velocitySample={at:velocityRecord.at_ms,seq:velocityRecord.sequence,vx,vy};
      }
    } else this.velocitySample = null;
    if (online && height !== null && s.tMs-this.trailAt>100) {
      const last=s.trail.at(-1);
      if (!last || Math.hypot(s.x-last.x,s.y-last.y,s.height-last.height)>.025) {
        s.trail=[...s.trail.slice(-599),{x:s.x,y:s.y,height:s.height}];
      }
      this.trailAt=s.tMs;
    }
    const motorValues = Array.isArray(fc?.motor) && fc.motor.length===4 && fc.motor.every(v=>number(v,750,2250)!==null)
      && fc.motor_ok === true ? fc.motor as number[] : null;
    const motorPower = motorValues ? Math.min(1,Math.max(0,...motorValues.map(v=>(v-1000)/1000))) : 0;
    const armed = fc?.status === true && typeof fc.armed==='boolean' ? fc.armed : null;
    return {flight:{...s,trail:s.trail.slice()}, online, roll, pitch, heading, height, rightSpeed, forwardSpeed,
      climbSpeed, flowQuality, positionValid, positionNote, armed,
      voltage:fc?.battery===true?number(fc.vbat,0,100):null, amps:fc?.battery===true?number(fc.amps,0,1000):null,
      motors:motorValues,motorPower,rc:Array.isArray(msp?.fc_rc)&&msp.fc_rc.length>=4&&msp.fc_rc.every(v=>number(v,750,2250)!==null)?msp.fc_rc as number[]:null,
      authority:msp?.authority_fresh===true?(msp.authority==='manual'?'조종기 수동':msp.authority==='auto'?'보드 자동':'미확인'):'미확인',
      fcLinked:msp?.fc_link===true};
  }
}
