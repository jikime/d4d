import {inZone,type MissionTag,type MissionWorld} from './missionWorld';
export interface VisionPose {x:number;y:number;height:number;yaw:number;vx:number;vy:number}
export interface TagDetection {id:string;code:number;corners:[number,number][];center:[number,number];pixels:number;right:number;forward:number;quality:number;valid:boolean;reason:string;margin?:number;hamming?:number}
// 표지 판은 바닥 장식 선(상단 0.056m) 위에 놓여야 코드가 가려지지 않는다.
export const TAG_CAMERA={width:320,height:240,fx:160/Math.tan(55*Math.PI/360),offset:.026,heightOffset:.115,tagHeight:.061};

/** 배치 검증·단위 시험용 시야 예측. 실제 자동 비행과 카메라 오버레이에서는 사용하지 않는다. */
export function observeTag(pose:VisionPose,tag:MissionTag,world:MissionWorld,hidden=false,dusk=false):TagDetection {
  const sin=Math.sin(pose.yaw),cos=Math.cos(pose.yaw),x=pose.x+sin*TAG_CAMERA.offset,y=pose.y+cos*TAG_CAMERA.offset,h=pose.height-TAG_CAMERA.heightOffset-TAG_CAMERA.tagHeight;
  const dx=tag.x-x,dy=tag.y-y,right=dx*cos-dy*sin,forward=dx*sin+dy*cos;
  const project=(a:number,b:number):[number,number]=>[160+(a*cos-b*sin)*TAG_CAMERA.fx/Math.max(.01,h),120-(a*sin+b*cos)*TAG_CAMERA.fx/Math.max(.01,h)];
  const half=tag.size/2,corners=([[-half,-half],[half,-half],[half,half],[-half,half]] as [number,number][]).map(([a,b])=>project(dx+a,dy+b));
  const pixels=tag.size*TAG_CAMERA.fx/Math.max(.01,h),fits=h>.15&&corners.every(([u,v])=>u>=4&&u<=316&&v>=4&&v<=236);
  let occluded=hidden;
  for(let i=1;i<20&&!occluded;i++){const t=i/20,p={x:x+dx*t,y:y+dy*t};if(h*(1-t)<3.8&&world.zones.some(z=>z.kind==='obstacle'&&inZone(p,z)))occluded=true;}
  const motion=Math.hypot(pose.vx,pose.vy),quality=Math.max(0,Math.min(1,(pixels-10)/32))*(dusk?.82:1)*Math.max(.2,1-motion*.18);
  const reason=occluded?'태그 가림':!fits?'시야 밖':pixels<14?'태그 크기 부족':motion>2?'이동 흐림':quality<.28?'검출 품질 부족':'검출';
  return {id:tag.id,code:tag.code,corners,center:project(dx,dy),pixels,right,forward,quality,valid:reason==='검출',reason};
}
