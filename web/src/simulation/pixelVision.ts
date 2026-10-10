import {TAG_CAMERA,type TagDetection,type VisionPose} from './tagVision';
import type {MissionTag} from './missionWorld';

export interface DecodedTag {
  id:number;family:string;hamming:number;decisionMargin:number;
  center:{x:number;y:number};corners:{x:number;y:number}[];
}
export interface VisionCapture {session:number;tMs:number;pose:VisionPose}
export interface PixelFrame extends VisionCapture {sequence:number;detections:DecodedTag[];processingMs:number}
export type VisionStatus='loading'|'ready'|'error';
export const VISION_MAX_AGE_MS=700;

/** ID는 픽셀 디코더의 결과만 사용한다. 등록표에서는 물리 크기와 이름만 가져온다. */
export function measuredTags(frame:PixelFrame,tags:readonly MissionTag[]):TagDetection[] {
  return frame.detections.map(raw=>{
    const tag=tags.find(t=>t.code===raw.id),corners=raw.corners.map(p=>[p.x,p.y] as [number,number]);
    const pixels=corners.length===4?corners.reduce((sum,p,i)=>sum+Math.hypot(p[0]-corners[(i+1)%4][0],p[1]-corners[(i+1)%4][1]),0)/4:0;
    const finite=[raw.id,raw.hamming,raw.decisionMargin,raw.center.x,raw.center.y,...corners.flat()].every(Number.isFinite);
    const fits=corners.length===4&&corners.every(([x,y])=>x>=2&&x<=318&&y>=2&&y<=238);
    const reason=!finite?'잘못된 영상 결과':raw.family!=='tag36h11'?'태그 계열 불일치':!tag?'등록되지 않은 ID':raw.hamming>1?'비트 오류 초과':raw.decisionMargin<30?'디코딩 신뢰도 부족':!fits?'태그 일부 잘림':pixels<16?'태그 크기 부족':'영상 디코딩';
    const scale=tag?tag.size/Math.max(1,pixels):0;
    return {id:tag?.id??`미등록 ${raw.id}`,code:raw.id,corners,center:[raw.center.x,raw.center.y],pixels,
      right:(raw.center.x-TAG_CAMERA.width/2)*scale,forward:(TAG_CAMERA.height/2-raw.center.y)*scale,
      quality:Math.max(0,Math.min(1,raw.decisionMargin/100)),valid:reason==='영상 디코딩',reason,margin:raw.decisionMargin,hamming:raw.hamming};
  });
}

/** 지상 기준의 영상 측정점. 지도에 저장된 태그 좌표는 읽지 않는다. */
export function imageFix(frame:VisionCapture,detection:TagDetection){
  const {pose}=frame,sin=Math.sin(pose.yaw),cos=Math.cos(pose.yaw);
  return {x:pose.x+sin*TAG_CAMERA.offset+detection.right*cos+detection.forward*sin,
    y:pose.y+cos*TAG_CAMERA.offset-detection.right*sin+detection.forward*cos};
}

/** 한 프레임만 전송하는 backpressure. Worker에는 픽셀 외에 지도·예상 ID를 보내지 않는다. */
export class AprilTagCamera {
  private worker:Worker;private pending:(VisionCapture&{sequence:number})|null=null;
  private sequence=0;private disposed=false;private timer:ReturnType<typeof setTimeout>|undefined;
  status:VisionStatus='loading';
  constructor(private onFrame:(frame:PixelFrame)=>void,private onStatus:(status:VisionStatus,message:string)=>void){
    this.worker=new Worker(`${import.meta.env.BASE_URL}vendor/apriltag/detector-worker.js`);
    this.armTimeout(15000);
    this.worker.onerror=()=>this.fail('AprilTag 영상 디코더를 불러오지 못했습니다.');
    this.worker.onmessage=({data})=>{
      if(this.disposed)return;
      if(data.type==='initialized')this.worker.postMessage({type:'setup',family:'tag36h11',hammingDist:1,params:{quadDecimate:1,quadSigma:0,refineEdges:true,decodeSharpening:.25,debug:false}});
      else if(data.type==='setup-complete'){clearTimeout(this.timer);this.status='ready';this.onStatus('ready','');}
      else if(data.type==='detection-result'){
        clearTimeout(this.timer);const pending=this.pending;this.pending=null;
        if(pending)this.onFrame({...pending,detections:data.detections,processingMs:data.processingTime});
      }else if(data.type==='error')this.fail('AprilTag 영상 디코딩 오류 · 카메라를 다시 연결해 주세요.');
    };
  }
  capture(pixels:Uint8ClampedArray,meta:VisionCapture){
    if(this.disposed||this.status!=='ready'||this.pending)return;
    this.pending={...meta,pose:{...meta.pose},sequence:++this.sequence};
    const buffer=pixels.slice().buffer;
    this.armTimeout(4000);
    this.worker.postMessage({type:'detect',imageData:buffer,width:320,height:240},[buffer]);
  }
  private armTimeout(ms:number){clearTimeout(this.timer);this.timer=setTimeout(()=>this.fail('카메라 영상 디코더의 응답이 지연됩니다. 다시 연결해 주세요.'),ms);}
  private fail(message:string){if(this.disposed)return;this.status='error';this.pending=null;clearTimeout(this.timer);this.worker.terminate();this.onStatus('error',message);}
  dispose(){this.disposed=true;clearTimeout(this.timer);this.pending=null;this.worker.terminate();}
}
