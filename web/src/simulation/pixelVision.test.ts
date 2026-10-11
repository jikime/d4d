import {beforeAll,describe,expect,it} from 'vitest';
import {readFileSync} from 'node:fs';
import {createContext,runInContext} from 'node:vm';
import {TAG_BITS,TAG_CODES} from './tag36h11';
import {imageFix,measuredTags,type DecodedTag,type PixelFrame} from './pixelVision';
import type {MissionTag} from './missionWorld';

let decoder:{detect:(pixels:Uint8Array,w:number,h:number)=>DecodedTag[]};
beforeAll(async()=>{
  // 제품과 동일한 C/WASM 바이너리를 격리된 브라우저 환경에서 실행한다.
  const source=(name:string)=>readFileSync(new URL(`../../public/vendor/apriltag/${name}`,import.meta.url),'utf8');
  const context=createContext({console,performance,TextDecoder,TextEncoder,WebAssembly,Uint8Array,Uint8ClampedArray,
    setTimeout,clearTimeout,window:{},document:{currentScript:{src:'https://test.invalid/apriltag_wasm.js'}},
    fetch:async()=>new Response(readFileSync(new URL('../../public/vendor/apriltag/apriltag_wasm.wasm',import.meta.url)),{headers:{'Content-Type':'application/wasm'}})});
  for(const file of ['tag-families.js','apriltag_wasm.js','apriltag-wasm-wrapper.js'])runInContext(source(file),context);
  decoder=await runInContext(`(async()=>{const d=new AprilTagDetector();await d.initialize();d.setupDetector('tag36h11',1);d.setParameters({quadDecimate:1,quadSigma:0,refineEdges:true,decodeSharpening:.25});return d;})()`,context);
});

function pixels(id:number,angle=0,blackSize=72,covered=false){
  const out=new Uint8Array(320*240).fill(146),code=TAG_CODES[id],cells=Array.from({length:10},()=>new Array<number>(10).fill(255));
  for(let y=1;y<9;y++)for(let x=1;x<9;x++)cells[y][x]=0;
  TAG_BITS.forEach(([x,y],i)=>{if(Math.floor(code/2**(35-i))%2)cells[y+1][x+1]=255;});
  const side=blackSize*1.25,sin=Math.sin(angle),cos=Math.cos(angle);
  for(let y=0;y<240;y++)for(let x=0;x<320;x++){
    const dx=x-160,dy=y-120,u=Math.floor((dx*cos+dy*sin)/side*10+5),v=Math.floor((-dx*sin+dy*cos)/side*10+5);
    if(u>=0&&u<10&&v>=0&&v<10)out[y*320+x]=covered?80:cells[v][u];
  }
  return out;
}
const tag:MissionTag={id:'checkpoint',code:1,size:.4,x:999,y:999,name:'위치와 무관한 등록 정보',role:'marker'};
function frame(detections:DecodedTag[]):PixelFrame{return {session:1,sequence:1,tMs:100,pose:{x:2,y:3,height:2.2,yaw:0,vx:0,vy:0},processingMs:1,detections};}

describe('실제 AprilTag WASM 픽셀 디코딩',()=>{
  it.each([0,Math.PI/2,Math.PI,Math.PI*1.5,.39])('회전 %s 라디안의 비트 패턴에서 ID 1을 읽는다',angle=>{
    const result=decoder.detect(pixels(1,angle),320,240);
    expect(result.map(d=>d.id)).toEqual([1]);expect(result[0].family).toBe('tag36h11');expect(result[0].hamming).toBe(0);expect(result[0].decisionMargin).toBeGreaterThan(30);
  });
  it('작은 26px 태그에서도 ID를 읽고, 다른 비트 패턴은 다른 ID로 읽는다',()=>{
    expect(decoder.detect(pixels(24,0,26),320,240).map(d=>d.id)).toEqual([24]);
    expect(decoder.detect(pixels(586,.2),320,240).map(d=>d.id)).toEqual([586]);
  });
  it.each([0,Math.PI/2,Math.PI,Math.PI*1.5,.39])('TAG 119를 회전 %s에서도 실제 코드로 읽는다',angle=>{
    expect(decoder.detect(pixels(119,angle),320,240).map(d=>d.id)).toEqual([119]);
  });
  it('태그가 없는 프레임과 가려진 프레임에서는 ID를 만들어 내지 않는다',()=>{
    expect(decoder.detect(new Uint8Array(320*240).fill(160),320,240)).toEqual([]);
    expect(decoder.detect(pixels(1,0,72,true),320,240)).toEqual([]);
  });
  it('미등록 코드와 낮은 신뢰도는 비행 제어에 사용하지 않는다',()=>{
    const decoded=decoder.detect(pixels(1),320,240),unknown=decoder.detect(pixels(586),320,240);
    expect(measuredTags(frame(decoded),[tag])[0].valid).toBe(true);
    expect(measuredTags(frame(unknown),[tag])[0].reason).toBe('등록되지 않은 ID');
    expect(measuredTags(frame([{...decoded[0],decisionMargin:10}]),[tag])[0].valid).toBe(false);
    expect(measuredTags(frame([{...decoded[0],hamming:2}]),[tag])[0].valid).toBe(false);
  });
  it('지도 좌표가 달라도 픽셀 중심·태그 크기·촬영 자세에서 같은 이동 오차를 얻는다',()=>{
    const f=frame(decoder.detect(pixels(1),320,240)),a=measuredTags(f,[tag])[0],b=measuredTags(f,[{...tag,x:-500,y:800}])[0];
    expect(a).toEqual(b);const fix=imageFix(f,a);expect(fix.x).toBeCloseTo(2,2);expect(fix.y).toBeCloseTo(3.026,2);
  });
});
