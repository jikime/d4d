import {useEffect,useRef,useState} from 'react';
import {useFrame,useThree} from '@react-three/fiber';
import * as THREE from 'three';
import {t, useLanguage} from '../i18n';

/** Canvas 밖의 별도 React 루트를 만들지 않는 3D 라벨. 화면 전환·경유점 교체 시 함께 정리된다. */
export default function SceneLabel({text,subtitle,position,width,screenOffset=[0,0]}:{text:string;subtitle?:string;position:[number,number,number];width?:number;screenOffset?:[number,number]}) {
  const language = useLanguage();
  const sprite=useRef<THREE.Sprite>(null),[texture,setTexture]=useState<THREE.CanvasTexture|null>(null);
  const dimensions=useRef({width:66,height:22});
  const {camera,size}=useThree();
  useEffect(()=>{
    const canvas=document.createElement('canvas');let map:THREE.CanvasTexture|undefined;
    const paint=()=>{
    const ctx=canvas.getContext('2d')!;
    ctx.font='600 29px Pretendard Variable';const titleWidth=ctx.measureText(t(text)).width;
    ctx.font='23px Pretendard Variable';const subtitleWidth=subtitle?ctx.measureText(t(subtitle)).width:0;
    // 고정 폭 대신 두 줄의 실제 글자 폭에 좌우 5px의 화면 여백만 더한다.
    canvas.width=Math.max(60,Math.min((width??300)*3,Math.ceil(Math.max(titleWidth,subtitleWidth)+30)));
    canvas.height=subtitle?102:66;dimensions.current={width:canvas.width/3,height:canvas.height/3};
    ctx.fillStyle='#182013eb';ctx.fillRect(0,0,canvas.width,canvas.height);ctx.strokeStyle='#abbc7d';ctx.lineWidth=2;ctx.strokeRect(1,1,canvas.width-2,canvas.height-2);
    ctx.fillStyle='#e4edc9';ctx.textAlign='center';ctx.textBaseline='middle';ctx.font='600 29px Pretendard Variable';ctx.fillText(t(text),canvas.width/2,subtitle?34:33,canvas.width-16);
    if(subtitle){ctx.fillStyle='#adc193';ctx.font='23px Pretendard Variable';ctx.fillText(t(subtitle),canvas.width/2,75,canvas.width-16);}
    const previous=map;map=new THREE.CanvasTexture(canvas);map.colorSpace=THREE.SRGBColorSpace;map.minFilter=THREE.LinearFilter;setTexture(map);previous?.dispose();
    };
    paint();let disposed=false;void document.fonts.ready.then(()=>{if(!disposed)paint();});
    return()=>{disposed=true;map?.dispose();};
  },[text,subtitle,language,width]);
  useFrame(()=>{
    if(!sprite.current)return;const perspective=camera as THREE.PerspectiveCamera;
    const pixel=2*Math.tan(perspective.fov*Math.PI/360)/perspective.zoom/Math.max(1,size.height);
    const {width:w,height:h}=dimensions.current;
    sprite.current.scale.set(w*pixel,h*pixel,1);
    sprite.current.center.set(.5-screenOffset[0]/w,.5+screenOffset[1]/h);
  });
  return texture?<sprite ref={sprite} name="scene-label" position={position} renderOrder={100}><spriteMaterial map={texture} transparent sizeAttenuation={false} depthTest={false} depthWrite={false}/></sprite>:null;
}
