import {useEffect,useRef,useState} from 'react';
import {useFrame,useThree} from '@react-three/fiber';
import * as THREE from 'three';

/** Canvas 밖의 별도 React 루트를 만들지 않는 3D 라벨. 화면 전환·경유점 교체 시 함께 정리된다. */
export default function SceneLabel({text,subtitle,position}:{text:string;subtitle?:string;position:[number,number,number]}) {
  const sprite=useRef<THREE.Sprite>(null),[texture,setTexture]=useState<THREE.CanvasTexture|null>(null);
  const {camera,size}=useThree();
  useEffect(()=>{
    const canvas=document.createElement('canvas');canvas.width=subtitle?396:198;canvas.height=subtitle?102:66;
    const ctx=canvas.getContext('2d')!;ctx.fillStyle='#182013eb';ctx.fillRect(0,0,canvas.width,canvas.height);ctx.strokeStyle='#abbc7d';ctx.lineWidth=2;ctx.strokeRect(1,1,canvas.width-2,canvas.height-2);
    ctx.fillStyle='#e4edc9';ctx.textAlign='center';ctx.textBaseline='middle';ctx.font='600 29px system-ui';ctx.fillText(text,canvas.width/2,subtitle?34:33);
    if(subtitle){ctx.fillStyle='#adc193';ctx.font='23px system-ui';ctx.fillText(subtitle,canvas.width/2,75);}
    const map=new THREE.CanvasTexture(canvas);map.colorSpace=THREE.SRGBColorSpace;map.minFilter=THREE.LinearFilter;setTexture(map);return()=>map.dispose();
  },[text,subtitle]);
  useFrame(()=>{
    if(!sprite.current)return;const perspective=camera as THREE.PerspectiveCamera;
    const pixel=2*Math.tan(perspective.fov*Math.PI/360)/perspective.zoom/Math.max(1,size.height);
    sprite.current.scale.set((subtitle?132:66)*pixel,(subtitle?34:22)*pixel,1);
  });
  return texture?<sprite ref={sprite} name="scene-label" position={position} renderOrder={100}><spriteMaterial map={texture} transparent sizeAttenuation={false} depthTest={false} depthWrite={false}/></sprite>:null;
}
