import {useEffect,useMemo,type RefObject} from 'react';
import {useFrame} from '@react-three/fiber';
import * as THREE from 'three';
import type {SimulationSceneProps} from '../SimulationScene';
import {seededRandom} from './flightVisuals';
import {weatherWind} from './weather';

const vertex=`
  attribute float aPhase;
  uniform float uTime,uRain,uIntensity,uSize,uViewportHeight,uPower;
  uniform vec3 uBounds,uWind,uAircraft;
  varying float vAlpha;varying vec2 vStreak;
  void main(){
    float fall=mix(1.15,14.0,uRain)*(0.8+aPhase*0.4);
    vec3 p=position;
    p.y=mod(position.y-uTime*fall,uBounds.y);
    p.x=mod(p.x+uWind.x*uTime,uBounds.x)-uBounds.x*0.5;
    p.z=mod(p.z+uWind.z*uTime,uBounds.z)-uBounds.z*0.5;
    p.x+=sin(uTime*1.4+aPhase*80.0)*(1.0-uRain)*0.45;
    p.z+=cos(uTime+aPhase*33.0)*(1.0-uRain)*0.35;
    vec2 away=p.xz-uAircraft.xz;float distanceToRotor=length(away);
    float wash=(1.0-uRain)*uPower*max(0.0,1.0-uAircraft.y/4.0)*exp(-distanceToRotor*0.8)*max(0.0,1.0-p.y/3.0);
    p.xz+=normalize(away+vec2(0.001))*wash*(1.0+sin(uTime*3.0+aPhase*40.0))*1.8;
    p.y+=wash*abs(sin(uTime*2.0+aPhase*22.0));
    vec4 mv=modelViewMatrix*vec4(p,1.0);gl_Position=projectionMatrix*mv;
    gl_PointSize=clamp(uSize*projectionMatrix[1][1]*uViewportHeight*0.5/max(0.5,-mv.z),uRain>0.5?1.5:1.0,32.0);
    vec3 motion=mat3(viewMatrix)*vec3(uWind.x,-fall,uWind.z);vStreak=normalize(vec2(motion.x,-motion.y)+vec2(0.0,0.001));
    vAlpha=(0.35+aPhase*0.5)*smoothstep(0.0,0.2,p.y);
  }`;
const fragment=`
  uniform float uRain; varying float vAlpha;varying vec2 vStreak;
  void main(){vec2 p=gl_PointCoord-0.5;
    float rain=(1.0-smoothstep(0.018,0.07,abs(dot(p,vec2(-vStreak.y,vStreak.x)))))*(1.0-smoothstep(0.3,0.5,abs(dot(p,vStreak))));
    float snow=1.0-smoothstep(0.15,0.5,length(p));float alpha=mix(snow,rain,uRain)*vAlpha;
    if(alpha<0.02)discard;gl_FragColor=vec4(mix(vec3(0.94,0.97,1.0),vec3(0.65,0.79,0.87),uRain),alpha);
    #include <tonemapping_fragment>
    #include <colorspace_fragment>
  }`;

/** 입자와 빗방울 파문은 각각 한 번의 GPU 그리기로 처리한다. 카메라별로 복제하지 않는다. */
export default function WeatherScene({source}:{source:RefObject<SimulationSceneProps>}){
  const world=source.current.mission;
  const width=(world?.width??40)+16,depth=(world?.depth??32)+16;
  const seed=source.current.missionState?.weather.seed??41011;
  const data=useMemo(()=>{
    const random=seededRandom(seed),count=20000,positions=new Float32Array(count*3),phases=new Float32Array(count);
    for(let i=0;i<count;i++){positions.set([random()*width,random()*18,random()*depth],i*3);phases[i]=random();}
    const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(positions,3));geometry.setAttribute('aPhase',new THREE.BufferAttribute(phases,1));
    const uniforms={uTime:{value:0},uRain:{value:0},uIntensity:{value:0},uSize:{value:.12},uViewportHeight:{value:1000},uPower:{value:0},uBounds:{value:new THREE.Vector3(width,18,depth)},uWind:{value:new THREE.Vector3()},uAircraft:{value:new THREE.Vector3()}};
    const material=new THREE.ShaderMaterial({uniforms,vertexShader:vertex,fragmentShader:fragment,transparent:true,depthWrite:false});
    const points=new THREE.Points(geometry,material);points.frustumCulled=false;points.name='training-precipitation';points.renderOrder=4;
    const viewport=new THREE.Vector4();points.onBeforeRender=renderer=>{uniforms.uViewportHeight.value=renderer.getCurrentViewport(viewport).w;};
    const plane=new THREE.PlaneGeometry(1,1),rippleGeo=new THREE.InstancedBufferGeometry();
    rippleGeo.index=plane.index;rippleGeo.attributes=plane.attributes;
    const offsets=new Float32Array(450*3),cycles=new Float32Array(450);
    for(let i=0;i<450;i++){offsets.set([(random()-.5)*(world?.width??36),.064,(random()-.5)*(world?.depth??28)],i*3);cycles[i]=random();}
    rippleGeo.setAttribute('aOffset',new THREE.InstancedBufferAttribute(offsets,3));rippleGeo.setAttribute('aCycle',new THREE.InstancedBufferAttribute(cycles,1));rippleGeo.instanceCount=450;
    const rippleMaterial=new THREE.ShaderMaterial({uniforms,transparent:true,depthWrite:false,side:THREE.DoubleSide,
      vertexShader:`attribute vec3 aOffset;attribute float aCycle;uniform float uTime,uIntensity;varying vec2 vUv;varying float vFade;
      void main(){vUv=uv;float age=fract(uTime*1.8+aCycle);vFade=(1.0-age)*uIntensity*.35;vec3 p=aOffset+vec3(position.x,0.0,position.y)*(0.06+age*.75);gl_Position=projectionMatrix*modelViewMatrix*vec4(p,1.0);}`,
      fragmentShader:`varying vec2 vUv;varying float vFade;void main(){float ring=1.0-smoothstep(0.012,0.04,abs(length(vUv-.5)-.42));gl_FragColor=vec4(.7,.82,.86,ring*vFade);}`});
    const ripples=new THREE.Mesh(rippleGeo,rippleMaterial);ripples.frustumCulled=false;ripples.renderOrder=3;ripples.name='rain-ground-ripples';
    return {points,ripples,uniforms,geometry,material,rippleGeo,rippleMaterial};
  },[width,depth,world?.width,world?.depth,seed]);
  useEffect(()=>()=>{data.geometry.dispose();data.material.dispose();data.rippleGeo.dispose();data.rippleMaterial.dispose();},[data]);
  useFrame(()=>{
    const p=source.current,w=p.missionState?.weather;
    data.points.visible=!!w&&w.kind!=='clear'&&w.intensity>0;data.ripples.visible=w?.kind==='rain'&&w.intensity>0;
    if(!w)return;
    const wind=weatherWind(w),u=data.uniforms;
    u.uTime.value=w.elapsed;u.uRain.value=w.kind==='rain'?1:0;u.uIntensity.value=w.intensity;u.uSize.value=w.kind==='rain'?.12:.07;
    u.uWind.value.set(wind.x*.6,0,-wind.y*.6);u.uAircraft.value.set(p.flight?.x??0,p.flight?.height??0,-(p.flight?.y??0));u.uPower.value=p.missionState?.motors??0;
    data.geometry.setDrawRange(0,Math.floor((p.quality==='high'?20000:12000)*w.intensity*(w.kind==='snow'?.7:1)));
  },-.5);
  return <group dispose={null}><primitive object={data.points}/><primitive object={data.ripples}/></group>;
}
