import * as THREE from 'three';
import type {WeatherState} from './weather';

/** 지면·지붕·태그를 같은 세계 좌표의 눈으로 덮는다. 센서 카메라도 이 재질을 렌더링한다. */
export function weatherMaterials(root:THREE.Object3D){
  const uniforms={uWeatherWet:{value:0},uWeatherSnow:{value:0},uWeatherSeed:{value:0}};
  const seen=new Set<THREE.Material>();
  root.traverse(node=>{
    if(!(node instanceof THREE.Mesh))return;
    for(const material of Array.isArray(node.material)?node.material:[node.material]){
      if(seen.has(material)||!(material instanceof THREE.MeshStandardMaterial||material instanceof THREE.MeshBasicMaterial))continue;
      seen.add(material);
      material.onBeforeCompile=shader=>{
        Object.assign(shader.uniforms,uniforms);
        shader.vertexShader=shader.vertexShader.replace('#include <common>','#include <common>\nvarying vec3 vWeatherPosition; varying vec3 vWeatherNormal;');
        shader.vertexShader=shader.vertexShader.replace('#include <worldpos_vertex>',`#include <worldpos_vertex>
          vec4 weatherPosition=vec4(transformed,1.0); vec3 weatherNormal=normal;
          #ifdef USE_INSTANCING
            weatherPosition=instanceMatrix*weatherPosition; weatherNormal=mat3(instanceMatrix)*weatherNormal;
          #endif
          vWeatherPosition=(modelMatrix*weatherPosition).xyz; vWeatherNormal=normalize(mat3(modelMatrix)*weatherNormal);`);
        shader.fragmentShader=shader.fragmentShader.replace('#include <common>',`#include <common>
          varying vec3 vWeatherPosition; varying vec3 vWeatherNormal;
          uniform float uWeatherWet; uniform float uWeatherSnow; uniform float uWeatherSeed;
          float weatherHash(vec2 p){return fract(sin(dot(p,vec2(127.1,311.7))+uWeatherSeed)*43758.5453);}
          float weatherNoise(vec2 p){vec2 i=floor(p),f=fract(p);f=f*f*(3.0-2.0*f);return mix(mix(weatherHash(i),weatherHash(i+vec2(1,0)),f.x),mix(weatherHash(i+vec2(0,1)),weatherHash(i+vec2(1,1)),f.x),f.y);}`);
        shader.fragmentShader=shader.fragmentShader.replace('#include <color_fragment>',`#include <color_fragment>
          float weatherUp=smoothstep(0.25,0.85,normalize(vWeatherNormal).y);
          diffuseColor.rgb*=1.0-uWeatherWet*weatherUp*0.24;
          if(uWeatherSnow>0.001){
            float noiseValue=weatherNoise(vWeatherPosition.xz*5.7)*0.7+weatherNoise(vWeatherPosition.xz*19.0)*0.3;
            float snow=(uWeatherSnow>=0.999?1.0:smoothstep(noiseValue-0.12,noiseValue+0.12,uWeatherSnow))*weatherUp;
            diffuseColor.rgb=mix(diffuseColor.rgb,vec3(0.91,0.94,0.98),snow);
          }`);
        if(material instanceof THREE.MeshStandardMaterial)shader.fragmentShader=shader.fragmentShader.replace('#include <roughnessmap_fragment>','#include <roughnessmap_fragment>\nroughnessFactor=mix(roughnessFactor,0.28,uWeatherWet*weatherUp);');
      };
      material.customProgramCacheKey=()=>`mango-weather-v1-${material.type}`;
      material.needsUpdate=true;
    }
  });
  return (weather?:WeatherState)=>{uniforms.uWeatherWet.value=weather?.wetness??0;uniforms.uWeatherSnow.value=weather?.snowCover??0;uniforms.uWeatherSeed.value=weather?.seed??0;};
}
