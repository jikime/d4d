import * as THREE from 'three';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import { add, addTagMarker, batchStatic, box, cylinder, material, sign, texture, tube } from './sceneModels';
import { seededRandom, terrainHeight } from './flightVisuals';

/** 원본 메시와 재질로 구성한 가상 훈련장. 실재 부대나 지도 데이터가 아니다. */
export function buildFieldRange() {
  const random=seededRandom(73142);
  const ground=new THREE.Group(); const facilities=new THREE.Group();
  const soilMap=texture(512,512,(ctx)=>{
    ctx.fillStyle='#7e7964';ctx.fillRect(0,0,512,512);
    for(let i=0;i<46000;i++){
      const value=70+Math.floor(random()*90);ctx.fillStyle=`rgba(${value+12},${value+8},${value-10},${0.2+random()*0.5})`;
      const size=random()<0.96?1:2.5;ctx.fillRect(random()*512,random()*512,size,size);
    }
  });
  soilMap.wrapS=soilMap.wrapT=THREE.RepeatWrapping;soilMap.repeat.set(32,32);
  const albedo=new THREE.TextureLoader().load('/terrain/field-ground-albedo-v1.png');
  albedo.colorSpace=THREE.SRGBColorSpace;albedo.wrapS=albedo.wrapT=THREE.RepeatWrapping;albedo.repeat.set(32,32);albedo.anisotropy=8;
  const terrain=new THREE.PlaneGeometry(90,90,180,180);terrain.rotateX(-Math.PI/2);
  const positions=terrain.attributes.position;const colors=new Float32Array(positions.count*3);const color=new THREE.Color();
  for(let i=0;i<positions.count;i++){
    const x=positions.getX(i),z=positions.getZ(i);positions.setY(i,terrainHeight(x,z));
    const road=Math.exp(-Math.pow((x-3-Math.sin(z*.12)*2)/1.5,2));
    const grass=0.5+Math.sin(x*.35+z*.11)*Math.cos(z*.4)*.18;
    color.setRGB(.75+road*.12,.72+grass*.06+road*.06,.61+road*.10);color.toArray(colors,i*3);
  }
  terrain.setAttribute('color',new THREE.BufferAttribute(colors,3));terrain.computeVertexNormals();
  const soil=new THREE.MeshStandardMaterial({map:albedo,bumpMap:soilMap,bumpScale:.004,roughness:.98,vertexColors:true});
  add(ground,terrain,soil).castShadow=false;
  const concrete=material('#7d8171',.95);const olive=material('#4a5140',.85,.12);const dark=material('#1e251a',.75,.2);const steel=material('#89917b',.45,.65);const stripe=material('#c5c1a0',.87);
  const padMap=texture(1024,1024,(ctx)=>{
    ctx.fillStyle='#656a5b';ctx.fillRect(0,0,1024,1024);
    for(let i=0;i<60000;i++){ctx.fillStyle=random()>.5?'#ddd9ba13':'#080d061a';ctx.fillRect(random()*1024,random()*1024,1+random()*2,1+random()*2);}
    ctx.strokeStyle='#c0bea2';ctx.lineWidth=5;ctx.strokeRect(62,62,900,900);ctx.setLineDash([18,20]);ctx.lineWidth=2;ctx.strokeRect(102,102,820,820);
    ctx.setLineDash([]);ctx.font='600 32px Pretendard Variable';ctx.fillStyle='#d1ccaf';ctx.fillText('MANGO   /   LZ 04',78,936);ctx.font='16px Pretendard Variable';ctx.fillText('PRECISION FLIGHT TRAINING',78,902);
    ctx.strokeStyle='#2c332855';ctx.lineWidth=1;for(let i=0;i<16;i++){let x=random()*1024,y=random()*1024;ctx.beginPath();ctx.moveTo(x,y);for(let j=0;j<5;j++){x+=random()*35-12;y+=random()*40;ctx.lineTo(x,y);}ctx.stroke();}
  });
  box(ground,[2.9,.027,2.9],[0,-.017,0],concrete,.018);
  const pad=add(ground,new THREE.PlaneGeometry(2.86,2.86),new THREE.MeshStandardMaterial({map:padMap,bumpMap:padMap,bumpScale:.0015,roughness:.95}),[0,-.001,0]);pad.rotation.x=-Math.PI/2;
  addTagMarker(ground);
  // 낮은 경계석과 이동식 장비. 접근 구역 2.9m 안쪽은 비워 둔다.
  for(const x of [-2.4,2.4])for(const z of [-2.3,2.3]){
    box(facilities,[.32,.06,.32],[x,0,z],dark,.01);
    add(facilities,new THREE.ConeGeometry(.10,.34,16),material('#b49152',.9),[x,.20,z]);
    cylinder(facilities,.061,.055,[x,.24,z],stripe);
  }
  const shelter=new THREE.Group();shelter.position.set(-5.7,terrainHeight(-5.7,-5),-5);facilities.add(shelter);
  box(shelter,[4.2,.14,2.8],[0,.05,0],concrete,.035);
  box(shelter,[4,2.4,2.55],[0,1.3,-.06],olive,.06);
  for(let i=0;i<23;i++)box(shelter,[.028,2.3,.035],[-1.92+i*.174,1.3,1.23],steel,.003);
  box(shelter,[4.25,.13,2.9],[0,2.55,0],dark,.035);
  box(shelter,[.86,1.97,.035],[1.07,1.12,1.25],dark,.014);
  box(shelter,[.025,.15,.075],[1.36,1.05,1.29],steel,.004);
  box(shelter,[1.4,.65,.04],[-.70,1.48,1.26],material('#252f2c',.2,.5),.01);
  sign(shelter,1.3,.33,[-.65,2.15,1.27],'FIELD CONTROL','MANGO / TRAINING RANGE',true);
  for(let i=0;i<5;i++){
    const x=-3.2+(i%3)*.5,z=-1.65-Math.floor(i/3)*.5,y=terrainHeight(x,z);
    box(facilities,[.43,.29,.37],[x,y+.17,z],olive,.025);
    box(facilities,[.45,.036,.39],[x,y+.33,z],dark,.012);
    for(const dx of [-.14,.14])box(facilities,[.028,.20,.018],[x+dx,y+.19,z+.192],steel,.002);
    box(facilities,[.15,.033,.012],[x,y+.24,z+.207],dark,.005);
  }
  // 풍향을 읽을 수 있는 관측 마스트와 경계 펜스.
  const mastY=terrainHeight(4,-4);
  cylinder(facilities,.035,3.2,[4,mastY+1.6,-4],steel);
  for(let i=0;i<3;i++){const a=i*Math.PI*2/3;tube(facilities,[[4,mastY+1.3,-4],[4+Math.cos(a)*.65,mastY+.025,-4+Math.sin(a)*.65]],.012,dark);}
  box(facilities,[.22,.14,.3],[4,mastY+2.7,-4],dark,.02);
  for(let x=-12;x<=12;x+=2){const y=terrainHeight(x,-11);cylinder(facilities,.026,1.3,[x,y+.65,-11],steel);}
  for(const h of [.45,1.05]){const pts:[number,number,number][]=[];for(let x=-12;x<=12;x+=1)pts.push([x,terrainHeight(x,-11)+h,-11]);tube(facilities,pts,.008,steel);}
  // 바위·수목은 인스턴스로 묶어 GPU 드로 콜을 줄인다.
  const rockGeometry=new THREE.DodecahedronGeometry(1,1);const rocks=new THREE.InstancedMesh(rockGeometry,material('#696958',.98),110);
  const dummy=new THREE.Object3D();
  for(let i=0;i<110;i++){
    let x=(random()-.5)*60,z=(random()-.5)*60;if(Math.hypot(x,z)<4){x+=5;z+=5;}
    const scale=.06+random()*.4;dummy.position.set(x,terrainHeight(x,z)+scale*.18,z);dummy.scale.set(scale*1.5,scale*.65,scale);dummy.rotation.set(random(),random()*Math.PI,random());dummy.updateMatrix();rocks.setMatrixAt(i,dummy.matrix);
  }
  rocks.castShadow=rocks.receiveShadow=true;ground.add(rocks);
  const trunks=new THREE.InstancedMesh(new THREE.CylinderGeometry(.10,.20,1,8),material('#464134',1),48);
  const foliage=new THREE.InstancedMesh(new THREE.IcosahedronGeometry(1,2),material('#424e2b',.95),48*5);
  for(let i=0;i<48;i++){
    const a=random()*Math.PI*2,d=13+random()*23,x=Math.cos(a)*d,z=Math.sin(a)*d,h=3+random()*3, y=terrainHeight(x,z);
    dummy.position.set(x,y+h*.45,z);dummy.rotation.set(0,random()*6,0);dummy.scale.set(1,h,1);dummy.updateMatrix();trunks.setMatrixAt(i,dummy.matrix);
    for(let j=0;j<5;j++){
      dummy.position.set(x+(random()-.5)*1.3,y+h*.7+random()*1.4,z+(random()-.5)*1.3);dummy.scale.set(1+random()*.8,1+random(),1+random()*.8);dummy.updateMatrix();foliage.setMatrixAt(i*5+j,dummy.matrix);
      foliage.setColorAt(i*5+j,new THREE.Color().setHSL(.20+random()*.05,.22+random()*.14,.19+random()*.08));
    }
  }
  trunks.castShadow=true;foliage.castShadow=true;ground.add(trunks,foliage);
  const root=new THREE.Group();root.name='spider-field-range';root.add(ground);
  const room=batchStatic(facilities);room.name='field-facilities';root.add(room);
  const cover=box(root,[.21,.012,.21],[0,.017,0],dark,.006);cover.name='tag-occluder';cover.visible=false;
  const windsock=new THREE.Group();windsock.position.set(4,mastY+3.07,-4);root.add(windsock);
  const sock=add(windsock,new THREE.CylinderGeometry(.08,.13,.65,20,1,true),material('#ad8753',.9),[.36,0,0]);sock.rotation.z=Math.PI/2;
  for(const x of [.14,.38,.60]){const band=add(windsock,new THREE.CylinderGeometry(.132-x*.06,.132-x*.06,.055,20,1,true),stripe,[x,0,0]);band.rotation.z=Math.PI/2;}
  return {root,room,cover,windsock};
}

export function buildGrass() {
  const random=seededRandom(618);const count=6000;const blades:THREE.BufferGeometry[]=[];
  for(let leaf=0;leaf<6;leaf++){
    const bladeGeometry=new THREE.PlaneGeometry(.012,.26,1,5);bladeGeometry.translate(0,.13,0);const blade=bladeGeometry.attributes.position;
    for(let i=0;i<blade.count;i++){
      const h=blade.getY(i)/.26;blade.setX(i,blade.getX(i)*(1-h*.99)+h*h*.055);blade.setZ(i,h*h*.025);
    }
    bladeGeometry.scale(1,.45+random()*.7,1);bladeGeometry.rotateY(random()*Math.PI*2);bladeGeometry.translate((random()-.5)*.05,0,(random()-.5)*.05);blades.push(bladeGeometry);
  }
  const geometry=mergeGeometries(blades)!;blades.forEach((part)=>part.dispose());geometry.computeVertexNormals();
  const uniforms={time:{value:0},power:{value:0},drone:{value:new THREE.Vector3()},wind:{value:new THREE.Vector2()}};
  const mat=new THREE.MeshStandardMaterial({color:'#b1b097',roughness:1,side:THREE.DoubleSide});
  mat.onBeforeCompile=(shader)=>{
    shader.uniforms.uGrassTime=uniforms.time;shader.uniforms.uWash=uniforms.power;shader.uniforms.uAircraft=uniforms.drone;shader.uniforms.uWind=uniforms.wind;
    shader.vertexShader='uniform float uGrassTime; uniform float uWash; uniform vec3 uAircraft; uniform vec2 uWind;\n'+shader.vertexShader;
    shader.vertexShader=shader.vertexShader.replace('#include <begin_vertex>',`#include <begin_vertex>
      vec3 anchor=(modelMatrix * instanceMatrix * vec4(0.0,0.0,0.0,1.0)).xyz;
      vec2 away=anchor.xz-uAircraft.xz;
      float distanceToDrone=length(away);
      float radius=.48+uAircraft.y*.42;
      float wash=uWash*exp(-uAircraft.y*.52)*exp(-pow(distanceToDrone/radius,2.0));
      float tip=pow(clamp(position.y/.26,0.0,1.0),2.0);
      vec2 bend=tip*(normalize(away+vec2(.0001))*wash*.38+uWind*.008+vec2(sin(uGrassTime*2.1+anchor.x*.7),cos(uGrassTime*1.6+anchor.z))*.025);
      vec3 worldBend=vec3(bend.x,0.0,bend.y);
      transformed.x+=dot(instanceMatrix[0].xyz,worldBend)/dot(instanceMatrix[0].xyz,instanceMatrix[0].xyz);
      transformed.z+=dot(instanceMatrix[2].xyz,worldBend)/dot(instanceMatrix[2].xyz,instanceMatrix[2].xyz);
      transformed.y-=tip*wash*.11;
    `);
  };
  const mesh=new THREE.InstancedMesh(geometry,mat,count);mesh.name='downwash-responsive-grass';mesh.frustumCulled=false;
  const dummy=new THREE.Object3D();
  for(let i=0;i<count;i++){
    const radius=i<4500?1.53+Math.pow(random(),.75)*6:7+random()*30,a=random()*Math.PI*2;
    let x=Math.cos(a)*radius,z=Math.sin(a)*radius;
    if(Math.abs(x)<1.53&&Math.abs(z)<1.53){const k=1.55/Math.max(Math.abs(x),Math.abs(z));x*=k;z*=k;}
    dummy.position.set(x,terrainHeight(x,z),z);dummy.rotation.set(0,random()*Math.PI,0);const s=.35+random()*.9;dummy.scale.set(.8+random(),s,1);dummy.updateMatrix();mesh.setMatrixAt(i,dummy.matrix);
    mesh.setColorAt(i,new THREE.Color().setHSL(.15+random()*.09,.10+random()*.22,.32+random()*.23));
  }
  mesh.receiveShadow=true;
  return {mesh,uniforms};
}

export function buildDust() {
  const map=texture(64,64,(ctx)=>{const gradient=ctx.createRadialGradient(32,32,0,32,32,32);gradient.addColorStop(0,'#ffffffa0');gradient.addColorStop(.35,'#ffffff60');gradient.addColorStop(1,'#ffffff00');ctx.fillStyle=gradient;ctx.fillRect(0,0,64,64);});
  const positions=new Float32Array(160*3);const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(positions,3));
  const mat=new THREE.PointsMaterial({map,color:'#bcb395',size:.16,sizeAttenuation:true,transparent:true,opacity:.12,depthWrite:false});
  const points=new THREE.Points(geometry,mat);points.name='rotor-wash-dust';points.frustumCulled=false;
  return {points,positions,mat};
}
