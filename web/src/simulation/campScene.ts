import * as THREE from 'three';
import {add,batchStatic,box,cylinder,material,sign,texture,tube} from './sceneModels';
import {seededRandom} from './flightVisuals';
import type {MissionWorld} from './missionWorld';
import {TAG_CAMERA} from './tagVision';
import {TAG_BITS,TAG_CODES} from './tag36h11';

/** 검은 외곽선의 한 변이 size다. 주변 흰 여백까지 포함하면 1.25배다. */
export function aprilTagTexture(id:number){
  const code=TAG_CODES[id];if(code===undefined)throw new Error(`tag36h11 코드 범위 밖: ${id}`);
  const map=texture(80,80,ctx=>{ctx.fillStyle='#fff';ctx.fillRect(0,0,80,80);ctx.fillStyle='#050505';ctx.fillRect(8,8,64,64);ctx.fillStyle='#fff';TAG_BITS.forEach(([x,y],i)=>{if(Math.floor(code/2**(35-i))%2)ctx.fillRect((x+1)*8,(y+1)*8,8,8);});});
  map.magFilter=THREE.NearestFilter;map.minFilter=THREE.LinearMipmapLinearFilter;return map;
}

export function buildTrainingCamp(world:MissionWorld){
  const random=seededRandom(48117),ground=new THREE.Group(),buildings=new THREE.Group(),details=new THREE.Group();
  const concrete=material('#929184',.96),curb=material('#b1ac99',.95),olive=material('#626855',.87,.08),steel=material('#525b4e',.55,.55),dark=material('#242c28',.8),paint=material('#d3c9a5',.94),asphalt=material('#50544b',.98),glass=material('#334b4b',.18,.5);
  const groundMap=new THREE.TextureLoader().load('/terrain/field-ground-albedo-v1.png');groundMap.colorSpace=THREE.SRGBColorSpace;groundMap.wrapS=groundMap.wrapT=THREE.RepeatWrapping;groundMap.repeat.set((world.width+40)/3,(world.depth+40)/3);groundMap.anisotropy=8;
  const geo=new THREE.PlaneGeometry(world.width+75,world.depth+75,120,100);geo.rotateX(-Math.PI/2);
  for(let i=0;i<geo.attributes.position.count;i++){const x=geo.attributes.position.getX(i),z=geo.attributes.position.getZ(i),d=Math.max(0,Math.abs(x)-world.width/2-4,Math.abs(z)-world.depth/2-4);geo.attributes.position.setY(i,-.035+Math.min(9,d*.32)*(1+Math.sin(x*.21)*Math.cos(z*.15)*.5));}geo.computeVertexNormals();
  add(ground,geo,new THREE.MeshStandardMaterial({map:groundMap,color:'#929580',roughness:1})).castShadow=false;
  // 도로와 부지는 지도와 같은 원점·축척을 사용한다.
  const roadway=(x:number,y:number,w:number,h:number)=>{box(ground,[w,.026,h],[x,.007,-y],asphalt);};
  const ox=world.width/2,oy=world.depth/2;
  for(const y of [5,17,29].filter(y=>y<world.depth-2))roadway(0,y-oy,world.width-2,3.6);
  for(const x of [6,23,41].filter(x=>x<world.width-2))roadway(x-ox,0,3.6,world.depth-2);
  for(let x=-world.width/2+2;x<world.width/2-2;x+=2.4)for(const y of [5,17,29].filter(y=>y<world.depth-2))box(ground,[.9,.008,.05],[x,.025,-y+oy],paint);
  for(let y=-world.depth/2+2;y<world.depth/2-2;y+=2.4)for(const x of [6,23,41].filter(x=>x<world.width-2))box(ground,[.05,.008,.9],[x-ox,.025,-y],paint);
  if(world.width>=40&&world.depth>=30){box(ground,[5,.034,7],[-world.width/2+5,.01,-1],concrete);for(let j=0;j<5;j++)box(ground,[3.5,.008,.065],[-world.width/2+5,.036,-3+j*1.2],paint);}
  const roofMap=texture(256,256,ctx=>{ctx.fillStyle='#4f594b';ctx.fillRect(0,0,256,256);for(let y=0;y<256;y+=16){ctx.fillStyle='#879079';ctx.fillRect(0,y,256,2);ctx.fillStyle='#303a2f';ctx.fillRect(0,y+3,256,2);}});roofMap.wrapS=roofMap.wrapT=THREE.RepeatWrapping;roofMap.repeat.set(2,3);
  const roof=new THREE.MeshStandardMaterial({map:roofMap,roughness:.65,metalness:.28});
  world.zones.filter(z=>z.kind==='obstacle').forEach((z,i)=>{
    if(z.shape==='rect'){
      const w=z.w??2,d=z.h??2,x=(z.x??0)+w/2,y=(z.y??0)+d/2,h=w<2?3.1:(z.name?.includes('정비')?4.3:3.3),hangar=z.name?.includes('정비');
      const group=new THREE.Group();group.name=`facility-${i}`;group.position.set(x,0,-y);buildings.add(group);
      box(group,[w,.24,d],[0,.1,0],concrete);box(group,[w-.08,h-.24,d-.08],[0,h/2+.12,0],i%2?olive:material('#b1b09c',.96));
      box(group,[w+.24,.14,d+.24],[0,h+.05,0],steel);
      if(w>2&&d>2){
        const rise=Math.min(1,d*.18),a=Math.atan2(rise,d/2),slope=Math.hypot(d/2,rise);for(const dir of [-1,1]){const panel=box(group,[w+.4,.07,slope+.15],[0,h+rise/2+.15,dir*d/4],roof);panel.rotation.x=-dir*a;}
        box(group,[w+.48,.065,.12],[0,h+rise+.18,0],steel);
        for(const side of [-1,1]){
          for(let p=-w/2+.8;p<w/2-.5;p+=1.7){if(hangar&&side===1)continue;box(group,[1.05,.90,.09],[p,1.95,side*(d/2+.025)],dark);box(group,[.91,.76,.014],[p,1.95,side*(d/2+.08)],glass);box(group,[.035,.8,.03],[p,1.95,side*(d/2+.095)],curb);box(group,[1.0,.045,.035],[p,1.94,side*(d/2+.095)],curb);box(group,[1.14,.1,.16],[p,1.48,side*(d/2+.05)],concrete);}
          tube(group,[[-w/2+.1,.1,side*d/2],[-w/2+.1,h+.05,side*d/2],[w/2,h+.05,side*d/2]],.035,steel);
        }
        const dw=hangar?w*.66:1.05,dh=hangar?3:2.15;box(group,[dw,dh,.08],[0,dh/2+.15,d/2+.025],dark);
        for(let r=0;r<(hangar?17:6);r++)box(group,[dw-.06,.055,.04],[0,.25+r*(dh-.2)/(hangar?17:6),d/2+.08],steel);
        box(group,[dw+.5,.12,.42],[0,dh+.24,d/2+.16],steel);box(group,[dw+.4,.14,.5],[0,.065,d/2+.18],concrete);
        if(!hangar)box(group,[.025,.18,.07],[.35,1.18,d/2+.1],paint);
        sign(group,Math.min(2.6,w*.42),.36,[0,h-.28,d/2+.051],z.name??`시설 ${i+1}`,'CHEONGSOL / TRAINING',true);
        for(const x of [-w*.32,w*.32]){box(group,[.52,.27,.46],[x,h+.42,-d*.18],steel,.025);for(let j=0;j<5;j++)box(group,[.4,.015,.008],[x,h+.34+j*.037,-d*.18+.24],dark);}
        box(group,[.8,.5,.33],[w/2+.16,.65,0],concrete,.02);for(let j=0;j<7;j++)box(group,[.013,.35,.25],[w/2+.32+j*.008,.67,0],steel);
      }
    }else if(z.shape==='circle'){const r=z.r??1;cylinder(buildings,r,3.2,[z.x??0,1.6,-(z.y??0)],olive,32);cylinder(buildings,r+.07,.13,[z.x??0,3.23,-(z.y??0)],steel);}
    else if(z.points?.length){const shape=new THREE.Shape(z.points.map(([x,y])=>new THREE.Vector2(x,y)));const g=new THREE.ExtrudeGeometry(shape,{depth:3.4,bevelEnabled:false});g.rotateX(-Math.PI/2);add(buildings,g,olive);}
  });
  // 경계 펜스와 조명을 부지 바깥쪽에 배치한다.
  const fenceX=world.width/2+.8,fenceY=world.depth/2+.8;
  for(const side of [-1,1])for(let x=-fenceX;x<=fenceX;x+=2.8){cylinder(details,.032,1.65,[x,.825,side*fenceY],steel,8);}
  for(const side of [-1,1])for(let y=-fenceY;y<=fenceY;y+=2.8)cylinder(details,.032,1.65,[side*fenceX,.825,y],steel,8);
  for(const h of [.35,.9,1.5]){for(const side of [-1,1]){tube(details,[[-fenceX,h,side*fenceY],[fenceX,h,side*fenceY]],.01,steel);tube(details,[[side*fenceX,h,-fenceY],[side*fenceX,h,fenceY]],.01,steel);}}
  for(let i=0;i<6;i++){const x=(i%2?1:-1)*(world.width/2-.7),z=-world.depth/2+3+Math.floor(i/2)*(world.depth-6)/2;cylinder(details,.055,4,[x,2,z],steel,12);box(details,[.55,.10,.30],[x,4,z],dark);box(details,[.42,.018,.23],[x,3.94,z],new THREE.MeshStandardMaterial({color:'#e1d4aa',emissive:'#e1d4aa',emissiveIntensity:.7}));}
  // 수목은 지도 외곽에 배치해 경로 계산에 없는 장애물이 생기지 않게 한다.
  const treeN=95,trunks=new THREE.InstancedMesh(new THREE.CylinderGeometry(.10,.22,1,7),material('#5b5544',1),treeN),crowns=new THREE.InstancedMesh(new THREE.ConeGeometry(1,1,9),material('#3c4c31',.95),treeN*3),dummy=new THREE.Object3D();
  for(let i=0;i<treeN;i++){const a=random()*Math.PI*2,r=1.14+random()*.4,x=Math.cos(a)*(world.width/2+4)*r,z=Math.sin(a)*(world.depth/2+4)*r;if(Math.abs(x)<world.width/2+1&&Math.abs(z)<world.depth/2+1){i--;continue;}const h=4+random()*3;dummy.position.set(x,h/2,z);dummy.scale.set(1,h,1);dummy.updateMatrix();trunks.setMatrixAt(i,dummy.matrix);for(let k=0;k<3;k++){dummy.position.y=h*.55+k*.85;dummy.scale.set(1.6-k*.3,2.5-k*.2,1.6-k*.3);dummy.updateMatrix();crowns.setMatrixAt(i*3+k,dummy.matrix);crowns.setColorAt(i*3+k,new THREE.Color().setHSL(.23+random()*.05,.22,.19+random()*.12));}}
  trunks.castShadow=crowns.castShadow=true;ground.add(trunks,crowns);
  const tags=new THREE.Group();tags.name='mission-tags';
  for(const tag of world.tags){
    box(ground,[1.16,.03,1.16],[tag.x,.033,-tag.y],concrete,.02);const plate=add(tags,new THREE.PlaneGeometry(tag.size*1.25,tag.size*1.25),new THREE.MeshBasicMaterial({map:aprilTagTexture(tag.code)}),[tag.x,TAG_CAMERA.tagHeight,-tag.y]);plate.rotation.x=-Math.PI/2;plate.name=`apriltag-${tag.id}`;plate.castShadow=false;
    for(const dir of [-1,1]){box(tags,[.18,.006,.025],[tag.x+dir*.43,.053,-tag.y-.43],paint);box(tags,[.025,.006,.18],[tag.x+dir*.43,.053,-tag.y-.35],paint);}
    const label=sign(tags,.67,.16,[tag.x,.054,-tag.y+.40],`TAG ${tag.id}`,'',true);label.rotation.x=-Math.PI/2;
  }
  for(const point of [world.start,world.goal]){
    const ring=add(ground,new THREE.RingGeometry(1.3,1.36,64),paint,[point.x,.037,-point.y]);ring.rotation.x=-Math.PI/2;
    for(const dir of [-1,1])box(ground,[.09,.01,.85],[point.x+dir*.29,.04,-point.y],paint);box(ground,[.58,.01,.09],[point.x,.04,-point.y],paint);
  }
  // 도로·표지 받침대는 재질별 병합, 수목은 기존 인스턴싱을 유지한다.
  ground.remove(trunks,crowns);
  const root=new THREE.Group();root.name='cheongsol-training-camp';root.add(batchStatic(ground),trunks,crowns,tags);
  const room=batchStatic(buildings),furniture=batchStatic(details);room.name='camp-buildings';room.add(furniture);root.add(room);
  const cover=box(root,[.54,.02,.54],[0,.065,0],dark);cover.name='tag-occlusion-cover';cover.visible=false;
  const windsock=new THREE.Group();windsock.position.set(world.start.x-2.5,3.2,-world.start.y);root.add(windsock);cylinder(room,.035,3.2,[world.start.x-2.5,1.6,-world.start.y],steel);
  const sock=add(windsock,new THREE.ConeGeometry(.14,.65,12,1,true),material('#bb9457',.9),[.3,0,0]);sock.rotation.z=-Math.PI/2;
  return {root,room,cover,windsock};
}
