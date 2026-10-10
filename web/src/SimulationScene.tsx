import {t, useLanguage} from "./i18n";
import { Component, useEffect, useMemo, useRef, useState, type ReactNode, type RefObject } from "react";
import { Canvas, useFrame, useThree } from "@react-three/fiber";
import { Grid, Line, OrbitControls, Sky } from "@react-three/drei";
import { Bloom, EffectComposer, ToneMapping } from "@react-three/postprocessing";
import { ToneMappingMode } from "postprocessing";
import type { OrbitControls as Controls } from "three-stdlib";
import * as THREE from "three";
import { RoomEnvironment } from "three/addons/environments/RoomEnvironment.js";
import { OutputPass } from "three/addons/postprocessing/OutputPass.js";
import type { Disturbance, SimFrame } from "./approachSim";
import { buildAircraft, buildTestBay, buildTelemetryBay, disposeModel } from "./simulation/sceneModels";
import { SENSOR_ASPECT, SENSOR_VERTICAL_FOV, sensorFootprint, worldPosition } from "./simulation/sceneMath";
import { drawFallback } from "./simulation/flatScene";
import { buildDust, buildFieldRange, buildGrass } from "./simulation/sceneTerrain";
import { clamp, damp, ROTOR_DIRECTIONS, rotorWash, terrainHeight, visualAttitude, type CameraInput } from "./simulation/flightVisuals";
import type { FlightSnapshot } from './simulation/pilotFlight';
import SceneLabel from './simulation/SceneLabel';
import {buildTrainingCamp} from './simulation/campScene';
import {terrainLayoutKey,type MissionWorld} from './simulation/missionWorld';
import type {MissionSnapshot} from './simulation/missionFlight';
import {VISION_MAX_AGE_MS,type VisionCapture} from './simulation/pixelVision';
import {aircraftDisplayScale} from './simulation/aircraftDisplay';
import "./simulationScene.css";

export type SimulationView = "orbit" | "follow" | "top" | "camera" | "fpv";
export interface SimulationSceneProps {
  frame: SimFrame | null;
  frames: readonly SimFrame[];
  disturbance: Disturbance;
  heightCm: number;
  startXcm: number;
  startYcm: number;
  arriveTolCm: number;
  view: SimulationView;
  showGuides: boolean;
  showRoom: boolean;
  showPath: boolean;
  dashboardVisible: boolean;
  panelVisible: boolean;
  quality: "high" | "standard";
  running: boolean;
  resetKey: number;
  environment: "field" | "lab";
  daylight: "day" | "dusk";
  playbackSpeed: number;
  zoomCommand: { sequence: number; factor: number };
  onZoom?: (value: number) => void;
  cameraInput: CameraInput;
  compactDisplay: boolean;
  sensorCanvas: RefObject<HTMLCanvasElement | null>;
  onReady?: (ok: boolean) => void;
  flight?:FlightSnapshot;
  mission?:MissionWorld;
  trainingPreview?:MissionWorld;
  missionState?:MissionSnapshot;
  frontCanvas?:RefObject<HTMLCanvasElement|null>;
  emphasizeAircraft?:boolean;
  liveTelemetry?:{motorPower:number};
  onVisionCapture?:(pixels:Uint8ClampedArray,meta:VisionCapture)=>void;
  canCaptureVision?:()=>boolean;
}
const location = (p: SimulationSceneProps) => worldPosition(p.frame?.x ?? p.startXcm, p.frame?.y ?? p.startYcm, p.heightCm);

function Lighting({ onReady, onLost, environment, daylight, camp=false, live=false }: { onReady?: (ok: boolean) => void; onLost: () => void; environment: SimulationSceneProps['environment']; daylight: SimulationSceneProps['daylight'];camp?:boolean;live?:boolean }) {
  const language = useLanguage();
  const { gl, scene, camera } = useThree();
  const field=environment==='field'; const dusk=daylight==='dusk';
  useEffect(() => {
    if (!import.meta.env.DEV) return;
    const host = window as unknown as { spiderScene?: { gl: THREE.WebGLRenderer; scene: THREE.Scene; camera: THREE.Camera } };
    const state = { gl, scene, camera }; host.spiderScene = state;
    return () => { if (host.spiderScene === state) delete host.spiderScene; };
  }, [gl, scene, camera]);
  useEffect(() => {
    const generator = new THREE.PMREMGenerator(gl); const room = new RoomEnvironment();
    const env = generator.fromScene(room, 0.04); scene.environment = env.texture; scene.environmentIntensity = 0.4;
    room.dispose(); generator.dispose();
    const lost = (event: Event) => { event.preventDefault(); onLost(); };
    gl.domElement.addEventListener("webglcontextlost", lost);
    onReady?.(true);
    return () => { gl.domElement.removeEventListener("webglcontextlost", lost); scene.environment = null; env.dispose(); };
  }, [gl, scene, onReady, onLost, live]);
  useEffect(() => {
    gl.domElement.setAttribute('aria-label', t(live?'실제 센서값으로 자세와 높이를 표시하는 3D 기체':'3D 시험장: 실제 시뮬레이션 좌표의 기체와 태그'));
  }, [gl, live, language]);
  return <>
    <color attach="background" args={[field ? dusk ? '#646a60' : '#aeb7a4' : '#0b1009']} />
    {field && <><fog attach="fog" args={[dusk?'#777968':'#a6af94',camp?65:18,camp?180:65]} /><Sky distance={450000} sunPosition={dusk?[-25,3,15]:[-12,18,9]} turbidity={5} rayleigh={1.2} mieCoefficient={0.008} /></>}
    <ambientLight intensity={dusk?0.20:0.35} /><hemisphereLight args={[dusk?'#9fa69e':'#e0e8dc', '#646044', dusk?.85:1.15]} />
    <directionalLight position={camp?[-24,dusk?18:38,22]:field?dusk?[-14,4,9]:[-8,12,6]:[-3,8,5]} color={dusk?'#f2b06a':'#fff0d1'} intensity={dusk?1.5:2.6} castShadow shadow-mapSize={[2048, 2048]} shadow-camera-left={camp?-36:field?-12:-4.5} shadow-camera-right={camp?36:field?12:4.5} shadow-camera-top={camp?36:field?12:4.5} shadow-camera-bottom={camp?-36:field?-12:-4.5} shadow-camera-near={0.1} shadow-camera-far={camp?110:45} shadow-normalBias={0.012} shadow-bias={-0.0001} shadow-radius={3} />
    <directionalLight position={[4, 3, -2]} color="#c1caba" intensity={dusk?.3:.7} />
    {!field && <Grid position={[0, -0.245, 0]} args={[20, 20]} cellSize={0.5} sectionSize={2} cellColor="#293329" sectionColor="#454c37" cellThickness={0.45} sectionThickness={0.8} fadeDistance={26} fadeStrength={1.5} infiniteGrid raycast={() => null} />}
  </>;
}

function World({ source, pose, environment,mission,aircraft }: { source: RefObject<SimulationSceneProps>; pose: RefObject<THREE.Vector3>; environment: SimulationSceneProps['environment'];mission?:MissionWorld;aircraft:RefObject<THREE.Group|null> }) {
  useLanguage();
  const invalidate=useThree(state=>state.invalidate);
  const terrainKey=useMemo(()=>mission?terrainLayoutKey(mission):'', [mission]);
  const [models, setModels] = useState<{ bay: ReturnType<typeof buildTestBay> | ReturnType<typeof buildFieldRange> | ReturnType<typeof buildTelemetryBay>; drone: ReturnType<typeof buildAircraft>; grass?: ReturnType<typeof buildGrass>; dust?: ReturnType<typeof buildDust> } | null>(null);
  const motion = useRef({ previousTime: -1, power: 0, seconds: 0 });
  useEffect(() => {
    // effect에서 생성·해제해 StrictMode의 시험 마운트에서도 GPU 자원을 회수한다.
    const bay = source.current.liveTelemetry?buildTelemetryBay():mission?buildTrainingCamp(mission,invalidate):environment==='field'?buildFieldRange(invalidate):buildTestBay(); const drone = buildAircraft();
    aircraft.current=drone.root;
    const grass=environment==='field'&&!mission?buildGrass():undefined;const dust=environment==='field'?buildDust():undefined;
    if(grass)bay.root.add(grass.mesh);if(dust)bay.root.add(dust.points);
    drone.root.position.set(...location(source.current));pose.current.copy(drone.root.position); setModels({ bay, drone, grass, dust });
    motion.current={previousTime:-1,power:0,seconds:0};
    return () => { if(aircraft.current===drone.root)aircraft.current=null;disposeModel(bay.root); disposeModel(drone.root); };
  }, [source, pose, environment,terrainKey,invalidate]);
  useFrame((_, delta) => {
    if (!models) return;
    const p = source.current; const time = p.frame?.tMs ?? 0; const { bay, drone, grass, dust } = models; const m=motion.current;
    const jump=!p.running||m.previousTime<0||time<m.previousTime||time-m.previousTime>250;
    const at=location(p);const blend=jump?1:1-Math.exp(-delta*28);
    pose.current.set(THREE.MathUtils.lerp(pose.current.x,at[0],blend),THREE.MathUtils.lerp(pose.current.y,at[1],blend),THREE.MathUtils.lerp(pose.current.z,at[2],blend));drone.root.position.copy(pose.current);
    const attitude=p.flight??visualAttitude(p.frame?.out.rollUs??0,p.frame?.out.pitchUs??0);
    drone.root.rotation.order='YXZ';const yaw=p.flight?-p.flight.yaw:0;drone.root.rotation.y=jump?yaw:drone.root.rotation.y+Math.atan2(Math.sin(yaw-drone.root.rotation.y),Math.cos(yaw-drone.root.rotation.y))*blend;
    drone.root.rotation.x=THREE.MathUtils.lerp(drone.root.rotation.x,attitude.pitch,blend);drone.root.rotation.z=THREE.MathUtils.lerp(drone.root.rotation.z,attitude.roll,blend);
    // 20Hz 제어 계산과 표시 보간을 분리한다. 일시정지/탐색 시 정확한 좌표로 맞춘다.
    const dt=Math.min(delta,.05)*p.playbackSpeed;
    if(jump)m.seconds=time/1000;else m.seconds+=dt;
    const desiredPower=p.liveTelemetry?.motorPower??(p.disturbance.kill?0:p.missionState?.motors??1);
    if(p.running)m.power=damp(m.power,desiredPower,p.disturbance.kill?5:3,dt);
    else m.power=p.liveTelemetry?.motorPower??p.missionState?.motors??(p.disturbance.kill?0:time>0?1:0);
    drone.rotors.forEach((rotor,i)=>{if(p.running){const trim=p.flight?1+(i<2?-1:1)*p.flight.roll*.3+(i%2?1:-1)*p.flight.pitch*.3+p.flight.input.climb*.09+ROTOR_DIRECTIONS[i]*p.flight.input.yaw*.05:1+(i<2?-1:1)*(p.frame?.out.rollUs??0)*.001+(i%2?1:-1)*(p.frame?.out.pitchUs??0)*.001;rotor.rotation.y+=dt*ROTOR_DIRECTIONS[i]*(65+m.power*170)*m.power*trim;}});
    drone.blur.forEach((disk)=>{disk.visible=p.running&&m.power>.12;(disk.material as THREE.MeshBasicMaterial).opacity=m.power*.12;});
    drone.gimbal.rotation.x=-drone.root.rotation.x;drone.gimbal.rotation.z=-drone.root.rotation.z;
    drone.navLights.forEach((mat,i)=>{mat.emissiveIntensity=(i%2?.9:1.4)+Math.pow(Math.max(0,Math.sin(m.seconds*3)),16)*1.8;});
    drone.led.emissive.set(p.disturbance.kill ? "#ed6650" : p.frame?.out.state === "HOLD" ? "#edb145" : "#78dc82");
    if(grass){grass.uniforms.time.value=m.seconds;grass.uniforms.power.value=m.power;grass.uniforms.drone.value.copy(pose.current);grass.uniforms.wind.value.set(p.disturbance.windRightCmS2,-p.disturbance.windFwdCmS2);}
    if(dust){
      const wash=rotorWash(at[1]-(p.mission?0:terrainHeight(at[0],at[2])),m.power,0);dust.mat.opacity=wash*.20;dust.points.visible=wash>.01;
      for(let i=0;i<160;i++){const phase=(m.seconds*.37+i*.6180339887)%1,angle=i*2.39996+phase*.9,radius=.10+phase*(.8+at[1]*.5);
        dust.positions[i*3]=pose.current.x+Math.cos(angle)*radius+p.disturbance.windRightCmS2*.02*phase;
        dust.positions[i*3+2]=pose.current.z+Math.sin(angle)*radius-p.disturbance.windFwdCmS2*.02*phase;
        dust.positions[i*3+1]=(p.mission?0:Math.max(0,terrainHeight(dust.positions[i*3],dust.positions[i*3+2])))+.03+Math.sin(phase*Math.PI)*.21;
      }dust.points.geometry.attributes.position.needsUpdate=true;
    }
    if('windsock' in bay){bay.windsock.rotation.y=Math.atan2(p.disturbance.windFwdCmS2,p.disturbance.windRightCmS2||1);bay.windsock.rotation.z=-.12+Math.sin(m.seconds*3)*.045;}
    bay.room.visible = p.showRoom; bay.cover.visible = p.disturbance.tagOccluded;
    if(p.mission){const tag=p.mission.tags.find(t=>t.id===p.missionState?.activeTag);if(tag)bay.cover.position.set(tag.x,.067,-tag.y);else bay.cover.visible=false;}
    m.previousTime=time;
  }, -3);
  return models && <><primitive object={models.bay.root} dispose={null} /><primitive object={models.drone.root} dispose={null} /></>;
}

function AircraftPresentation({source,aircraft}:{source:RefObject<SimulationSceneProps>;aircraft:RefObject<THREE.Group|null>}) {
  const local=useMemo(()=>new THREE.Vector3(),[]);
  useFrame(({camera,size})=>{
    const drone=aircraft.current;if(!drone)return;
    const p=source.current,overview=p.view==='orbit'||p.view==='top',perspective=camera as THREE.PerspectiveCamera;
    camera.updateMatrixWorld();local.copy(drone.position).applyMatrix4(camera.matrixWorldInverse);
    drone.scale.setScalar(p.mission&&p.emphasizeAircraft&&overview?aircraftDisplayScale(-local.z,perspective.fov,perspective.zoom,size.height,drone.position.y):1);
    drone.userData.displayScale=drone.scale.x;
  },-1.5);
  return null;
}

function CameraRig({ source, pose }: { source: RefObject<SimulationSceneProps>; pose: RefObject<THREE.Vector3> }) {
  useLanguage();
  const { camera, size, gl, scene, invalidate } = useThree(); const controls = useRef<Controls>(null);
  const previousCamera=useMemo(()=>new THREE.Vector3(),[]);
  const followRay=useMemo(()=>new THREE.Raycaster(),[]);
  const [view, setView] = useState(source.current.view); const last = useRef(""); const lastView=useRef(''); const framing = useRef(""); const moving = useRef(true);
  const goal = useMemo(() => new THREE.Vector3(), []); const target = useMemo(() => new THREE.Vector3(), []);
  const reduced = useMemo(() => window.matchMedia("(prefers-reduced-motion: reduce)").matches, []);
  const zoom=useRef({sequence:0,follow:1,camera:1,reported:0,baseline:6.2});
  const deltaVector=useMemo(()=>new THREE.Vector3(),[]);
  const stickVectors=useMemo(()=>({spherical:new THREE.Spherical(),right:new THREE.Vector3(),forward:new THREE.Vector3(),pan:new THREE.Vector3()}),[]);
  useEffect(()=>{
    const applyZoom=(factor:number)=>{
      invalidate();
      if(source.current.view==='follow')zoom.current.follow=clamp(zoom.current.follow/factor,.4,3.5);
      else {zoom.current.camera=clamp(zoom.current.camera*factor,1,4);const perspective=camera as THREE.PerspectiveCamera;perspective.zoom=zoom.current.camera;perspective.updateProjectionMatrix();}
    };
    const wheel=(event:WheelEvent)=>{
      const p=source.current;if(p.view!=='follow'&&p.view!=='camera')return;
      event.preventDefault();applyZoom(Math.exp(clamp(-event.deltaY*.001,-.2,.2)));
    };
    // 추적·하향 시점은 OrbitControls를 잠그므로 두 손가락 확대는 별도로 연결한다.
    let pinchDistance=0;
    const touchDistance=(event:TouchEvent)=>event.touches.length===2?Math.hypot(event.touches[0].clientX-event.touches[1].clientX,event.touches[0].clientY-event.touches[1].clientY):0;
    const touchStart=(event:TouchEvent)=>{pinchDistance=touchDistance(event);};
    const touchMove=(event:TouchEvent)=>{
      const p=source.current;if(p.view!=='follow'&&p.view!=='camera'){pinchDistance=0;return;}
      const distance=touchDistance(event);
      if(distance&&pinchDistance){event.preventDefault();applyZoom(clamp(distance/pinchDistance,.5,2));}
      pinchDistance=distance;
    };
    const canvas=gl.domElement;
    canvas.addEventListener('wheel',wheel,{passive:false});canvas.addEventListener('touchstart',touchStart,{passive:true});canvas.addEventListener('touchmove',touchMove,{passive:false});canvas.addEventListener('touchend',touchStart,{passive:true});canvas.addEventListener('touchcancel',touchStart,{passive:true});
    return()=>{canvas.removeEventListener('wheel',wheel);canvas.removeEventListener('touchstart',touchStart);canvas.removeEventListener('touchmove',touchMove);canvas.removeEventListener('touchend',touchStart);canvas.removeEventListener('touchcancel',touchStart);};
  },[gl,camera,source]);
  useFrame((_, delta) => {
    previousCamera.copy(camera.position);
    const p = source.current; const perspective = camera as THREE.PerspectiveCamera;
    const viewKey=`${p.view}-${p.resetKey}-${p.environment}-${p.mission?.key??''}`;const key = `${viewKey}-${size.width}-${size.height}`;
    if (key !== last.current) {
      last.current = key;
      if(lastView.current!==viewKey){
      lastView.current=viewKey;moving.current = true; setView(p.view);
      const fit = Math.max(1, 1.4 / (size.width / size.height));
      if (p.view === "orbit") { goal.set(...(p.environment==='field'?[3.5,2.6,4.1]:[4.4,4.2,5.7]) as [number,number,number]).multiplyScalar(fit); target.set(0, 0.52, -0.15); }
      if (p.view === "top") { goal.set(0, 7.4 * fit, 0.001); target.set(0, 0, -0.05); }
      if(p.flight&&!p.liveTelemetry&&(p.view==='orbit'||p.view==='top')){
        const points=p.flight.route.points,minX=Math.min(...points.map(v=>v.x)),maxX=Math.max(...points.map(v=>v.x)),minY=Math.min(...points.map(v=>v.y)),maxY=Math.max(...points.map(v=>v.y));
        const span=Math.max(6,maxX-minX,maxY-minY);target.set((minX+maxX)/2,1.1,-(minY+maxY)/2);
        goal.copy(target).add(p.view==='top'?deltaVector.set(0,span*1.65*fit,.001):deltaVector.set(span*.65,span*.7,span*.85).multiplyScalar(fit));
      }
      if(p.mission&&(p.view==='orbit'||p.view==='top')){const span=Math.max(p.mission.width,p.mission.depth);target.set(0,0,0);goal.copy(target).add(p.view==='top'?deltaVector.set(0,span*1.45*fit,.001):deltaVector.set(span*.56,span*.79,span*.91).multiplyScalar(fit));}
      if(p.liveTelemetry&&(p.view==='orbit'||p.view==='top')){target.copy(pose.current);goal.copy(target).add(p.view==='top'?deltaVector.set(0,3*fit,.001):deltaVector.set(1.35,1.05,1.7).multiplyScalar(fit));}
      zoom.current.baseline=goal.distanceTo(target);
      perspective.fov = p.view === "camera" ? SENSOR_VERTICAL_FOV : p.view==='fpv'?64:37;
      zoom.current.follow=1;zoom.current.camera=1;perspective.zoom=1;
      // 넓은 맵의 바닥 도색·태그가 깊이 버퍼에서 같은 면으로 뭉개지지 않게 한다.
      perspective.near=p.mission&&(p.view==='orbit'||p.view==='top')?.2:.01;
      const downward=p.view==='camera'||p.view==='top';
      perspective.up.set(0,downward?0:1,downward?-1:0);
      }
    }
    if(zoom.current.sequence!==p.zoomCommand.sequence){
      zoom.current.sequence=p.zoomCommand.sequence;
      if(p.view==='follow')zoom.current.follow=clamp(zoom.current.follow/p.zoomCommand.factor,.4,3.5);
      else if(p.view==='camera'){zoom.current.camera=clamp(zoom.current.camera*p.zoomCommand.factor,1,4);perspective.zoom=zoom.current.camera;perspective.updateProjectionMatrix();}
      else if(controls.current){deltaVector.copy(camera.position).sub(controls.current.target);const distance=clamp(deltaVector.length()/p.zoomCommand.factor,.55,p.flight?100:35);deltaVector.setLength(distance);camera.position.copy(controls.current.target).add(deltaVector);moving.current=false;controls.current.update();}
    }
    const frameKey = `${key}-${p.dashboardVisible}-${p.panelVisible}`;
    if (frameKey !== framing.current) {
      framing.current = frameKey;
      const hud = p.dashboardVisible && !p.mission && !p.compactDisplay && p.panelVisible && size.width > 900 && p.view !== "camera";
      if (hud) perspective.setViewOffset(size.width, size.height, Math.min(185, size.width * 0.115), -size.height * 0.03, size.width, size.height);
      else perspective.clearViewOffset();
      perspective.updateProjectionMatrix();
    }
    const input=p.cameraInput;
    if(controls.current&&(input.panX||input.panY||input.rotateX||input.rotateY)&&(p.view==='orbit'||p.view==='top')){
      moving.current=false;const control=controls.current;const {spherical,right,forward,pan}=stickVectors;
      deltaVector.copy(camera.position).sub(control.target);spherical.setFromVector3(deltaVector);
      const dt=Math.min(delta,.05);
      if(p.view==='orbit'){spherical.theta-=input.rotateX*dt*1.4;spherical.phi=clamp(spherical.phi+input.rotateY*dt,0.10,Math.PI*.465);}
      right.setFromMatrixColumn(camera.matrixWorld,0);right.y=0;right.normalize();forward.setFromMatrixColumn(camera.matrixWorld,2);forward.y=0;forward.normalize();
      pan.copy(right).multiplyScalar(input.panX).addScaledVector(forward,input.panY).multiplyScalar(dt*Math.max(.6,spherical.radius*.24));control.target.add(pan);
      camera.position.copy(control.target).add(deltaVector.setFromSpherical(spherical));control.update();
    }
    const at = pose.current.toArray();
    const zoomValue=p.view==='camera'?zoom.current.camera:p.view==='follow'?1/zoom.current.follow:zoom.current.baseline/Math.max(.1,camera.position.distanceTo(controls.current?.target??target));
    if(Math.abs(zoomValue-zoom.current.reported)>.025){zoom.current.reported=zoomValue;p.onZoom?.(zoomValue);}
    const yaw=p.flight?.yaw??0;
    if(p.view==='fpv'){camera.up.set(0,1,0);camera.position.set(at[0]+Math.sin(yaw)*.24,at[1]+.025,at[2]-Math.cos(yaw)*.24);camera.lookAt(at[0]+Math.sin(yaw)*8,at[1]-1.1,at[2]-Math.cos(yaw)*8);return;}
    if (p.view === "camera") { const x=at[0]+Math.sin(yaw)*.026,z=at[2]-Math.cos(yaw)*.026;camera.up.set(Math.sin(yaw),0,-Math.cos(yaw));camera.position.set(x, at[1] - 0.115, z); camera.lookAt(x,at[1]-10,z); return; }
    if (p.view === "follow") {
      target.set(at[0],at[1]-.03,at[2]);
      if(p.liveTelemetry)deltaVector.set(.85*Math.cos(yaw)-1.2*Math.sin(yaw),.65,.85*Math.sin(yaw)+1.2*Math.cos(yaw));
      else if(p.flight)deltaVector.set(1.9*Math.cos(yaw)-3.1*Math.sin(yaw),1.35,1.9*Math.sin(yaw)+3.1*Math.cos(yaw));else deltaVector.set(.85,.46,-1.03);
      goal.copy(target).add(deltaVector.multiplyScalar(zoom.current.follow));moving.current=true;
    }
    if (moving.current && controls.current) {
      const blend = reduced ? 1 : 1 - Math.exp(-Math.min(delta, 0.1) * 5);
      camera.position.lerp(goal, blend); controls.current.target.lerp(target, blend);
      if(p.view==='follow'&&p.mission){
        // 추적 카메라가 건물 뒤로 들어가 기체를 가리지 않도록 시선을 확보한다.
        const buildings=scene.getObjectByName('camp-buildings');
        if(buildings){const distance=deltaVector.copy(camera.position).sub(target).length();followRay.set(target,deltaVector.normalize());followRay.far=distance;
          const hit=followRay.intersectObject(buildings)[0];if(hit)camera.position.copy(target).addScaledVector(deltaVector,Math.max(.45,hit.distance-.3));}
        controls.current.target.copy(target);
      }
      controls.current.update();
      if (p.view !== "follow" && camera.position.distanceTo(goal) < 0.006) moving.current = false;
    }
    if(!p.running&&camera.position.distanceToSquared(previousCamera)>1e-7)invalidate();
  }, -2);
  return <OrbitControls ref={controls} makeDefault enabled={view === "orbit" || view === "top"} enableRotate={view === "orbit"} enableDamping={!reduced} dampingFactor={0.08} minDistance={0.55} maxDistance={source.current.flight?100:35} maxPolarAngle={Math.PI * 0.485} zoomSpeed={0.65} onStart={() => { moving.current = false; }} />;
}

function FlightGuides({props:p,guideRef}:{props:SimulationSceneProps;guideRef:RefObject<THREE.Group|null>}) {
  useLanguage();
  const s=p.flight!;
  const route=useMemo(()=>s.route.points.map(v=>worldPosition(v.x*100,v.y*100,v.height*100)),[s.route]);
  const trail=useMemo(()=>s.trail.map(v=>worldPosition(v.x*100,v.y*100,v.height*100)),[s.trail]);
  const preview=useMemo(()=>p.trainingPreview?.route.points.map(v=>[v.x,v.height,-v.y] as [number,number,number])??[],[p.trainingPreview]);
  const overlay=p.trainingPreview??p.mission;
  return <group ref={guideRef} visible={p.view!=='camera'&&p.view!=='fpv'}>
    {overlay?.grid&&overlay.zones.filter(z=>z.kind==='keepout'||z.kind==='height_limit').map((z,i)=><group key={`grid-zone-${i}`} position={[(z.x??0)+(z.w??0)/2,.085,-(z.y??0)-(z.h??0)/2]}>
      <mesh rotation-x={-Math.PI/2}><planeGeometry args={[z.w??0,z.h??0]}/><meshBasicMaterial color={z.kind==='keepout'?'#cb7054':'#d8b36a'} transparent opacity={.22} depthWrite={false}/></mesh>
      <Line points={[[-(z.w??0)/2,0,-(z.h??0)/2],[(z.w??0)/2,0,-(z.h??0)/2],[(z.w??0)/2,0,(z.h??0)/2],[-(z.w??0)/2,0,(z.h??0)/2],[-(z.w??0)/2,0,-(z.h??0)/2]]} color={z.kind==='keepout'?'#ce8065':'#d5b97a'} lineWidth={1.3}/>
      {p.showGuides&&<SceneLabel position={[0,.35,0]} width={105} text={`${z.name} · ${t(z.kind==='keepout'?'폐쇄':`${z.value}m 고도 제한`)}`}/>}
    </group>)}
    {preview.length>1&&<Line points={preview} color="#9ce5c3" lineWidth={2.6} dashed dashSize={.35} gapSize={.12}/>}
    {p.mission&&p.showGuides&&s.height>.5&&p.view!=='follow'&&<SceneLabel text="MANGO 01" width={92} screenOffset={[0,-56]} position={[s.x,s.height,-s.y]}/>}
    {p.showPath&&<>
      {!p.liveTelemetry&&route.length>1&&<Line points={route} color="#d4b575" lineWidth={1.4} dashed dashSize={.18} gapSize={.13} transparent opacity={.75}/>}
      {trail.length>1&&<Line points={trail} color="#c6e697" lineWidth={2.4}/>}
      {!p.mission&&!p.liveTelemetry&&route.map((at,i)=><group key={i} position={at}>
        <mesh rotation-x={-Math.PI/2}><torusGeometry args={[i===s.waypoint ? .20 : .13,.015,6,32]}/><meshBasicMaterial color={i<s.waypoint||s.completed?'#b2d488':i===s.waypoint?'#f0d690':'#8f9c73'} transparent opacity={.85}/></mesh>
        <Line points={[[0,0,0],[0,-at[1]+Math.max(0,terrainHeight(at[0],at[2]))+.025,0]]} color="#b4a477" transparent opacity={.3} dashed dashSize={.08} gapSize={.08}/>
        {p.showGuides&&i===s.waypoint&&!s.completed&&<SceneLabel position={[0,.3,0]} text={t(`WP ${String(i+1).padStart(2,'0')}`)}/>}
      </group>)}
      {p.mission?.tags.map(tag=><group key={tag.id} position={[tag.x,.065,-tag.y]}><mesh rotation-x={-Math.PI/2}><ringGeometry args={[.61,.65,48]}/><meshBasicMaterial color={p.missionState?.visited.includes(tag.id)?'#c7e59e':tag.id===p.missionState?.activeTag?'#f0ca76':'#bcc4ac'} transparent opacity={.8}/></mesh>{p.showGuides&&(p.view!=='follow'||tag.id===p.missionState?.activeTag)&&(!p.mission?.grid||tag.id===p.missionState?.activeTag||p.mission?.stops.some(s=>s.tagId===tag.id))&&<SceneLabel text={t(`TAG ${tag.id}`)} subtitle={t(tag.name)} screenOffset={p.view==='follow'?[0,35]:[0,0]} position={[0,p.view==='follow'?.06:.92,0]}/>}</group>)}
    </>}
  </group>;
}

function Guides({ props: p, guideRef }: { props: SimulationSceneProps; guideRef: RefObject<THREE.Group | null> }) {
  useLanguage();
  const at = location(p); const [width, height] = sensorFootprint(p.heightCm - 11.5);
  const trail = useMemo(() => p.frames.slice(-900).map((f) => worldPosition(f.x, f.y, 1.3)), [p.frame?.tMs, p.resetKey, p.frames]);
  const corners: [number, number, number][] = [[at[0]-width/2,0.018,at[2]-height/2], [at[0]+width/2,0.018,at[2]-height/2], [at[0]+width/2,0.018,at[2]+height/2], [at[0]-width/2,0.018,at[2]+height/2]];
  if(p.flight)return <FlightGuides props={p} guideRef={guideRef}/>;
  return <group ref={guideRef}>
    {p.showPath && p.view !== "camera" && trail.length > 1 && <Line points={trail} color="#298a79" lineWidth={2.5} />}
    {p.showGuides && p.view !== "camera" && <>
      <Line points={[...corners, corners[0]]} color="#378e89" transparent opacity={0.65} lineWidth={1} />
      <Line points={[[at[0], 0.018, at[2]], at]} color="#477a78" dashed dashSize={0.04} gapSize={0.025} lineWidth={1} />
      <Line points={[[0, 0.02, 0], [at[0], 0.02, at[2]]]} color="#be943e" dashed dashSize={0.025} gapSize={0.018} lineWidth={1.5} />
      <mesh rotation-x={-Math.PI/2} position-y={0.012}><ringGeometry args={[Math.max(0.001, p.arriveTolCm/100 - 0.004), p.arriveTolCm/100, 64]} /><meshBasicMaterial color="#739b3d" side={THREE.DoubleSide} /></mesh>
      {p.dashboardVisible && !p.compactDisplay && <>
        <SceneLabel position={[at[0]+.22,at[1]+.07,at[2]]} text="MANGO 01" subtitle={t(`고도 ${p.heightCm.toFixed(0)} cm`)}/>
        <SceneLabel position={[.22,.04,.12]} text={t("착륙 기준 태그")} subtitle={t(p.disturbance.tagOccluded?'태그 가림':'기준점 0, 0')}/>
      </>}
    </>}
  </group>;
}

function SensorFeed({ source, guideRef, pose,aircraft }: { source: RefObject<SimulationSceneProps>; guideRef: RefObject<THREE.Group | null>; pose: RefObject<THREE.Vector3>;aircraft:RefObject<THREE.Group|null> }) {
  const language=useLanguage();
  const {gl,scene,invalidate}=useThree();
  const r=useMemo(()=>({camera:new THREE.PerspectiveCamera(SENSOR_VERTICAL_FOV,SENSOR_ASPECT,.008,180),target:new THREE.WebGLRenderTarget(320,240,{type:THREE.HalfFloatType}),pass:new OutputPass(),viewport:new THREE.Vector4(),scissor:new THREE.Vector4(),last:-1,generation:0,disposed:false,async:true,
    feeds:[false,true].map(front=>({front,output:new THREE.WebGLRenderTarget(320,240),pixels:new Uint8Array(320*240*4),image:new ImageData(320,240),pending:false,signature:''}))}),[]);
  useEffect(()=>{r.disposed=false;return()=>{r.disposed=true;r.generation++;r.target.dispose();r.feeds.forEach(f=>f.output.dispose());r.pass.dispose();};},[r]);
  useFrame(()=>{
    // R3F는 demand/always 전환 시 장면 시계를 초기화하므로 단조 증가하는 벽시계를 쓴다.
    const p=source.current,now=performance.now()/1000;if((p.running||p.liveTelemetry)&&now-r.last<.085)return;r.last=now;
    const oldTarget=gl.getRenderTarget(),oldTone=gl.toneMapping,oldScissor=gl.getScissorTest(),oldAutoClear=gl.autoClear,oldShadowUpdate=gl.shadowMap.autoUpdate;gl.getViewport(r.viewport);gl.getScissor(r.scissor);
    const visible=guideRef.current?.visible;if(guideRef.current)guideRef.current.visible=false;
    const drone=aircraft.current,displayScale=drone?.scale.x??1;
    if(drone){drone.scale.setScalar(1);drone.updateMatrixWorld(true);}
    // 센서 시점은 메인 화면의 그림자 지도를 재사용한다.
    gl.shadowMap.autoUpdate=false;
    try{
      const at=pose.current,yaw=p.flight?.yaw??0,sin=Math.sin(yaw),cos=Math.cos(yaw);
      for(const feed of r.feeds){
        const {front}=feed,canvas=front?p.frontCanvas?.current:p.sensorCanvas.current;
        const displayed=p.dashboardVisible&&!!canvas?.offsetParent;
        const signature=`${p.frame?.tMs}-${at.x}-${at.y}-${at.z}-${yaw}-${p.missionState?.visionSession}-${p.resetKey}-${p.daylight}-${p.disturbance.tagOccluded}-${displayed}-${language}-${p.missionState?.vision.status}-${scene.getObjectByName('cheongsol-training-camp')?.uuid}`;
        if(feed.pending||(!p.running&&feed.signature===signature)||(!displayed&&(front||!p.onVisionCapture)))continue;
        if(!front&&p.onVisionCapture&&p.canCaptureVision&&!p.canCaptureVision())continue;
        if(front){r.camera.fov=64;r.camera.up.set(0,1,0);r.camera.position.set(at.x+sin*.24,at.y+.025,at.z-cos*.24);r.camera.lookAt(at.x+sin*8,at.y-1.1,at.z-cos*8);}
        else {r.camera.fov=SENSOR_VERTICAL_FOV;r.camera.up.set(sin,0,-cos);r.camera.position.set(at.x+sin*.026,at.y-.115,at.z-cos*.026);r.camera.lookAt(at.x+sin*.026,at.y-10,at.z-cos*.026);}
        const meta:VisionCapture|undefined=p.missionState?{session:p.missionState.visionSession,tMs:p.missionState.tMs,capturedAt:performance.now(),pose:{x:at.x,y:-at.z,height:at.y,yaw,vx:p.missionState.vx,vy:p.missionState.vy}}:undefined;
        r.camera.updateProjectionMatrix();gl.toneMapping=THREE.ACESFilmicToneMapping;gl.autoClear=true;gl.setScissorTest(false);gl.setRenderTarget(r.target);gl.clear();gl.render(scene,r.camera);
        r.pass.render(gl,feed.output,r.target,0,false);
        feed.pending=true;feed.signature=signature;const generation=r.generation;
        // GPU 결과를 기다리는 동안 UI·조종 입력을 막지 않는다. 각 카메라는 한 프레임만 처리한다.
        const read=r.async?gl.readRenderTargetPixelsAsync(feed.output,0,0,320,240,feed.pixels):Promise.resolve(gl.readRenderTargetPixels(feed.output,0,0,320,240,feed.pixels));
        void read.then(()=>{
          if(r.disposed||generation!==r.generation||meta&&source.current.missionState?.visionSession!==meta.session)return;
          const flipped=feed.image.data;
          for(let row=0;row<240;row++)flipped.set(feed.pixels.subarray(row*1280,(row+1)*1280),(239-row)*1280);
          // 인식에는 ID·사각형을 그리기 전 원본 픽셀만 전달한다.
          if(!front&&meta)p.onVisionCapture?.(flipped,meta);
          if(!displayed||!canvas)return;
          const ctx=canvas.getContext('2d');if(!ctx)return;ctx.putImageData(feed.image,0,0);
          if(p.missionState&&p.mission&&!front){
            const s=p.missionState;ctx.lineWidth=1.3;ctx.font='10px Pretendard Variable';
            for(const detection of s.detections){if(!detection.valid||s.tMs-s.vision.frameMs>VISION_MAX_AGE_MS)continue;
              ctx.strokeStyle=detection.id===s.activeTag?s.lock>0?'#bfe79b':'#ffd685':'#8faebc';ctx.fillStyle=ctx.strokeStyle;ctx.beginPath();detection.corners.forEach(([x,y],i)=>i?ctx.lineTo(x,y):ctx.moveTo(x,y));ctx.closePath();ctx.stroke();detection.corners.forEach(([x,y])=>ctx.fillRect(x-2,y-2,4,4));
              const [x,y]=detection.center;ctx.fillText(`ID ${detection.code} / ${detection.pixels.toFixed(0)}px`,Math.max(6,x-35),Math.max(16,Math.min(...detection.corners.map(p=>p[1]))-8));ctx.beginPath();ctx.moveTo(x,120);ctx.lineTo(x,y);ctx.lineTo(160,y);ctx.stroke();
            }
            if(p.running&&['search','align','confirm'].includes(s.phase)){ctx.fillStyle='#cfdfa318';ctx.fillRect(0,18+(s.tMs/35)%203,320,3);}
            ctx.strokeStyle='#e2ecc477';ctx.beginPath();ctx.moveTo(150,120);ctx.lineTo(170,120);ctx.moveTo(160,110);ctx.lineTo(160,130);ctx.stroke();
          }
        }).catch(()=>{if(!r.disposed&&generation===r.generation){r.async=false;feed.signature='';invalidate();}}).finally(()=>{feed.pending=false;if(!r.disposed&&!source.current.running)invalidate(2);});
      }
    }finally{if(drone){drone.scale.setScalar(displayScale);drone.updateMatrixWorld(true);}gl.shadowMap.autoUpdate=oldShadowUpdate;gl.setRenderTarget(oldTarget);gl.toneMapping=oldTone;gl.autoClear=oldAutoClear;gl.setViewport(r.viewport);gl.setScissor(r.scissor);gl.setScissorTest(oldScissor);if(guideRef.current)guideRef.current.visible=visible??true;}
  });return null;
}

/** 정지 중에는 설정·카메라 조작·폰트 로딩 등 화면이 달라질 때만 깨운다. */
function SceneWake({props}:{props:SimulationSceneProps}){
  const invalidate=useThree(state=>state.invalidate);
  useEffect(()=>{invalidate(2);},[props,invalidate]);
  useEffect(()=>{let alive=true;void document.fonts.ready.then(()=>{if(alive)invalidate();});return()=>{alive=false;};},[invalidate]);
  return null;
}

function AdaptiveResolution() {
  const { setDpr, viewport } = useThree(); const sample = useRef({ time: 0, frames: 0, warmup: 4 });
  useFrame((_, dt) => {
    const m = sample.current; m.warmup -= dt; if (m.warmup > 0) return; m.time += dt; m.frames++;
    if (m.time > 3) { if (m.frames / m.time < 35 && viewport.dpr > 1) setDpr(1); m.time = 0; m.frames = 0; m.warmup = 5; }
  });
  return null;
}

function RenderEffects({ quality }: { quality: SimulationSceneProps["quality"] }) {
  useLanguage();
  const gl = useThree((state) => state.gl);
  const effects = useMemo(() => <><Bloom luminanceThreshold={2.5} intensity={0.12} mipmapBlur /><ToneMapping mode={ToneMappingMode.ACES_FILMIC} /></>, []);
  useFrame(() => {
    // 후처리 비활성 시 기본 렌더러가 색 변환과 버퍼 초기화를 맡는다.
    gl.toneMapping = quality === "high" ? THREE.NoToneMapping : THREE.ACESFilmicToneMapping;
    gl.autoClear = quality !== "high";
  }, -1);
  if (gl.getContext().isContextLost()) return null;
  // 화질 전환 때 GPU 파이프라인을 재생성하지 않아 컨텍스트 상실과의 경합을 피한다.
  return <EffectComposer enabled={quality === "high"} multisampling={2}>{effects}</EffectComposer>;
}

function FlatScene({ props, retry }: { props: SimulationSceneProps; retry: () => void }) {
  useLanguage();
  const canvas = useRef<HTMLCanvasElement>(null);
  useEffect(() => { props.onReady?.(false); }, [props.onReady]);
  useEffect(() => {
    if (canvas.current) drawFallback(canvas.current, props);
    const observer = new ResizeObserver(() => { if (canvas.current) drawFallback(canvas.current, props); });
    if (canvas.current) observer.observe(canvas.current); return () => observer.disconnect();
  }, [props]);
  return <div className="simulation-scene__flat"><canvas ref={canvas} aria-label={t("3D 대신 동일한 시뮬레이션 좌표로 그린 평면도")} /><div className="simulation-scene__fallback-note"><b>{t("3D 화면을 사용할 수 없어요")}</b><span>{t("같은 좌표의 평면도로 표시합니다.")}</span><button type="button" onClick={retry}>{t("3D 다시 시도")}</button></div></div>;
}
class SceneBoundary extends Component<{ children: ReactNode; fallback: ReactNode }, { failed: boolean }> {
  state = { failed: false };
  static getDerivedStateFromError() { return { failed: true }; }
  render() { return this.state.failed ? this.props.fallback : this.props.children; }
}

export default function SimulationScene(props: SimulationSceneProps) {
  useLanguage();
  const source = useRef(props); source.current = props; const guides = useRef<THREE.Group | null>(null);
  const aircraft=useRef<THREE.Group|null>(null);
  const pose=useRef(new THREE.Vector3(...location(props)));
  const [lost, setLost] = useState(false); const [generation, setGeneration] = useState(0);
  const onLost = useMemo(() => () => setLost(true), []);
  const retry = () => { setLost(false); setGeneration((n) => n + 1); };
  const flat = <FlatScene props={props} retry={retry} />;
  return <div className={`simulation-scene${props.view === "camera" ? " is-camera" : ""}`}>
    <div className="simulation-scene__webgl">
      {lost ? flat : <SceneBoundary key={generation} fallback={flat}>
        <Canvas frameloop={props.running||props.liveTelemetry?"always":"demand"} shadows dpr={props.quality === "high" ? [1, 1.5] : 1} camera={{ position: [3.5,2.6,4.1], fov: 37, near: 0.01, far: 180 }} gl={{ antialias: true, alpha: false, powerPreference: "high-performance" }} fallback={<span>{t("이 브라우저는 3D 캔버스를 지원하지 않습니다.")}</span>}>
          <SceneWake props={props}/><Lighting onReady={props.onReady} onLost={onLost} environment={props.environment} daylight={props.daylight} camp={!!props.mission} live={!!props.liveTelemetry}/><World source={source} pose={pose} environment={props.environment} mission={props.mission} aircraft={aircraft}/><CameraRig source={source} pose={pose}/><AircraftPresentation source={source} aircraft={aircraft}/><AdaptiveResolution />
          <Guides props={props} guideRef={guides} />{!props.liveTelemetry&&<SensorFeed source={source} guideRef={guides} pose={pose} aircraft={aircraft}/>}
          <RenderEffects quality={props.quality} />
        </Canvas>
      </SceneBoundary>}
    </div>
    {props.view === "camera" && <div className="simulation-scene__crosshair" aria-hidden="true" />}
  </div>;
}
