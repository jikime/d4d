import { Component, useEffect, useMemo, useRef, useState, type ReactNode, type RefObject } from "react";
import { Canvas, useFrame, useThree } from "@react-three/fiber";
import { Grid, Html, Line, OrbitControls, Sky } from "@react-three/drei";
import { Bloom, EffectComposer, ToneMapping } from "@react-three/postprocessing";
import { ToneMappingMode } from "postprocessing";
import type { OrbitControls as Controls } from "three-stdlib";
import * as THREE from "three";
import { RoomEnvironment } from "three/addons/environments/RoomEnvironment.js";
import { OutputPass } from "three/addons/postprocessing/OutputPass.js";
import type { Disturbance, SimFrame } from "./approachSim";
import { buildAircraft, buildTestBay, disposeModel } from "./simulation/sceneModels";
import { SENSOR_ASPECT, SENSOR_VERTICAL_FOV, sensorFootprint, worldPosition } from "./simulation/sceneMath";
import { drawFallback } from "./simulation/flatScene";
import { buildDust, buildFieldRange, buildGrass } from "./simulation/sceneTerrain";
import { clamp, damp, ROTOR_DIRECTIONS, rotorWash, visualAttitude, type CameraInput } from "./simulation/flightVisuals";
import "./simulationScene.css";

export type SimulationView = "orbit" | "follow" | "top" | "camera";
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
}
const location = (p: SimulationSceneProps) => worldPosition(p.frame?.x ?? p.startXcm, p.frame?.y ?? p.startYcm, p.heightCm);

function Lighting({ onReady, onLost, environment, daylight }: { onReady?: (ok: boolean) => void; onLost: () => void; environment: SimulationSceneProps['environment']; daylight: SimulationSceneProps['daylight'] }) {
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
    gl.domElement.setAttribute("aria-label", "3D 시험장: 실제 시뮬레이션 좌표의 기체와 태그");
    onReady?.(true);
    return () => { gl.domElement.removeEventListener("webglcontextlost", lost); scene.environment = null; env.dispose(); };
  }, [gl, scene, onReady, onLost]);
  return <>
    <color attach="background" args={[field ? dusk ? '#646a60' : '#aeb7a4' : '#0b1009']} />
    {field && <><fog attach="fog" args={[dusk?'#777968':'#a6af94',18,65]} /><Sky distance={450000} sunPosition={dusk?[-25,3,15]:[-12,18,9]} turbidity={5} rayleigh={1.2} mieCoefficient={0.008} /></>}
    <ambientLight intensity={dusk?0.20:0.35} /><hemisphereLight args={[dusk?'#9fa69e':'#e0e8dc', '#646044', dusk?.85:1.15]} />
    <directionalLight position={field?dusk?[-14,4,9]:[-8,12,6]:[-3,8,5]} color={dusk?'#f2b06a':'#fff0d1'} intensity={dusk?1.5:3.1} castShadow shadow-mapSize={[2048, 2048]} shadow-camera-left={field?-12:-4.5} shadow-camera-right={field?12:4.5} shadow-camera-top={field?12:4.5} shadow-camera-bottom={field?-12:-4.5} shadow-camera-near={0.1} shadow-camera-far={45} shadow-normalBias={0.004} shadow-bias={-0.0001} shadow-radius={3} />
    <directionalLight position={[4, 3, -2]} color="#c1caba" intensity={dusk?.3:.7} />
    {!field && <Grid position={[0, -0.245, 0]} args={[20, 20]} cellSize={0.5} sectionSize={2} cellColor="#293329" sectionColor="#454c37" cellThickness={0.45} sectionThickness={0.8} fadeDistance={26} fadeStrength={1.5} infiniteGrid raycast={() => null} />}
  </>;
}

function World({ source, pose, environment }: { source: RefObject<SimulationSceneProps>; pose: RefObject<THREE.Vector3>; environment: SimulationSceneProps['environment'] }) {
  const [models, setModels] = useState<{ bay: ReturnType<typeof buildTestBay> | ReturnType<typeof buildFieldRange>; drone: ReturnType<typeof buildAircraft>; grass?: ReturnType<typeof buildGrass>; dust?: ReturnType<typeof buildDust> } | null>(null);
  const motion = useRef({ previousTime: -1, power: 0, seconds: 0 });
  useEffect(() => {
    // effect에서 생성·해제해 StrictMode의 시험 마운트에서도 GPU 자원을 회수한다.
    const bay = environment==='field'?buildFieldRange():buildTestBay(); const drone = buildAircraft();
    const grass=environment==='field'?buildGrass():undefined;const dust=environment==='field'?buildDust():undefined;
    if(grass)bay.root.add(grass.mesh);if(dust)bay.root.add(dust.points);
    drone.root.position.set(...location(source.current));pose.current.copy(drone.root.position); setModels({ bay, drone, grass, dust });
    motion.current={previousTime:-1,power:0,seconds:0};
    return () => { disposeModel(bay.root); disposeModel(drone.root); };
  }, [source, pose, environment]);
  useFrame((_, delta) => {
    if (!models) return;
    const p = source.current; const time = p.frame?.tMs ?? 0; const { bay, drone, grass, dust } = models; const m=motion.current;
    const jump=!p.running||m.previousTime<0||time<m.previousTime||time-m.previousTime>250;
    const at=location(p);const blend=jump?1:1-Math.exp(-delta*28);
    pose.current.set(THREE.MathUtils.lerp(pose.current.x,at[0],blend),at[1],THREE.MathUtils.lerp(pose.current.z,at[2],blend));drone.root.position.copy(pose.current);
    const attitude=visualAttitude(p.frame?.out.rollUs??0,p.frame?.out.pitchUs??0);
    drone.root.rotation.x=THREE.MathUtils.lerp(drone.root.rotation.x,attitude.pitch,blend);drone.root.rotation.z=THREE.MathUtils.lerp(drone.root.rotation.z,attitude.roll,blend);
    // 20Hz 제어 계산과 표시 보간을 분리한다. 일시정지/탐색 시 정확한 좌표로 맞춘다.
    const dt=Math.min(delta,.05)*p.playbackSpeed;
    if(jump)m.seconds=time/1000;else m.seconds+=dt;
    if(p.running)m.power=damp(m.power,p.disturbance.kill?0:1,p.disturbance.kill?5:3,dt);
    else m.power=p.disturbance.kill?0:time>0?1:0;
    drone.rotors.forEach((rotor,i)=>{if(p.running){const trim=1+(i<2?-1:1)*(p.frame?.out.rollUs??0)*.001+(i%2?1:-1)*(p.frame?.out.pitchUs??0)*.001;rotor.rotation.y+=dt*ROTOR_DIRECTIONS[i]*(65+m.power*170)*m.power*trim;}});
    drone.blur.forEach((disk)=>{disk.visible=p.running&&m.power>.12;(disk.material as THREE.MeshBasicMaterial).opacity=m.power*.12;});
    drone.gimbal.rotation.x=-drone.root.rotation.x;drone.gimbal.rotation.z=-drone.root.rotation.z;
    drone.navLights.forEach((mat,i)=>{mat.emissiveIntensity=(i%2?.9:1.4)+Math.pow(Math.max(0,Math.sin(m.seconds*3)),16)*1.8;});
    drone.led.emissive.set(p.disturbance.kill ? "#ed6650" : p.frame?.out.state === "HOLD" ? "#edb145" : "#78dc82");
    if(grass){grass.uniforms.time.value=m.seconds;grass.uniforms.power.value=m.power;grass.uniforms.drone.value.copy(pose.current);grass.uniforms.wind.value.set(p.disturbance.windRightCmS2,-p.disturbance.windFwdCmS2);}
    if(dust){
      const wash=rotorWash(at[1],m.power,0);dust.mat.opacity=wash*.20;dust.points.visible=wash>.01;
      for(let i=0;i<160;i++){const phase=(m.seconds*.37+i*.6180339887)%1,angle=i*2.39996+phase*.9,radius=.10+phase*(.8+at[1]*.5);
        dust.positions[i*3]=pose.current.x+Math.cos(angle)*radius+p.disturbance.windRightCmS2*.02*phase;
        dust.positions[i*3+1]=.03+Math.sin(phase*Math.PI)*.21;
        dust.positions[i*3+2]=pose.current.z+Math.sin(angle)*radius-p.disturbance.windFwdCmS2*.02*phase;
      }dust.points.geometry.attributes.position.needsUpdate=true;
    }
    if('windsock' in bay){bay.windsock.rotation.y=Math.atan2(p.disturbance.windFwdCmS2,p.disturbance.windRightCmS2||1);bay.windsock.rotation.z=-.12+Math.sin(m.seconds*3)*.045;}
    bay.room.visible = p.showRoom; bay.cover.visible = p.disturbance.tagOccluded; m.previousTime=time;
  }, -3);
  return models && <><primitive object={models.bay.root} dispose={null} /><primitive object={models.drone.root} dispose={null} /></>;
}

function CameraRig({ source, pose }: { source: RefObject<SimulationSceneProps>; pose: RefObject<THREE.Vector3> }) {
  const { camera, size, gl } = useThree(); const controls = useRef<Controls>(null);
  const [view, setView] = useState(source.current.view); const last = useRef(""); const lastView=useRef(''); const framing = useRef(""); const moving = useRef(true);
  const goal = useMemo(() => new THREE.Vector3(), []); const target = useMemo(() => new THREE.Vector3(), []);
  const reduced = useMemo(() => window.matchMedia("(prefers-reduced-motion: reduce)").matches, []);
  const zoom=useRef({sequence:0,follow:1,camera:1,reported:0,baseline:6.2});
  const deltaVector=useMemo(()=>new THREE.Vector3(),[]);
  const stickVectors=useMemo(()=>({spherical:new THREE.Spherical(),right:new THREE.Vector3(),forward:new THREE.Vector3(),pan:new THREE.Vector3()}),[]);
  useEffect(()=>{
    const applyZoom=(factor:number)=>{
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
    const p = source.current; const perspective = camera as THREE.PerspectiveCamera;
    const viewKey=`${p.view}-${p.resetKey}-${p.environment}`;const key = `${viewKey}-${size.width}-${size.height}`;
    if (key !== last.current) {
      last.current = key;
      if(lastView.current!==viewKey){
      lastView.current=viewKey;moving.current = true; setView(p.view);
      const fit = Math.max(1, 1.4 / (size.width / size.height));
      if (p.view === "orbit") { goal.set(...(p.environment==='field'?[3.5,2.6,4.1]:[4.4,4.2,5.7]) as [number,number,number]).multiplyScalar(fit); target.set(0, 0.52, -0.15); }
      if (p.view === "top") { goal.set(0, 7.4 * fit, 0.001); target.set(0, 0, -0.05); }
      zoom.current.baseline=goal.distanceTo(target);
      perspective.fov = p.view === "camera" ? SENSOR_VERTICAL_FOV : 37;
      zoom.current.follow=1;zoom.current.camera=1;perspective.zoom=1;
      perspective.up.set(0, p.view === "camera" ? 0 : 1, p.view === "camera" ? -1 : 0);
      }
    }
    if(zoom.current.sequence!==p.zoomCommand.sequence){
      zoom.current.sequence=p.zoomCommand.sequence;
      if(p.view==='follow')zoom.current.follow=clamp(zoom.current.follow/p.zoomCommand.factor,.4,3.5);
      else if(p.view==='camera'){zoom.current.camera=clamp(zoom.current.camera*p.zoomCommand.factor,1,4);perspective.zoom=zoom.current.camera;perspective.updateProjectionMatrix();}
      else if(controls.current){deltaVector.copy(camera.position).sub(controls.current.target);const distance=clamp(deltaVector.length()/p.zoomCommand.factor,.55,35);deltaVector.setLength(distance);camera.position.copy(controls.current.target).add(deltaVector);moving.current=false;controls.current.update();}
    }
    const frameKey = `${key}-${p.dashboardVisible}-${p.panelVisible}`;
    if (frameKey !== framing.current) {
      framing.current = frameKey;
      const hud = p.dashboardVisible && p.panelVisible && size.width > 900 && p.view !== "camera";
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
    if (p.view === "camera") { camera.position.set(at[0], at[1] - 0.115, at[2] - 0.026); camera.lookAt(at[0], 0, at[2] - 0.026); return; }
    if (p.view === "follow") { target.set(at[0], at[1] - 0.03, at[2]); goal.copy(target).add(deltaVector.set(.85,.46,-1.03).multiplyScalar(zoom.current.follow)); moving.current = true; }
    if (moving.current && controls.current) {
      const blend = reduced ? 1 : 1 - Math.exp(-Math.min(delta, 0.1) * 5);
      camera.position.lerp(goal, blend); controls.current.target.lerp(target, blend); controls.current.update();
      if (p.view !== "follow" && camera.position.distanceTo(goal) < 0.006) moving.current = false;
    }
  }, -2);
  return <OrbitControls ref={controls} makeDefault enabled={view === "orbit" || view === "top"} enableRotate={view === "orbit"} enableDamping={!reduced} dampingFactor={0.08} minDistance={0.55} maxDistance={35} maxPolarAngle={Math.PI * 0.485} zoomSpeed={0.65} onStart={() => { moving.current = false; }} />;
}

function Guides({ props: p, guideRef }: { props: SimulationSceneProps; guideRef: RefObject<THREE.Group | null> }) {
  const { gl } = useThree(); const portal = useMemo(() => ({ current: gl.domElement.parentElement! }), [gl]);
  const at = location(p); const [width, height] = sensorFootprint(p.heightCm - 11.5);
  const trail = useMemo(() => p.frames.slice(-900).map((f) => worldPosition(f.x, f.y, 1.3)), [p.frame?.tMs, p.resetKey, p.frames]);
  const corners: [number, number, number][] = [[at[0]-width/2,0.018,at[2]-height/2], [at[0]+width/2,0.018,at[2]-height/2], [at[0]+width/2,0.018,at[2]+height/2], [at[0]-width/2,0.018,at[2]+height/2]];
  return <group ref={guideRef}>
    {p.showPath && p.view !== "camera" && trail.length > 1 && <Line points={trail} color="#298a79" lineWidth={2.5} />}
    {p.showGuides && p.view !== "camera" && <>
      <Line points={[...corners, corners[0]]} color="#378e89" transparent opacity={0.65} lineWidth={1} />
      <Line points={[[at[0], 0.018, at[2]], at]} color="#477a78" dashed dashSize={0.04} gapSize={0.025} lineWidth={1} />
      <Line points={[[0, 0.02, 0], [at[0], 0.02, at[2]]]} color="#be943e" dashed dashSize={0.025} gapSize={0.018} lineWidth={1.5} />
      <mesh rotation-x={-Math.PI/2} position-y={0.012}><ringGeometry args={[Math.max(0.001, p.arriveTolCm/100 - 0.004), p.arriveTolCm/100, 64]} /><meshBasicMaterial color="#739b3d" side={THREE.DoubleSide} /></mesh>
      {p.dashboardVisible && !p.compactDisplay && <>
        <Html position={[at[0] + 0.22, at[1] + 0.07, at[2]]} portal={portal} zIndexRange={[12, 1]} center style={{ pointerEvents: "none" }}><div className="scene-pin scene-pin--drone"><i />SPIDER 01<small>고도 {p.heightCm.toFixed(0)} cm</small></div></Html>
        <Html position={[0.22, 0.04, 0.12]} portal={portal} zIndexRange={[11, 1]} center style={{ pointerEvents: "none" }}><div className="scene-pin scene-pin--tag"><i />착륙 기준 태그<small>{p.disturbance.tagOccluded ? "태그 가림" : "기준점 0, 0"}</small></div></Html>
      </>}
    </>}
  </group>;
}

function SensorFeed({ source, guideRef, pose }: { source: RefObject<SimulationSceneProps>; guideRef: RefObject<THREE.Group | null>; pose: RefObject<THREE.Vector3> }) {
  const { gl, scene } = useThree();
  const r = useMemo(() => ({ camera: new THREE.PerspectiveCamera(SENSOR_VERTICAL_FOV, SENSOR_ASPECT, 0.008, 30), target: new THREE.WebGLRenderTarget(320, 240, { type: THREE.HalfFloatType }), output: new THREE.WebGLRenderTarget(320, 240), pass: new OutputPass(), pixels: new Uint8Array(320 * 240 * 4), flipped: new Uint8ClampedArray(320 * 240 * 4), viewport: new THREE.Vector4(), scissor: new THREE.Vector4(), last: -1 }), []);
  useEffect(() => { r.camera.up.set(0, 0, -1); return () => { r.target.dispose(); r.output.dispose(); r.pass.dispose(); }; }, [r]);
  useFrame(({ clock }) => {
    const p = source.current; const canvas = p.sensorCanvas.current;
    if (!p.dashboardVisible || !p.panelVisible || !canvas?.offsetParent || clock.elapsedTime - r.last < 0.12) return;
    r.last = clock.elapsedTime;
    const at = pose.current.toArray(); r.camera.position.set(at[0], at[1] - 0.115, at[2] - 0.026); r.camera.lookAt(at[0], 0, at[2] - 0.026);
    const oldTarget = gl.getRenderTarget(); const oldTone = gl.toneMapping; const oldScissor = gl.getScissorTest();
    gl.getViewport(r.viewport); gl.getScissor(r.scissor); const visible = guideRef.current?.visible;
    if (guideRef.current) guideRef.current.visible = false;
    try {
      gl.toneMapping = THREE.ACESFilmicToneMapping; gl.setScissorTest(false); gl.setRenderTarget(r.target); gl.clear(); gl.render(scene, r.camera);
      // 선형 HDR 장면을 화면과 같은 ACES/sRGB로 변환한 뒤 캔버스에 복사한다.
      r.pass.render(gl, r.output, r.target, 0, false);
      gl.readRenderTargetPixels(r.output, 0, 0, 320, 240, r.pixels);
      for (let row = 0; row < 240; row++) r.flipped.set(r.pixels.subarray(row * 1280, (row + 1) * 1280), (239 - row) * 1280);
      canvas.getContext("2d")?.putImageData(new ImageData(new Uint8ClampedArray(r.flipped), 320, 240), 0, 0);
    } finally {
      gl.setRenderTarget(oldTarget); gl.toneMapping = oldTone; gl.setViewport(r.viewport); gl.setScissor(r.scissor); gl.setScissorTest(oldScissor);
      if (guideRef.current) guideRef.current.visible = visible ?? true;
    }
  });
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
  const canvas = useRef<HTMLCanvasElement>(null);
  useEffect(() => { props.onReady?.(false); }, [props.onReady]);
  useEffect(() => {
    if (canvas.current) drawFallback(canvas.current, props);
    const observer = new ResizeObserver(() => { if (canvas.current) drawFallback(canvas.current, props); });
    if (canvas.current) observer.observe(canvas.current); return () => observer.disconnect();
  }, [props]);
  return <div className="simulation-scene__flat"><canvas ref={canvas} aria-label="3D 대신 동일한 시뮬레이션 좌표로 그린 평면도" /><div className="simulation-scene__fallback-note"><b>3D 화면을 사용할 수 없어요</b><span>같은 좌표의 평면도로 표시합니다.</span><button type="button" onClick={retry}>3D 다시 시도</button></div></div>;
}
class SceneBoundary extends Component<{ children: ReactNode; fallback: ReactNode }, { failed: boolean }> {
  state = { failed: false };
  static getDerivedStateFromError() { return { failed: true }; }
  render() { return this.state.failed ? this.props.fallback : this.props.children; }
}

export default function SimulationScene(props: SimulationSceneProps) {
  const source = useRef(props); source.current = props; const guides = useRef<THREE.Group | null>(null);
  const pose=useRef(new THREE.Vector3(...location(props)));
  const [lost, setLost] = useState(false); const [generation, setGeneration] = useState(0);
  const onLost = useMemo(() => () => setLost(true), []);
  const retry = () => { setLost(false); setGeneration((n) => n + 1); };
  const flat = <FlatScene props={props} retry={retry} />;
  return <div className={`simulation-scene${props.view === "camera" ? " is-camera" : ""}`}>
    <div className="simulation-scene__webgl">
      {lost ? flat : <SceneBoundary key={generation} fallback={flat}>
        <Canvas shadows dpr={props.quality === "high" ? [1, 1.5] : 1} camera={{ position: [3.5,2.6,4.1], fov: 37, near: 0.01, far: 180 }} gl={{ antialias: true, alpha: false, powerPreference: "high-performance" }} fallback={<span>이 브라우저는 3D 캔버스를 지원하지 않습니다.</span>}>
          <Lighting onReady={props.onReady} onLost={onLost} environment={props.environment} daylight={props.daylight} /><World source={source} pose={pose} environment={props.environment} /><CameraRig source={source} pose={pose} /><AdaptiveResolution />
          <Guides props={props} guideRef={guides} /><SensorFeed source={source} guideRef={guides} pose={pose} />
          <RenderEffects quality={props.quality} />
        </Canvas>
      </SceneBoundary>}
    </div>
    {props.view === "camera" && <div className="simulation-scene__crosshair" aria-hidden="true" />}
  </div>;
}
