import * as THREE from "three";
import { RoundedBoxGeometry } from "three/addons/geometries/RoundedBoxGeometry.js";
import { mergeGeometries } from "three/addons/utils/BufferGeometryUtils.js";
import {t, subscribeLanguage} from '../i18n';

type V3 = [number, number, number];
export const material = (color: string, roughness = 0.65, metalness = 0) => new THREE.MeshStandardMaterial({ color, roughness, metalness });

export function add(parent: THREE.Object3D, geometry: THREE.BufferGeometry, mat: THREE.Material, position: V3 = [0, 0, 0]) {
  const mesh = new THREE.Mesh(geometry, mat);
  mesh.position.set(...position); mesh.castShadow = true; mesh.receiveShadow = true;
  parent.add(mesh); return mesh;
}
export function box(parent: THREE.Object3D, size: V3, position: V3, mat: THREE.Material, radius = 0) {
  return add(parent, radius ? new RoundedBoxGeometry(...size, 2, radius) : new THREE.BoxGeometry(...size), mat, position);
}
export function cylinder(parent: THREE.Object3D, radius: number, length: number, position: V3, mat: THREE.Material, sides = 24) {
  return add(parent, new THREE.CylinderGeometry(radius, radius, length, sides), mat, position);
}
export function tube(parent: THREE.Object3D, points: V3[], radius: number, mat: THREE.Material) {
  return add(parent, new THREE.TubeGeometry(new THREE.CatmullRomCurve3(points.map((p) => new THREE.Vector3(...p))), 20, radius, 6, false), mat);
}
export function texture(width: number, height: number, paint: (ctx: CanvasRenderingContext2D) => void, usesFont=false) {
  const canvas = document.createElement("canvas"); canvas.width = width; canvas.height = height;
  const ctx = canvas.getContext("2d"); if (!ctx) throw new Error("2D 텍스처 생성 실패");
  paint(ctx);
  const result = new THREE.CanvasTexture(canvas); result.colorSpace = THREE.SRGBColorSpace; result.anisotropy = 4;
  if(usesFont){
    let disposed=false;
    const repaint=()=>{if(!disposed){paint(ctx);result.needsUpdate=true;}};
    const unsubscribe=subscribeLanguage(repaint);
    result.addEventListener('dispose',()=>{disposed=true;unsubscribe();});
    void document.fonts.ready.then(repaint);
  }
  return result;
}
export function sign(parent: THREE.Object3D, width: number, height: number, position: V3, title: string, subtitle: string, dark = false) {
  const map = texture(1024, 256, (ctx) => {
    ctx.fillStyle = dark ? "#1b2c35" : "#e9ece4"; ctx.fillRect(0, 0, 1024, 256);
    ctx.fillStyle = "#a3d44a"; ctx.fillRect(38, 40, 10, 173);
    ctx.fillStyle = dark ? "#f5f6ee" : "#273d45"; ctx.font = "700 75px Pretendard Variable"; ctx.fillText(t(title), 78, 129, 890);
    ctx.fillStyle = dark ? "#a8b8b5" : "#657a79"; ctx.font = "23px Pretendard Variable"; ctx.fillText(t(subtitle), 82, 187, 880);
  },true);
  const panel = add(parent, new THREE.PlaneGeometry(width, height), new THREE.MeshStandardMaterial({ map, roughness: 0.82 }), position);
  panel.castShadow = false; return panel;
}

/** 움직이지 않는 시험장 메시를 재질별로 병합해 드로 콜을 줄인다. */
export function batchStatic(source: THREE.Group) {
  source.updateMatrixWorld(true);
  const buckets = new Map<THREE.Material, THREE.BufferGeometry[]>();
  source.traverse((node) => {
    if (!(node instanceof THREE.Mesh) || Array.isArray(node.material)) return;
    const geometry = (node.geometry.index ? node.geometry.toNonIndexed() : node.geometry.clone()).applyMatrix4(node.matrixWorld);
    const list = buckets.get(node.material) ?? []; list.push(geometry); buckets.set(node.material, list);
    node.geometry.dispose();
  });
  const group = new THREE.Group();
  for (const [mat, geometries] of buckets) {
    const merged = mergeGeometries(geometries);
    if (merged) add(group, merged, mat);
    geometries.forEach((geometry) => geometry.dispose());
  }
  return group;
}

/** 실제 기체 관측용 기준 바닥. 실제 지형이나 태그 위치로 오인할 장식은 두지 않는다. */
export function buildTelemetryBay() {
  const root=new THREE.Group(),room=new THREE.Group(),cover=new THREE.Group();
  const ground=material('#20291e',.95),line=material('#45523a',.85),edge=material('#a7bc7b',.75);
  box(root,[8,.05,8],[0,-.055,0],ground);
  for(let n=-8;n<=8;n++){
    box(root,[8,.002,.006],[0,-.028,n*.5],line);
    box(root,[.006,.002,8],[n*.5,-.028,0],line);
  }
  box(root,[.45,.004,.012],[0,-.023,0],edge);
  box(root,[.012,.004,.45],[0,-.023,0],edge);
  root.add(room,cover);cover.visible=false;
  return {root,room,cover};
}

export function buildTestBay() {
  const base = new THREE.Group(); const architecture = new THREE.Group();
  const chalk = material("#e6e8dc", 0.82); const edge = material("#647a77", 0.52, 0.35);
  const steel = material("#a2ada6", 0.3, 0.75); const navy = material("#243b41", 0.65, 0.3);
  const trim = material("#c9d3c7", 0.75); const dark = material("#263537", 0.7); const lime = material("#adc95d", 0.7);
  const amber = material("#dfaf4e", 0.72);
  box(base, [6.6, 0.22, 5.7], [0, -0.125, -0.25], edge, 0.06);
  box(base, [6.48, 0.035, 5.58], [0, -0.018, -0.25], chalk, 0.02);
  box(base, [6.1, 0.017, 0.013], [0, -0.12, 2.607], lime);
  const floorMap = texture(1536, 1536, (ctx) => {
    ctx.fillStyle = "#cbd0c5"; ctx.fillRect(0, 0, 1536, 1536);
    let seed = 7129;
    for (let i = 0; i < 22000; i++) {
      seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0;
      const x = seed % 1536; seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0;
      ctx.fillStyle = i % 2 ? "#f4f4e915" : "#374b4012"; ctx.fillRect(x, seed % 1536, 2, 2);
    }
    ctx.strokeStyle = "#b8c2b2"; ctx.lineWidth = 1.5;
    for (let i = 0; i <= 1536; i += 128) { ctx.beginPath(); ctx.moveTo(i, 0); ctx.lineTo(i, 1536); ctx.moveTo(0, i); ctx.lineTo(1536, i); ctx.stroke(); }
  });
  const floor = add(base, new THREE.PlaneGeometry(6.35, 5.46), new THREE.MeshStandardMaterial({ map: floorMap, roughness: 0.93 }), [0, 0.001, -0.25]); floor.rotation.x = -Math.PI / 2;

  // 실제 센서 좌표를 읽기 좋은, 지면과 거의 같은 높이의 시험 구획.
  const mat = add(base, new THREE.PlaneGeometry(2.65, 2.65), material("#bbc7b9", 0.96), [0, 0.003, 0]); mat.rotation.x = -Math.PI / 2;
  for (const x of [-1.3, 1.3]) for (const z of [-1.3, 1.3]) {
    box(base, [0.28, 0.002, 0.02], [x - Math.sign(x) * 0.13, 0.006, z], amber);
    box(base, [0.02, 0.002, 0.28], [x, 0.006, z - Math.sign(z) * 0.13], amber);
  }
  for (let i = -10; i <= 10; i++) {
    box(base, [i % 5 === 0 ? 0.06 : 0.03, 0.002, 0.006], [-1.40, 0.007, i / 10], edge);
    box(base, [0.006, 0.002, i % 5 === 0 ? 0.06 : 0.03], [i / 10, 0.007, 1.4], edge);
  }
  const floorSign = sign(base, 1.45, 0.36, [-0.5, 0.008, 1.75], "FLIGHT AREA  01", "PRECISION APPROACH / 1 : 1"); floorSign.rotation.x = -Math.PI / 2;

  // 지붕과 앞벽을 덜어낸 시험실: 어느 방향에서든 기체·태그·궤적을 관찰할 수 있다.
  box(architecture, [6.4, 2.2, 0.12], [0, 1.1, -3], chalk);
  box(architecture, [6.45, 0.10, 0.20], [0, 2.21, -3], trim, 0.015);
  box(architecture, [6.4, 0.16, 0.035], [0, 0.10, -2.925], edge);
  for (const x of [-3.14, -1.7, 0, 1.7, 3.14]) box(architecture, [0.042, 2.15, 0.07], [x, 1.1, -2.914], trim);
  box(architecture, [0.12, 1.05, 4.3], [-3.15, 0.525, -0.8], chalk);
  box(architecture, [0.16, 0.055, 4.3], [-3.15, 1.06, -0.8], edge);
  sign(architecture, 2.2, 0.55, [-0.75, 1.58, -2.924], "MANGO / FLIGHT LAB", "AUTONOMOUS SYSTEMS  —  TEST FACILITY 04", true);
  sign(architecture, 0.85, 0.21, [2.25, 1.98, -2.914], "04 / ACCESS", "SIMULATION ONLY");
  box(architecture, [1.06, 1.82, 0.065], [2.25, 0.93, -2.902], edge, 0.025);
  box(architecture, [0.87, 1.68, 0.022], [2.25, 0.91, -2.85], navy, 0.01);
  box(architecture, [0.6, 0.60, 0.025], [2.25, 1.22, -2.834], material("#8daaa3", 0.22, 0.4));
  box(architecture, [0.02, 0.18, 0.06], [2.57, 0.89, -2.8], steel, 0.005);
  const light = new THREE.MeshStandardMaterial({ color: "#ffffe8", emissive: "#ffffcf", emissiveIntensity: 2.5, roughness: 0.4 });
  for (const x of [-2.0, 0.65]) { box(architecture, [1.6, 0.06, 0.13], [x, 2.12, -2.76], edge); box(architecture, [1.5, 0.01, 0.075], [x, 2.084, -2.746], light); }

  // 계측대: 모니터, 데이터 로거, 전원 장치, 케이블, 보관 케이스.
  box(architecture, [2.4, 0.075, 0.67], [-0.75, 0.78, -2.36], trim, 0.012);
  for (const x of [-1.85, 0.35]) for (const z of [-2.6, -2.11]) box(architecture, [0.055, 0.73, 0.055], [x, 0.37, z], edge);
  for (const x of [-1.3, -0.25]) {
    box(architecture, [0.38, 0.022, 0.19], [x, 0.83, -2.36], dark, 0.012);
    box(architecture, [0.055, 0.19, 0.055], [x, 0.91, -2.48], edge);
    box(architecture, [0.70, 0.43, 0.045], [x, 1.12, -2.48], dark, 0.018);
    const screenMap = texture(512, 300, (ctx) => {
      ctx.fillStyle = "#132c34"; ctx.fillRect(0, 0, 512, 300); ctx.fillStyle = "#c7e8c0"; ctx.font = "20px Pretendard Variable"; ctx.fillText("MANGO / TELEMETRY", 20, 36);
      ctx.strokeStyle = "#274f59"; ctx.lineWidth = 1;
      for (let y = 70; y < 300; y += 40) { ctx.beginPath(); ctx.moveTo(18, y); ctx.lineTo(490, y); ctx.stroke(); }
      for (const [offset, color] of [[0, "#9eca72"], [45, "#67bdd0"]] as const) { ctx.strokeStyle = color; ctx.lineWidth = 3; ctx.beginPath(); for (let i = 0; i < 475; i++) { const y = 158 + offset + Math.sin(i * 0.055) * Math.exp(-i / 230) * 50; if (i) ctx.lineTo(i + 18, y); else ctx.moveTo(i + 18, y); } ctx.stroke(); }
    });
    add(architecture, new THREE.PlaneGeometry(0.655, 0.375), new THREE.MeshStandardMaterial({ map: screenMap, emissiveMap: screenMap, emissive: "#ffffff", emissiveIntensity: 0.35 }), [x, 1.12, -2.453]);
    box(architecture, [0.36, 0.018, 0.12], [x, 0.837, -2.18], dark, 0.004);
  }
  box(architecture, [0.3, 0.16, 0.27], [0.27, 0.9, -2.33], chalk, 0.02);
  for (let i = 0; i < 3; i++) cylinder(architecture, 0.018, 0.02, [0.2 + i * 0.06, 0.915, -2.185], dark).rotation.x = Math.PI / 2;
  tube(architecture, [[-0.15, 0.83, -2.3], [0.1, 0.75, -2.7], [0.28, 0.1, -2.7]], 0.008, dark);
  for (const x of [-2.7, -2.14]) {
    box(architecture, [0.47, 0.27, 0.39], [x, 0.15, -2.45], navy, 0.045);
    box(architecture, [0.49, 0.03, 0.41], [x, 0.28, -2.45], dark, 0.015);
    box(architecture, [0.18, 0.025, 0.025], [x, 0.33, -2.45], steel, 0.008);
    for (const sx of [-0.16, 0.16]) box(architecture, [0.035, 0.07, 0.022], [x + sx, 0.25, -2.24], steel, 0.004);
  }
  // 공간과 높이 감각을 주는 고정 센서 삼각대.
  const tripod = new THREE.Group(); tripod.position.set(2.15, 0, -1.18); architecture.add(tripod);
  cylinder(tripod, 0.016, 1.1, [0, 0.65, 0], steel);
  for (let i = 0; i < 3; i++) { const a = i * Math.PI * 2 / 3; tube(tripod, [[0, 0.65, 0], [Math.cos(a) * 0.24, 0.025, Math.sin(a) * 0.24]], 0.012, dark); }
  box(tripod, [0.13, 0.08, 0.08], [0, 1.23, 0], navy, 0.02);
  cylinder(tripod, 0.026, 0.055, [-0.08, 1.23, 0], dark).rotation.z = Math.PI / 2;
  // 난간은 시뮬레이션 영역 바깥에만 둔다.
  for (const z of [-1.85, -0.5, 0.85, 2.1]) {
    cylinder(architecture, 0.025, 0.75, [3.05, 0.375, z], steel);
    cylinder(architecture, 0.075, 0.018, [3.05, 0.012, z], edge);
  }
  tube(architecture, [[3.05, 0.76, -1.85], [3.05, 0.76, 2.1]], 0.021, edge);
  tube(architecture, [[3.05, 0.35, -1.85], [3.05, 0.35, 2.1]], 0.006, steel);

  addTagMarker(base);
  const root = batchStatic(base); root.name = "spider-test-bay";
  const room = batchStatic(architecture); room.name = "test-bay-architecture"; root.add(room);
  const cover = box(root, [0.21, 0.012, 0.21], [0, 0.017, 0], dark, 0.006); cover.name = "tag-occluder"; cover.visible = false;
  return { root, room, cover };
}

export function addTagMarker(base: THREE.Group) {
  // 현재의 시각화용 태그 패턴을 유지한다. 실물 AprilTag 내보내기 기능은 별도다.
  const bits = [1,0,1,1,0,1,0,1,1,0,0,1,1,1,0,1,1,0,0,1,1,0,1,0,1,0,1,1,1,0,0,1,0,1,1,0];
  const tagMap = texture(512, 512, (ctx) => {
    ctx.fillStyle = "#fffff7"; ctx.fillRect(0, 0, 512, 512); ctx.fillStyle = "#0c1518"; ctx.fillRect(56, 56, 400, 400);
    ctx.fillStyle = "#fffff7"; bits.forEach((bit, i) => { if (bit) ctx.fillRect(106 + i % 6 * 50, 106 + Math.floor(i / 6) * 50, 50, 50); });
  });
  tagMap.magFilter = THREE.NearestFilter;
  const tag = add(base, new THREE.PlaneGeometry(0.17, 0.17), new THREE.MeshStandardMaterial({ map: tagMap, roughness: 1 }), [0, 0.009, 0]); tag.rotation.x = -Math.PI / 2;
}

export function buildAircraft() {
  const root = new THREE.Group(); root.name = "spider-aircraft";
  const weave = texture(128, 128, (ctx) => {
    ctx.fillStyle = "#202b30"; ctx.fillRect(0, 0, 128, 128);
    for (let x = 0; x < 128; x += 8) for (let y = 0; y < 128; y += 8) {
      ctx.fillStyle = (x + y) % 16 ? "#39454b" : "#1d292f"; ctx.fillRect(x, y, 8, 8);
      ctx.fillStyle = "#53636b44"; ctx.fillRect(x, y, 1, 8);
    }
  });
  weave.wrapS = weave.wrapT = THREE.RepeatWrapping; weave.repeat.set(3, 3);
  const carbon = new THREE.MeshPhysicalMaterial({ map: weave, roughness: 0.38, metalness: 0.45, clearcoat: 0.4 });
  const black = material("#141813", 0.45, 0.4); const silver = material("#939c89", 0.25, 0.8);
  const copper = material("#b97b45", 0.27, 0.9); const white = material("#687354", 0.54, 0.18);
  const lime = material("#afbc83", 0.4, 0.15); const pcb = material("#374b2b", 0.57, 0.2);
  const rubber = material("#25343b", 0.9); const led = new THREE.MeshStandardMaterial({ color: "#b5e987", emissive: "#78dc82", emissiveIntensity: 3 });
  box(root, [0.14, 0.012, 0.20], [0, -0.014, 0], carbon, 0.016);
  box(root, [0.12, 0.010, 0.17], [0, 0.032, 0], carbon, 0.014);
  box(root, [0.08, 0.013, 0.086], [0, 0.008, 0.018], pcb, 0.004);
  for (const x of [-0.05, 0.05]) for (const z of [-0.066, 0.066]) {
    cylinder(root, 0.004, 0.04, [x, 0.005, z], silver, 8);
    cylinder(root, 0.0048, 0.003, [x, 0.039, z], black, 6);
  }
  box(root, [0.088, 0.047, 0.14], [0, 0.063, 0.012], white, 0.012);
  for (const z of [-0.031, 0.052]) box(root, [0.096, 0.006, 0.015], [0, 0.089, z], black, 0.003);
  const sticker = sign(root, 0.060, 0.025, [0, 0.091, 0.01], "MANGO", "04 / 4S"); sticker.rotation.x = -Math.PI / 2;
  box(root, [0.108, 0.025, 0.035], [0, 0.034, -0.075], lime, 0.01);
  const navGlass=new THREE.MeshPhysicalMaterial({color:'#18342c',roughness:.09,metalness:.65,clearcoat:1});
  for (const x of [-0.025, 0.025]) {
    cylinder(root, 0.013, 0.018, [x, 0.035, -0.098], black).rotation.x = Math.PI / 2;
    cylinder(root,.009,.003,[x,.035,-.108],navGlass).rotation.x=Math.PI/2;
    cylinder(root,.0035,.003,[x,.035,-.110],black).rotation.x=Math.PI/2;
  }
  tube(root, [[0.043, 0.065, 0.075], [0.06, 0.08, 0.095], [0.04, 0.008, 0.06]], 0.003, material("#da6c49"));
  tube(root, [[-0.04, 0.02, 0.07], [-0.043, 0.12, 0.105], [-0.043, 0.17, 0.12]], 0.002, black);
  cylinder(root, 0.007, 0.025, [-0.043, 0.17, 0.12], rubber);

  // 방진 외피, 방열 홈, 퀵 릴리스 배터리와 전방 짐벌의 독립 기구부.
  box(root, [0.115, 0.035, 0.19], [0, 0.035, -0.006], white, 0.012);
  for (const x of [-0.059, 0.059]) {
    for (let i=0; i<7; i++) box(root,[0.004,0.011,0.002],[x,0.037,-0.025+i*0.008],black,0.0005);
    box(root,[0.004,0.018,0.019],[x,0.07,0.055],black,0.002);
    for (const z of [-0.075,0.072]) {
      const screw=cylinder(root,0.003,0.002,[x*0.85,0.055,z],silver,6); screw.rotation.y=Math.PI/6;
    }
  }
  for (let i=0;i<4;i++) box(root,[0.006,0.001,0.002],[0.029,0.088,0.038+i*0.005],i<3?led:black,0.0005);
  const gimbal = new THREE.Group(); gimbal.name="stabilized-camera-gimbal"; gimbal.position.set(0,-0.034,-0.118); root.add(gimbal);
  box(gimbal,[0.069,0.012,0.04],[0,0,0],black,0.004);
  for(const x of [-0.03,0.03]) box(gimbal,[0.007,0.046,0.034],[x,-0.024,0],silver,0.003);
  const payload = new THREE.Group(); payload.position.y=-0.032; gimbal.add(payload);
  box(payload,[0.047,0.041,0.038],[0,0,0],black,0.01);
  const glass=new THREE.MeshPhysicalMaterial({color:'#132323',roughness:0.07,metalness:0.6,clearcoat:1});
  for(const [x,r] of [[-0.01,0.010],[0.015,0.006]]) {
    cylinder(payload,r+0.003,0.006,[x,0,-0.021],silver).rotation.x=Math.PI/2;
    cylinder(payload,r,0.007,[x,0,-0.024],glass).rotation.x=Math.PI/2;
  }
  const navLights: THREE.MeshStandardMaterial[]=[];

  const rotors: THREE.Group[] = []; const blur: THREE.Mesh[] = [];
  for (const sx of [-1, 1]) for (const sz of [-1, 1]) {
    const x = sx * 0.15; const z = sz * 0.145;
    const arm = box(root, [0.026, 0.014, Math.hypot(x, z)], [x / 2, -0.012, z / 2], carbon, 0.004); arm.rotation.y = Math.atan2(x, z);
    tube(root, [[sx * 0.05, -0.002, sz * 0.045], [x * 0.7, -0.001, z * 0.7], [x, 0, z]], 0.0018, copper);
    cylinder(root, 0.022, 0.028, [x, 0.008, z], black);
    cylinder(root, 0.0185, 0.014, [x, 0.01, z], copper);
    for (const y of [0, 0.022]) cylinder(root, 0.0225, 0.004, [x, y, z], silver);
    for (let i = 0; i < 8; i++) { const a = i * Math.PI / 4; box(root, [0.006, 0.022, 0.004], [x + Math.cos(a) * 0.019, 0.009, z + Math.sin(a) * 0.019], black); }
    const rotor = new THREE.Group(); rotor.name=`rotor-${rotors.length}`; rotor.position.set(x, 0.028, z); root.add(rotor); rotors.push(rotor);
    cylinder(rotor, 0.009, 0.008, [0, 0.005, 0], silver, 12);
    cylinder(rotor, 0.0038, 0.003, [0,0.01,0],black,6);
    const hinge=box(root,[0.033,0.023,0.028],[sx*0.053,-0.012,sz*0.063],white,0.006); hinge.rotation.y=sx*sz*0.5;
    const shape = new THREE.Shape(); shape.moveTo(0.004, -0.004); shape.bezierCurveTo(0.038, -0.019, 0.083, -0.011, 0.091, 0.006); shape.bezierCurveTo(0.072, 0.015, 0.025, 0.008, 0.004, 0.004); shape.closePath();
    const bladeGeometry = new THREE.ExtrudeGeometry(shape, { depth: 0.0015, bevelEnabled: false });
    for (let i = 0; i < 2; i++) { const pivot = new THREE.Group(); pivot.rotation.y = i * Math.PI; rotor.add(pivot); const blade = add(pivot, bladeGeometry, sz < 0 ? lime : black); blade.rotation.x = -Math.PI / 2; }
    const disk = add(root, new THREE.CircleGeometry(0.091, 48), new THREE.MeshBasicMaterial({ color: sz < 0 ? "#d2e4a5" : "#9eb2bb", transparent: true, opacity: 0.16, side: THREE.DoubleSide, depthWrite: false }), [x, 0.028, z]); disk.rotation.x = -Math.PI / 2; disk.castShadow = false; disk.receiveShadow = false; blur.push(disk);
    tube(root, [[x * 0.6, -0.025, z * 0.55], [x * 0.72, -0.092, z * 0.67], [x * 0.72, -0.105, z * 0.9]], 0.006, black);
    box(root, [0.019, 0.010, 0.035], [x * 0.72, -0.105, z * 0.9], rubber, 0.005);
    const nav=new THREE.MeshStandardMaterial({color:sz<0?'#b75a42':'#819a61',emissive:sz<0?'#e13f25':'#84c87a',emissiveIntensity:1.8}); navLights.push(nav);
    box(root, [0.016, 0.005, 0.009], [x * 0.80, -0.025, z * 0.80], nav, 0.002);
  }
  // 아래쪽 카메라와 MTF 시각 모델. 렌즈 아래에서 가상 영상을 렌더링한다.
  box(root, [0.053, 0.020, 0.042], [0, -0.041, -0.022], pcb, 0.005);
  box(root, [0.025, 0.026, 0.025], [0, -0.061, -0.026], black, 0.004);
  cylinder(root, 0.010, 0.014, [0, -0.080, -0.026], silver);
  cylinder(root, 0.0078, 0.002, [0, -0.088, -0.026], new THREE.MeshPhysicalMaterial({ color: "#0d2e3d", roughness: 0.07, metalness: 0.5, clearcoat: 1 }));
  box(root, [0.033, 0.015, 0.025], [0, -0.036, 0.045], white, 0.004);
  const fixed=new THREE.Group();
  for(const part of [...root.children])if(part!==gimbal&&!rotors.includes(part as THREE.Group)&&!blur.includes(part as THREE.Mesh))fixed.add(part);
  root.add(batchStatic(fixed));
  return { root, rotors, blur, led, gimbal, navLights };
}

export function disposeModel(root: THREE.Object3D) {
  const geometries = new Set<THREE.BufferGeometry>(); const materials = new Set<THREE.Material>(); const textures = new Set<THREE.Texture>();
  root.traverse((node) => {
    if (!(node instanceof THREE.Mesh) && !(node instanceof THREE.Points)) return;
    geometries.add(node.geometry);
    if(node instanceof THREE.InstancedMesh)node.dispose();
    for (const mat of Array.isArray(node.material) ? node.material : [node.material]) {
      materials.add(mat); for (const value of Object.values(mat)) if (value instanceof THREE.Texture) textures.add(value);
    }
  });
  geometries.forEach((geometry) => geometry.dispose()); materials.forEach((mat) => mat.dispose()); textures.forEach((map) => map.dispose());
}
