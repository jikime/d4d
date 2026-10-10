import {t} from '../i18n';
import type { SimulationSceneProps } from "../SimulationScene";
import { sensorFootprint } from "./sceneMath";

export function drawFallback(canvas: HTMLCanvasElement, props: SimulationSceneProps): void {
  const ctx = canvas.getContext("2d"); if (!ctx) return;
  const w = canvas.clientWidth; const h = canvas.clientHeight; const dpr = Math.min(window.devicePixelRatio || 1, 2);
  if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) { canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr); }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.fillStyle = "#d5dcdb"; ctx.fillRect(0, 0, w, h);
  const mission=props.mission;
  const range = Math.max(85, Math.abs(props.frame?.x ?? props.startXcm) + 25, Math.abs(props.frame?.y ?? props.startYcm) + 25,...(props.flight?.route.points.flatMap(p=>[Math.abs(p.x)*100+60,Math.abs(p.y)*100+60])??[]));
  const scale = mission?Math.min(w/((mission.width+6)*100),h/((mission.depth+6)*100)):Math.min(w, h) / (range * 2);
  const X = (x: number) => w / 2 + x * scale; const Y = (y: number) => h / 2 - y * scale;
  ctx.strokeStyle = "rgba(63,80,86,.13)"; ctx.lineWidth = 1;
  for (let t = -range; t <= range; t += props.flight?100:10) { ctx.beginPath(); ctx.moveTo(X(t), 0); ctx.lineTo(X(t), h); ctx.moveTo(0, Y(t)); ctx.lineTo(w, Y(t)); ctx.stroke(); }
  if(mission){
    ctx.strokeStyle='#60704d';ctx.lineWidth=2;ctx.strokeRect(X(-mission.width*50),Y(mission.depth*50),mission.width*100*scale,mission.depth*100*scale);
    for(const z of mission.zones){
      ctx.fillStyle=z.kind==='obstacle'?'#757b65':z.kind==='keepout'?'#bb827088':'#8f9b6a33';ctx.beginPath();
      if(z.shape==='rect')ctx.rect(X((z.x??0)*100),Y(((z.y??0)+(z.h??0))*100),(z.w??0)*100*scale,(z.h??0)*100*scale);
      else if(z.shape==='circle')ctx.arc(X((z.x??0)*100),Y((z.y??0)*100),(z.r??0)*100*scale,0,Math.PI*2);
      else z.points?.forEach(([x,y],i)=>i?ctx.lineTo(X(x*100),Y(y*100)):ctx.moveTo(X(x*100),Y(y*100)));
      ctx.closePath();ctx.fill();
    }
    for(const tag of mission.tags){
      const x=X(tag.x*100),y=Y(tag.y*100),size=Math.max(5,tag.size*100*scale);
      ctx.fillStyle='#fff';ctx.fillRect(x-size*.7,y-size*.7,size*1.4,size*1.4);
      ctx.fillStyle=props.disturbance.tagOccluded&&tag.id===props.missionState?.activeTag?'#827b6c':'#1b2715';ctx.fillRect(x-size/2,y-size/2,size,size);
      if(props.showGuides){ctx.fillStyle='#26371d';ctx.font='11px Pretendard Variable';ctx.fillText(`TAG ${tag.id}`,x+size,y-7);}
    }
  }else{
    ctx.fillStyle = "#f9faf5"; ctx.fillRect(X(-8.5), Y(8.5), 17 * scale, 17 * scale);
    ctx.fillStyle = "#19242a"; ctx.fillRect(X(-6), Y(6), 12 * scale, 12 * scale);
    ctx.fillStyle = "#f9faf5"; ctx.fillRect(X(-3), Y(3), 3 * scale, 3 * scale); ctx.fillRect(X(1), Y(-1), 3 * scale, 3 * scale);
    if (props.disturbance.tagOccluded) { ctx.fillStyle = "#333d40"; ctx.fillRect(X(-10.3), Y(10.3), 20.6 * scale, 20.6 * scale); }
  }
  const x = props.frame?.x ?? props.startXcm; const y = props.frame?.y ?? props.startYcm;
  if (props.showPath) {
    ctx.strokeStyle = "#078b8b"; ctx.lineWidth = 2; ctx.beginPath();
    const trail=props.flight?props.flight.trail.map(p=>({x:p.x*100,y:p.y*100})):props.frames;
    trail.forEach((f, i) => i ? ctx.lineTo(X(f.x), Y(f.y)) : ctx.moveTo(X(f.x), Y(f.y))); ctx.stroke();
    if(props.flight){ctx.strokeStyle='#9d7138';ctx.setLineDash([5,4]);ctx.beginPath();props.flight.route.points.forEach((p,i)=>i?ctx.lineTo(X(p.x*100),Y(p.y*100)):ctx.moveTo(X(p.x*100),Y(p.y*100)));ctx.stroke();ctx.setLineDash([]);}
  }
  if (props.showGuides) {
    const [widthM, heightM] = sensorFootprint(props.heightCm - 11.5);
    ctx.setLineDash([5, 4]); ctx.strokeStyle = "rgba(9,123,139,.6)";
    ctx.strokeRect(X(x - widthM * 50), Y(y + heightM * 50), widthM * 100 * scale, heightM * 100 * scale);
    const target=mission?.tags.find(t=>t.id===props.missionState?.activeTag);
    ctx.setLineDash([]); ctx.beginPath(); ctx.arc(X((target?.x??0)*100), Y((target?.y??0)*100), props.arriveTolCm * scale, 0, Math.PI * 2); ctx.stroke();
  }
  ctx.strokeStyle = "#27363d"; ctx.lineWidth = 5; ctx.beginPath(); ctx.moveTo(X(x) - 19, Y(y) - 19); ctx.lineTo(X(x) + 19, Y(y) + 19); ctx.moveTo(X(x) + 19, Y(y) - 19); ctx.lineTo(X(x) - 19, Y(y) + 19); ctx.stroke();
  for (const sx of [-1, 1]) for (const sy of [-1, 1]) { ctx.beginPath(); ctx.arc(X(x) + sx * 19, Y(y) + sy * 19, 11, 0, Math.PI * 2); ctx.stroke(); }
  ctx.fillStyle = "#d59734"; ctx.fillRect(X(x) - 7, Y(y) - 12, 14, 24);
  ctx.fillStyle = "#30414a"; ctx.font = "12px Pretendard Variable"; ctx.fillText(t(props.flight?`높이 ${(props.heightCm/100).toFixed(1)}m · 좌표 ${(x/100).toFixed(1)} / ${(y/100).toFixed(1)}m`:`높이 ${props.heightCm}cm · 좌표 ${x.toFixed(1)} / ${y.toFixed(1)}cm`), 18, h - 20);
}
