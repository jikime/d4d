import type { SimulationSceneProps } from "../SimulationScene";
import { sensorFootprint } from "./sceneMath";

export function drawFallback(canvas: HTMLCanvasElement, props: SimulationSceneProps): void {
  const ctx = canvas.getContext("2d"); if (!ctx) return;
  const w = canvas.clientWidth; const h = canvas.clientHeight; const dpr = Math.min(window.devicePixelRatio || 1, 2);
  if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) { canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr); }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.fillStyle = "#d5dcdb"; ctx.fillRect(0, 0, w, h);
  const range = Math.max(85, Math.abs(props.frame?.x ?? props.startXcm) + 25, Math.abs(props.frame?.y ?? props.startYcm) + 25,...(props.flight?.route.points.flatMap(p=>[Math.abs(p.x)*100+60,Math.abs(p.y)*100+60])??[]));
  const scale = Math.min(w, h) / (range * 2);
  const X = (x: number) => w / 2 + x * scale; const Y = (y: number) => h / 2 - y * scale;
  ctx.strokeStyle = "rgba(63,80,86,.13)"; ctx.lineWidth = 1;
  for (let t = -range; t <= range; t += props.flight?100:10) { ctx.beginPath(); ctx.moveTo(X(t), 0); ctx.lineTo(X(t), h); ctx.moveTo(0, Y(t)); ctx.lineTo(w, Y(t)); ctx.stroke(); }
  ctx.fillStyle = "#f9faf5"; ctx.fillRect(X(-8.5), Y(8.5), 17 * scale, 17 * scale);
  ctx.fillStyle = "#19242a"; ctx.fillRect(X(-6), Y(6), 12 * scale, 12 * scale);
  ctx.fillStyle = "#f9faf5"; ctx.fillRect(X(-3), Y(3), 3 * scale, 3 * scale); ctx.fillRect(X(1), Y(-1), 3 * scale, 3 * scale);
  if (props.disturbance.tagOccluded) { ctx.fillStyle = "#333d40"; ctx.fillRect(X(-10.3), Y(10.3), 20.6 * scale, 20.6 * scale); }
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
    ctx.setLineDash([]); ctx.beginPath(); ctx.arc(X(0), Y(0), props.arriveTolCm * scale, 0, Math.PI * 2); ctx.stroke();
  }
  ctx.strokeStyle = "#27363d"; ctx.lineWidth = 5; ctx.beginPath(); ctx.moveTo(X(x) - 19, Y(y) - 19); ctx.lineTo(X(x) + 19, Y(y) + 19); ctx.moveTo(X(x) + 19, Y(y) - 19); ctx.lineTo(X(x) - 19, Y(y) + 19); ctx.stroke();
  for (const sx of [-1, 1]) for (const sy of [-1, 1]) { ctx.beginPath(); ctx.arc(X(x) + sx * 19, Y(y) + sy * 19, 11, 0, Math.PI * 2); ctx.stroke(); }
  ctx.fillStyle = "#d59734"; ctx.fillRect(X(x) - 7, Y(y) - 12, 14, 24);
  ctx.fillStyle = "#30414a"; ctx.font = "12px system-ui"; ctx.fillText(props.flight?`높이 ${(props.heightCm/100).toFixed(1)}m · 좌표 ${(x/100).toFixed(1)} / ${(y/100).toFixed(1)}m`:`높이 ${props.heightCm}cm · 좌표 ${x.toFixed(1)} / ${y.toFixed(1)}cm`, 18, h - 20);
}
