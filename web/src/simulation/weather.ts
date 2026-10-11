import {clamp} from './flightVisuals';

export type WeatherKind='clear'|'rain'|'snow';
export interface WeatherSettings {
  kind:WeatherKind; intensity:number; windSpeed:number; windDirection:number; gust:number; snowBase:number; seed:number;
}
export interface WeatherState extends WeatherSettings {elapsed:number;wetness:number;snowCover:number}
export const WEATHER_LABEL:Record<WeatherKind,string>={clear:'맑음',rain:'비',snow:'눈'};
export const DEFAULT_WEATHER:WeatherSettings={kind:'clear',intensity:.45,windSpeed:0,windDirection:90,gust:.25,snowBase:.06,seed:41011};
export const TAG_LOSS_LIMIT=30;
const finite=(value:unknown,fallback:number)=>typeof value==='number'&&Number.isFinite(value)?value:fallback;

export function normalizeWeather(value:Partial<WeatherSettings>={}):WeatherSettings {
  return {kind:value.kind==='rain'||value.kind==='snow'?value.kind:'clear',intensity:clamp(finite(value.intensity,.45),0,1),
    windSpeed:clamp(finite(value.windSpeed,0),0,8),windDirection:((finite(value.windDirection,90)%360)+360)%360,
    gust:clamp(finite(value.gust,.25),0,1),snowBase:clamp(finite(value.snowBase,.06),0,1),seed:Math.abs(Math.trunc(finite(value.seed,41011)))%100000};
}
export function createWeather(settings:Partial<WeatherSettings>=DEFAULT_WEATHER):WeatherState {
  const value=normalizeWeather(settings);
  return {...value,elapsed:0,wetness:value.kind==='rain'?value.intensity*.25:0,snowCover:value.kind==='snow'?value.snowBase:0};
}
/** 벽시계가 아닌 비행 시간만 사용해 일시정지·배속·기록 재생을 일치시킨다. */
export function advanceWeather(state:WeatherState,dt:number):WeatherState {
  const step=Math.max(0,Number.isFinite(dt)?dt:0);
  return {...state,elapsed:state.elapsed+step,
    wetness:clamp(state.wetness+step*(state.kind==='rain'?state.intensity*.035:-.012),0,1),
    snowCover:clamp(state.snowCover+step*(state.kind==='snow'?state.intensity*.0015:-.003),0,1)};
}
export function changeWeather(state:WeatherState,patch:Partial<WeatherSettings>):WeatherState {
  const config=normalizeWeather({...state,...patch}),changed=config.kind!==state.kind;
  return {...state,...config,wetness:changed&&config.kind==='rain'?Math.max(state.wetness,config.intensity*.25):state.wetness,
    snowCover:patch.snowBase!==undefined||changed?(config.kind==='snow'?config.snowBase:state.snowCover):state.snowCover};
}
/** 풍향은 바람이 향하는 방향이다. 0°=북쪽, 90°=동쪽. 외란은 훈련용 근사 모델이다. */
export function weatherWind(weather:WeatherState){
  const wave=Math.sin(weather.elapsed*.73+weather.seed*.013)*.65+Math.sin(weather.elapsed*1.91+weather.seed*.007)*.35;
  const speed=Math.max(0,weather.windSpeed*(1+weather.gust*wave));
  const angle=weather.windDirection*Math.PI/180;
  return {speed,x:Math.sin(angle)*speed,y:Math.cos(angle)*speed};
}
export function weatherKey(weather:Partial<WeatherSettings>){return JSON.stringify(normalizeWeather(weather));}
export const weatherActive=(weather:WeatherState)=>weather.kind!=='clear'||weather.windSpeed>0||weather.snowCover>0||weather.wetness>0;
export function readWeather():WeatherSettings {try{return normalizeWeather(JSON.parse(localStorage.getItem('mango-weather-v1')??'null')??DEFAULT_WEATHER);}catch{return {...DEFAULT_WEATHER};}}
export function saveWeather(weather:WeatherSettings){try{localStorage.setItem('mango-weather-v1',JSON.stringify(normalizeWeather(weather)));}catch{/* 저장이 제한되어도 현재 훈련은 계속 사용할 수 있다. */}}

/** 렌즈의 물방울을 원본 영상에 합성한다. 동일한 픽셀을 화면과 AprilTag 디코더에 전달한다. */
export function wetCameraPixels(pixels:Uint8ClampedArray,original:Uint8ClampedArray,weather:WeatherState,front:boolean){
  if(weather.kind!=='rain'||weather.intensity<=0)return;
  original.set(pixels);
  const fract=(n:number)=>n-Math.floor(n),hash=(n:number)=>fract(Math.sin(n*127.1+weather.seed)*43758.5453);
  const count=Math.floor(weather.intensity*(front?13:8));
  for(let i=0;i<count;i++){
    const cycle=Math.floor((weather.elapsed+i*3)/11),x=12+hash(i+cycle*.3)*296;
    const y=12+fract(hash(i+31)+weather.elapsed*(front?.009:.004))*216,r=3+hash(i+12)*8*weather.intensity;
    for(let py=Math.max(0,Math.floor(y-r));py<Math.min(240,y+r);py++)for(let px=Math.max(0,Math.floor(x-r));px<Math.min(320,x+r);px++){
      const dx=(px-x)/r,dy=(py-y)/r,d=Math.hypot(dx,dy);if(d>1)continue;
      const sx=clamp(Math.round(px+dx*4*(1-d)),0,319),sy=clamp(Math.round(py+dy*4*(1-d)),0,239),target=(py*320+px)*4,source=(sy*320+sx)*4;
      const haze=(d>.83?.3:.10)*weather.intensity;
      for(let c=0;c<3;c++)pixels[target+c]=original[source+c]*(1-haze)+(c===2?205:190)*haze;
    }
  }
}
