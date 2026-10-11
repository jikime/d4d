import {Popover} from 'radix-ui';
import {CloudRain,CloudSun,Snowflake,Wind,X} from 'lucide-react';
import {Button} from './components/ui/button';
import {FormSelect,FormSlider} from './components/form-controls';
import {t,useLanguage} from './i18n';
import {WEATHER_LABEL,weatherWind,weatherActive,type WeatherSettings,type WeatherState,type WeatherKind} from './simulation/weather';
import './weatherControls.css';

export default function WeatherControls({weather,onChange,onOpen}:{weather:WeatherState;onChange:(patch:Partial<WeatherSettings>)=>void;onOpen:()=>void}){
  useLanguage();const Icon=weather.kind==='rain'?CloudRain:weather.kind==='snow'?Snowflake:CloudSun;
  return <Popover.Root onOpenChange={open=>{if(open)onOpen();}}><Popover.Trigger asChild><Button variant="outline" aria-label={t('날씨 환경 설정')} className="weather-trigger"><Icon size={16}/>{t(WEATHER_LABEL[weather.kind])}</Button></Popover.Trigger>
    <Popover.Portal><Popover.Content className="weather-popover" data-slot="weather-content" align="end" sideOffset={10} collisionPadding={12} aria-label={t('훈련 날씨')}>
      <header><div><small>TRAINING WEATHER</small><h3>{t('훈련 날씨')}</h3></div><Popover.Close asChild><Button variant="ghost" size="icon" aria-label={t('날씨 설정 닫기')}><X size={17}/></Button></Popover.Close></header>
      <FormSelect aria-label={t('날씨 종류')} value={weather.kind} onValueChange={kind=>onChange({kind:kind as WeatherKind})} options={Object.entries(WEATHER_LABEL).map(([value,label])=>({value,label}))}/>
      <label><span>{t('강수 강도')}<b>{Math.round(weather.intensity*100)}%</b></span><FormSlider aria-label={t('강수 강도')} min={0} max={1} step={.05} value={weather.intensity} disabled={weather.kind==='clear'} onValueChange={intensity=>onChange({intensity})}/></label>
      {weather.kind==='snow'&&<label><span>{t('시작 적설량')}<b>{Math.round(weather.snowBase*100)}%</b></span><FormSlider aria-label={t('시작 적설량')} min={0} max={1} step={.05} value={weather.snowBase} onValueChange={snowBase=>onChange({snowBase})}/></label>}
      <div className="weather-wind-title"><Wind size={15}/>{t('바람과 돌풍')}<span style={{transform:`rotate(${weather.windDirection}deg)`}}>↑</span></div>
      <label><span>{t('기준 풍속')}<b>{weather.windSpeed.toFixed(1)} m/s</b></span><FormSlider aria-label={t('기준 풍속')} min={0} max={8} step={.5} value={weather.windSpeed} onValueChange={windSpeed=>onChange({windSpeed})}/></label>
      <label><span>{t('바람 이동 방향')}<b>{weather.windDirection}°</b></span><FormSlider aria-label={t('바람 이동 방향')} min={0} max={345} step={15} value={weather.windDirection} onValueChange={windDirection=>onChange({windDirection})}/></label>
      <label><span>{t('돌풍 변화량')}<b>{Math.round(weather.gust*100)}%</b></span><FormSlider aria-label={t('돌풍 변화량')} min={0} max={1} step={.05} value={weather.gust} onValueChange={gust=>onChange({gust})}/></label>
      <p>{t('0° 북쪽 · 90° 동쪽으로 부는 바람입니다. 기체 반응은 훈련용 근사 모델입니다.')}</p>
      <div className="weather-surface"><span>{t('현재 적설')} <b>{Math.round(weather.snowCover*100)}%</b></span><span>{t('지면 젖음')} <b>{Math.round(weather.wetness*100)}%</b></span></div>
      <p>{t('날씨는 비행 시간에 맞춰 진행하며 일시정지하면 함께 멈춥니다. 적설은 실제 태그 영상에도 반영됩니다.')}</p>
    </Popover.Content></Popover.Portal>
  </Popover.Root>;
}

export function WeatherTelemetry({weather,attempts,matches,loss,crossTrack,stopped}:{weather:WeatherState;attempts:number;matches:number;loss:number;crossTrack:number;stopped?:boolean}){
  useLanguage();const wind=weatherWind(weather);
  return <section className="mc-weather-telemetry" aria-label={t('환경과 인식 상태')}><header><Wind size={13}/><strong>{t(WEATHER_LABEL[weather.kind])}</strong><span>{t('모의 환경')}</span></header><dl>
    <div><dt>{t('현재 풍속')}</dt><dd>{wind.speed.toFixed(1)} m/s</dd></div><div><dt>{t('경로 이탈 오차')}</dt><dd>{crossTrack.toFixed(2)} m</dd></div>
    <div><dt>{t('목적지 인식률')}</dt><dd>{attempts?Math.round(matches/attempts*100)+'%':'—'}</dd></div><div className={loss>5?'weather-loss':''}><dt>{t('연속 미인식')}</dt><dd>{loss.toFixed(1)}{weatherActive(weather)?' / 30':''} s</dd></div>
  </dl><small>{t('최근 5초의 목적지 관측 기준 · 이동 구간 제외')}</small>{!stopped&&loss>5&&weatherActive(weather)&&<p role="status">{t('위치를 유지하며 재인식 중입니다. 30초 연속 미인식 시 환경 훈련을 중단합니다.')}</p>}</section>;
}
