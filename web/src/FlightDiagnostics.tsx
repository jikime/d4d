import {memo,useEffect,useState,type RefObject} from 'react';
import {t,useLanguage} from './i18n';
import type {MissionFlight} from './simulation/missionFlight';
export interface CameraMetrics {received:number[];latency:number;lastCapture:number}
export const emptyCameraMetrics=():CameraMetrics=>({received:[],latency:0,lastCapture:0});

export default memo(function FlightDiagnostics({running,speed,engine,metrics}:{running:boolean;speed:number;engine:RefObject<MissionFlight|null>;metrics:RefObject<CameraMetrics>}){
  useLanguage();
  const [sample,setSample]=useState({fps:0,age:0,latency:0,rate:0,status:'loading'});
  useEffect(()=>{
    let wall=performance.now(),sim=engine.current!.state.tMs;
    const update=()=>{const now=performance.now(),state=engine.current!.state,m=metrics.current;
      const recent=m.received.filter(t=>now-t<2000),fps=recent.length>1?(recent.length-1)*1000/(recent.at(-1)!-recent[0]):0;
      setSample({fps,age:m.lastCapture?now-m.lastCapture:0,latency:m.latency,rate:running?Math.max(0,(state.tMs-sim)/(now-wall||1)):0,status:state.vision.status});wall=now;sim=state.tMs;
    };update();const timer=setInterval(update,1000);return()=>clearInterval(timer);
  },[running,speed,engine,metrics]);
  const reason=!running?'카메라 일시정지':sample.status==='error'?'영상 연결 오류':sample.status==='loading'?'디코더 준비 중':sample.age>1500||sample.fps===0?'새 영상 대기':sample.rate<speed*.8?'처리 속도에 맞춰 재생 중':'정상 재생';
  return <div className="mc-diagnostics" aria-label={t('영상·재생 상태')}><div><span>{t('인식 영상')}</span><b>{running?sample.fps.toFixed(1):'—'} fps</b></div><div><span>{t('영상 처리 지연')}</span><b>{sample.latency?Math.round(sample.latency):'—'} ms</b></div><div><span>{t('영상 경과 시간')}</span><b>{running&&sample.age?Math.round(sample.age):'—'} ms</b></div><div><span>{t('실제 / 선택 배속')}</span><b>{sample.rate.toFixed(1)}× / {speed}×</b></div><small>{t(reason)}</small></div>;
});
