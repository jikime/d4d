import {lazy, Suspense, useEffect, useState} from 'react';
import {RefreshCw} from 'lucide-react';
import {t, useLanguage} from './i18n';
import type {SimulatorLaunch} from './Simulator';

const Simulator = lazy(() => import('./Simulator'));

// 실기체 패널을 마운트하지 않는 시연에서도 정밀 접근 시험을 열 수 있게 한다.
export default function SimulationDialogHost() {
  useLanguage();
  const [launch, setLaunch] = useState<SimulatorLaunch | null>(null);
  useEffect(() => {
    const open = (event: Event) => setLaunch((event as CustomEvent<SimulatorLaunch>).detail ?? {initialMode: 'flight'});
    window.addEventListener('spider:open-simulator', open);
    return () => window.removeEventListener('spider:open-simulator', open);
  }, []);
  if (!launch) return null;
  return <Suspense fallback={<div className="sim-loading" role="status"><RefreshCw className="spin"/><strong>{t('가상 시험 공간 준비 중')}</strong><button className="button ghost" onClick={() => setLaunch(null)}>{t('취소')}</button></div>}>
    <Simulator launch={launch} onClose={() => setLaunch(null)}/>
  </Suspense>;
}
