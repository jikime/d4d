import {t, useLanguage} from "./i18n";
import {useState} from 'react';
import {Activity, Radio} from 'lucide-react';
import {FormSelect} from './components/form-controls';
import {signalPath, type ThrottleReading} from './throttleTelemetry';

export function PulseMeter({value, label, text, tone = 'input'}: {value: number | null; label: string; text: string; tone?: 'input' | 'motor'}) {
  useLanguage();
  return <div className={'live-pulse-meter ' + tone + (value === null ? ' is-unavailable' : '')}
    role={value === null ? 'img' : 'meter'} aria-label={t(label)} aria-valuemin={value === null ? undefined : 0}
    aria-valuemax={value === null ? undefined : 100} aria-valuenow={value ?? undefined} aria-valuetext={t(text)}>
    <i style={{width: (value ?? 0) + '%'}}/>
    {value !== null && <b style={{left: Math.min(99, value) + '%'}}/>}
  </div>;
}

export default function ThrottlePanel({reading}: {reading: ThrottleReading}) {
  useLanguage();
  const [windowSeconds, setWindowSeconds] = useState('30');
  const windowMs = Number(windowSeconds) * 1000;
  const inputPath = signalPath(reading.inputHistory, reading.now, windowMs);
  const motorPath = signalPath(reading.motorHistory, reading.now, windowMs);
  return <section className="live-card live-throttle" aria-label={t("실제 스로틀 모니터링")}>
    <div className="live-card-heading"><Radio size={16}/><h3>{t("스로틀 모니터")}</h3><span className={reading.inputUs === null ? '' : 'is-good'}>{t(reading.inputUs === null ? '수신 대기' : '실측')}</span></div>
    <div className="live-throttle-body">
      <div className="live-throttle-label"><span>{t("조종기 스로틀 입력")}</span><small>{t("조절: 실제 조종기")}</small></div>
      <div className="live-throttle-value"><output aria-label={t("실제 스로틀 입력")}>{t(reading.inputUs ?? '—')}<small>µs</small></output><strong>{t(reading.inputPercent === null ? '—' : reading.inputPercent.toFixed(1))}<small>%</small></strong></div>
      <PulseMeter value={reading.inputPercent} label={t("조종기 스로틀 막대")} text={t(reading.inputUs === null ? '수신 대기' : reading.inputUs + ' 마이크로초')}/>
      <div className="live-meter-scale"><span>1,000</span><span>1,500</span><span>2,000 µs</span></div>
      <div className="live-output-label"><span>{t("FC 모터 출력 평균")}</span><output aria-label={t("모터 출력 평균")}>{t(reading.motorMeanUs === null ? '—' : reading.motorMeanUs.toFixed(0))} <small>µs</small></output></div>
      <PulseMeter value={reading.motorPercent} label={t("FC 모터 평균 출력 막대")} text={t(reading.motorMeanUs === null ? '수신 대기' : reading.motorMeanUs.toFixed(0) + ' 마이크로초')} tone="motor"/>
      <p className="live-throttle-caption">{t("막대는 1,000–2,000µs 기준입니다. 추력·RPM 비율과는 다릅니다.")}</p>
      <div className="live-trend-heading"><span><Activity size={12}/>{t("출력 변화")}</span><FormSelect aria-label={t("스로틀 기록 표시 구간")} value={windowSeconds} onValueChange={setWindowSeconds} options={[{value:'15',label:'최근 15초'},{value:'30',label:'최근 30초'},{value:'60',label:'최근 60초'}]}/></div>
      <div className="live-throttle-chart">
        <svg viewBox="0 0 280 88" role="img" aria-label={t('최근 ' + windowSeconds + '초 조종기 입력과 FC 모터 평균 출력 그래프')}>
          <path className="live-chart-grid" d="M8,12 H272 M8,44 H272 M8,76 H272"/>
          <text x="8" y="10">2,000</text><text x="8" y="42">1,500</text>
          <path className="live-chart-input" d={inputPath}/><path className="live-chart-motor" d={motorPath}/>
        </svg>
        {!inputPath && !motorPath && <span className="live-chart-empty">{t("수신한 값부터 기록합니다")}</span>}
      </div>
      <div className="live-chart-legend"><span><i/>{t("조종기")}</span><span><i/>{t("모터 평균")}</span><small>{t(windowSeconds)}{t("초 전 → 현재")}</small></div>
      <p className="live-throttle-source">{t(reading.inputAge === null ? '유효한 조종기 입력을 기다리고 있습니다.' : '입력 수신 ' + (reading.inputAge / 1000).toFixed(1) + '초 전')}<br/>{t(reading.boardControl === false ? '현재 기체의 스로틀 권한은 조종기에 있습니다.' : '웹에서는 수신값을 모니터링합니다.')}</p>
    </div>
  </section>;
}
