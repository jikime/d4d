import {useEffect} from 'react';
import {Languages} from 'lucide-react';
import {FormSelect} from '../components/form-controls';
import {setLanguage, t, useLanguage, type Language} from '.';
import './language.css';

export function LanguageSwitcher() {
  const language = useLanguage();
  return <div className="language-switcher">
    <Languages size={14} aria-hidden="true"/>
    <FormSelect aria-label={t('언어')} value={language} onValueChange={value => setLanguage(value as Language)}
      options={[{value:'en',label:'EN'},{value:'ko',label:'한국어'}]}/>
  </div>;
}
export function LanguageMetadata() {
  const language = useLanguage();
  useEffect(() => {
    document.documentElement.lang = language;
    document.title = t('MANGO 전술지휘소');
    document.querySelector('meta[name="description"]')?.setAttribute('content', t('MANGO 임무 계획·검증·승인과 3D 접근 제어 시뮬레이터'));
  }, [language]);
  return <a className="skip-link" href="#main">{t('주요 콘텐츠로 이동')}</a>;
}
