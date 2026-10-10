import {useSyncExternalStore} from 'react';
import {messages} from './messages';

export type Language = 'en' | 'ko';
export const LANGUAGE_STORAGE_KEY = 'spider-language';
export const DEFAULT_LANGUAGE: Language = 'en';
export function readLanguage(storage?: Pick<Storage, 'getItem'>): Language {
  try { return storage?.getItem(LANGUAGE_STORAGE_KEY) === 'ko' ? 'ko' : DEFAULT_LANGUAGE; }
  catch { return DEFAULT_LANGUAGE; }
}
let language: Language = (() => { try { return readLanguage(window.localStorage); } catch { return DEFAULT_LANGUAGE; } })();
const listeners = new Set<() => void>();
export const getLanguage = () => language;
export const subscribeLanguage = (listener: () => void) => { listeners.add(listener); return () => { listeners.delete(listener); }; };
function publish(next: Language) { if (language !== next) { language = next; listeners.forEach(listener => listener()); } }
export function setLanguage(next: Language) {
  if (next !== 'en' && next !== 'ko') return;
  try { window.localStorage.setItem(LANGUAGE_STORAGE_KEY, next); } catch { /* 메모리에서는 계속 전환한다. */ }
  publish(next);
}
if (typeof window !== 'undefined') window.addEventListener('storage', event => {
  if (event.key === LANGUAGE_STORAGE_KEY || event.key === null) publish(event.newValue === 'ko' ? 'ko' : DEFAULT_LANGUAGE);
});
export const useLanguage = () => useSyncExternalStore(subscribeLanguage, getLanguage, () => DEFAULT_LANGUAGE);
export const locale = (value = language) => value === 'ko' ? 'ko-KR' : 'en-US';

const escapeRegex = (value: string) => value.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
// 상태·이벤트의 원문은 유지하고 표시할 때 번역한다. 기존 기록도 언어 전환에 함께 반응한다.
const patterns = Object.entries(messages).filter(([source]) => /\{\d+\}/.test(source)).map(([source, target]) => {
  const ids: string[] = [];
  const parts = source.split(/(\{\d+\})/g).map(part => /^\{\d+\}$/.test(part) ? (ids.push(part.slice(1, -1)), '([\\s\\S]*?)') : escapeRegex(part));
  return {regex: new RegExp('^' + parts.join('') + '$'), ids, target, specificity: source.replace(/\{\d+\}/g, '').length};
}).sort((a,b) => b.specificity - a.specificity);
const cache = new Map<string, string>();
export function translate(value: string, targetLanguage: Language = language, depth = 0): string {
  if (targetLanguage === 'ko' || !/[가-힣]/.test(value)) return value;
  const source = value.trim().replace(/\s+/g, ' ');
  let result = messages[source] ?? cache.get(source);
  if (result === undefined && depth < 4) {
    for (const pattern of patterns) {
      const match = pattern.regex.exec(source);
      if (!match) continue;
      result = pattern.target.replace(/\{(\d+)\}/g, (_, id: string) => translate(match[pattern.ids.indexOf(id) + 1] ?? '', targetLanguage, depth + 1));
      break;
    }
    if (result === undefined && source.includes(' · ')) {
      result = source.split(' · ').map(part => translate(part, targetLanguage, depth + 1)).join(' · ');
    }
    if (result === undefined && source.startsWith(': ')) result = ': ' + translate(source.slice(2), targetLanguage, depth + 1);
  }
  if (result === undefined) return value; // 사용자 임무명·원시 장치 로그는 임의로 바꾸지 않는다.
  if (cache.size > 2000) cache.clear();
  cache.set(source, result);
  return (value.match(/^\s*/)?.[0] ?? '') + result + (value.match(/\s*$/)?.[0] ?? '');
}
export function t(value: string, targetLanguage?: Language): string;
export function t<T extends string | number | boolean | null | undefined>(value: T, targetLanguage?: Language): T;
export function t(value: string | number | boolean | null | undefined, targetLanguage?: Language) {
  // 기존 임무·저장 기록의 브랜드는 표시할 때만 바꿔 원본 ID와 통신 값을 유지한다.
  return typeof value === 'string' ? translate(value, targetLanguage).replace(/spider|스파이더/gi, 'MANGO') : value;
}
