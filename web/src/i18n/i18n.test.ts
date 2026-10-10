import {afterEach, describe, expect, it} from 'vitest';
import {getLanguage, readLanguage, setLanguage, subscribeLanguage, t, translate} from '.';
import {messages} from './messages';
import {buildMissionWorld} from '../simulation/missionWorld';
import training from '../../../scenarios/demo_training_camp.json';

afterEach(() => setLanguage('en'));
describe('interface language', () => {
  it('defaults to English and accepts only supported saved preferences', () => {
    expect(readLanguage()).toBe('en');
    for (const value of [null, 'en', 'fr', '', 'KO']) expect(readLanguage({getItem: () => value})).toBe('en');
    expect(readLanguage({getItem: () => 'ko'})).toBe('ko');
    expect(readLanguage({getItem: () => {throw new Error('Storage unavailable');}})).toBe('en');
  });
  it('switches existing display text in memory without needing storage', () => {
    let notifications = 0;
    const unsubscribe = subscribeLanguage(() => notifications++);
    setLanguage('ko');
    expect(getLanguage()).toBe('ko');
    expect(t('기체 연결')).toBe('기체 연결');
    setLanguage('en');
    expect(t('기체 연결')).toBe('Connect aircraft');
    setLanguage('en');
    expect(notifications).toBe(2);
    unsubscribe();setLanguage('ko');expect(notifications).toBe(2);
  });
  it('localizes recorded dynamic events and nested place names at display time', () => {
    const event = '이착륙장으로 이동 · TAG 1';
    expect(translate(event, 'en')).toBe('Moving to Landing pad · TAG 1');
    expect(translate(event, 'ko')).toBe(event);
    expect(translate('영상 ID 3 확인 · 오차 2.4cm', 'en')).toBe('Image ID 3 verified · Error 2.4cm');
    expect(translate('입력 수신 1.2초 전', 'en')).toBe('Input received 1.2s ago');
    expect(translate('모터 2 출력', 'en')).toBe('Motor 2 output');
    expect(translate('전방 비행 카메라 확대', 'en')).toBe('Expand Forward flight camera');
    expect(translate('정보 없음 · MTF-01P 데이터 없음','en')).toBe('No information · No MTF-01P data');
    expect(translate('속도 P 게인 (vel_kp)','en')).toBe('Velocity P gain (vel_kp)');
    expect(translate('MTF-01P 거리·광류 경로 미확인: MTF-01P 데이터 없음','en')).toBe('MTF-01P range/optical-flow path unverified: No MTF-01P data');
  });
  it('preserves custom names, device identifiers, spacing, and scalar values', () => {
    for (const value of ['내가 만든 임무 123', 'D4D_Hacker_01', '{"type":"ap","mode":"off"}', '1516 µs']) expect(translate(value,'en')).toBe(value);
    expect(translate('  기체 연결 ','en')).toBe('  Connect aircraft ');
    expect(t(1516)).toBe(1516);expect(t(null)).toBe(null);expect(t(undefined)).toBeUndefined();
  });
  it('keeps simulation layout, route coordinates, and persistence keys independent of language', () => {
    setLanguage('en');const before = buildMissionWorld(training, []);
    setLanguage('ko');expect(buildMissionWorld(training, [])).toEqual(before);
  });
  it('keeps interpolation slots intact throughout the catalog', () => {
    const slots = (s:string) => [...new Set(s.match(/\{\d+\}/g) ?? [])].sort();
    for (const [source,target] of Object.entries(messages)) expect(slots(target),source).toEqual(slots(source));
  });
});
