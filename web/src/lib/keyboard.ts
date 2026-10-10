/** 폼과 포털 메뉴의 키 입력을 비행 조작으로 해석하지 않는다. */
export const FLIGHT_INPUT_EXCLUSIONS = 'input,select,textarea,a,summary,[contenteditable=true],[role=combobox],[role=listbox],[role=option],[role=slider],[role=switch],[role=checkbox],[role=tab],[data-slot=select-content]';

export function ownsKeyboardInput(target: EventTarget | null, key?: string): boolean {
  if(!(target instanceof Element))return false;
  if(target.closest(FLIGHT_INPUT_EXCLUSIONS))return true;
  // 버튼을 누른 뒤에도 비행 단축키는 유지한다. Space/Enter는 버튼의 기본 동작이다.
  return (!key||key===' '||key==='Enter')&&!!target.closest('button');
}
